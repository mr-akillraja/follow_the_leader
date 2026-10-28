#!/usr/bin/env python3
"""
pso_leader_node.py  –  Closed-Loop Swarm Formation Controller
==============================================================
Manages a 3-drone swarm (alpha = leader, beta_1 / beta_2 = followers).

Key fixes over the previous version
-------------------------------------
1. Velocity clamping now works for BOTH positive AND negative values.
   (old code used `min(v, MAX)` which never clamped negative velocities)

2. Per-axis low-pass filter on follower commands prevents jerky oscillation.

3. Obstacle repulsion is normalised and capped — no more runaway commands
   when a drone is very close to an obstacle.

4. Formation transition uses a blended interpolation so drones don't
   jump target positions abruptly.

5. Yaw control uses a proper angular P-controller with clamping, replacing
   the raw `3.0 * yaw_err` that caused spinning.

6. Followers match the leader's velocity feed-forward so they don't lag
   when the swarm is moving at speed.

7. Dead-band on position error stops constant micro-corrections at rest.

Coordinate convention
---------------------
  +X  = Forward    +Y = Left    +Z = Up  (ROS standard)
  Yaw = CCW positive

Formation offsets (leader-relative, local frame)
-------------------------------------------------
  Triangle :  beta_1 → (-1.8m,  1.5m)   beta_2 → (-1.8m, -1.5m)
  Column   :  beta_1 → (-2.0m,  0.0m)   beta_2 → (-4.0m,  0.0m)
  Column is automatically selected when any obstacle is within 2.5 m of alpha.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
import math


# ──────────────────────────────────────────────────────────────────────────────
#  Tuneable constants
# ──────────────────────────────────────────────────────────────────────────────

# Formation gains
KV_POSITION      = 1.8   # P-gain for position error → translational velocity
KV_YAW           = 2.0   # P-gain for yaw error → angular velocity
MAX_FOLLOWER_VEL = 3.5   # m/s  – per-axis translational speed limit
MAX_YAW_RATE     = 1.8   # rad/s – angular speed limit for followers

# Feed-forward fraction: followers inherit this proportion of leader velocity
FF_GAIN          = 0.8

# Low-pass filter coefficient (0=no smoothing, 1=no change)
LP_ALPHA         = 0.35

# Obstacle avoidance
OBS_SAFETY_MARGIN = 0.6   # m added to obstacle radius
OBS_REP_GAIN      = 4.0   # repulsion magnitude gain
OBS_REP_CAP       = 2.5   # m/s  – max repulsion contribution per obstacle
OBS_INFLUENCE_DIST = 2.5  # m    – formation switches to column inside this

# Dead-band: ignore errors smaller than this
POS_DEADBAND     = 0.05  # m
YAW_DEADBAND     = 0.03  # rad


def clamp(value: float, limit: float) -> float:
    """Symmetrically clamp value to [-limit, +limit]."""
    return max(-abs(limit), min(abs(limit), value))


def angle_wrap(a: float) -> float:
    """Wrap angle to (-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


class LowPassFilter:
    """First-order low-pass filter for a scalar signal."""

    def __init__(self, alpha: float = LP_ALPHA):
        """alpha ∈ (0, 1) — higher alpha = more smoothing (slower response)."""
        self._alpha = alpha
        self._value = 0.0
        self._initialised = False

    def update(self, new_val: float) -> float:
        if not self._initialised:
            self._value = new_val
            self._initialised = True
        else:
            self._value = self._alpha * self._value + (1.0 - self._alpha) * new_val
        return self._value

    def reset(self, val: float = 0.0):
        self._value = val
        self._initialised = True


class DroneState:
    """Holds the latest pose estimate for one drone."""

    def __init__(self):
        self.x   = 0.0
        self.y   = 0.0
        self.z   = 0.0
        self.yaw = 0.0

    @staticmethod
    def quat_to_yaw(q) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)


class FollowerController:
    """
    Per-follower state + low-pass filters for smooth output.
    """

    def __init__(self, follower_index: int):
        self.idx = follower_index
        self.lpf_vx  = LowPassFilter()
        self.lpf_vy  = LowPassFilter()
        self.lpf_wz  = LowPassFilter()

    def reset_filters(self):
        for f in (self.lpf_vx, self.lpf_vy, self.lpf_wz):
            f.reset(0.0)


class PSOLeaderNode(Node):
    """
    Closed-Loop Swarm Formation Node.

    Subscribes to /tf for real-time drone poses.
    Subscribes to /cmd_vel_keyboard for manual alpha commands.
    Publishes /{name}/cmd_vel_pso for each drone.
    """

    DRONE_NAMES = ['alpha', 'beta_1', 'beta_2']

    # Formation offsets [local frame, per follower]
    # Key: formation name  →  list indexed by drone index (index 0 unused)
    FORMATIONS = {
        'triangle': [None, (-1.8,  1.5), (-1.8, -1.5)],
        'column'  : [None, (-2.0,  0.0), (-4.0,  0.0)],
    }

    # Obstacles: (x, y, radius)
    OBSTACLES = [
        (3.0,  2.0, 0.8),
        (5.0, -3.0, 0.8),
        (7.0,  3.0, 0.8),
        (8.0, -1.0, 0.8),
    ]

    def __init__(self):
        super().__init__('pso_leader_node')

        # Drone state objects
        self.states = {name: DroneState() for name in self.DRONE_NAMES}

        # Follower controllers (indices 1 and 2)
        self.followers = {
            'beta_1': FollowerController(1),
            'beta_2': FollowerController(2),
        }

        # Latest keyboard command for alpha
        self.leader_cmd = Twist()

        # Current and target formation (blended during transition)
        self._formation      = 'triangle'
        self._prev_formation = 'triangle'
        self._blend_t        = 1.0   # 1.0 = fully in current formation

        # ── ROS publishers ──
        self.cmd_pubs: dict[str, rclpy.publisher.Publisher] = {}
        for name in self.DRONE_NAMES:
            self.cmd_pubs[name] = self.create_publisher(
                Twist, f'/{name}/cmd_vel_pso', 10
            )

        # ── ROS subscribers ──
        self.create_subscription(Twist, '/cmd_vel_keyboard',
                                 self._keyboard_cb, 10)
        self.create_subscription(TFMessage, '/tf',
                                 self._tf_cb, 10)

        # ── Control loop: 20 Hz ──
        self.dt = 0.05
        self.create_timer(self.dt, self._control_loop)

        self.get_logger().info('Closed-Loop Swarm Formation Controller started.')
        self.get_logger().info(f'Drones : {self.DRONE_NAMES}')
        self.get_logger().info(f'Obstacles : {self.OBSTACLES}')

    # ──────────────────────────────────────────────────────────
    #  Callbacks
    # ──────────────────────────────────────────────────────────

    def _tf_cb(self, msg: TFMessage):
        """Update drone poses from /tf."""
        for tf in msg.transforms:
            if tf.header.frame_id != 'swarm_world':
                continue
            child = tf.child_frame_id
            if child not in self.states:
                continue

            t = tf.transform
            s = self.states[child]
            s.x   = t.translation.x
            s.y   = t.translation.y
            s.z   = t.translation.z
            s.yaw = DroneState.quat_to_yaw(t.rotation)

    def _keyboard_cb(self, msg: Twist):
        """Store latest manual velocity command for alpha."""
        self.leader_cmd = msg

    # ──────────────────────────────────────────────────────────
    #  Formation selection
    # ──────────────────────────────────────────────────────────

    def _nearest_obstacle_distance(self, x: float, y: float) -> float:
        """Return the closest clear distance from (x,y) to any obstacle."""
        min_d = float('inf')
        for ox, oy, orad in self.OBSTACLES:
            d = math.hypot(x - ox, y - oy) - orad
            min_d = min(min_d, d)
        return min_d

    def _update_formation(self, alpha: DroneState):
        desired = (
            'column'
            if self._nearest_obstacle_distance(alpha.x, alpha.y)
               < OBS_INFLUENCE_DIST
            else 'triangle'
        )
        if desired != self._formation:
            self._prev_formation = self._formation
            self._formation      = desired
            self._blend_t        = 0.0   # restart blend
            self.get_logger().info(f'Formation switch: {self._prev_formation} → {desired}')

        # Advance blend (0 → 1 over 1 second at 20 Hz → 20 ticks)
        self._blend_t = min(1.0, self._blend_t + self.dt)

    def _get_blended_offset(self, follower_name: str) -> tuple[float, float]:
        """Return smoothly interpolated local-frame offset for a follower."""
        idx  = self.DRONE_NAMES.index(follower_name)
        prev = self.FORMATIONS[self._prev_formation][idx]
        curr = self.FORMATIONS[self._formation][idx]

        t = self._blend_t
        # Smooth-step interpolation
        t_smooth = t * t * (3 - 2 * t)

        ox = prev[0] + (curr[0] - prev[0]) * t_smooth
        oy = prev[1] + (curr[1] - prev[1]) * t_smooth
        return ox, oy

    # ──────────────────────────────────────────────────────────
    #  Obstacle repulsion helper
    # ──────────────────────────────────────────────────────────

    def _obstacle_repulsion(self, px: float, py: float) -> tuple[float, float]:
        """
        Compute combined repulsive velocity vector (global frame) for position (px, py).
        Each obstacle contributes a capped repulsion scaled by inverse distance.
        """
        rvx, rvy = 0.0, 0.0
        for ox, oy, orad in self.OBSTACLES:
            dx  = px - ox
            dy  = py - oy
            d   = math.hypot(dx, dy)
            safety = orad + OBS_SAFETY_MARGIN

            if d < safety and d > 1e-6:
                # Repulsion magnitude: large when close, zero at safety boundary
                mag = OBS_REP_GAIN * (1.0 / d - 1.0 / safety)
                # Unit direction (away from obstacle) × magnitude
                rvx += clamp((dx / d) * mag, OBS_REP_CAP)
                rvy += clamp((dy / d) * mag, OBS_REP_CAP)

        return rvx, rvy

    # ──────────────────────────────────────────────────────────
    #  Main control loop
    # ──────────────────────────────────────────────────────────

    def _control_loop(self):
        alpha = self.states['alpha']

        # ── 1. Alpha: direct pass-through ──
        self.cmd_pubs['alpha'].publish(self.leader_cmd)

        # ── 2. Formation selection ──
        self._update_formation(alpha)

        # ── 3. Followers ──
        for fname, fc in self.followers.items():
            follower = self.states[fname]
            self._compute_follower_cmd(alpha, follower, fname, fc)

    def _compute_follower_cmd(
        self,
        alpha   : DroneState,
        follower: DroneState,
        fname   : str,
        fc      : FollowerController,
    ):
        # ── Target position (global frame) ──
        ox_loc, oy_loc = self._get_blended_offset(fname)

        cos_a = math.cos(alpha.yaw)
        sin_a = math.sin(alpha.yaw)

        # Rotate local offset into global frame
        tx = alpha.x + ox_loc * cos_a - oy_loc * sin_a
        ty = alpha.y + ox_loc * sin_a + oy_loc * cos_a

        # ── Position error (global) ──
        ex = tx - follower.x
        ey = ty - follower.y
        dist = math.hypot(ex, ey)

        # Apply dead-band to reduce micro-corrections when close
        if dist < POS_DEADBAND:
            ex, ey = 0.0, 0.0

        # ── Proportional velocity command (global frame) ──
        vx_glob = KV_POSITION * ex
        vy_glob = KV_POSITION * ey

        # ── Feed-forward: inherit a fraction of alpha's velocity ──
        vx_glob += FF_GAIN * self.leader_cmd.linear.x
        vy_glob += FF_GAIN * self.leader_cmd.linear.y

        # ── Obstacle repulsion (global frame) ──
        rvx, rvy = self._obstacle_repulsion(follower.x, follower.y)
        vx_glob += rvx
        vy_glob += rvy

        # ── Convert global → drone LOCAL frame ──
        cos_f =  math.cos(follower.yaw)
        sin_f =  math.sin(follower.yaw)
        vx_loc =  vx_glob * cos_f + vy_glob * sin_f
        vy_loc = -vx_glob * sin_f + vy_glob * cos_f

        # ── Clamp translational velocities ──
        vx_loc = clamp(vx_loc, MAX_FOLLOWER_VEL)
        vy_loc = clamp(vy_loc, MAX_FOLLOWER_VEL)

        # ── Yaw control ──
        # Decide desired heading
        speed_glob = math.hypot(vx_glob - rvx, vy_glob - rvy)   # excluding repulsion
        if speed_glob > 0.3:
            # Face the direction of travel
            desired_yaw = math.atan2(vy_glob - rvy, vx_glob - rvx)
        else:
            # Stay aligned with leader when almost stationary
            desired_yaw = alpha.yaw

        yaw_err = angle_wrap(desired_yaw - follower.yaw)

        # Apply dead-band
        if abs(yaw_err) < YAW_DEADBAND:
            yaw_err = 0.0

        wz = clamp(KV_YAW * yaw_err, MAX_YAW_RATE)

        # ── Low-pass filter ──
        vx_loc = fc.lpf_vx.update(vx_loc)
        vy_loc = fc.lpf_vy.update(vy_loc)
        wz     = fc.lpf_wz.update(wz)

        # ── Publish ──
        cmd = Twist()
        cmd.linear.x  = vx_loc
        cmd.linear.y  = vy_loc
        cmd.angular.z = wz
        self.cmd_pubs[fname].publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PSOLeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()