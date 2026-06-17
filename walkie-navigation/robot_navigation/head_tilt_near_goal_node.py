#!/usr/bin/env python3
"""Tilt the ZED head down when the robot is near its goal.

Problem this solves
-------------------
The Spatio-Temporal Voxel Layer marks tabletop obstacles from the ZED head, and
Nav2 keeps the robot off them. But at very close range the table drops below the
head camera's field of view (a "blind spot"); with nothing re-marking it the
voxels decay and the robot drives into the table.

What this node does
-------------------
Instead of stopping and freezing the costmap, we keep the obstacle in view: as
the robot closes on the goal we pitch the ZED head down so the near tabletop
stays in the camera FOV and STVL keeps marking it live (no freeze, no stop).

Cooperative, edge-triggered, fire-once
-------------------------------------
It does NOT own the head. It starts a SINGLE move on the moments that matter and
is otherwise silent, so any other system can drive the head between those
moments:

  * crossing INTO  near_distance  -> move head down to `down_angle`  (once)
  * crossing OUT of far_distance  -> move head to `default_angle`    (once)
  * goal ends (feedback idle)     -> move head to `default_angle`    (once)

The latch re-arms on every new goal, so it tilts on every approach (it never
relies on a remembered "current pose", which would silently desync if anything
else moved the head). A plain volatile publisher is used so we never hold/latch
the topic.

A down-tilt only fires once the robot has genuinely been OUTSIDE the near zone
for the current goal (an "approach armed" flag). This means a goal that only
changes heading -- robot already at the goal position, distance_remaining never
exceeds `near_distance` -- never tilts, since there was no real approach.

Yielding the camera
-------------------
An external (non-ROS) agent sometimes aims the camera too. A `SetBool` service
(`~/enable`, i.e. `/head_tilt_near_goal/enable`) gates this node: call it with
`data: false` and the node goes fully silent -- it issues no head commands and
drops any in-flight motion, leaving the head wherever it is for the other
controller. `data: true` hands control back; the next approach re-evaluates from
a clean slate.

Motion profile
--------------
The position controller jumps to whatever setpoint it last received, so a single
command snaps the head over at full speed. To soften that, each move is shaped
into a velocity-limited, smoothstep-eased ramp: instead of one command we stream
intermediate setpoints at `profile_rate` Hz, easing in and out so the head
glides to the target over `|delta| / tilt_speed` seconds. Outside an active
move the profile timer is idle, so the cooperative "silent between edges"
behaviour is preserved -- a move is just spread over a short window now.

The head is the `head_servo_joint`, driven by `head_servo_controller`
(JointGroupPositionController), commanded with a Float64MultiArray of one angle
(rad). The joint limit is +/-0.785398 rad; commands are clamped to it.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
from nav2_msgs.action import NavigateToPose

# The wrapper message published on the hidden action feedback topic. It carries
# the goal_id (so we can detect a new goal) plus the NavigateToPose feedback.
FeedbackMessage = NavigateToPose.Impl.FeedbackMessage

# Hard limit of head_servo_joint (see servo_camera.urdf.xacro).
JOINT_LIMIT = 0.785398163


class HeadTiltNearGoal(Node):
    def __init__(self):
        super().__init__("head_tilt_near_goal")

        self.declare_parameter("head_command_topic", "/head_servo_controller/commands")
        # Tilt down when crossing into near_distance; return to default when
        # crossing back past far_distance (far > near gives hysteresis).
        self.declare_parameter("near_distance", 1.5)
        self.declare_parameter("far_distance", 1.7)
        # Distances at or below this are ignored as "not a real approach": a
        # heading-only / in-place-rotation goal sits at a tiny distance_remaining
        # the whole time, so treating it as noise keeps the head from tilting.
        self.declare_parameter("near_ignore_distance", 0.3)
        # Head angles (rad). default = forward-ish nav pose, down = look at table.
        self.declare_parameter("default_angle", 0.25)
        self.declare_parameter("down_angle", 0.785)
        # If we tilted down and feedback then goes silent this long, the goal has
        # ended (feedback only streams while the action is active) -> return to
        # default once so the head isn't left looking down between goals.
        self.declare_parameter("idle_return_timeout", 2.0)
        self.declare_parameter("feedback_topic", "/navigate_to_pose/_action/feedback")
        # Motion profile: peak tilt speed (rad/s) and setpoint stream rate (Hz).
        # Lower tilt_speed = gentler/slower head motion. A move of size |delta|
        # takes about |delta| / tilt_speed seconds, eased in and out.
        self.declare_parameter("tilt_speed", 0.6)
        self.declare_parameter("profile_rate", 30.0)
        # Runtime gate. While disabled this node issues no head commands at all,
        # so an external camera controller can own the head. start_enabled sets
        # the boot state; enable_service is the SetBool service name.
        self.declare_parameter("enable_service", "~/enable")
        self.declare_parameter("start_enabled", True)

        self._cmd_topic = self.get_parameter("head_command_topic").value
        self._near = self.get_parameter("near_distance").value
        self._far = self.get_parameter("far_distance").value
        self._near_ignore = self.get_parameter("near_ignore_distance").value
        self._default_angle = self._clamp(self.get_parameter("default_angle").value)
        self._down_angle = self._clamp(self.get_parameter("down_angle").value)
        self._idle_timeout = self.get_parameter("idle_return_timeout").value
        feedback_topic = self.get_parameter("feedback_topic").value
        self._tilt_speed = max(1e-3, float(self.get_parameter("tilt_speed").value))
        self._profile_rate = max(1.0, float(self.get_parameter("profile_rate").value))
        self._enabled = bool(self.get_parameter("start_enabled").value)

        # True once we have issued the down command for the current approach. Reset
        # to False on a new goal and after we return to default. This is a one-shot
        # latch for OUR command edges only -- it is NOT a belief about where the
        # head physically is, so other systems moving the head can't desync us.
        self._down_sent = False
        # Armed once we've seen the robot genuinely OUTSIDE the near zone for the
        # current goal. Required before any down-tilt, so a goal that is already
        # near at acceptance (heading-only / in-place rotation) never tilts.
        self._approach_armed = False
        # goal_id of the goal we are currently tracking.
        self._last_goal_id = None
        # clock time of the most recent feedback (None = none yet).
        self._last_feedback_time = None

        # Motion-profile state. We assume the head starts at default; the ramp
        # start is always the last setpoint WE published, so a move retargeted
        # mid-flight (e.g. crossing back out while still tilting) eases smoothly
        # from wherever we currently are rather than snapping.
        self._last_commanded_angle = self._default_angle
        self._ramp_active = False
        self._ramp_start_angle = self._default_angle
        self._ramp_target_angle = self._default_angle
        self._ramp_start_time = None
        self._ramp_duration = 0.0

        # Plain volatile publisher: we publish single shots and never latch/hold
        # the topic, leaving the head free for other systems between our edges.
        self._cmd_pub = self.create_publisher(Float64MultiArray, self._cmd_topic, 10)

        self.create_subscription(FeedbackMessage, feedback_topic, self._feedback_cb, 10)

        # Watchdog: return the head to default once feedback has been silent past
        # the idle timeout (the goal ended).
        self._watchdog_timer = self.create_timer(0.5, self._idle_watchdog)

        # Profile timer: streams eased setpoints while a move is in flight, and
        # returns immediately (silent) when no move is active.
        self._profile_timer = self.create_timer(1.0 / self._profile_rate, self._profile_tick)

        # Runtime enable/disable gate so an external controller can own the head.
        enable_service_name = self.get_parameter("enable_service").value
        self._enable_srv = self.create_service(SetBool, enable_service_name, self._set_enabled_cb)

        self.get_logger().info(
            f"Head-tilt manager up ({'ENABLED' if self._enabled else 'DISABLED'}): tilt to "
            f"{self._down_angle:.3f} rad on '{self._cmd_topic}' when distance_remaining "
            f"crosses <= {self._near} m (back to {self._default_angle:.3f} rad past {self._far} m). "
            f"Toggle via SetBool service '{enable_service_name}'."
        )

    @staticmethod
    def _clamp(angle):
        return max(-JOINT_LIMIT, min(JOINT_LIMIT, float(angle)))

    def _set_enabled_cb(self, request, response):
        self._enabled = bool(request.data)
        if not self._enabled:
            # Go silent immediately: drop any in-flight ramp so we stop publishing
            # and leave the head wherever it is for the external controller.
            self._ramp_active = False
        else:
            # Re-evaluate cleanly on the next feedback edge after regaining control.
            self._down_sent = False
        response.success = True
        response.message = "enabled" if self._enabled else "disabled"
        self.get_logger().info(f"Head-tilt {'ENABLED' if self._enabled else 'DISABLED'} via service")
        return response

    def _feedback_cb(self, msg: FeedbackMessage):
        self._last_feedback_time = self.get_clock().now()

        goal_id = bytes(msg.goal_id.uuid)
        if goal_id != self._last_goal_id:
            # New goal -> re-arm. If we were left tilted from a previous approach,
            # restore the default once so this approach starts clean.
            self._last_goal_id = goal_id
            if self._down_sent:
                self._command(self._default_angle, "default (new goal)")
            self._down_sent = False
            self._approach_armed = False
            return

        distance = msg.feedback.distance_remaining
        # distance_remaining is 0.0 until the first path is available, and tiny
        # for a heading-only / in-place-rotation goal; ignore anything at or
        # below near_ignore_distance so such goals never drive the head.
        if distance <= self._near_ignore:
            return

        # Only treat this as a real approach once the robot has been clearly
        # outside the near zone; a heading-only goal stays below `near` and so
        # never arms, never tilts.
        if distance > self._near:
            self._approach_armed = True

        if self._approach_armed and distance <= self._near and not self._down_sent:
            self._command(self._down_angle, "DOWN (near goal)")
            self._down_sent = True
        elif distance >= self._far and self._down_sent:
            self._command(self._default_angle, "default (left near zone)")
            self._down_sent = False

    def _idle_watchdog(self):
        # Only acts if we tilted down and feedback then stopped (goal ended).
        if not self._enabled or not self._down_sent or self._last_feedback_time is None:
            return
        idle = (self.get_clock().now() - self._last_feedback_time).nanoseconds * 1e-9
        if idle >= self._idle_timeout:
            self._command(self._default_angle, f"default (goal ended, idle {idle:.1f}s)")
            self._down_sent = False

    def _command(self, angle: float, why: str):
        # Start a profiled move from our last setpoint to `angle`. The profile
        # timer streams the eased setpoints; we don't publish the full command
        # here. Retargeting an in-flight move just re-seeds the ramp from the
        # current setpoint, so motion stays continuous.
        if not self._enabled:
            # Disabled -> the external controller owns the head; stay silent.
            return
        angle = self._clamp(angle)
        start = self._last_commanded_angle
        distance = abs(angle - start)
        if distance < 1e-4:
            # Already there (or as good as) -- publish once and skip the ramp.
            self._publish(angle)
            self._ramp_active = False
            return

        self._ramp_start_angle = start
        self._ramp_target_angle = angle
        self._ramp_duration = distance / self._tilt_speed
        self._ramp_start_time = self.get_clock().now()
        self._ramp_active = True
        self.get_logger().info(
            f"Head -> {angle:.3f} rad over {self._ramp_duration:.2f}s: {why}"
        )

    def _profile_tick(self):
        # Silent unless a move is in flight (and we still own the head).
        if not self._ramp_active or not self._enabled:
            return
        elapsed = (self.get_clock().now() - self._ramp_start_time).nanoseconds * 1e-9
        t = elapsed / self._ramp_duration if self._ramp_duration > 0.0 else 1.0
        if t >= 1.0:
            self._publish(self._ramp_target_angle)
            self._ramp_active = False
            return
        # Smoothstep easing (ease in/out): zero velocity at both ends, peak in
        # the middle, so the head accelerates and decelerates gently.
        s = t * t * (3.0 - 2.0 * t)
        angle = self._ramp_start_angle + (self._ramp_target_angle - self._ramp_start_angle) * s
        self._publish(angle)

    def _publish(self, angle: float):
        self._cmd_pub.publish(Float64MultiArray(data=[angle]))
        self._last_commanded_angle = angle


def main(args=None):
    rclpy.init(args=args)
    node = HeadTiltNearGoal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
