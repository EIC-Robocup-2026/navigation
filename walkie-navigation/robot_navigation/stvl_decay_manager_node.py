#!/usr/bin/env python3
"""Freeze the local-costmap STVL layer when the robot is near its goal.

Problem this solves
-------------------
The Spatio-Temporal Voxel Layer marks tabletop obstacles from the ZED head and
the 3D lidar, and Nav2 (MPPI + collision monitor) keeps the robot off them.
But at very close range the table leaves the camera/lidar field of view (a
"blind spot"). With nothing re-marking it, the voxels decay away after
``voxel_decay`` seconds, the obstacle vanishes from the costmap, and the robot
drives into the table.

What this node does
-------------------
STVL has no runtime-settable ``voxel_decay``. The only lever that freezes decay
is ``mapping_mode``: when true, STVL skips ``ClearFrustums()`` so the flattened
cost map is held frozen at its last state (see the matching patch in
spatio_temporal_voxel_layer.cpp). This node watches the distance remaining to
the active NavigateToPose goal and:

  * distance <= freeze_distance   -> set stvl_layer.mapping_mode = true  (freeze)
  * distance >= unfreeze_distance -> set stvl_layer.mapping_mode = false (normal)
  * a NEW goal arrives            -> force mapping_mode = false (resume normal)

The two thresholds give hysteresis so it does not flap on the boundary.

The distance comes from the NavigateToPose action feedback, so it works no
matter how the goal was sent (the Flask backend's goToPose action, the human
follower's /goal_pose, RViz, ...) and naturally stops between goals.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

from nav2_msgs.action import NavigateToPose

# The wrapper message published on the hidden action feedback topic. It carries
# the goal_id (so we can detect a new goal) plus the NavigateToPose feedback.
FeedbackMessage = NavigateToPose.Impl.FeedbackMessage


class StvlDecayManager(Node):
    def __init__(self):
        super().__init__("stvl_decay_manager")

        # Node that hosts the costmap layer params, and the layer name.
        self.declare_parameter("costmap_node", "/local_costmap/local_costmap")
        self.declare_parameter("layer_name", "stvl_layer")
        # Freeze at/under this distance to goal; only unfreeze once back above the
        # upper threshold (hysteresis). distance is the action's distance_remaining.
        self.declare_parameter("freeze_distance", 1.4)
        self.declare_parameter("unfreeze_distance", 1.6)
        # Don't freeze the instant we cross freeze_distance. The robot pauses near
        # the goal (see the verify-pause gate in nav_to_pose_real_recovery_bt.xml);
        # we hold STVL in normal (decaying) mode for confirm_duration first so real
        # obstacles stay solid while transients decay, then freeze a verified
        # snapshot. Keep this < the BT Wait so the freeze lands during the pause.
        self.declare_parameter("confirm_duration", 2.0)
        # If we are frozen and no action feedback has arrived for this long, the
        # goal has ended (succeeded/canceled/aborted) — feedback only streams
        # while the action is active. Auto-unfreeze so STVL resumes clearing and
        # we don't strand mapping_mode=true between goals (which freezes every
        # transient into a permanent ghost). Keep it a bit above the feedback
        # period so a momentary gap doesn't unfreeze us mid-task.
        self.declare_parameter("idle_unfreeze_timeout", 2.0)
        self.declare_parameter(
            "feedback_topic", "/navigate_to_pose/_action/feedback"
        )

        self._costmap_node = self.get_parameter("costmap_node").value
        self._layer = self.get_parameter("layer_name").value
        self._freeze_dist = self.get_parameter("freeze_distance").value
        self._unfreeze_dist = self.get_parameter("unfreeze_distance").value
        self._confirm_duration = self.get_parameter("confirm_duration").value
        self._idle_unfreeze_timeout = self.get_parameter("idle_unfreeze_timeout").value
        feedback_topic = self.get_parameter("feedback_topic").value
        self._param_name = f"{self._layer}.mapping_mode"

        # frozen state we have actually pushed to STVL (None = unknown yet)
        self._frozen = None
        # when distance first dropped <= freeze_dist (None = not currently below)
        self._below_since = None
        # goal_id of the goal we are currently tracking
        self._last_goal_id = None
        # clock time of the most recent feedback (None = none yet). Used by the
        # idle watchdog to detect that the action has ended.
        self._last_feedback_time = None
        # guard so we never have two in-flight set_parameters calls
        self._busy = False

        self._param_client = AsyncParameterClient(self, self._costmap_node)

        self.create_subscription(
            FeedbackMessage, feedback_topic, self._feedback_cb, 10
        )

        # Push a known-good initial state (normal decay) once the costmap node's
        # parameter service is up, in case a previous run left it frozen.
        self._init_timer = self.create_timer(1.0, self._ensure_initial_state)

        # Watchdog: while frozen, unfreeze once feedback has been silent past the
        # idle timeout (the goal ended), so STVL never stays frozen between goals.
        self._watchdog_timer = self.create_timer(0.5, self._idle_watchdog)

        self.get_logger().info(
            f"STVL decay manager up: freezing '{self._param_name}' on "
            f"'{self._costmap_node}' when distance_remaining <= "
            f"{self._freeze_dist} m (unfreeze >= {self._unfreeze_dist} m)."
        )

    def _ensure_initial_state(self):
        # Wait for the lifecycle costmap node's parameter services, then make
        # sure we start in normal (unfrozen) mode, then stop this timer.
        if not self._param_client.services_are_ready():
            return
        self._init_timer.cancel()
        self._apply(False)

    def _feedback_cb(self, msg: FeedbackMessage):
        self._last_feedback_time = self.get_clock().now()
        goal_id = bytes(msg.goal_id.uuid)
        if goal_id != self._last_goal_id:
            # New goal -> resume normal STVL behaviour for this goal. We return
            # without evaluating distance so STVL gets at least one normal cycle
            # to rebuild a fresh (decayed) costmap before any re-freeze.
            self._last_goal_id = goal_id
            self._below_since = None
            self.get_logger().info("New goal detected -> STVL decay back to normal")
            self._apply(False)
            return

        distance = msg.feedback.distance_remaining
        # distance_remaining is 0.0 until the first path is available; ignore it.
        if distance <= 0.0:
            return

        now = self.get_clock().now()
        if distance <= self._freeze_dist:
            # Start (or continue) the confirmation dwell. Stay in normal decay
            # mode until it elapses, so transients clear and only an obstacle
            # still present after confirm_duration gets frozen.
            if self._below_since is None:
                self._below_since = now
                self.get_logger().info(
                    f"Within {self._freeze_dist} m -> confirming for "
                    f"{self._confirm_duration:.1f}s before freeze"
                )
            elif (now - self._below_since).nanoseconds * 1e-9 >= self._confirm_duration:
                self._apply(True)
        elif distance >= self._unfreeze_dist:
            self._below_since = None
            self._apply(False)
        # between the thresholds: hold current state and keep the dwell running

    def _idle_watchdog(self):
        # Only relevant once we have actually frozen. If feedback has gone silent
        # past the timeout, the action ended while frozen -> resume normal decay.
        if not self._frozen or self._last_feedback_time is None:
            return
        idle = (self.get_clock().now() - self._last_feedback_time).nanoseconds * 1e-9
        if idle >= self._idle_unfreeze_timeout:
            self._below_since = None
            self.get_logger().info(
                f"No goal feedback for {idle:.1f}s (goal ended) -> STVL decay back to normal"
            )
            self._apply(False)

    def _apply(self, freeze: bool):
        if freeze == self._frozen or self._busy:
            return
        if not self._param_client.services_are_ready():
            return

        self._busy = True
        future = self._param_client.set_parameters(
            [Parameter(self._param_name, Parameter.Type.BOOL, freeze)]
        )
        future.add_done_callback(lambda f, v=freeze: self._on_set_done(f, v))

    def _on_set_done(self, future, freeze: bool):
        self._busy = False
        try:
            results = future.result().results
        except Exception as exc:  # noqa: BLE001 - log and retry on next tick
            self.get_logger().warn(f"Failed to set {self._param_name}: {exc}")
            return
        if results and not results[0].successful:
            self.get_logger().warn(
                f"{self._costmap_node} rejected {self._param_name}: "
                f"{results[0].reason}"
            )
            return
        self._frozen = freeze
        self.get_logger().info(
            f"STVL {'FROZEN (mapping_mode=true)' if freeze else 'normal (mapping_mode=false)'}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StvlDecayManager()
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
