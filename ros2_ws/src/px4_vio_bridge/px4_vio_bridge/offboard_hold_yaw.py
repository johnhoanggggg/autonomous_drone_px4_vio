"""Short offboard position-hold and relative-yaw flight test.

Sequence: take off while holding the initial NED x/y/yaw, turn by the requested
relative yaw angle, hold, return to the precisely latched initial yaw, stabilize,
then request AUTO.LAND. Safety and engagement behavior come from OffboardHover.
"""
import math

import rclpy

from px4_vio_bridge.offboard_hover import OffboardHover


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class OffboardHoldYaw(OffboardHover):
    def __init__(self):
        super().__init__("offboard_hold_yaw")
        self.declare_parameter("yaw_angle_deg", 15.0)
        self.declare_parameter("yaw_tolerance_deg", 5.0)
        self.declare_parameter("yaw_timeout", 8.0)
        self.declare_parameter("return_hold_time", 2.0)
        # Tighter than max_horizontal_error: the yaw test must start from a
        # stable hover, not from the edge of the abort envelope.
        self.declare_parameter("pre_yaw_max_horizontal_error", 0.15)

        self.yaw_angle = math.radians(
            float(self.get_parameter("yaw_angle_deg").value)
        )
        self.yaw_tolerance = math.radians(
            float(self.get_parameter("yaw_tolerance_deg").value)
        )
        self.yaw_timeout = float(self.get_parameter("yaw_timeout").value)
        self.return_hold_time = float(
            self.get_parameter("return_hold_time").value
        )
        self.pre_yaw_max_horizontal_error = float(
            self.get_parameter("pre_yaw_max_horizontal_error").value
        )
        self.get_logger().warn(
            f"yaw test: turn={math.degrees(self.yaw_angle):.1f}deg, "
            f"yaw hold={self.hold_time:.1f}s, return hold={self.return_hold_time:.1f}s"
        )

    @property
    def yaw_target(self):
        return wrap_pi(self.yaw0 + self.yaw_angle)

    def yaw_error(self, target):
        if self.pos is None or not math.isfinite(self.pos.heading):
            return math.inf
        return abs(wrap_pi(target - self.pos.heading))

    def check_flight_position(self):
        if self.auto_arm and not self.pos_valid():
            self.trigger_landing("lost local position in flight")
            return False
        return True

    def wait_for_yaw(self, target, next_state, description):
        self.publish_setpoint(self.hover_height, target)
        if not self.check_flight_position():
            return
        # A props-off dry run cannot physically follow the yaw setpoint. Validate
        # the rate limiter/state machine against the commanded yaw instead.
        if self.auto_arm:
            error = self.yaw_error(target)
        else:
            error = abs(wrap_pi(target - self.yaw_cmd))
        if error <= self.yaw_tolerance:
            self.get_logger().warn(
                f"{description} reached "
                f"({'vehicle' if self.auto_arm else 'dry-run command'} "
                f"error={math.degrees(error):.1f}deg)"
            )
            self.set_state(next_state)
        elif self.t >= self.yaw_timeout:
            self.trigger_landing(
                f"timeout waiting for {description} "
                f"(error={math.degrees(error):.1f}deg)"
            )

    def handle_flight_state(self):
        if self.state == "CLIMB_HOLD":
            self.publish_setpoint(self.hover_height, self.yaw0)
            if not self.check_flight_position():
                return True
            if self.auto_arm:
                # Armed: the yaw test may only start from a verified stable
                # hover. A climb timeout means something is wrong -> LAND, never
                # fall through into the yaw sequence.
                altitude_ok = self.pos is not None and (
                    abs((-self.pos.z) - self.hover_height) <= self.reach_tol
                )
                horizontal_ok = (
                    self.horizontal_error is not None
                    and self.horizontal_error <= self.pre_yaw_max_horizontal_error
                )
                if altitude_ok and horizontal_ok:
                    self.get_logger().warn(
                        "reached hover altitude inside pre-yaw gate"
                    )
                    self.set_state("YAW_OUT")
                elif self.t > self.climb_timeout:
                    self.trigger_landing(
                        "climb timeout without a stable hover "
                        f"(altitude_ok={altitude_ok}, "
                        f"horizontal_error="
                        f"{float('nan') if self.horizontal_error is None else self.horizontal_error:.2f}m"
                        f"/{self.pre_yaw_max_horizontal_error:.2f}m)"
                    )
            elif self.t > self.climb_timeout:
                # Dry run cannot physically climb; proceed on timeout to
                # exercise the yaw state machine against the commanded yaw.
                self.get_logger().warn(
                    "dry run: climb timeout; proceeding with yaw state machine"
                )
                self.set_state("YAW_OUT")
            return True

        if self.state == "YAW_OUT":
            self.wait_for_yaw(self.yaw_target, "YAW_HOLD", "+yaw target")
            return True

        if self.state == "YAW_HOLD":
            self.publish_setpoint(self.hover_height, self.yaw_target)
            if not self.check_flight_position():
                return True
            if self.t >= self.hold_time:
                self.set_state("YAW_BACK")
            return True

        if self.state == "YAW_BACK":
            self.wait_for_yaw(self.yaw0, "RETURN_HOLD", "initial yaw")
            return True

        if self.state == "RETURN_HOLD":
            self.publish_setpoint(self.hover_height, self.yaw0)
            if not self.check_flight_position():
                return True
            if self.t >= self.return_hold_time:
                self.set_state("LAND")
            return True

        return False


def main(args=None):
    rclpy.init(args=args)
    node = OffboardHoldYaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
