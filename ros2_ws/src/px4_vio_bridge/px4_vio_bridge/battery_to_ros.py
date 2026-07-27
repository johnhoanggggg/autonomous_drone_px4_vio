"""Republish PX4 battery telemetry as plain ROS types Foxglove can display.

`/fmu/out/battery_status_v1` is a px4_msgs/BatteryStatus, which Foxglove can only
show as raw fields — and its `remaining` is 0..1, which reads badly on a gauge.
This node flattens the fields that matter into std_msgs so a Gauge panel can bind
straight to `/battery/percent .data`, plus a one-line `/battery/status` string for
an Indicator or Raw Message panel.

Written after a 2026-07-27 waypoint flight completed at 11% state of charge with
nobody watching: the data was in the bag all along but was not on screen.

`level` is the field to drive a Foxglove Indicator panel from:
    0 OK   1 LOW   2 CRITICAL   3 EMPTY
It is the worse of this node's own percent thresholds and PX4's own `warning`
enum, so a PX4 low-voltage warning is never masked by an optimistic SoC estimate.
"""
import rclpy
from px4_msgs.msg import BatteryStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, Int32, String

LEVEL_NAMES = ("OK", "LOW", "CRITICAL", "EMPTY")

# px4_msgs BatteryStatus.warning -> our level. EMERGENCY and FAILED both mean
# "on the ground now", so they collapse into EMPTY.
PX4_WARNING_TO_LEVEL = {0: 0, 1: 1, 2: 2, 3: 3, 4: 3}


class BatteryToRos(Node):
    def __init__(self):
        super().__init__("battery_to_ros")

        self.declare_parameter("input_topic", "/fmu/out/battery_status_v1")
        self.declare_parameter("warn_percent", 40.0)
        self.declare_parameter("critical_percent", 25.0)
        self.declare_parameter("empty_percent", 15.0)
        # Voltage is the honest signal under load when the SoC estimate drifts.
        # Defaults are per-cell x 3S; cell_count from PX4 is used when it is sane.
        self.declare_parameter("warn_cell_volts", 3.60)
        self.declare_parameter("critical_cell_volts", 3.45)
        self.declare_parameter("empty_cell_volts", 3.30)
        self.declare_parameter("default_cell_count", 3)
        self.declare_parameter("log_throttle_s", 10.0)

        self.warn_percent = float(self.get_parameter("warn_percent").value)
        self.critical_percent = float(self.get_parameter("critical_percent").value)
        self.empty_percent = float(self.get_parameter("empty_percent").value)
        self.warn_cell = float(self.get_parameter("warn_cell_volts").value)
        self.critical_cell = float(self.get_parameter("critical_cell_volts").value)
        self.empty_cell = float(self.get_parameter("empty_cell_volts").value)
        self.default_cell_count = int(self.get_parameter("default_cell_count").value)
        self.log_throttle_s = float(self.get_parameter("log_throttle_s").value)

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Foxglove reads these; keep-last-1 volatile is plenty and matches the
        # rest of the visualization topics in this package.
        out_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.percent_pub = self.create_publisher(Float32, "/battery/percent", out_qos)
        self.voltage_pub = self.create_publisher(Float32, "/battery/voltage", out_qos)
        self.current_pub = self.create_publisher(Float32, "/battery/current", out_qos)
        self.power_pub = self.create_publisher(Float32, "/battery/power", out_qos)
        self.cell_pub = self.create_publisher(Float32, "/battery/cell_voltage", out_qos)
        self.level_pub = self.create_publisher(Int32, "/battery/level", out_qos)
        self.status_pub = self.create_publisher(String, "/battery/status", out_qos)

        self.create_subscription(
            BatteryStatus,
            str(self.get_parameter("input_topic").value),
            self.on_battery,
            sub_qos,
        )

        self.last_level = None
        self.get_logger().info(
            f"battery_to_ros: warn<{self.warn_percent:.0f}% "
            f"critical<{self.critical_percent:.0f}% empty<{self.empty_percent:.0f}%, "
            f"per-cell {self.warn_cell:.2f}/{self.critical_cell:.2f}/{self.empty_cell:.2f} V"
        )

    def cell_count(self, msg):
        count = int(msg.cell_count)
        return count if count > 0 else self.default_cell_count

    def level_from(self, percent, cell_v, px4_warning):
        """Worst of SoC, per-cell voltage, and PX4's own warning enum."""
        level = 0
        if percent is not None:
            if percent < self.empty_percent:
                level = max(level, 3)
            elif percent < self.critical_percent:
                level = max(level, 2)
            elif percent < self.warn_percent:
                level = max(level, 1)
        if cell_v is not None and cell_v > 0.0:
            if cell_v < self.empty_cell:
                level = max(level, 3)
            elif cell_v < self.critical_cell:
                level = max(level, 2)
            elif cell_v < self.warn_cell:
                level = max(level, 1)
        return max(level, PX4_WARNING_TO_LEVEL.get(int(px4_warning), 0))

    def on_battery(self, msg):
        if not msg.connected:
            return
        voltage = float(msg.voltage_v)
        current = float(msg.current_a)
        # PX4 marks these invalid rather than omitting them: voltage 0, current
        # -1, remaining -1. Publishing those as-is would put a 0 V / -100% on a
        # gauge, which is worse than publishing nothing.
        percent = float(msg.remaining) * 100.0 if msg.remaining >= 0.0 else None
        current = current if current >= 0.0 else None
        cell_v = voltage / self.cell_count(msg) if voltage > 0.0 else None

        level = self.level_from(percent, cell_v, msg.warning)

        if percent is not None:
            self.percent_pub.publish(Float32(data=percent))
        if voltage > 0.0:
            self.voltage_pub.publish(Float32(data=voltage))
        if cell_v is not None:
            self.cell_pub.publish(Float32(data=cell_v))
        if current is not None:
            self.current_pub.publish(Float32(data=current))
            if voltage > 0.0:
                self.power_pub.publish(Float32(data=voltage * current))
        self.level_pub.publish(Int32(data=level))

        text = " ".join(
            part
            for part in (
                LEVEL_NAMES[level],
                f"{percent:.0f}%" if percent is not None else "--%",
                f"{voltage:.2f}V" if voltage > 0.0 else "--V",
                f"({cell_v:.2f}V/cell)" if cell_v is not None else "",
                f"{current:.1f}A" if current is not None else "",
                f"{voltage * current:.0f}W"
                if current is not None and voltage > 0.0
                else "",
            )
            if part
        )
        self.status_pub.publish(String(data=text))

        # Escalation is worth a line in the flight log; steady state is not.
        if level != self.last_level and level > 0:
            log = self.get_logger().error if level >= 2 else self.get_logger().warn
            log(f"BATTERY {text}")
        self.last_level = level
        self.get_logger().info(f"battery {text}", throttle_duration_sec=self.log_throttle_s)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryToRos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
