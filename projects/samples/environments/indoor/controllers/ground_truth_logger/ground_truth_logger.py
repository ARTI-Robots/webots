from controller import Supervisor

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GroundTruthTfPublisher(Node):
    def __init__(self):
        super().__init__("ground_truth_tf_publisher")

        self.tf_broadcaster = TransformBroadcaster(self)

        self.odom_publisher = self.create_publisher(
            Odometry,
            "/ground_truth/odom",
            10,
        )


webots = Supervisor()
timestep = int(webots.getBasicTimeStep())

rosbot = webots.getFromDef("ROSBOT_XL")

if rosbot is None:
    print("[GT] ERROR: DEF ROSBOT_XL not found.", flush=True)
    raise SystemExit(1)

rclpy.init()
ros_node = GroundTruthTfPublisher()

# Establish odom at the robot's initial Webots pose.
initial_position = rosbot.getPosition()
initial_orientation = rosbot.getOrientation()

x0 = initial_position[0]
y0 = initial_position[1]

yaw0 = math.atan2(
    initial_orientation[3],
    initial_orientation[0],
)

cos_yaw0 = math.cos(yaw0)
sin_yaw0 = math.sin(yaw0)

print(
    f"[GT] Initial Webots pose: "
    f"x={x0:+.6f} y={y0:+.6f} "
    f"yaw={math.degrees(yaw0):+.3f} deg",
    flush=True,
)

print(
    "[GT] Publishing ground-truth odom -> base_link",
    flush=True,
)

while webots.step(timestep) != -1:
    rclpy.spin_once(ros_node, timeout_sec=0.0)

    position = rosbot.getPosition()
    orientation = rosbot.getOrientation()

    world_dx = position[0] - x0
    world_dy = position[1] - y0

    # Express translation in the robot's initial frame.
    x = (
        cos_yaw0 * world_dx +
        sin_yaw0 * world_dy
    )

    y = (
        -sin_yaw0 * world_dx +
        cos_yaw0 * world_dy
    )

    current_yaw = math.atan2(
        orientation[3],
        orientation[0],
    )

    yaw = normalize_angle(current_yaw - yaw0)

    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)

    stamp = ros_node.get_clock().now().to_msg()

    transform = TransformStamped()

    transform.header.stamp = stamp
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"

    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.translation.z = 0.0

    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = qz
    transform.transform.rotation.w = qw

    ros_node.tf_broadcaster.sendTransform(transform)

    odom = Odometry()

    odom.header.stamp = stamp
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0

    odom.pose.pose.orientation.x = 0.0
    odom.pose.pose.orientation.y = 0.0
    odom.pose.pose.orientation.z = qz
    odom.pose.pose.orientation.w = qw

    ros_node.odom_publisher.publish(odom)

ros_node.destroy_node()
rclpy.shutdown()
