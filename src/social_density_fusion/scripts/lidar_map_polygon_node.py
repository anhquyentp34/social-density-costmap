#!/usr/bin/env python3
"""Node độc lập: dựng đa giác LiDAR trong map bằng TF đồng bộ theo thời gian scan."""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import Point32, PolygonStamped
from sensor_msgs.msg import LaserScan


def rotate_point_by_quaternion(x, y, z, qx, qy, qz, qw):
    """Quay vector bằng quaternion (Hamilton product rút gọn)."""
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)

    rx = x + qw * tx + (qy * tz - qz * ty)
    ry = y + qw * ty + (qz * tx - qx * tz)
    rz = z + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


class LidarMapPolygonNode:
    def __init__(self):
        rospy.init_node("lidar_map_polygon_node", anonymous=True)

        scan_topic = rospy.get_param("~scan_topic", "/scan")
        polygon_topic = rospy.get_param("~polygon_topic", "/lidar_map_polygon")
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.tf_lookup_timeout = float(rospy.get_param("~tf_lookup_timeout", 0.05))
        self.max_tf_cache_age = float(rospy.get_param("~max_tf_cache_age", 0.30))
        self.last_valid_transform = None
        self.last_valid_transform_time = rospy.Time(0)

        self.pub = rospy.Publisher(polygon_topic, PolygonStamped, queue_size=2)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(scan_topic, LaserScan, self._scan_cb, queue_size=1)
        rospy.loginfo(
            "lidar_map_polygon_node: scan=%s, polygon=%s, target_frame=%s",
            scan_topic,
            polygon_topic,
            self.target_frame,
        )

    def _lookup_transform(self, scan_msg):
        src_frame = scan_msg.header.frame_id
        if not src_frame:
            rospy.logwarn_throttle(5.0, "LaserScan frame_id rong, bo qua scan.")
            return None

        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                src_frame,
                scan_msg.header.stamp,
                rospy.Duration(self.tf_lookup_timeout),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # Fallback: dùng transform mới nhất nếu chưa có đúng timestamp (giảm rớt khung lúc startup)
            try:
                rospy.logwarn_throttle(
                    2.0,
                    "Khong co TF dung timestamp scan, fallback sang TF moi nhat %s <- %s.",
                    self.target_frame,
                    src_frame,
                )
                return self.tf_buffer.lookup_transform(
                    self.target_frame,
                    src_frame,
                    rospy.Time(0),
                    rospy.Duration(self.tf_lookup_timeout),
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                # Last-resort: tái sử dụng TF hợp lệ gần nhất nếu còn mới để tránh rỗng costmap ngắn hạn.
                if self.last_valid_transform is not None:
                    age = (rospy.Time.now() - self.last_valid_transform_time).to_sec()
                    if age <= self.max_tf_cache_age:
                        rospy.logwarn_throttle(
                            2.0,
                            "Dung lai TF hop le gan nhat (age=%.3fs) cho %s <- %s.",
                            age,
                            self.target_frame,
                            src_frame,
                        )
                        return self.last_valid_transform
                rospy.logwarn_throttle(
                    2.0,
                    "Khong lookup duoc TF %s <- %s, bo qua scan.",
                    self.target_frame,
                    src_frame,
                )
                return None

    def _scan_cb(self, scan_msg):
        transform = self._lookup_transform(scan_msg)
        if transform is None:
            return
        self.last_valid_transform = transform
        self.last_valid_transform_time = rospy.Time.now()

        t = transform.transform.translation
        q = transform.transform.rotation

        points_xy = []
        angle = scan_msg.angle_min
        for raw_range in scan_msg.ranges:
            r = raw_range
            if not math.isfinite(r) or r > scan_msg.range_max:
                r = scan_msg.range_max
            if r < scan_msg.range_min or r > scan_msg.range_max:
                angle += scan_msg.angle_increment
                continue

            local_x = r * math.cos(angle)
            local_y = r * math.sin(angle)
            local_z = 0.0

            rot_x, rot_y, _ = rotate_point_by_quaternion(local_x, local_y, local_z, q.x, q.y, q.z, q.w)
            world_x = t.x + rot_x
            world_y = t.y + rot_y
            points_xy.append((world_x, world_y))
            angle += scan_msg.angle_increment

        if len(points_xy) < 3:
            return

        out = PolygonStamped()
        out.header.stamp = scan_msg.header.stamp
        out.header.frame_id = self.target_frame
        for x, y in points_xy:
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            out.polygon.points.append(p)

        self.pub.publish(out)


if __name__ == "__main__":
    LidarMapPolygonNode()
    rospy.spin()
