from typing import Optional

import rospy
import tf2_ros
from geometry_msgs.msg import Pose, TransformStamped


class Transform:
    def __init__(self) -> None:
        self.node_name = rospy.get_name()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

    def _create_transform_stamped(
        self, target_frame: str, source_frame: str, pose: Pose
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = source_frame
        transform.child_frame_id = target_frame
        transform.transform.translation = pose.position
        transform.transform.rotation = pose.orientation
        return transform

    def send_static_transform(
        self, target_frame: str, source_frame: str, pose: Pose
    ) -> None:
        transform = self._create_transform_stamped(target_frame, source_frame, pose)
        self.static_broadcaster.sendTransform(transform)

    def send_transform(self, target_frame: str, source_frame: str, pose: Pose) -> None:
        transform = self._create_transform_stamped(target_frame, source_frame, pose)
        self.broadcaster.sendTransform(transform)

    def get_pose(
        self,
        target_frame: str,
        source_frame: str,
        time=0.0,
        timeout=1.0,
    ) -> Optional[Pose]:
        try:
            transform = self.buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(time),
                rospy.Duration(timeout),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logwarn(f"[{self.node_name}]: {e}")
            rospy.logwarn(f"[{self.node_name}]: Transform FAILURE.")
            return None

        pose = Pose(
            position=transform.transform.translation,
            orientation=transform.transform.rotation.x,
        )
        return pose

    def get_pose_with_offset(
        self,
        target_frame: str,
        source_frame: str,
        buffer_frame: str,
        offset: Pose,
        time=0.0,
        timeout=1.0,
    ) -> Optional[Pose]:
        self.send_transform(buffer_frame, source_frame, offset)
        pose = self.get_pose(target_frame, buffer_frame, time, timeout)
        return pose
