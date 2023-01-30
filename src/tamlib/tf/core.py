from typing import Optional, Tuple

import numpy as np
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped


def pose_to_matrix(pose: Pose):
    p, q = pose.position, pose.orientation
    trans = (p.x, p.y, p.z)
    rot = (q.x, q.y, q.z, q.w)
    trans_mat = tf.transformations.translation_matrix(trans)
    rot_mat = tf.transformations.quaternion_matrix(rot)
    return np.dot(trans_mat, rot_mat)


class Transform:
    def __init__(self) -> None:
        self.node_name = rospy.get_name()

        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._broadcaster = tf2_ros.TransformBroadcaster()
        self._buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buffer)

    def _create_transform_stamped(
        self, target_frame: str, source_frame: str, pose: Pose
    ) -> TransformStamped:
        """TransformStampedメッセージを作成する

        Args:
            target_frame (str): ターゲットフレーム名．
            source_frame (str): ソースフレーム名．
            pose (Pose): 座標．

        Returns:
            TransformStamped: 作成されたメッセージ．
        """
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
        """/static_tfを配信する

        Args:
            target_frame (str): 配信する座標系名．
            source_frame (str): 基準となる座標系名．
            pose (Pose): 基準座標系からの相対座標．
        """
        transform = self._create_transform_stamped(target_frame, source_frame, pose)
        self._static_broadcaster.sendTransform(transform)

    def send_transform(self, target_frame: str, source_frame: str, pose: Pose) -> None:
        """/tfを配信する

        Args:
            target_frame (str): 配信する座標系名．
            source_frame (str): 基準となる座標系名．
            pose (Pose): 基準座標系からの相対座標．
        """
        transform = self._create_transform_stamped(target_frame, source_frame, pose)
        self._broadcaster.sendTransform(transform)

    def get_pose(
        self,
        target_frame: str,
        source_frame: str,
        time=0.0,
        timeout=1.0,
    ) -> Optional[Pose]:
        """座標変換を行う

        Args:
            target_frame (str): 変換後の座標系名．
            source_frame (str): 変換前の座標系名．
            time (float, optional): 座標変換する特定の時間. 0で最新． Defaults to 0.0.
            timeout (float, optional): タイムアウト. Defaults to 1.0.

        Returns:
            Optional[Pose]: 変換後の座標．タイムアウトの場合，None．
        """
        try:
            transform = self._buffer.lookup_transform(
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
            orientation=transform.transform.rotation,
        )
        return pose

    def get_pose_with_offset(
        self,
        target_frame: str,
        source_frame: str,
        offset: Pose,
        buffer_frame: Optional[str] = None,
        time=0.0,
        timeout=1.0,
    ) -> Optional[Pose]:
        """オフセット付きの座標変換を行う

        Args:
            target_frame (str): 変換後の座標系名．
            source_frame (str): 変換前の座標系名．
            offset (Pose): オフセット座標．
            buffer_frame (str, optional): バッファーの座標系名．Defaults to None.
            time (float, optional): 座標変換する特定の時間. 0で最新． Defaults to 0.0.
            timeout (float, optional): タイムアウト．負の場合一回のみ実行. Defaults to 1.0.

        Returns:
            Optional[Pose]: 変換後の座標．タイムアウトの場合，None．
        """
        pose = self.get_pose(target_frame, source_frame, time, timeout)
        if pose is None:
            return None

        mat1 = pose_to_matrix(pose)
        mat2 = pose_to_matrix(offset)
        mat = np.dot(mat1, mat2)
        trans = tf.transformations.translation_from_matrix(mat)
        rot = tf.transformations.quaternion_from_matrix(mat)
        result = Pose(Point(*trans), Quaternion(*rot))
        if buffer_frame is not None:
            self.send_transform(buffer_frame, target_frame, result)
        return result

    @staticmethod
    def euler2quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(*q)

    @staticmethod
    def quaternion2euler(quaternion: Quaternion) -> Tuple[float, float, float]:
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        )
        return (roll, pitch, yaw)
