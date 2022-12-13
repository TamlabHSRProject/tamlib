import copy
from ctypes import POINTER, c_float, c_uint32, cast, pointer
from typing import List, Optional, Union

import numpy as np
import open3d as o3d
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose
from open3d.geometry import PointCloud as PCD
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header

_FIELDS_XYZ = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]
_FIELDS_XYZRGB = _FIELDS_XYZ + [
    PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1)
]
_BIT_MOVE_16 = 2**16
_BIT_MOVE_8 = 2**8


class Open3D:
    def __init__(self) -> None:
        self.node_name = rospy.get_name()

        self._rgbuint32_to_tuple = lambda rgb_uint32: (
            (rgb_uint32 & 0x00FF0000) >> 16,
            (rgb_uint32 & 0x0000FF00) >> 8,
            (rgb_uint32 & 0x000000FF),
        )
        self._rgbfloat_to_tuple = lambda rgb_float: self._rgbuint32_to_tuple(
            int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
        )

    def show(self, pcd: Union[PCD, List[PCD]]) -> None:
        """ポイントクラウドの描画

        Args:
            pcd (Union[PCD, List[PCD]]): ポイントクラウド（複数可）．
        """
        if not isinstance(pcd, list):
            pcd = [pcd]
        o3d.visualization.draw_geometries(pcd)

    def paint(self, pcd: PCD, r: float, g: float, b: float) -> PCD:
        """ポイントクラウドの塗りつぶし

        Args:
            pcd (PCD): ポイントクラウド．
            r (float): Red（0〜1）．
            g (float): Green（0〜1）．
            b (float): Blue（0〜1）

        Returns:
            PCD: 塗りつぶされたポイントクラウド．
        """
        pcd_paint = copy.deepcopy(pcd)
        pcd_paint.paint_uniform_color([r, g, b])
        return pcd_paint

    def pcd2msg_to_pcd(self, pcd_msg: PointCloud2, has_color=True) -> Optional[PCD]:
        """PointCloud2メッセージからOpen3D形式のポイントクラウドに変換する

        Args:
            pcd_msg (PointCloud2): PointCloud2メッセージ．
            has_color (bool, optional): 変換したポイントクラウドに色を付けるかどうか．Defaults to True.

        Returns:
            Optional[PCD]: Open3D形式のポイントクラウド．変換できない場合はNoneを返す．
        """
        field_names = [field.name for field in pcd_msg.fields]
        cloud_data = list(
            pc2.read_points(pcd_msg, skip_nans=True, field_names=field_names)
        )
        pcd = o3d.geometry.PointCloud()
        if len(cloud_data) == 0:
            rospy.logwarn(f"[{self.node_name}]: PointCloud2 message is empty.")
            rospy.logwarn(f"[{self.node_name}]: Convert FAILURE.")
            return None

        if "rgb" in field_names and has_color:
            xyz = [(x, y, z) for x, y, z, _ in cloud_data]
            if type(cloud_data[0][3]) == float:
                rgb = [self._rgbfloat_to_tuple(rgb) for *_, rgb in cloud_data]
            else:
                rgb = [self._rgbuint32_to_tuple(rgb) for *_, rgb in cloud_data]
            pcd.points = o3d.utility.Vector3dVector(np.array(xyz))
            pcd.colors = o3d.utility.Vector3dVector(np.array(rgb))
        else:
            xyz = [(x, y, z) for x, y, z in cloud_data]
            pcd.points = o3d.utility.Vector3dVector(np.array(xyz))
        return pcd

    def pcd_to_pcd2msg(self, pcd: PCD, frame_id: str, has_color=True) -> PointCloud2:
        """Open3D形式のポイントクラウドからPointCloud2メッセージに変換する

        Args:
            pcd (PCD): Open3D形式のポイントクラウド．
            frame_id (str): PointCloud2の基準座標系名．
            has_color (bool, optional): 変換したポイントクラウドに色を付けるかどうか． Defaults to True.

        Returns:
            PointCloud2: PointCloud2メッセージ．
        """
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id

        points = np.ascontiguousarray(pcd.points)
        if not pcd.colors:
            fields = _FIELDS_XYZ
            cloud_data = points
        else:
            fields = _FIELDS_XYZRGB
            colors = np.floor(np.ascontiguousarray(pcd.colors) * 255)
            colors = (
                colors[:, 0] * _BIT_MOVE_16 + colors[:, 1] * _BIT_MOVE_8 + colors[:, 2]
            )
            cloud_data = np.c_[points, colors]
        return pc2.create_cloud(header, fields, cloud_data)

    def depth_to_pcd(self, depth: np.ndarray, camera_info: CameraInfo) -> PCD:
        intrinsics = o3d.camera.PinholeCameraIntrinsic()
        intrinsics.set_intrinsics(
            width=camera_info.width,
            height=camera_info.height,
            fx=camera_info.K[0],
            fy=camera_info.K[4],
            cx=camera_info.K[2],
            cy=camera_info.K[5],
        )
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            o3d.geometry.Image(depth), intrinsics
        )
        return pcd

    def pass_through_filter(
        self,
        pcd: PCD,
        xmin: float,
        xmax: float,
        ymin: float,
        ymax: float,
        zmin: float,
        zmax: float,
    ) -> PCD:
        """パススルーフィルタ

        Args:
            pcd (PCD): Open3D形式のポイントクラウド．
            xmin (float): x座標の最小．
            xmax (float): x座標の最大．
            ymin (float): y座標の最小．
            ymax (float): y座標の最大．
            zmin (float): z座標の最小．
            zmax (float): z座標の最大．

        Returns:
            PCD: パススルーフィルタを適用したOpen3D形式のポイントクラウド．
        """
        pcd_ptf = copy.deepcopy(pcd)
        points = np.ascontiguousarray(pcd_ptf.points)
        colors = np.ascontiguousarray(pcd_ptf.colors)
        x_range = np.logical_and(points[:, 0] >= xmin, points[:, 0] <= xmax)
        y_range = np.logical_and(points[:, 1] >= ymin, points[:, 1] <= ymax)
        z_range = np.logical_and(points[:, 2] >= zmin, points[:, 2] <= zmax)
        pass_through_filter = np.logical_and(x_range, np.logical_and(y_range, z_range))
        pcd_ptf.points = o3d.utility.Vector3dVector(points[pass_through_filter])
        pcd_ptf.colors = o3d.utility.Vector3dVector(colors[pass_through_filter])
        return pcd_ptf

    def transform(self, pcd: PCD, pose: Pose) -> PCD:
        pcd_rot = copy.deepcopy(pcd)
        q = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        rotation_matrix = Rotation.from_quat(q)
        T = np.eye(4)
        T[:3, :3] = rotation_matrix.as_matrix()
        T[0, 3] = pose.position.x
        T[1, 3] = pose.position.y
        T[2, 3] = pose.position.z
        return pcd_rot.transform(T)
