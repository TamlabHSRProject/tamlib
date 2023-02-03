from typing import Tuple

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid


class Map:
    def __init__(self, map_topic="/rtabmap/grid_map") -> None:
        self.map = rospy.wait_for_message(map_topic, OccupancyGrid)

    @property
    def resolution(self):
        return self.map.info.resolution

    @property
    def width(self):
        return self.map.info.width

    @property
    def height(self):
        return self.map.info.height

    @property
    def origin(self):
        return self.map.info.origin

    @property
    def reference_frame(self):
        return self.map.header.frame_id

    @property
    def map_data(self):
        return self.map.data

    def pose2map(self, x: float, y: float) -> Tuple[float, float]:
        """Map座標系の座標を環境地図上の座標に変換する

        Args:
            x (float): Map座標系のx座標．
            y (float): Map座標系のy座標．

        Returns:
            Tuple[float, float]: 環境地図上のxy座標．
        """
        map_x = int(round((x - self.origin.position.x) / self.resolution))
        map_y = int(round((y - self.origin.position.y) / self.resolution))
        return (map_x, map_y)

    def map2pose(self, x: int, y: int) -> Tuple[float, float]:
        """環境地図上の座標をMap座標系の座標に変換する

        Args:
            x (int): 環境地図上のx座標．
            y (int): 環境地図上のy座標．

        Returns:
            Tuple[float, float]: Map座標系上のxy座標．
        """
        pose_x = x * self.resolution + self.origin.position.x
        ppse_y = y * self.resolution + self.origin.position.y
        return (pose_x, ppse_y)

    def get_map_image(self) -> np.ndarray:
        """環境地図の画像を取得する

        Returns:
            np.ndarray: 環境地図の画像．
        """
        if hasattr(self, "map_image"):
            return getattr(self, "map_image")

        data = np.array(self.map_data)
        data2d = data.reshape((self.height, self.width))
        map_image = np.where(data2d < 0, 127, 0)
        map_image = np.where(data2d == 0, 255, map_image)
        map_image = np.where(data2d > 0, 0, map_image).astype(np.uint8)

        self.map_image = map_image
        return map_image
