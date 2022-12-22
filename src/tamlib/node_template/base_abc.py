from abc import ABC, abstractmethod

import rospy


class NodeABC(ABC):
    def __init__(self):
        rospy.sleep(0.01)
        self.node_name = rospy.get_name()

    @abstractmethod
    def delete(self) -> None:
        """Destructor"""
        ...

    @abstractmethod
    def run(self) -> None:
        """実行関数"""
        ...
