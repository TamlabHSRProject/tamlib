from typing import Any

import rospy

from .ifdict import InterfaceDict


class Publisher:
    def __init__(self):
        self.node_name = rospy.get_name()
        self._pub = InterfaceDict()

    @property
    def pub(self):
        return self._pub

    @pub.setter
    def pub(self, name: str, publisher: rospy.Publisher):
        self._pub[name] = publisher

    def register(self, name: str, topic: str, msg_type: Any, queue_size=10) -> None:
        """Publisherを登録する

        Args:
            name (str): 登録するPublisherの名前．self.pub.{name}で取り出せる．
            topic (str): Publishするtopic名
            msg_type (Any): メッセージの型
            queue_size (int, optional): キューサイズ. Defaults to 1.
        """
        self._pub[name] = rospy.Publisher(topic, msg_type, queue_size=queue_size)
