#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

import rospy
from action_msgs.msg import BuriAction
from hoge_msgs.msg import Fuga, Hoge, Piyo
from node_template import Node


class TemplateClass(Node):
    def __init__(self) -> None:
        super().__init__()

        # Publisher
        self.pub_register("hoge", "/topic/hoge", Hoge)

        # Subscriber
        self.fuga = Fuga()
        self.sub_register("fuga", "/topic/fuga")
        # self.sub_register("fuga", "/topic/fuga", execute_func=self.run)

        # Sync subscriber
        self.piyo = Piyo()
        self.buri = Piyo()
        sync_piyoburi = {
            "piyo": "/topic/piyo",
            "buri": "/topic/buri",
        }
        self.sync_sub_register(
            "piyoburi",
            sync_piyoburi,
            que_size=10,
            delay=0.1,
        )

        # ActionServer
        self.action_server_register(
            "buri",
            "/action/buri",
            BuriAction,
            self.action_cb,
        )

        # ActionClient
        self.action_client_register(
            "buri",
            "/action/buri",
            BuriAction,
            timeout=5,
        )

    def action_cb(self, goal) -> None:
        """write your code"""

    def run(self) -> None:
        if self.run_enable is False:
            return

        """write your code"""


def main():
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", 30)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = TemplateClass()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            cls.run()
        except rospy.exceptions.ROSException:
            rospy.logerr(f"[{rospy.get_name()}]: FAILURE")
        loop_wait.sleep()


if __name__ == "__main__":
    main()
