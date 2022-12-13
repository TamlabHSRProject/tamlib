import rospy


class Open3D:
    def __init__(self) -> None:
        self.node_name = rospy.get_name()
