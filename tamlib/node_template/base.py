import inspect
from typing import Any, Callable, Dict, Optional

import rospy
from std_srvs.srv import Empty, EmptyResponse

from ..rosutils import Action, Publisher, Subscriber
from .base_abc import NodeABC


class Node(NodeABC, Publisher, Subscriber, Action):
    def __init__(self) -> None:
        super().__init__()
        Publisher.__init__(self)
        Subscriber.__init__(self)
        Action.__init__(self)

        self.run_enable = rospy.get_param(self.node_name + "/run_enable", True)
        rospy.Service(self.node_name + "/start", Empty, self.srvf_start)
        rospy.Service(self.node_name + "/stop", Empty, self.srvf_stop)

    def delete(self) -> None:
        for sub in self.sub.values():
            sub.unregister()

    def logdebug(self, message) -> None:
        rospy.logdebug(f"[{self.node_name}]: {message}")

    def loginfo(self, message) -> None:
        rospy.loginfo(f"[{self.node_name}]: {message}")

    def logwarn(self, message) -> None:
        rospy.logwarn(f"[{self.node_name}]: {message}")

    def logerr(self, message) -> None:
        rospy.logerr(f"[{self.node_name}]: {message}")

    def logfatal(self, message) -> None:
        rospy.logfatal(f"[{self.node_name}]: {message}")

    def set_update_ros_time(self, name: Optional[str] = None) -> None:
        """更新時間（ROS時間）をセットする

        Args:
            name (Optional[str], optional):
                指定されたkeyで記録される．指定しない場合，呼び出した関数名がkeyになる. Defaults to None.
        """
        if name is None:
            name = inspect.stack()[1].function
        self.update_ros_time[name] = rospy.Time.now()

    def srvf_start(self, req: Empty) -> Empty:
        """ノード開始用のサービス

        Args:
            req (Empty): 空．

        Returns:
            Empty: 空．
        """
        self.run_enable = True
        self.set_update_ros_time(name="start")
        return EmptyResponse()

    def srvf_stop(self, req: Empty) -> Empty:
        """ノード停止用のサービス

        Args:
            req (Empty): 空．

        Returns:
            Empty: 空．
        """
        self.run_enable = False
        self.set_update_ros_time(name="stop")
        return EmptyResponse()

    def pub_register(self, name: str, topic: str, msg_type: Any, queue_size=10) -> None:
        """Publisherを登録する

        Args:
            name (str): 登録するPublisherの名前．self.pub.{name}で取り出せる．
            topic (str): Publishするtopic名
            msg_type (Any): メッセージの型
            queue_size (int, optional): キューサイズ. Defaults to 1.
        """
        Publisher.register(self, name, topic, msg_type, queue_size)

    def sub_register(
        self,
        name: str,
        topic: str,
        queue_size=10,
        execute_func: Optional[Callable] = None,
    ) -> None:
        """Subscriberを登録する

        Args:
            name (str): 登録するSubsciberの名前．self.sub.{name}で取り出せる．
                また，self.subf_{name}でcallback関数が作成される．変数はself.{name}で取り出せる．
            topic (str): Subscribeするtopic名
            queue_size (int, optional): キューサイズ. Defaults to 10.
            execute_func (Optional[Callable], optional):
                コールバック関数内で実行する関数．Defaults to None.

        Raises:
            AttributeError: callback関数用の変数が定義されていない場合．
        """
        Subscriber.register(self, name, topic, queue_size, execute_func)

    def sync_sub_register(
        self,
        name: str,
        topics: Dict[str, str],
        queue_size=10,
        delay=0.1,
        allow_headerless=False,
        execute_func: Optional[Callable] = None,
    ) -> None:
        """複数トピック同期のSubscriberを登録する

        Args:
            name (str): 登録するSubsciberの名前．self.sub.{name}で取り出せる．
                また，self.subf_{name}でcallback関数が作成される．
                変数はtopicsで指定した名前で取り出せる．(e.g. self.{name1}, self.{name2})．
            topics (Dict[str, str]): Subscribeするtopic名の辞書．
                dict = {name1: topic1, name2: topic2} で指定する．
            queue_size (int, optional): キューサイズ. Defaults to 10.
            delay (float, optional): 許容する同期ズレ [sec]. Defaults to 0.1.
            allow_headerless (bool, optional): Trueの場合，headerがなくても同期する（非推奨）.
                Defaults to False.
            execute_func (Optional[Callable], optional):
                コールバック関数内で実行する関数．Defaults to None.
        """
        Subscriber.sync_register(
            self, name, topics, queue_size, delay, allow_headerless, execute_func
        )

    def wait_for_message(
        self,
        topic_name: str,
        timeout: Optional[float] = None,
    ) -> Any:
        """新規メッセージを取得する

        Args:
            topic_name (str): トピックの変数名（sub_regster()で登録したもの）．
            timeout (Optional[float], optional): タイムアウト. Defaults to None.

        Raises:
            AttributeError: トピックのコールバック関数が定義されていない場合．

        Returns:
            Any: 取得したメッセージ．タイムアウトの場合，False．
        """
        return Subscriber.wait_for_message(self, topic_name, timeout)

    def action_server_register(
        self,
        name: str,
        ns: str,
        action_type: Any,
        execute_cb: Optional[Callable] = None,
    ) -> None:
        """ActionServerを登録する

        Args:
            name (str): 登録するActionServerの変数名．self.action.server.{name}で取り出せる．
            ns (str): 登録するActionServerの名前．
            action_type (Any): ActionServerの型．
            execute_cb (Optional[Callable], optional):
                クライアントからゴールが投げられたときに実行する関数．Defaults to None.
        """
        Action.server_register(self, name, ns, action_type, execute_cb)

    def client_register(
        self,
        name: str,
        ns: str,
        action_type: Any,
        wait_for_server=True,
        timeout=-1.0,
    ) -> None:
        """ActionClientを登録する

        Args:
            name (str): 登録するActionClientの変数名．self.action.client.{name}で取り出せる．
            ns (str): 呼び出すActionServerの名前．
            action_type (Any): 呼び出すActionServerの型．
            wait_for_server (bool, optional): ActionServerとの接続を待つか.
                Defaults to True.
            timeout (float, optional): ActionServerとの接続を待つ場合のタイムアウト時間 [sec].
                Defaults to -1.0.

        Raises:
            TimeoutError: 指定したtimeout秒が経過した場合．
        """
        Action.client_register(self, name, ns, action_type, wait_for_server, timeout)
