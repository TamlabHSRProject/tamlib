import inspect
from types import MethodType
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import actionlib
import message_filters
import rospy
from std_srvs.srv import Empty, EmptyResponse

from .base_abc import NodeABC
from .ifdict import InterfaceDict


class Node(NodeABC):
    def __init__(self) -> None:
        rospy.sleep(0.01)
        self.node_name = rospy.get_name()

        self.update_ros_time = dict(init=rospy.Time.now())  # name:update time

        self.run_enable = rospy.get_param(self.node_name + "/run_enable", True)

        rospy.Service(self.node_name + "/start", Empty, self.srvf_start)
        rospy.Service(self.node_name + "/stop", Empty, self.srvf_stop)

        self.pub = InterfaceDict()
        self.sub = InterfaceDict()
        self.action = InterfaceDict()
        self.action["server"] = InterfaceDict()
        self.action["client"] = InterfaceDict()

    def delete(self):
        ...

    def logdebug(self, message):
        rospy.logdebug(f"[{self.node_name}]: {message}")

    def loginfo(self, message):
        rospy.loginfo(f"[{self.node_name}]: {message}")

    def logwarn(self, message):
        rospy.logwarn(f"[{self.node_name}]: {message}")

    def logerr(self, message):
        rospy.logerr(f"[{self.node_name}]: {message}")

    def logfatal(self, message):
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

    def pub_register(self, name: str, topic: str, msg_type: Any, queue_size=1):
        """Publisherを登録する

        Args:
            name (str): 登録するPublisherの名前．self.pub.{name}で取り出せる．
            topic (str): Publishするtopic名
            msg_type (Any): メッセージの型
            queue_size (int, optional): キューサイズ. Defaults to 1.
        """
        self.pub[name] = rospy.Publisher(topic, msg_type, queue_size=queue_size)

    def _make_callback(
        self,
        name: str,
        execute_func: Callable,
    ) -> MethodType:
        """動的にcallback関数を作成する

        Args:
            name (str): self.subf_{name}でcallback関数が作成される．変数はself.{name}で取り出せる．
            execute_func (Callable): コールバック関数内で実行する関数．

        Returns:
            MethodType: Subscriber method
        """
        func_name = f"subf_{name}"
        exec_func_name = f"exec_{func_name}"

        def callback(self, msg):
            if hasattr(self, exec_func_name):
                getattr(self, exec_func_name)(msg)
            else:
                setattr(self, name, msg)
            self.set_update_ros_time(func_name)

        if execute_func is not None:
            setattr(self, exec_func_name, execute_func)
        setattr(self, func_name, callback.__get__(self, self.__class__))
        return getattr(self, func_name)

    def _make_sync_callback(
        self,
        name: str,
        var_names: Union[Tuple[str], List[str]],
        execute_func: Callable,
    ) -> MethodType:
        """動的にcallback関数を作成する

        Args:
            name (str): self.subf_{name}でcallback関数が作成される．変数はself.{name}で取り出せる．
            var_names (tuple, list): 登録する変数名リスト．
            execute_func (Callable): コールバック関数内で実行する関数．

        Returns:
            MethodType: Subscriber method
        """
        func_name = f"subf_{name}"
        exec_func_name = f"exec_{func_name}"

        def sync_callback(self, *msgs):
            if hasattr(self, exec_func_name):
                getattr(self, exec_func_name)(*msgs)
            else:
                for i, msg in enumerate(msgs):
                    setattr(self, var_names[i], msg)
            self.set_update_ros_time(func_name)

        if execute_func is not None:
            setattr(self, exec_func_name, execute_func)
        setattr(self, func_name, sync_callback.__get__(self, self.__class__))
        return getattr(self, func_name)

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
        if not hasattr(self, name):
            raise AttributeError(
                f"'{self.__class__.__name__}' object has no attribute '{name}'."
            )
        msg_type = type(getattr(self, name))
        new_cb = self._make_callback(name, execute_func)
        self.sub[name] = rospy.Subscriber(
            topic,
            msg_type,
            new_cb,
            queue_size=queue_size,
        )

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
        interfaces = []
        var_names = []
        for var_name, topic in topics.items():
            interfaces.append(
                message_filters.Subscriber(
                    topic,
                    type(getattr(self, var_name)),
                )
            )
            var_names.append(var_name)

        synchronizer = message_filters.ApproximateTimeSynchronizer(
            interfaces, queue_size, delay, allow_headerless=allow_headerless
        )
        new_cb = self._make_sync_callback(name, var_names, execute_func)
        self.sub[name] = synchronizer.registerCallback(new_cb)

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
        func_name = f"subf_{topic_name}"
        if not hasattr(self, func_name):
            raise AttributeError(f"'self' has no attribute '{func_name}'.")

        start_ros_time = rospy.Time.now()
        prev_ros_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if timeout is not None:
                if prev_ros_time - start_ros_time > rospy.Duration(secs=timeout):
                    self.logwarn(f"wait for '{topic_name}' is timeout!")
                    return False
                prev_ros_time = rospy.Time.now()

            if func_name not in self.update_ros_time.keys():
                continue
            if start_ros_time >= self.update_ros_time[func_name]:
                return getattr(self, topic_name)

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
        self.action.server[name] = actionlib.SimpleActionServer(
            ns, action_type, execute_cb, auto_start=False
        )
        self.action.server[name].start()

    def action_client_register(
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
        self.action.client[name] = actionlib.SimpleActionClient(
            ns,
            action_type,
        )
        if wait_for_server:
            _timeout = rospy.Duration(timeout) if timeout > 0 else rospy.Duration()
            result = self.action.client[name].wait_for_server(_timeout)
            if result is False:
                raise TimeoutError(
                    f"ActionServer: '{ns}' is timeout ({timeout} secs elapsed).",
                )
