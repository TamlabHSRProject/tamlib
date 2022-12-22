import inspect
from types import MethodType
from typing import Any, Callable, Dict, List, Optional, Tuple, Union

import message_filters
import rospy

from .ifdict import InterfaceDict


class Subscriber:
    def __init__(self) -> None:
        self.node_name = rospy.get_name()
        self.update_ros_time: Dict[str, rospy.Time] = {
            "init": rospy.Time.now()
        }  # name:update time
        self._sub = InterfaceDict()

    def __del__(self):
        for sub in self.sub.values():
            sub.unregister()

    @property
    def sub(self):
        return self._sub

    @sub.setter
    def sub(self, name: str, subscriber: rospy.Subscriber):
        self._sub[name] = subscriber

    def set_update_ros_time(self, name: Optional[str] = None) -> None:
        """更新時間（ROS時間）をセットする

        Args:
            name (Optional[str], optional):
                指定されたkeyで記録される．指定しない場合，呼び出した関数名がkeyになる. Defaults to None.
        """
        if name is None:
            name = inspect.stack()[1].function
        self.update_ros_time[name] = rospy.Time.now()

    def _make_callback(
        self,
        name: str,
        execute_func: Optional[Callable],
    ) -> MethodType:
        """動的にcallback関数を作成する

        Args:
            name (str): self.subf_{name}でcallback関数が作成される．変数はself.{name}で取り出せる．
            execute_func (Optional[Callable]): コールバック関数内で実行する関数．

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
            self.set_update_ros_time(name)

        if execute_func is not None:
            setattr(self, exec_func_name, execute_func)
        setattr(self, func_name, callback.__get__(self, self.__class__))
        return getattr(self, func_name)

    def _make_sync_callback(
        self,
        name: str,
        var_names: Union[Tuple[str], List[str]],
        execute_func: Optional[Callable],
    ) -> MethodType:
        """動的にcallback関数を作成する

        Args:
            name (str): self.subf_{name}でcallback関数が作成される．変数はself.{name}で取り出せる．
            var_names (tuple, list): 登録する変数名リスト．
            execute_func (Optional[Callable]): コールバック関数内で実行する関数．

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
            self.set_update_ros_time(name)

        if execute_func is not None:
            setattr(self, exec_func_name, execute_func)
        setattr(self, func_name, sync_callback.__get__(self, self.__class__))
        return getattr(self, func_name)

    def register(
        self,
        name: str,
        topic: str,
        queue_size=10,
        callback_func: Optional[Callable] = None,
        execute_func: Optional[Callable] = None,
    ) -> None:
        """Subscriberを登録する

        Args:
            name (str): 登録するSubsciberの名前．self.sub.{name}で取り出せる．
                また，self.subf_{name}でcallback関数が作成される．変数はself.{name}で取り出せる．
            topic (str): Subscribeするtopic名
            queue_size (int, optional): キューサイズ. Defaults to 10.
            callback_func (Optional[Callable], optional):
                コールバック関数．Defaults to None.
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
        if callback_func is None:
            callback_func = self._make_callback(name, execute_func)
        self._sub[name] = rospy.Subscriber(
            topic,
            msg_type,
            callback_func,
            queue_size=queue_size,
        )

    def sync_register(
        self,
        name: str,
        topics: Dict[str, str],
        queue_size=10,
        delay=0.1,
        allow_headerless=False,
        callback_func: Optional[Callable] = None,
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
            callback_func (Optional[Callable], optional):
                コールバック関数．Defaults to None.
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
        if callback_func is None:
            callback_func = self._make_sync_callback(name, var_names, execute_func)
        self.sub[name] = synchronizer.registerCallback(callback_func)

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
        if not hasattr(self, topic_name):
            raise AttributeError(f"'self' has no attribute '{topic_name}'.")

        start_ros_time = rospy.Time.now()
        prev_ros_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if timeout is not None:
                if prev_ros_time - start_ros_time > rospy.Duration(secs=timeout):
                    message = f"'{topic_name}' timed out."
                    rospy.logwarn(f"[{self.node_name}]: {message}")
                    return False
                prev_ros_time = rospy.Time.now()

            if topic_name not in self.update_ros_time.keys():
                continue
            if start_ros_time >= self.update_ros_time[topic_name]:
                return getattr(self, topic_name)
