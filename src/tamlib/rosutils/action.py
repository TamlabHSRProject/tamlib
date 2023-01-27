from typing import Any, Callable, Optional, Union

import actionlib
import rospy

from ..utils import InterfaceDict


class Action:
    def __init__(self) -> None:
        self.node_name = rospy.get_name()
        self._action = InterfaceDict()
        self._action["server"] = InterfaceDict()
        self._action["client"] = InterfaceDict()

    @property
    def action(self):
        return self._action

    @action.setter
    def action(
        self,
        name: str,
        action: Union[actionlib.SimpleActionServer, actionlib.SimpleActionClient],
    ):
        if isinstance(action, actionlib.SimpleActionServer):
            self._action.server[name] = action
        elif isinstance(action, actionlib.SimpleActionClient):
            self._action.client[name] = action

    def server_register(
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
        self._action.server[name] = actionlib.SimpleActionServer(
            ns, action_type, execute_cb, auto_start=False
        )
        self._action.server[name].start()

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
        self._action.client[name] = actionlib.SimpleActionClient(
            ns,
            action_type,
        )
        if wait_for_server:
            _timeout = rospy.Duration(timeout) if timeout > 0 else rospy.Duration()
            result = self._action.client[name].wait_for_server(_timeout)
            if result is False:
                raise TimeoutError(
                    f"'{ns}' timed out ({timeout} secs elapsed).",
                )
