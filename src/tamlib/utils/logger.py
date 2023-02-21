import sys
from loguru import logger
import rospy

class Logger:
    def __init__(self, logger_name=None, loglevel='INFO',
                 format='<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green>  <level>{level: <8}</level> | {extra[logger_name]} |<cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>',) -> None:
        if logger_name is None:
            self._logger_name = rospy.get_name()
        else:
            self._logger_name = logger_name
        self._logger = logger.bind(logger_name=self._logger_name)
        self._logger.remove()
        self._logger.add(sys.stdout, format=format, level=loglevel, backtrace=False, diagnose=False)

    def start_dump(self, path: str, level='DEBUG'):
        self._logger.add(path, level=level,
                         filter=lambda record: record["extra"]["logger_name"] == self._logger_name)

    def logdebug(self, message) -> None:
        self._logger.opt(depth=1).debug(message)

    def loginfo(self, message) -> None:
        self._logger.opt(depth=1).info(message)

    def logwarn(self, message) -> None:
        self._logger.opt(depth=1).warning(message)

    def logerr(self, message) -> None:
        self._logger.opt(depth=1).error(message)

    def logfatal(self, message) -> None:
        self._logger.opt(depth=1).critical(message)
