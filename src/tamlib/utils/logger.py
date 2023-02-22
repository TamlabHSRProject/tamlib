import sys
from loguru import logger
import rospy
import copy

class Logger:
    def __init__(self, logger_name=None, loglevel='INFO',
                 format='<green>{time:HH:mm:ss}</green> | <level>{level: <8}</level> | {extra[logger_name]} |<light-cyan>{name}</light-cyan>:<light-cyan>{function}</light-cyan>:<light-cyan>{line}</light-cyan>: - <level>{message}</level>',) -> None:
        """Initialize function

        Parameters
        ----------
        logger_name : str, optional
            logger_name. When not set, get from rospy.get_name() , by default None
        loglevel : str, optional
            standard output log level, by default 'INFO'
        format : str, optional
            log format, by default '<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | <level>{level: <8}</level> | {extra[logger_name]} |<cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> - <level>{message}</level>'
        """
        if logger_name is None:
            self._logger_name = rospy.get_name()
        else:
            self._logger_name = logger_name 
        logger.remove()
        self._logger = copy.deepcopy(logger.bind(logger_name=self._logger_name))
        self._logger.add(sys.stdout, format=format, level=loglevel)
        self._logger.level("TRACE", color="<white>")
        self._logger.level("DEBUG", color="<blue><bold>")
        self._logger.level("INFO", color="<cyan><bold>")
        self._logger.level("SUCCESS", color="<green><bold>")
        self._logger.level("WARNING", color="<yellow><bold>")
        self._logger.level("ERROR", color="<red><bold>")
        self._logger.level("CRITICAL", color="<RED><bold>")

    def start_dump_log(self, path: str, loglevel='DEBUG'):
        """function to start dump log to file

        Parameters
        ----------
        path : str
            logfile path
        loglevel : str, optional
            dump log level, by default 'DEBUG'
        """
        self._logger.add(path, level=loglevel)

    def logdebug(self, message) -> None:
        self._logger.opt(depth=1).debug(message)

    def loginfo(self, message: str) -> None:
        self._logger.opt(depth=1).info(message)

    def logwarn(self, message: str) -> None:
        self._logger.opt(depth=1).warning(message)

    def logerr(self, message: str) -> None:
        self._logger.opt(depth=1).error(message)

    def logfatal(self, message: str) -> None:
        self._logger.opt(depth=1).critical(message)
