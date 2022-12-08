from abc import abstractmethod


class NodeABC:
    @abstractmethod
    def run(self) -> None:
        """実行関数"""
        ...
