from abc import abstractmethod


class NodeABC:
    @abstractmethod
    def delete(self) -> None:
        """Destructor"""
        ...

    @abstractmethod
    def run(self) -> None:
        """実行関数"""
        ...
