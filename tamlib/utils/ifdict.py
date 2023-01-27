from addict import Dict


class InterfaceDict(Dict):
    def __missing__(self, name: str):
        raise KeyError(name)

    def __getattr__(self, name: str):
        try:
            value = super().__getattr__(name)
        except KeyError:
            AttributeError(
                f"'{self.__class__.__name__}' object has no " f"attribute '{name}'"
            )
        except Exception as e:
            print(e)
        else:
            return value
        raise
