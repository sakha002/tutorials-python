

from enum import Enum

class Assb:
    pass


class AssbType(Enum):

    # this method assigns the string representation of the asset type into .value
    def __new__(cls, assb: Assb):
        print("Hey we are excuting new")
        obj = object.__new__(cls)
        # obj._value_ = cls._get_class_name(assb)
        return obj

    # initialize the enum variables
    def __init__(self, assb: Assb):
        print("we are doing init now")
        self._asset_type_name = self._get_class_name(assb)
        self._assb_type = assb


    @classmethod
    def _get_class_name(cls, assb: Assb) -> str:
        return getattr(assb, "__name__")

class ErtBees3:
    pass
class ErtBees2:
    pass

class ErtBees:
    pass

class ErtAssbType(AssbType, Enum):
    bees = ErtBees
    bees2 = ErtBees2
    bees3 = ErtBees3


print (" we are here")
a = ErtAssbType
print (" we are done")