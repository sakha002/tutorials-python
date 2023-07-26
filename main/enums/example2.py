

from enum import Enum

class Asset:
    pass


class AssetType(Enum):

    # this method assigns the string representation of the asset type into .value
    # def __new__(cls, asset: Asset):
    #     print("Hey we are excuting new")
    #     obj = object.__new__(cls)
    #     obj._value_ = cls._get_class_name(asset)
    #     return obj

    # initialize the enum variables
    def __init__(self, asset: Asset):
        print("we are doing init now")
        self._asset_type_name = self._get_class_name(asset)
        self._asset_type = asset


    @classmethod
    def _get_class_name(cls, asset: Asset) -> str:
        return getattr(asset, "__name__")

class ErcotBess:
    pass


class ErcotBess2:
    pass



class ErcottAssetType(AssetType, Enum):
    bess = ErcotBess
    bess2 = ErcotBess2


a = ErcottAssetType

print(a.bess.value)


b = ErcottAssetType.bess

print(b.name)

