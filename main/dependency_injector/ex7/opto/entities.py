from typing import Callable, List
from dataclasses import dataclass

@dataclass(frozen=True)
class Price:
        name: str
        market: str
        settlement_point: str

@dataclass(frozen=True)
class Asset:
        name: str
        type: str
        properties: dict # we will define static types for these as well
        
@dataclass(frozen=True)
class AssetGroup:
        name: str
        type: str
        price: Price
        assets: List[Asset]
        properties: dict

@dataclass(frozen=True)
class Portfolio:
        name: str
        asset_groups: List[AssetGroup]
        assets: List[Asset]
        


class PriceHolder: 
    prices = dict()
    
    def __init__(self, price_factory: Callable[..., Price])-> None:
        self._price_factory = price_factory
        return
    
    def add_all_prices(self, price_dict_list: List[dict])-> None:
        for price_dict in price_dict_list:
            self.prices[price_dict["name"]] = self._price_factory(**price_dict)
            
    def get_price_by_name(self, price_name) -> Price:
        return self.prices[price_name]

class AssetHolder:
    assets = dict()
    
    def __init__(self, asset_factory: Callable[..., Price])-> None:
        self._asset_factory = asset_factory
        return
    
    def add_all_assets(self, asset_dict_list: List[dict])-> None:
        for asset_dict in asset_dict_list:
            self.assets[asset_dict["name"]] = self._asset_factory(**asset_dict)
        return
    
    def get_all_assets(self) -> List[Asset]:
        return self.assets
            
    def get_assets_by_name(self, asset_names: List[str]) -> List[Asset]:
        return [self.assets[asset_name] for asset_name in asset_names]
    

