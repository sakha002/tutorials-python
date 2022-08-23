from typing import Callable, List


class Price:

    def __init__(self, name: str, market: str, settlement_point: str):
        self.name = str(name)
        self.market = str(market)
        self.settlement_point = str(settlement_point)


class PriceHolder:
    
    prices = dict()
    
    
    def __init__(self, price_factory: Callable[..., Price])-> None:
        self._price_factory = price_factory
        return
    
    def add_all_prices(self, price_list: List[dict])-> None:
        for price_dict in price_list:
            self.prices[price_dict["name"]] = self._price_factory(**price_dict)
            
    def get_price_by_name(self, price_name):
        return self.prices[price_name]
    
class Asset:
    def __init__(self, name: str,  type:str):
        self.name = name
        self.type = type
        
class AssetGroup:
    def __init__(self, name:str, assets: List[Asset], price: Price):
        self.name = name
        self.price = price
        self.assets = assets

class Portfolio:
    
    def __init__(self, name: str, asset_groups: List[AssetGroup], assets: List[Asset]):
        self.name = name,
        self.asset_groups = asset_groups
        self.assets = assets

