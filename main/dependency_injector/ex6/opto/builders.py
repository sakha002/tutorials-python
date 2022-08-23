
from re import A
from tkinter import N
from typing import Callable, List

from .entities import Asset, PriceHolder, AssetGroup, Portfolio, Price

class AssetBuilder:
    
    def build_asset(self, asset_dict:dict):
        return Asset(
            name = asset_dict.get("name"),
            type= asset_dict.get("type")
        )
        
class AssetGroupBuilder:
    
    def build_asset_group(self, price: Price, asset_group_dict: dict):
        return AssetGroup(
            name= asset_group_dict.get("name"),
            price= price,
            assets= [AssetBuilder().build_asset(asset_dict) for asset_dict in asset_group_dict.get("assets")],
        )
           
    
class PortfolioBuilder:
    
    def __init__(self, price_holder:PriceHolder):
        self._price_holder = price_holder
        return
    
    def build_portfolio(self, portfolio_dict: dict):
        
        return Portfolio(
            name = portfolio_dict.get("name"),
            assets=[AssetBuilder().build_asset(asset_dict) for asset_dict in portfolio_dict.get("assets")],
            asset_groups=[
                AssetGroupBuilder().build_asset_group(
                    price=self._price_holder.get_price_by_name(ag_dict.get("price_name")),
                    asset_group_dict= ag_dict,
                    
                )
                for ag_dict in portfolio_dict.get("asset_groups")
            ]
            
        )
   