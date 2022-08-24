
from typing import Callable, List
from .entities import Asset, AssetGroup, Portfolio, Price, AssetHolder, PriceHolder

        
class AssetGroupBuilder:
    def build_asset_group(self, price: Price, assets: List[Asset], asset_group_dict: dict):
        return AssetGroup(
            name= asset_group_dict.get("name"),
            price= price,
            assets= assets,
            type=asset_group_dict.get("type"),
            properties=asset_group_dict.get("properties"),
        )
           
    
class PortfolioBuilder:
    def __init__(self, price_holder:PriceHolder, asset_holder: AssetHolder):
        self._price_holder = price_holder
        self._asset_holder = asset_holder
        return
    
    def build_portfolio(self, portfolio_dict: dict): 
        return Portfolio(
            name = portfolio_dict.get("name"),
            assets= self._asset_holder.get_all_assets(),
            asset_groups=[
                AssetGroupBuilder().build_asset_group(
                    price=self._price_holder.get_price_by_name(ag_dict.get("price_name")),
                    assets=self._asset_holder.get_assets_by_name(ag_dict.get("asset_names")),
                    asset_group_dict= ag_dict,
                )
                for ag_dict in portfolio_dict.get("asset_groups")
            ]
            
        )
   