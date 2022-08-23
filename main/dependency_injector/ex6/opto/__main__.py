
import json
from dependency_injector.wiring import Provide, inject

from .builders import PortfolioBuilder
from .entities import PriceHolder
from .containers import OptoData


@inject
def build_portfolio(portfolio_dict: dict, portfolio_builder: PortfolioBuilder = Provide[OptoData.cont2.portfolio_builder]):
   portfolio = portfolio_builder.build_portfolio(portfolio_dict)
   
   print(portfolio.asset_groups[0].price.name)
   
   


@inject
def main(price_holder:  PriceHolder = Provide[OptoData.cont1.price_holder]) -> None:
    with open("./data/opto_inputs.json") as file:
        data_dict = json.load(file)
    price_holder.add_all_prices(price_list=data_dict.get("prices"))
    build_portfolio(data_dict.get("portfolio"))
    
    
   
if __name__ == "__main__":
    container = OptoData()
    container.wire(modules=[__name__])

    main()