
from dependency_injector import containers, providers

from . import  entities, builders


class PriceContainer(containers.DeclarativeContainer):

    # config = providers.Configuration(yaml_files=["config.yml"])
    
    price = providers.Factory(entities.Price)
    
    price_holder = providers.Singleton(
        entities.PriceHolder,
        price_factory = price.provider,
    )

    
    
class PortfolioContainer(containers.DeclarativeContainer):
    price_container = providers.DependenciesContainer()
    # config2 = providers.Configuration()
    
    # asset = providers.Factory(
    #     entities.Asset,
    # )

    portfolio_builder = providers.Singleton(
        builders.PortfolioBuilder,
        price_holder = price_container.price_holder,
    )


class  OptoData(containers.DeclarativeContainer):
    cont1 = providers.Container(
        PriceContainer
    )
    
    cont2 = providers.Container(
        PortfolioContainer,
        price_container = cont1
    )