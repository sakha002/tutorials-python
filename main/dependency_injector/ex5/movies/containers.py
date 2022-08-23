
from dependency_injector import containers, providers

from . import finders, listers, entities


class Container1(containers.DeclarativeContainer):

    config = providers.Configuration(yaml_files=["config.yml"])
    
    movie = providers.Factory(entities.Movie)

    csv_finder = providers.Singleton(
        finders.CsvMovieFinder,
        movie_factory=movie.provider,
        path=config.finder.csv.path,
        delimiter=config.finder.csv.delimiter,
    )

    sqlite_finder = providers.Singleton(
        finders.SqliteMovieFinder,
        movie_factory=movie.provider,
        path=config.finder.sqlite.path,
    )

    finder = providers.Selector(
        config.finder.type,
        csv=csv_finder,
        sqlite=sqlite_finder,
    )

    
class Container2(containers.DeclarativeContainer):
    cont1 = providers.DependenciesContainer()
    config2 = providers.Configuration()

    lister = providers.Factory(
        listers.MovieLister,
        movie_finder=cont1.finder,
        movie_director = config2.finder.director 
    )
    
class  Container3(containers.DeclarativeContainer):
    cont1 = providers.Container(
        Container1
    )
    
    cont2 = providers.Container(
        Container2,
        cont1 = cont1
    )