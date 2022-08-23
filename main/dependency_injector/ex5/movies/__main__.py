
from dependency_injector.wiring import Provide, inject
from .finders import CsvMovieFinder, MovieFinder
from .listers import MovieLister
from .containers import Container3, Container1, Container2


@inject
def print_my_list(year, lister: MovieLister = Provide[Container3.cont2.lister]):
    print("Francis Lawrence movies:")
    for movie in lister.movies_directed_by("Francis Lawrence"):
        print("\t-", movie)

    print("2016 movies:")
    for movie in lister.movies_released_in(year):
        print("\t-", movie)



@inject
def main(finder:  MovieFinder = Provide[Container3.cont1.finder]) -> None:
    finder.find_all()
    print_my_list(2016)
    
    
   
if __name__ == "__main__":
    container = Container3()
    container.cont2.config2.from_dict(
        {
            "finder": {
                "csv": {
                    "path" : "data/movies.csv",
                    "delimiter": ","
                },
                "sqlite": {
                    "path": "data/movies.db"

                },
                "director": "Francis Lawrence",
            }
        }
    )
    container.cont1.config.finder.type.from_env("MOVIE_FINDER_TYPE")
    container.wire(modules=[__name__])

    main()