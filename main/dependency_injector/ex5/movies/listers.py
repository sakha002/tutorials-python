
from .finders import MovieFinder


class MovieLister:

    def __init__(self, movie_finder: MovieFinder, movie_director: str):
        self._movie_finder = movie_finder
        self.movie_director = movie_director

    def movies_directed_by(self, director):
        return [
            movie for movie in self._movie_finder.give_all_movies()
            if movie.director == director
        ]

    def movies_released_in(self, year):
        print(f"target director is {self.movie_director}")
        return [
            movie for movie in self._movie_finder.give_all_movies()
            if (movie.year == year and self.movie_director == movie.director)
        ]