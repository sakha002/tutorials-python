import dataclasses

from dependency_injector import providers


@dataclasses.dataclass
class Foo:
    markets: list
    # option2: object


config = providers.Configuration(
    default={
        "regions": [
            {
                "name": 'r1',
                "markets" : ['option1', 'option2']
            },
            {
                "name": 'r2',
                'markets': ['option3', 'option4']
            }
        ]
    }
)


class ConfigOptionSelector(providers.Callable):

    def __init__(self, selector, sub_lists):
        super().__init__(
            lambda target, sub_lists: [item for sublist in sub_lists for item in sublist[target]],
            selector,
            sub_lists,
        )


foo = providers.Factory(
    Foo,
    markets=ConfigOptionSelector(
        'markets',
        config.regions,
    )
)


if __name__ == '__main__':
    # config.target.from_env('TARGET')
    f = foo()
    print(f.markets)
