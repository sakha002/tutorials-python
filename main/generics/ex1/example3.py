


from typing import Generic, TypeVar, List
from dataclasses import dataclass

@dataclass
class Content:
    name: str

@dataclass
class SenderBase:
    name: str

ContentT = TypeVar('ContentT', bound=Content)
SenderT = TypeVar('SenderT', bound=SenderBase)


class Box(Generic[ContentT, SenderT]):
    def __init__(self, content: ContentT):
        self.content = content
    
    def get_content(self) -> ContentT:
        return self.content

    def submit(self, senders: List[SenderT]):
        for sender in senders:
            print(f"Sender_{sender.name}_submitted_box_with_content_name_{self.content.name}")

@dataclass
class Toy(Content):
    name: str
    year_built: int

@dataclass
class ToySender(SenderBase):
    name: str
    preferred_shipping: str

class ToyBox(Box[Toy, ToySender]):
    def __init__(self, content: Toy):
        super().__init__(content)
    

def main():
    sender1 = ToySender(name='Amazon', preferred_shipping='overnight')
    sender2 = ToySender(name='Walmart', preferred_shipping='overnight')

    box = ToyBox(content=Toy(name='puzzle', year_built=2023))

    box.submit(senders=[sender1, sender2])


if __name__ == "__main__":
    main()
