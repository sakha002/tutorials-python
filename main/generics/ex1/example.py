from dataclasses import dataclass


from typing import Generic,TypeVar, List

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
            print(f"Sender_{sender.name}_submited_box_with_content_name_{self.content.name}")

@dataclass
class Toy(Content):
    name: str
    year_built:  int

@dataclass
class Fedex(SenderBase):
    name: str
    option: str

class ToyBox(Box[Toy, Fedex]):
    def __init__(self, content: Toy):
        super().__init__(content)
    

def main():

    sender1 = Fedex(name='Amazon', option='overnight')
    sender2 = Fedex(name='Walmart', option='overnight')

    box = ToyBox(content=Toy(name='puzzle', year_built=2023))

    box.submit(senders=[sender1, sender2])


if __name__ == "__main__":
    main()









# # Usage
# int_box = Box(123)
# str_box = Box("Hello")

# print(int_box.get_content())  # 123
# print(str_box.get_content())  # Hello