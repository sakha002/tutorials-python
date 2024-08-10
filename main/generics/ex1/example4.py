from typing import List
from dataclasses import dataclass

@dataclass
class Content:
    name: str

@dataclass
class SenderBase:
    name: str

class Box:
    def __init__(self, content: Content):
        self.content = content
    
    def get_content(self) -> Content:
        return self.content

    def submit(self, senders: List[SenderBase]):
        for sender in senders:
            print(f"Sender_{sender.name}_submitted_box_with_content_name_{self.content.name}")

@dataclass
class Toy(Content):
    name: str
    year_built: int

@dataclass
class Document(Content):
    name: str
    pages: int

@dataclass
class ToySender(SenderBase):
    name: str
    preferred_shipping: str

@dataclass
class DocumentSender(SenderBase):
    name: str
    email: str

class ToyBox(Box):
    def __init__(self, content: Toy):
        super().__init__(content)
    

def main():
    sender1 = DocumentSender(name='SomeCompany', email='contact@example.com')
    sender2 = ToySender(name='Walmart', preferred_shipping='overnight')

    box = ToyBox(content=Toy(name='puzzle', year_built=2023))

    box.submit(senders=[sender1, sender2])  # This should raise an error!


if __name__ == "__main__":
    main()
