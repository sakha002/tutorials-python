from dependency_injector import containers, providers


dict1 ={
    "param1": 1,
    "param2": 2.0
}

dict2 ={
    "param3": 3.0
}

class MyClass1:
    def __init__(self, param1:int, param2:float):
        # self.key = key
        self.param1=param1
        self.param2=param2

class MyClass2:
     def __init__(self,  param4:MyClass1, param3:int):
        self.param3=param3
        self.param4=param4
        
# class Lister:
#     objects = dict()
#     def add_object(self, obj):
#         self.objects[obj.key] = obj

#     def retrun_object(self, key):
#         return self.objects[key]
    
class  Container1(containers.DeclarativeContainer):
    factory1 = providers.Factory(
        MyClass1,
        param1 = dict1["param1"],
        param2= dict1["param2"],
    )
    
    
    
    
   
    

class Container2(containers.DeclarativeContainer):

    cont1 = providers.DependenciesContainer()
    # factory3 = providers.Factory(
    #     MyClass1,
    #     param1 = dict1["param1"],
    #     param2= dict1["param2"],
    # )
    
    factory2 = providers.Factory(
        MyClass2,
        param4=cont1.factory1,
        param3=4
    )

class Cont3(containers.DeclarativeContainer):
    cont1 = providers.Container(
        Container1
    )
    cont2 =  providers.Container(
        Container2,
        cont1
    )

if __name__ == "__main__":
    container = Cont3()
    # container2 = Container2()
    container.init_resources()
    # object1 = container2.factory1()
    object2 = container.cont2().factory2()
    # container2.wire(modules=[__name__])
    
    print(object2.param4)
    
    

    