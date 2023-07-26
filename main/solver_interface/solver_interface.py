from abc import ABC
from typing import List, Union
from enum import Enum

class IVar(ABC):
    def __init__(self, name, lower_boud: float, upper_bound: float, boolean: bool):
        raise NotImplementedError
    

class ILinExpr(IVar, ABC):
    def __init__(
        self,
        name,
        vars: List[Union[IVar, "ILinExpr"]],
        coefs: List[float],
    ):

        raise NotImplementedError

class ConstraintType(Enum):
    LESSEQUAL = 0
    EQUAL=1

class IModel(ABC):

    def add_variable(self, var: IVar):
        raise NotImplementedError
    
    def add_constraint(self, expr: ILinExpr, rhs: float, operator:ConstraintType):
        raise NotImplementedError
    
    def add_objective(self, expr: ILinExpr, sense):
        raise NotImplementedError

    
