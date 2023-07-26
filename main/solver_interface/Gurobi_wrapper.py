from abc import ABC
from typing import Union, List
from gurobipy import Model, LinExpr

from .solver_interface import(
    IVar,
    ILinExpr,
    ConstraintType,
    IModel,
)

class GurobiVar(IVar):
    def __init__(self, name, lower_boud: float, upper_bound: float, boolean: bool, model: Model):
        vtype = self.get_var_type(boolean)
        self.var = model.addVar( 
            ub = upper_bound,
            lb= lower_boud,
            name= name,
            vtype = vtype,
        )

class GurobiLinExpr(ILinExpr):
    def __init__(
        self,
        vars: List[Union[GurobiVar, "GurobiLinExpr"]],
        coefs: List[float],
    ):
        self.var = LinExpr(coefs, [var.var for var in vars])

class GurobiModel(IModel):
    def __init__(self):
        self.model = Model()

    def add_variable(self, var: IVar):
        pass
    
    def add_constraint(self, expr: GurobiLinExpr, rhs: float, operator:ConstraintType):
        gurobi_operator = self.get_operator(operator)
        self.model.addConstraint(expr,gurobi_operator,rhs )
    
    def add_objective(self, expr: GurobiLinExpr, sense):
        self.model.setObjective(expr, sense)