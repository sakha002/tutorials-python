import networkx
import pyomo.environ as pyo

model = pyo.ConcreteModel()

block_1 = pyo.Block()
block_1.x = pyo.Var(bounds=(0,3))
block_1.p = pyo.Param(initialize=5, mutable=True)

block_2 = pyo.Block()
block_2.x = pyo.Var(bounds=(2,5))
block_2.p = pyo.Param(initialize=-2, mutable=True)

def expr(b):
    return b.x * b.p

block_1.cost = pyo.Expression(rule=expr)
block_2.cost = pyo.Expression(rule=expr)

block_3 = pyo.Block()
block_3.block_1 = block_1
block_3.block_2 = block_2

model.block_3 = block_3

def obj_expr(m):
    return m.block_3.block_1.cost + m.block_3.block_2.cost
model.obj = pyo.Objective(rule=obj_expr)

p_scenarios = {}

p_scenarios["low"] = {"block_1": 2, "block_2": -10}
p_scenarios["med"] = {"block_1": 4, "block_2": -2}
p_scenarios["high"] = {"block_1": 6, "block_2": 4}

def pysp_instance_creation_callback(scenario_name, node_names):

    instance = model.clone()
    instance.block_3.block_1.p.store_values(p_scenarios[scenario_name]["block_1"])
    instance.block_3.block_2.p.store_values(p_scenarios[scenario_name]["block_2"])

    return instance

def pysp_scenario_tree_model_callback():
    # Return a NetworkX scenario tree.
    g = networkx.DiGraph()

    ce1 = "block_3.block_1.cost"
    ce2 = "block_3.block_2.cost"

    # ce1 = "CostExpressions[1]"
    g.add_node("Root",
               cost = ce1,
               variables = ["block_3.block_1.x"],
               derived_variables = [])

    # ce2 = "CostExpressions[2]"
    g.add_node("low",
               cost = ce2,
               variables = ["block_3.block_2.x"],
               derived_variables = [])
    g.add_edge("Root", "low", weight=0.3333)

    g.add_node("med",
               cost = ce2,
               variables = ["block_3.block_2.x"],
               derived_variables = [])
    g.add_edge("Root", "med", weight=0.3333)

    g.add_node("high",
               cost = ce2,
               variables = ["block_3.block_2.x"],
               derived_variables = [])
    g.add_edge("Root", "high", weight=0.3334)

    return g

if __name__ == "__main__":
    scenario_name = "high"
    instance = model.clone()
    instance.block_3.block_1.p.store_values(p_scenarios[scenario_name]["block_1"])
    instance.block_3.block_2.p.store_values(p_scenarios[scenario_name]["block_2"])

    opt = pyo.SolverFactory("glpk")
    opt.solve(instance)

    print(pyo.value(instance.block_3.block_1.x))
    print(pyo.value(instance.block_3.block_2.x))