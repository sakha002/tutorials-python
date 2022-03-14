from pyomo.environ import *

scenario_data = {}
scenario_data['Scenario1'] = (2.1, 23.5)
scenario_data['Scenario2'] = (0.2, 4.2)
scenario_data['Scenario3'] = (0.6, 0.1)
scenario_data['Scenario4'] = (1.1, -4.4)

def pysp_scenario_tree_model_callback():
    from pyomo.pysp.scenariotree.tree_structure_model \
        import CreateConcreteTwoStageScenarioTreeModel

    st_model = CreateConcreteTwoStageScenarioTreeModel(len(scenario_data))

    first_stage = st_model.Stages.first()
    second_stage = st_model.Stages.last()

    # First Stage
    st_model.StageCost[first_stage] = 'FirstStageCost'
    st_model.StageVariables[first_stage].add('v1')
    st_model.StageVariables[first_stage].add('xd')
    st_model.StageDerivedVariables[first_stage].add('xde')

    # Second Stage
    st_model.StageCost[second_stage] = 'SecondStageCost'
    st_model.StageVariables[second_stage].add('y')
    st_model.StageVariables[second_stage].add('v2')
    return st_model


# Creates an instance for each scenario
def pysp_instance_creation_callback(scenario_name, node_names):
    a, b = scenario_data[scenario_name]

    model = ConcreteModel()
    model.v1 = Block()
    model.v1.x = Var()
    model.y = Var()
    model.FirstStageCost = Expression(expr=model.v1.x )
    model.SecondStageCost = Expression(expr=a * (model.y - b) )
    model.obj = Objective(expr=model.FirstStageCost + model.SecondStageCost)

    model.xd = Var()
    model.xde = Expression(expr=model.v1.x)
    # model.yd = Var()
    # model.yde = Expression(expr=model.y - b)

    model.v2 = Block()
    v2 = model.v2
    v2.yd_test = Var()
    v2.yde = Expression(expr=model.y - b)

    model.c = ConstraintList()
    model.c.add(model.y == model.v1.x)
    model.c.add(model.xd == model.v1.x)
    model.c.add(v2.yd_test == model.y - b)
    return model