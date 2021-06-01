import pyomo.environ as aml

# Need to decide if we reference components based on name or id. ID needs to be known before instantiation if using this for reference.

## Pyomo concepts that we can do differently
# aml.variable_dict - array concept (use pointers?)
# aml.constraint_dict - array concept (use pointers?)
# self.model() - to reference the model container object 

class Solver:
    def __init__(
        self,
        solver: str = "cbc",
        solver_params: dict = None,
        ):
        self.optimizer = aml.SolverFactory(solver)

        if solver_params:
            for param, value in solver_params:
                self.optimizer.options[param] = value

    def solve(self, model, tee=True):
        result = self.optimizer.solve(model, tee=tee)
        return result


class Model:
    def __init__(
        self,
        ):
        super().__init__()

        self.model = aml.ConcreteModel()

        self.objective_terms = {}

        self.component_elements = {}

    def add_element(self, element):
        self.component_elements[element.id] = element
        setattr(self.model, element.id, element.block)

    def build(self):
        self.update_coupling()
        self.create_objective()

    def update_coupling(self):
        # could also just search for block component elements rather than needing to define additional lists
        # will also likely need to differentiate between physical and market/economic objects
        for component_element in self.component_elements.values():
            if hasattr(component_element, "update_coupling_constraints"):
                component_element.update_coupling_constraints()

    def create_objective(self):
        nested_all_objective_terms = [list(component_element.objective_terms.values()) for component_element in self.component_elements.values()]
        all_objective_terms = [item for sublist in nested_all_objective_terms for item in sublist]
        self.model.objective = aml.Objective(expr = sum(objective_term for objective_term in all_objective_terms))
                    


# todo time index, objective, load, coupling, construction, etc.

# debate - do we need indices/sets attached to the model - likely not. Still needs to be an object somewhere - i.e. period
# explore if we can use generator expressions for kernel container objects (i.e. list and dict comps)

class Generator:
    def __init__(
        self,
        interval_set,
        params: dict,
        ):
        super().__init__()

        self.block = aml.Block(concrete=True)

        ## Setup
        # self.id = uuid4()
        self.id = params["name"]
        self.objective_terms = {}

        ## Parameters
        self.block.maxMW = params["maxMW"]
        self.block.minMW = params["minMW"]
        self.block.max_ramp = params["max_ramp"]
        self.block.marginal_cost = params["marginal_cost"]
        self.block.initial_commit = params["initial_commit"]

        ## Variables
        self.block.output = aml.Var(interval_set, bounds = (0,self.block.maxMW))

        self.block.commit = aml.Var(interval_set, domain=aml.Binary)


        ## Expressions - this is where we define a common interface for model resource elements
        # Note in Optopy we have to have a generic and dynamic common interface for market products, different time indices, etc.
        # Will just use net_export as an interface for now
        def _net_export(b,idx):
            return b.output[idx]
        
        self.block.net_export = aml.Expression(interval_set, rule=_net_export)

        ## Constraints
        # Note constraints should work with regular pyomo expression syntax
        # Should be able to reduce the amount of boilerplate here

        def _output_commit_upper_bound(b,idx):
            return b.output[idx] - b.commit[idx] * b.maxMW <= 0

        self.block.con_output_commit_upper_bound = aml.Constraint(interval_set, rule=_output_commit_upper_bound)

        def _output_commit_lower_bound(b,idx):
            return b.commit[idx] * b.minMW - b.output[idx] <= 0

        self.block.con_output_commit_lower_bound = aml.Constraint(interval_set, rule=_output_commit_lower_bound)

        # todo Add constraints/costs for ramping, min up, min down, startup/shutdown costs, etc.

        ## Objective Terms
        # Unclear whether this expression object needs to be added to block/model - may be enough just to have it in the objective

        def _interval_cost(b,idx):
            return b.marginal_cost * b.output[idx]

        self.block.interval_cost = aml.Expression(interval_set, rule=_interval_cost)
        
        def _total_cost(b):
            return sum(b.interval_cost[idx] for idx in interval_set)

        self.block.total_cost = aml.Expression(rule=_total_cost)

        self.objective_terms["marginal_cost"] = self.block.total_cost
        

class Load:
    def __init__(
        self,
        interval_set,
        params: dict,
        ):
        super().__init__()

        ## Setup
        self.id = params["name"]
        self.objective_terms = {}

        self.block = aml.Block(concrete=True)


        ## Params
        # Decide if we want load as mutable param - need to use param object
        # not sure what difference is with raw numpy - may be pyomo mutability - check this
        # self.load = aml.parameter_dict()
        self.block.load = params["load"]
        # for interval in interval_set:
        #     self.load[interval] = params["load"][interval]

        ## Expressions
        def _net_export(b,idx):
            return -b.load[idx]

        self.block.net_export = aml.Expression(interval_set, rule=_net_export)


class Collection:
    def __init__(
        self,
        interval_set,
        params: dict,
        ):
        super().__init__()

        ## Setup
        self.id = params["name"]
        self.objective_terms = {}

        self.block = aml.Block(concrete=True)

        # Need to reference interval set in coupling method, either need to add it to class or pass it to function.
        self.interval_set = interval_set

        self.component_element_ids = params["component_element_names"] # resources or other collections

        ## Parameters
        self.block.import_limit = params["import_limit"] # assume positive
        self.block.export_limit = params["export_limit"]

        ## Expressions/Variables
        # Should we define net_export as a variable and set equal later - allows to define upper and lower bounds here
        # or should it be an expression here, and we reconstruct constraints later whenever we add elements - i.e. the coupling aspect of the problem
        # It really depends on how the model is "constructed" - one option is to define everything as constructor functions that are called when the model is constructed, but order of construction matters here!
        # If we define as a variable, then we decouple them. 
        
        self.block.net_export = aml.Var(interval_set, bounds = (-self.block.import_limit,self.block.export_limit))

    def update_coupling_constraints(self):
        
        def _coupling_net_export(b,idx):
            sum_of_component_element_net_exports_idx = sum(getattr(b.parent_block(),component_element_id).net_export[idx] for component_element_id in self.component_element_ids)
            return b.net_export[idx] - sum_of_component_element_net_exports_idx == 0

        self.block.con_coupling_net_export = aml.Constraint(self.interval_set, rule=_coupling_net_export)
