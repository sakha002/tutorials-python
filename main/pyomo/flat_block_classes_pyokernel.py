import pyomo.kernel as aml

## Notes
# name is a protected term in pyomo objects - use id as our reference attribute
# getting an error when trying to solve/write the model based on a variable_dict: "AttributeError: 'variable_dict' object has no attribute 'is_potentially_variable'"
# Only 3 variable dicts defined in the model objects, so should be easy to pin down. Same formulation works fine in vanilla Pyomo

## Pyomo concepts that we can do differently
# aml.variable_dict - array concept (use pointers?)
# aml.constraint_dict - array concept (use pointers?)
# aml.expression_dict - array concept (use pointers?)
# self.parent - to reference the model container object - basically a way of accessing a global address of variables contained in other blocks
# objective_terms - idea is that we can define expressions on individual blocks, tag them as objective terms (likely add cost/benefit markers as well), then these can be collected and summed in the model

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


class Model(aml.block):
    def __init__(
        self,
        ):
        super().__init__()

        self.objective_terms = {}

        self.component_element_ids = []

    def add_element(self, element):
        self.component_element_ids.append(element.id)
        setattr(self, element.id, element)

    def build(self):
        self.update_coupling()
        self.create_objective()

    def update_coupling(self):
        # could also just search for block component elements rather than needing to define additional lists
        # will also likely need to differentiate between physical and market/economic objects when searching over direct elements
        for component_element_id in self.component_element_ids:
            component_element = getattr(self,component_element_id)
            if hasattr(component_element, "update_coupling_constraints"):
                component_element.update_coupling_constraints()

    def create_objective(self):
        nested_all_objective_terms = [list(getattr(self,component_element_id).objective_terms.values()) for component_element_id in self.component_element_ids]
        all_objective_terms = [item for sublist in nested_all_objective_terms for item in sublist]
        self.objective = aml.objective(expr = sum(objective_term for objective_term in all_objective_terms))
                    

# debate - do we need indices/sets attached to the model - likely not. Still needs to be an object somewhere - i.e. period
# explore if we can use generator expressions for pyomo kernel container objects (i.e. list and dict comps)

class Generator(aml.block):
    def __init__(
        self,
        interval_set,
        params: dict,
        ):
        super().__init__()

        ## Setup
        # self.id = uuid4()
        self.id = params["name"]
        self.objective_terms = {}

        ## Parameters
        self.maxMW = aml.parameter(params["maxMW"])
        self.minMW = aml.parameter(params["minMW"])
        self.max_ramp = aml.parameter(params["max_ramp"])
        self.marginal_cost = aml.parameter(params["marginal_cost"])
        self.initial_commit = aml.parameter(params["initial_commit"])

        ## Variables
        self.output = aml.variable_dict()
        for interval in interval_set:
            self.output[interval] = aml.variable(lb=0,ub=self.maxMW)

        self.commit = aml.variable_dict()
        for interval in interval_set:
            self.commit[interval] = aml.variable(domain=aml.Binary)

        ## Expressions - this is where we define a common interface for model resource elements
        # Note in Optopy we have to have a generic and dynamic common interface for market products, different time indices, etc.
        # Will just use net_export as an interface for now
        self.net_export = aml.expression_dict()
        for interval in interval_set:
            self.net_export[interval] = aml.expression(expr=self.output)

        ## Constraints
        # Can express constraints abstractly (lhs, rhs, body, ub, lb, etc.) or with interpreted syntax - see below
        self.con_output_commit_upper_bound = aml.constraint_dict()
        for interval in interval_set:
            # body_expr = self.output[interval] - self.commit[interval] * self.maxMW
            # self.con_output_commit_upper_bound[interval] = aml.constraint(body=body_expr, ub=0)

            self.con_output_commit_upper_bound[interval] = aml.constraint(self.output[interval] - self.commit[interval] * self.maxMW <= 0)

        self.con_output_commit_lower_bound = aml.constraint_dict()
        for interval in interval_set:
            # body_expr =  self.commit[interval] * self.minMW - self.output[interval]
            # self.con_output_commit_lower_bound[interval] = aml.constraint(body=body_expr, ub=0)

            self.con_output_commit_lower_bound[interval] = aml.constraint(self.commit[interval] * self.minMW - self.output[interval] <=0)

        # todo Add constraints/costs for ramping, min up, min down, startup/shutdown costs, etc.

        ## Objective Terms
        # Unclear whether this expression object needs to be added to block/model - may be enough just to have it in the objective
        self.interval_cost = aml.expression_dict()
        for interval in interval_set:
            self.interval_cost[interval] = aml.expression(self.marginal_cost * self.output[interval])
        self.objective_terms["total_cost"] = sum(self.interval_cost[interval] for interval in interval_set)
        

class Load(aml.block):
    def __init__(
        self,
        interval_set,
        params: dict,
        ):
        super().__init__()

        ## Setup
        self.id = params["name"]
        self.objective_terms = {}

        ## Params
        # Decide if we want load as mutable param - need to use param object
        # not sure what difference is with raw numpy - may be pyomo mutability - check this
        # self.load = aml.parameter_dict()
        self.load = params["load"]
        # for interval in interval_set:
        #     self.load[interval] = params["load"][interval]

        ## Expressions
        self.net_export = aml.expression_dict()
        for interval in interval_set:
            self.net_export[interval] = aml.expression(-self.load[interval])


class Collection(aml.block):
    def __init__(
        self,
        interval_set,
        params: dict,
        ):
        super().__init__()

        ## Setup
        self.id = params["name"]
        self.objective_terms = {}


        # Need to reference interval set in coupling method, either need to add it to class or pass it to function.
        self.interval_set = interval_set

        self.component_element_ids = params["component_element_names"] # resources or other collections

        ## Parameters
        self.import_limit = aml.parameter(params["import_limit"]) # assume positive
        self.export_limit = aml.parameter(params["export_limit"])

        ## Expressions/Variables
        # Should we define net_export as a variable and set equal later - allows to define upper and lower bounds here
        # or should it be an expression here, and we reconstruct constraints later whenever we add elements - i.e. the coupling aspect of the problem
        # It really depends on how the model is "constructed" - one option is to define everything as constructor functions that are called when the model is constructed, but order of construction matters here!
        # If we define as a variable, then we decouple them. 
        self.net_export = aml.variable_dict()
        for interval in interval_set:
            self.net_export[interval] = aml.variable(lb=-self.import_limit,ub=self.export_limit)

    def update_coupling_constraints(self):
        self.con_coupling_net_export = aml.constraint_dict()
        for interval in self.interval_set:
            sum_of_component_element_net_exports_interval = sum(getattr(self.parent,component_element_id).net_export[interval] for component_element_id in self.component_element_ids)
            # body_expr =  self.net_export[interval] - sum_of_component_element_net_exports_interval
            # self.con_coupling_net_export[interval] = aml.constraint(body=body_expr, rhs=0)

            self.con_coupling_net_export[interval] = aml.constraint(self.net_export[interval] - sum_of_component_element_net_exports_interval == 0)
