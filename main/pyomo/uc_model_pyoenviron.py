import numpy as np
import pandas as pd
import pyomo.kernel as pyo

from flat_block_classes_pyoenviron import Model, Collection, Generator, Load, Solver

## Notes - need to decide how to pass in time interval set - to model, as separate object?

def instantiate_model():
    # This method effectively replicates the Optopy ModelBuilder class in a manual way. We would package this up based on a standard input schema

    # Data - assume we're running in project directory
    data_directory_path = "data"
    data_file_name = "benchmark_data.xlsx"

    # Generator names in excel file are non-unique
    gen_data = pd.read_excel(f"{data_directory_path}/{data_file_name}", sheet_name="Generator")
    load_data = pd.read_excel(f"{data_directory_path}/{data_file_name}", sheet_name="Load", index_col=0)

    # Model
    instance = Model()

    # Sets
    gen_index = set(gen_data.index)
    gen_names = [f"gen_name_{idx}" for idx in gen_index]

    interval_index = set(load_data.index) ## check this

    ## Parameter Definition
    all_gen_params = [
        {   
            "name": gen_names[idx],
            "maxMW": np.minimum(gen_data["Pmax"][idx],gen_data["Available Capacity"][idx]),
            "minMW": np.minimum(gen_data["Pmin"][idx],gen_data["Available Capacity"][idx]),
            "max_ramp": gen_data["Ramp"][idx],
            "marginal_cost": gen_data["Price"][idx],
            "initial_commit": gen_data["Status"][idx],
        } for idx in gen_index
    ]

    load_params = {
        "name": "load_0",
        "load": load_data["Load Profile"] * 0.9 * gen_data["Available Capacity"].sum()
    }

    collection_params = {
        "name": "collection_0",
        "import_limit": 0,
        "export_limit": 0,
        "component_element_names": gen_names + ["load_0"]
    }

    ## Element addition
    for idx in gen_index:
        instance.add_element(Generator(interval_index, all_gen_params[idx]))

    instance.add_element(Load(interval_index, load_params))

    instance.add_element(Collection(interval_index, collection_params))

    ## Model construction

    instance.build()

    return instance.model

def solve_instance(instance):
    opt = Solver()
    opt.solve(instance, tee=True)

def write_model_file(model, directory, filename, file_format="lp", symbolic_solver_labels=True):        
        file_path = f"{directory}/{filename}.{file_format}"
        model.write(file_path, io_options={"symbolic_solver_labels": symbolic_solver_labels})

def write_model_formulation(model, file_path=None):
    if file_path:
        with open(file_path, "w") as file:
            model.pprint(ostream=file)
    else:
        model.pprint()

def get_results(model):
    pyomo_objects = model.component_objects([pyo.Var, pyo.Expression, pyo.Param])
    return {
        obj.name: [[index, pyo.value(obj[index])] for index in obj] for obj in pyomo_objects
    }
    

if __name__ == "__main__":
    instance = instantiate_model()
    solve_instance(instance)

    results_dict = get_results(instance)
