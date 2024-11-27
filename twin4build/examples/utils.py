import os
def get_path(list_: str) -> str:
    """
    Get the full path to a file in the examples directory.
    """
    path = os.path.join(os.path.dirname(__file__))
    print(path)
    for path_ in list_:
        path = os.path.join(path, path_)

    return os.path.join(os.path.dirname(__file__), path)


def validate_schema(data):
    if not isinstance(data, dict):
        raise TypeError("Data should be a dictionary.")
    
    # Validate input exists
    if "input" not in data:
        raise ValueError("'input' key is required in the data.")
    if not isinstance(data["input"], dict):
        raise TypeError("'input' should be a dictionary.")
    
    # Validate each component in input
    for component, component_data in data["input"].items():
        if not isinstance(component_data, dict):
            raise TypeError(f"Component '{component}' data should be a dictionary.")
        
        # Validate each parameter in the component
        for param, param_data in component_data.items():
            if not isinstance(param_data, dict):
                raise TypeError(f"Parameter '{param}' in component '{component}' should be a dictionary.")
            
            # Check required keys for each parameter
            required_keys = {"min": (float, int), "max": (float, int), "description": str}
            for key, expected_type in required_keys.items():
                if key not in param_data:
                    raise ValueError(f"'{key}' key is required for parameter '{param}' in component '{component}'.")
                
                if not isinstance(param_data[key], expected_type):
                    raise TypeError(
                        f"'{key}' in parameter '{param}' under component '{component}' should be of type {expected_type.__name__}."
                    )
                
            if param_data["min"] > param_data["max"]:
                raise ValueError(
                    f"'min' value should be <= 'max' for parameter '{param}' in component '{component}'."
                )