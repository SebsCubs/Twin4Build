
"""
Neural policy controller for RL-based building control

Features:
- The controller is based on a neural network model that takes as input the current state of the building and outputs the control signal
- The neural network model is trained using reinforcement learning techniques to optimize building energy performance
- The input and output of the controller is defined by a JSON schema that contains the keys and types of the input and output signals
- The neural policy is initialized at instantiation and the weights are updated manually by the user, typically through a training process

"""
from twin4build.base import NeuralPolicyController
import sys
import os
import torch.nn as nn
import torch
import twin4build.utils.input_output_types as tps
import numpy as np
uppath = lambda _path,n: os.sep.join(_path.split(os.sep)[:-n])
file_path = uppath(os.path.abspath(__file__), 9)
sys.path.append(file_path)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


#TODO Add signature pattern

class NeuralPolicyControllerSystem(NeuralPolicyController):
    def __init__(self, 
                input_size = None,
                output_size = None,
                input_output_schema = None,
                policy_model = None,
                **kwargs):
        super().__init__(**kwargs)

        assert input_size is not None, "Input size must be defined"
        assert output_size is not None, "Output size must be defined"
        self.input_size = input_size
        self.output_size = output_size

        assert input_output_schema is not None, "Input and output schema must be defined"
        """
        Example of an input_output_dictionary:

        "[020B][020B_space_heater]": {
            "indoorTemperature": {
                "min": 0,
                "max": 40,
                "description": "Room 020B indoor temperature"
            },
            "indoorCo2Concentration": {
                "min": 0,
                "max": 4000,
                "description": "Room 020B indoor CO2 concentration"
            }
        }

        The outputs are the setpoints overrided from the input components that are setpoints, identified by the "scheduleValue" key.
        """
        #Validate the schema, will raise error if invalid
        self.validate_schema(input_output_schema)

        self.input_output_schema = input_output_schema

        self.is_training = False

        if policy_model is not None:
            self.policy = policy_model
        else:
            self.policy = nn.Sequential(
                nn.Linear(self.input_size, 128),
                nn.ReLU(),
                nn.Linear(128, 64),
                nn.ReLU(),
                nn.Linear(64, 64),
                nn.ReLU(),
                nn.Linear(64, 64),
                nn.ReLU(),
                nn.Linear(64, self.output_size),
                nn.Sigmoid()
            ).to(device)


        #Input and output can be any arbitrary vector
        self.input = {"actualValue": tps.Vector()}
        self.output = {} #Output will be set when connecting the controller to the model
        self.device =  device
        self._config = {"parameters": ["input_size", "output_size"]}
    @property
    def config(self):
        return self._config

    def cache(self,
            startTime=None,
            endTime=None,
            stepSize=None):
        pass    

    def initialize(self,
                    startTime=None,
                    endTime=None,
                    stepSize=None,
                    model=None):
        pass

    def normalize_input_data(self, data):
        normalized_data = []
        # Loop through each component in the input schema
        for component, params in self.input_output_schema["input"].items():
            # Loop through each parameter in the component
            for param_name, param_info in params.items():
                min_val = param_info["min"]
                max_val = param_info["max"]
                # Get the corresponding value from input data
                value = data[len(normalized_data)]  # Assuming data is ordered array matching schema
                # Normalize the value
                normalized_value = (value - min_val) / (max_val - min_val)
                normalized_data.append(normalized_value)
        return normalized_data
    
    def denormalize_output_data(self, data):
        """
        Denormalize the output data using the schema.
        Inputs: data (numpy array or tensor of shape (output_size,))
        Outputs: denormalized data (numpy array)
        The min and max values are stored in the input_output_schema["output"] dictionary.
        """
        if not isinstance(data, np.ndarray):
            data = np.array(data)
        keys = list(self.input_output_schema["output"].keys())
        min_vals = np.array([self.input_output_schema["output"][key]["min"] for key in keys])
        max_vals = np.array([self.input_output_schema["output"][key]["max"] for key in keys])
        denormalized_data = data * (max_vals - min_vals) + min_vals
        return denormalized_data
    
    def load_policy_model(self, policy_path):
        self.policy.load_state_dict(torch.load(policy_path))
        
    def validate_schema(self, data):
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
                
    
    def select_action(self, state):
        state = torch.FloatTensor(state)
        with torch.no_grad():
            mean, std = self.policy(state)
        dist = torch.distributions.Normal(mean, std)
        if self.is_training:
            action = dist.sample()
        else:
            action = mean
        action_logprob = dist.log_prob(action).sum()
        return action.numpy(), action_logprob.numpy()

    def do_step(self, secondTime=None, dateTime=None, stepSize=None):
        input_data = self.input["actualValue"].get()
        normalized_input = self.normalize_input_data(input_data)
        state = torch.tensor(normalized_input).float().to(self.device)
        action, action_logprob = self.select_action(state)

        #NOTE: The output is not normalized, I will test the results and assess whether normalization is needed or not
        for idx, output_key in enumerate(self.output.keys()):
            self.output[output_key].set(action[idx])


        """
        denormalized_output = self.denormalize_output_data(action)
        
        #The resulting denormalized output follows the same order as the input schema,
        for idx, key in enumerate(self.input_output_schema["output"]):
            output_key = key + "_input_signal"
            self.output[output_key].set(denormalized_output[idx])
        """
        



