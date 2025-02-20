from twin4build.base import ControlSignalMap
import sys
import os
import numpy as np
import twin4build.utils.input_output_types as tps

class ControlSignalMapSystem(ControlSignalMap):
    """
    This class allows to apply custom control signal maps coming from controllers
    to the input signal of a distribution device.
    It allows use cases like: 
    - mapping a PI controller output to more than one distribution device input, with custom logic
    - Applying normalizations and clamping to control signals
    - Any operation on the control signal can be applied here
    The input is a scalar and the output is a vector.
    The logic can be defined in a custom function, (see example)
    """

    def __init__(self, 
                **kwargs):
        super().__init__(**kwargs)
        self.input = {"actualValue": tps.Vector()}
        self.output = {} #Output will be set when connecting the component to the model
        self._config = {"parameters": []}

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

    
    def do_step(self, secondTime=None, dateTime=None, stepSize=None):
        """
        The do_step function is called every time step.
        It must follow certaing rules to ensure the correct behavior:
        - The input signal must be a scalar
        - The output signal must be a vector
        - The input signal must be accessed as self.input["actualValue"] 
        - The outbound connections will be defined when instantiating and connecting this model component in the fcn function. 
        - A custom, unique identifier must be given to each outbound connection.
        - The output signal must be defined as self.output["XXXXX"].set(value) where XXXXX is the identifier of the outbound connection
        """
        pass



