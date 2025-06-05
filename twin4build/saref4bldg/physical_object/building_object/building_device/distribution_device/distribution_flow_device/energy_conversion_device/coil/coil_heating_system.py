from .coil import Coil
import twin4build.saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_flow_device.energy_conversion_device.coil.coil as coil
from typing import Union
import twin4build.saref.measurement.measurement as measurement
from twin4build.utils.constants import Constants
import numpy as np
import twin4build.base as base
from twin4build.utils.signature_pattern.signature_pattern import SignaturePattern, Node, Exact, MultipleMatches, Optional, IgnoreIntermediateNodes
import twin4build.utils.input_output_types as tps

def get_signature_pattern():
    node0 = Node(cls=base.Coil, id="<n<SUB>1</SUB>(Coil)>")
    node1 = Node(cls=base.FlowJunction, id="<n<SUB>2</SUB>(FlowJunction)>")
    node2 = Node(cls=(base.Fan, base.AirToAirHeatRecovery, base.Coil), id="<Fan, AirToAirHeatRecovery, Coil\nn<SUB>3</SUB>>")

    node = Node(cls=base.PropertyValue, id="<PropertyValue\nn<SUB>23</SUB>>")
    node = Node(cls=(float, int), id="<Float, Int\nn<SUB>24</SUB>>")
    node = Node(cls=base.FlowCoefficient, id="<FlowCoefficient\nn<SUB>25</SUB>>")

    sp = SignaturePattern(ownedBy="CoilHeatingSystem", priority=0)

    sp.add_edge(IgnoreIntermediateNodes(object=node0, subject=node1, predicate="suppliesFluidTo"))
    sp.add_edge(IgnoreIntermediateNodes(object=node0, subject=node2, predicate="hasFluidSuppliedBy"))

    sp.add_modeled_node(node0)
    # sp.add_parameter("airFlowRateMax.hasValue", node13)
    sp.add_input("airFlowRate", node1, "airFlowRateIn")
    sp.add_input("inletAirTemperature", node2, ("outletAirTemperature", "primaryTemperatureOut", "outletAirTemperature"))

    return sp

class CoilHeatingSystem(coil.Coil):
    # sp = [get_signature_pattern()]
    def __init__(self,
                **kwargs):
        super().__init__(**kwargs)
        self.specificHeatCapacityAir = Constants.specificHeatCapacity["air"]

        self.input = {"inletAirTemperature": tps.Scalar(),
                      "outletAirTemperatureSetpoint": tps.Scalar(),
                      "airFlowRate": tps.Scalar()}
        self.output = {"Power": tps.Scalar(),
                       "outletAirTemperature": tps.Scalar()}
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
        '''
         updates the input and output variables of the coil and calculates the power output and air temperature based on the input air temperature, air flow rate, and air temperature setpoint. 
         If the air flow rate is zero, the output power and air temperature are set to NaN
        '''
        self.output.update(self.input)
        tol = 1e-5
        if self.input["airFlowRate"]>tol:
            if self.input["inletAirTemperature"] < self.input["outletAirTemperatureSetpoint"]:
                Q = self.input["airFlowRate"]*self.specificHeatCapacityAir*(self.input["outletAirTemperatureSetpoint"] - self.input["inletAirTemperature"])
                if np.isnan(Q):
                    raise ValueError("Q is not a number")
                self.output["outletAirTemperature"].set(self.input["outletAirTemperatureSetpoint"])
            else:
                Q = 0
            self.output["Power"].set(Q)
        else:
            # self.output["outletAirTemperature"] = self.input["outletAirTemperatureSetpoint"]
            self.output["outletAirTemperature"].set(self.input["inletAirTemperature"])
            self.output["Power"].set(0)

        


        