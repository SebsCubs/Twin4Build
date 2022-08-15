from dateutil.tz import tzutc
import datetime
import matplotlib.pyplot as plt
import networkx as nx
import os
import sys
import pandas as pd
import warnings

sys.path.append('c:\\Users\\jabj\\OneDrive - Syddansk Universitet\\PhD_Project_Jakob\\Twin4build\\python\\BuildingEnergyModel\\BuildingEnergyModel') ###Only for testing before distributing package
from twin4build.saref4syst.connection import Connection 
from twin4build.saref4syst.connection_point import ConnectionPoint
from twin4build.saref4syst.system import System
from twin4build.utils.weather_station import WeatherStation
from twin4build.utils.schedule import Schedule
from twin4build.utils.node import Node
from twin4build.saref.measurement.measurement import Measurement
from twin4build.saref.date_time.date_time import DateTime
from twin4build.saref4bldg.building_space.building_space import BuildingSpace
from twin4build.saref4bldg.building_space.building_space_model import BuildingSpaceModel, NoSpaceModelException
from twin4build.saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_device import DistributionDevice
from twin4build.saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_control_device.controller.controller_model import ControllerModel
from twin4build.saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_flow_device.energy_conversion_device.air_to_air_heat_recovery.air_to_air_heat_recovery_model import AirToAirHeatRecoveryModel
from twin4build.saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_flow_device.energy_conversion_device.coil.coil_heating_model import CoilHeatingModel
from twin4build.saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_flow_device.energy_conversion_device.coil.coil_cooling_model import CoilCoolingModel
from twin4build.saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_flow_device.flow_controller.damper.damper_model import DamperModel
from twin4build.saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_flow_device.flow_controller.flow_meter.flow_meter_model import FlowMeterModel
from twin4build.saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_flow_device.flow_controller.valve.valve_model import ValveModel
from twin4build.saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_flow_device.flow_moving_device.fan.fan_model import FanModel
from twin4build.saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_flow_device.flow_terminal.space_heater.space_heater_model import SpaceHeaterModel




class BuildingEnergyModel:
    def __init__(self,
                timeStep = None,
                startPeriod = None,
                endPeriod = None,
                createReport = False):
        self.timeStep = timeStep
        self.startPeriod = startPeriod
        self.endPeriod = endPeriod
        self.createReport = createReport
        self.system_graph = nx.MultiDiGraph() ###
        self.system_graph_node_attribute_dict = {}
        self.system_graph_edge_label_dict = {}

        self.initComponents = []
        self.activeComponents = None
        self.system_dict = {}
        self.component_dict = {}

    def add_edge_(self, a, b, label):
        if (a, b) in self.system_graph.edges:
            max_rad = max(x[2]['rad'] for x in self.system_graph.edges(data=True) if sorted(x[:2]) == sorted([a,b]))
        else:
            max_rad = 0
        self.system_graph.add_edge(a, b, rad=max_rad+0, label=label)


    def add_connection(self, sender_obj, reciever_obj, senderPropertyName, recieverPropertyName):
        sender_obj_connection = Connection(connectsSystem = sender_obj, senderPropertyName = senderPropertyName)
        sender_obj.connectedThrough.append(sender_obj_connection)
        reciever_obj_connection_point = ConnectionPoint(connectionPointOf = reciever_obj, connectsSystemThrough = sender_obj_connection, recieverPropertyName = recieverPropertyName)
        sender_obj_connection.connectsSystemAt = reciever_obj_connection_point
        reciever_obj.connectsAt.append(reciever_obj_connection_point)

        self.add_edge_(sender_obj.systemId, reciever_obj.systemId, label=senderPropertyName) ###

        
        self.system_graph_node_attribute_dict[sender_obj.systemId] = {"label": sender_obj.__class__.__name__}
        self.system_graph_node_attribute_dict[reciever_obj.systemId] = {"label": reciever_obj.__class__.__name__}
        self.system_graph_edge_label_dict[(sender_obj.systemId, reciever_obj.systemId)] = senderPropertyName

    
    def add_weather_station(self):
        weather_station = WeatherStation(
            startPeriod = self.startPeriod,
            endPeriod = self.endPeriod,
            input = {},
            output = {},
            savedInput = {},
            savedOutput = {},
            createReport = self.createReport,
            connectedThrough = [],
            connectsAt = [],
            systemId = "weather_station")
        self.component_dict["weather_station"] = weather_station

    def add_occupancy_schedule(self):
        occupancy_schedule = Schedule(
            startPeriod = self.startPeriod,
            timeStep = self.timeStep,
            rulesetDict = {
                "ruleset_default_value": 0,
                "ruleset_start_minute": [0,0,0,0,0],
                "ruleset_end_minute": [0,0,0,0,0],
                "ruleset_start_hour": [0,5,8,12,18],
                "ruleset_end_hour": [6,8,12,18,22],
                "ruleset_value": [0,10,30,25,0]}, #35
            input = {},
            output = {},
            savedInput = {},
            savedOutput = {},
            createReport = False,
            connectedThrough = [],
            connectsAt = [],
            systemId = "occupancy_schedule")
        self.component_dict["occupancy_schedule"] = occupancy_schedule

    def add_temperature_setpoint_schedule(self):
        temperature_setpoint_schedule = Schedule(
            startPeriod = self.startPeriod,
            timeStep = self.timeStep,
            rulesetDict = {
                "ruleset_default_value": 20,
                "ruleset_start_minute": [0,0],
                "ruleset_end_minute": [0,0],
                "ruleset_start_hour": [0,6],
                "ruleset_end_hour": [6,18],
                "ruleset_value": [20,22]},
            input = {},
            output = {},
            savedInput = {},
            savedOutput = {},
            createReport = False,
            connectedThrough = [],
            connectsAt = [],
            systemId = "temperature_setpoint_schedule")
        self.component_dict["temperature_setpoint_schedule"] = temperature_setpoint_schedule

    def add_co2_setpoint_schedule(self):
        co2_setpoint_schedule = Schedule(
            startPeriod = self.startPeriod,
            timeStep = self.timeStep,
            rulesetDict = {
                "ruleset_default_value": 600,
                "ruleset_start_minute": [],
                "ruleset_end_minute": [],
                "ruleset_start_hour": [],
                "ruleset_end_hour": [],
                "ruleset_value": []},
            input = {},
            output = {},
            savedInput = {},
            savedOutput = {},
            createReport = False,
            connectedThrough = [],
            connectsAt = [],
            systemId = "co2_setpoint_schedule")
        self.component_dict["co2_setpoint_schedule"] = co2_setpoint_schedule


    def read_config(self):
        file_path = "C:/Users/jabj/OneDrive - Syddansk Universitet/PhD_Project_Jakob/Twin4build/python/BuildingEnergyModel/BuildingEnergyModel/configuration_template.xlsx"
        df_Systems = pd.read_excel(file_path, sheet_name="Systems")
        df_Spaces = pd.read_excel(file_path, sheet_name="Spaces")
        df_Dampers = pd.read_excel(file_path, sheet_name="Dampers")
        df_SpaceHeaters = pd.read_excel(file_path, sheet_name="SpaceHeaters")
        df_Valves = pd.read_excel(file_path, sheet_name="Valves")
        df_HeatingCoils = pd.read_excel(file_path, sheet_name="HeatingCoils")
        df_CoolingCoils = pd.read_excel(file_path, sheet_name="CoolingCoils")
        df_AirToAirHeatRecovery = pd.read_excel(file_path, sheet_name="AirToAirHeatRecovery")
        df_Fans = pd.read_excel(file_path, sheet_name="Fan")
        df_Controller = pd.read_excel(file_path, sheet_name="Controller")
        df_Nodes = pd.read_excel(file_path, sheet_name="Node")

        for ventilation_system_name in df_Systems["Ventilation system name"].dropna():
            ventilation_system = DistributionDevice(subSystemOf = [], hasSubSystem = [], systemId = ventilation_system_name)
            self.system_dict[ventilation_system_name] = ventilation_system
        
        for heating_system_name in df_Systems["Heating system name"].dropna():
            heating_system = DistributionDevice(subSystemOf = [], hasSubSystem = [], systemId = heating_system_name)
            self.system_dict[heating_system_name] = heating_system

        for cooling_system_name in df_Systems["Cooling system name"].dropna():
            cooling_system = DistributionDevice(subSystemOf = [], hasSubSystem = [], systemId = cooling_system_name)
            self.system_dict[cooling_system_name] = cooling_system

        for row in df_Spaces.dropna(subset=["Space name"]).itertuples(index=False):
            space_name = row[df_Spaces.columns.get_loc("Space name")]
            try: 
                space = BuildingSpaceModel(
                    densityAir = 1.225,
                    airVolume = 50,
                    startPeriod = self.startPeriod,
                    timeStep = self.timeStep,
                    input = {"generationCo2Concentration": 0.06,
                            "outdoorCo2Concentration": 500,
                            "shadesPosition": 0},
                    output = {"indoorTemperature": 21.5,
                            "indoorCo2Concentration": 500},
                    savedInput = {},
                    savedOutput = {},
                    createReport = True,
                    connectedThrough = [],
                    connectsAt = [],
                    systemId = space_name)
                self.component_dict[space_name] = space
            except NoSpaceModelException: 
                print("No fitting space model for space " + "\"" + space_name + "\"")
                print("Continuing...")
            

        for row in df_Dampers.dropna(subset=["Damper name"]).itertuples(index=False):
            damper_name = row[df_Dampers.columns.get_loc("Damper name")]
            #Check that an appropriate space object exists
            if damper_name[4:] not in self.component_dict:
                warnings.warn("Cannot find a matching mathing BuildingSpace object for damper \"" + damper_name + "\"")
            else:
                ventilation_system = self.system_dict[row[df_Dampers.columns.get_loc("Ventilation system")]]
                damper = DamperModel(
                    nominalAirFlowRate = Measurement(hasValue=row[df_Dampers.columns.get_loc("nominalAirFlowRate")]),
                    subSystemOf = [ventilation_system],
                    input = {},
                    output = {"airFlowRate": 0},
                    savedInput = {},
                    savedOutput = {},
                    createReport = self.createReport,
                    connectedThrough = [],
                    connectsAt = [],
                    systemId = damper_name)
                # supply_damper.output["airFlowRate"] = 0 ########################
                self.component_dict[damper_name] = damper
                ventilation_system.hasSubSystem.append(damper)

        for row in df_SpaceHeaters.dropna(subset=["Space heater name"]).itertuples(index=False):
            space_heater_name = row[df_SpaceHeaters.columns.get_loc("Space heater name")]
            #Check that an appropriate space object exists
            if space_heater_name[3:] not in self.component_dict:
                warnings.warn("Cannot find a matching mathing BuildingSpace object for space heater \"" + space_heater_name + "\"")
            else:
                heating_system = self.system_dict[row[df_SpaceHeaters.columns.get_loc("Heating system")]]
                space_heater = SpaceHeaterModel(
                    specificHeatCapacityWater = Measurement(hasValue=4180),
                    outputCapacity = Measurement(hasValue=row[df_SpaceHeaters.columns.get_loc("outputCapacity")]),
                    temperatureClassification = row[df_SpaceHeaters.columns.get_loc("temperatureClassification")],
                    thermalMassHeatCapacity = Measurement(hasValue=row[df_SpaceHeaters.columns.get_loc("thermalMassHeatCapacity")]),
                    timeStep = self.timeStep, 
                    subSystemOf = [heating_system],
                    input = {"supplyWaterTemperature": 60},
                    output = {"radiatorOutletTemperature": 22,
                                "Energy": 0},
                    savedInput = {},
                    savedOutput = {},
                    createReport = self.createReport,
                    connectedThrough = [],
                    connectsAt = [],
                    systemId = space_heater_name)
                self.component_dict[space_heater_name] = space_heater
                heating_system.hasSubSystem.append(space_heater)

        for row in df_Valves.dropna(subset=["Valve name"]).itertuples(index=False):
            valve_name = row[df_Valves.columns.get_loc("Valve name")]
            #Check that an appropriate space object exists
            if valve_name[2:] not in self.component_dict:
                warnings.warn("Cannot find a matching mathing BuildingSpace object for valve \"" + valve_name + "\"")
            else:
                heating_system = self.component_dict[valve_name.replace("V_", "SH_")].subSystemOf[0]
                valve = ValveModel(
                    valveAuthority = Measurement(hasValue=0.8),
                    flowCoefficient = Measurement(hasValue=row[df_Valves.columns.get_loc("flowCoefficient")]),
                    testPressure = Measurement(hasValue=row[df_Valves.columns.get_loc("testPressure")]),
                    subSystemOf = [heating_system],
                    input = {},
                    output = {},
                    savedInput = {},
                    savedOutput = {},
                    createReport = self.createReport,
                    connectedThrough = [],
                    connectsAt = [],
                    systemId = valve_name)
                self.component_dict[valve_name] = valve
                heating_system.hasSubSystem.append(valve)

        for row in df_HeatingCoils.dropna(subset=["Heating coil name"]).itertuples(index=False):
            heating_coil_name = row[df_HeatingCoils.columns.get_loc("Heating coil name")]
            ventilation_system = self.system_dict[row[df_HeatingCoils.columns.get_loc("Ventilation system")]]
            heating_system = self.system_dict[row[df_HeatingCoils.columns.get_loc("Heating system")]]
            heating_coil = CoilHeatingModel(
                specificHeatCapacityAir = Measurement(hasValue=1000),
                subSystemOf = [ventilation_system, heating_system],
                input = {"supplyAirTemperatureSetpoint": 23},
                output = {},
                savedInput = {},
                savedOutput = {},
                createReport = self.createReport,
                connectedThrough = [],
                connectsAt = [],
                systemId = heating_coil_name)
            self.component_dict[heating_coil_name] = heating_coil
            ventilation_system.hasSubSystem.append(heating_coil)
            heating_system.hasSubSystem.append(heating_coil)

        for row in df_CoolingCoils.dropna(subset=["Cooling coil name"]).itertuples(index=False):
            cooling_coil_name = row[df_CoolingCoils.columns.get_loc("Cooling coil name")]
            ventilation_system = self.system_dict[row[df_CoolingCoils.columns.get_loc("Ventilation system")]]
            cooling_system = self.system_dict[row[df_CoolingCoils.columns.get_loc("Cooling system")]]
            cooling_coil = CoilCoolingModel(
                specificHeatCapacityAir = Measurement(hasValue=1000),
                subSystemOf = [ventilation_system, cooling_system],
                input = {"supplyAirTemperatureSetpoint": 23},
                output = {},
                savedInput = {},
                savedOutput = {},
                createReport = self.createReport,
                connectedThrough = [],
                connectsAt = [],
                systemId = cooling_coil_name)
            self.component_dict[cooling_coil_name] = cooling_coil
            ventilation_system.hasSubSystem.append(cooling_coil)
            cooling_system.hasSubSystem.append(cooling_coil)

        for row in df_AirToAirHeatRecovery.dropna(subset=["Air to air heat recovery name"]).itertuples(index=False):
            air_to_air_heat_recovery_name = row[df_AirToAirHeatRecovery.columns.get_loc("Air to air heat recovery name")]
            ventilation_system = self.system_dict[row[df_AirToAirHeatRecovery.columns.get_loc("Ventilation system")]]
            air_to_air_heat_recovery = air_to_air_heat_recovery = AirToAirHeatRecoveryModel(
                specificHeatCapacityAir = Measurement(hasValue=1000),
                eps_75_h = Measurement(hasValue=0.8),
                eps_75_c = Measurement(hasValue=0.8),
                eps_100_h = Measurement(hasValue=0.8),
                eps_100_c = Measurement(hasValue=0.8),
                primaryAirFlowRateMax = Measurement(hasValue=row[df_AirToAirHeatRecovery.columns.get_loc("primaryAirFlowRateMax")]),
                secondaryAirFlowRateMax = Measurement(hasValue=row[df_AirToAirHeatRecovery.columns.get_loc("secondaryAirFlowRateMax")]),
                subSystemOf = [ventilation_system],
                input = {},
                output = {},
                savedInput = {},
                savedOutput = {},
                createReport = self.createReport,
                connectedThrough = [],
                connectsAt = [],
                systemId = air_to_air_heat_recovery_name)
            self.component_dict[air_to_air_heat_recovery_name] = air_to_air_heat_recovery
            ventilation_system.hasSubSystem.append(air_to_air_heat_recovery)

        for row in df_Fans.dropna(subset=["Fan name"]).itertuples(index=False):
            fan_name = row[df_Fans.columns.get_loc("Fan name")]
            ventilation_system = self.system_dict[row[df_Fans.columns.get_loc("Ventilation system")]]
            fan = FanModel(
                c1=Measurement(hasValue=0),
                c2=Measurement(hasValue=0),
                c3=Measurement(hasValue=0),
                c4=Measurement(hasValue=1),
                nominalAirFlowRate = Measurement(hasValue=row[df_Fans.columns.get_loc("nominalAirFlowRate")]),
                nominalPowerRate = Measurement(hasValue=row[df_Fans.columns.get_loc("nominalPowerRate")]),
                subSystemOf = [ventilation_system],
                input = {},
                output = {},
                savedInput = {},
                savedOutput = {},
                createReport = self.createReport,
                connectedThrough = [],
                connectsAt = [],
                systemId = fan_name)
            self.component_dict[fan_name] = fan
            ventilation_system.hasSubSystem.append(fan)

        for row in df_Nodes.dropna(subset=["Node name"]).itertuples(index=False):
            node_name = row[df_Nodes.columns.get_loc("Node name")]
            ventilation_system = self.system_dict[row[df_Nodes.columns.get_loc("Ventilation system")]]
            node = Node(
                subSystemOf = [ventilation_system],
                input = {},
                output = {},
                savedInput = {},
                savedOutput = {},
                createReport = self.createReport,
                connectedThrough = [],
                connectsAt = [],
                systemId = node_name)
            self.component_dict[node_name] = node
            ventilation_system.hasSubSystem.append(node)

        for row in df_Controller.dropna(subset=["Controller name"]).itertuples(index=False):
            controller_name = row[df_Controller.columns.get_loc("Controller name")]
            if controller_name[4:] not in self.component_dict:
                warnings.warn("Cannot find a matching mathing BuildingSpace object for controller \"" + controller_name + "\"")
            else:
                controller = ControllerModel(
                    K_p = row[df_Controller.columns.get_loc("K_p")],
                    K_i = row[df_Controller.columns.get_loc("K_i")],
                    K_d = row[df_Controller.columns.get_loc("K_d")],
                    subSystemOf = [],
                    input = {},
                    output = {"inputSignal": 0},
                    savedInput = {},
                    savedOutput = {},
                    createReport = self.createReport,
                    connectedThrough = [],
                    connectsAt = [],
                    systemId = controller_name)
                self.component_dict[controller_name] = controller
            # ventilation_system.hasSubSystem.append(controller)

        

    def get_models_by_instance(self, type_):
        return [v for v in self.component_dict.values() if isinstance(v, type_)]

    def get_model_by_name(self, name):
        return self.component_dict[name]

    def connect(self):
        space_instances = self.get_models_by_instance(BuildingSpaceModel)
        damper_instances = self.get_models_by_instance(DamperModel)
        space_heater_instances = self.get_models_by_instance(SpaceHeaterModel)
        valve_instances = self.get_models_by_instance(ValveModel)
        coil_heating_instances = self.get_models_by_instance(CoilHeatingModel)
        coil_cooling_instances = self.get_models_by_instance(CoilCoolingModel)
        air_to_air_heat_recovery_instances = self.get_models_by_instance(AirToAirHeatRecoveryModel)
        fan_instances = self.get_models_by_instance(FanModel)
        node_instances = self.get_models_by_instance(Node)
        controller_instances = self.get_models_by_instance(ControllerModel)


        weather_station = self.component_dict["weather_station"]
        occupancy_schedule = self.component_dict["occupancy_schedule"]
        temperature_setpoint_schedule = self.component_dict["temperature_setpoint_schedule"]
        co2_setpoint_schedule = self.component_dict["co2_setpoint_schedule"]


        for space in space_instances:
            if "C_T_" + space.systemId in self.component_dict:
                temperature_controller = self.component_dict["C_T_" + space.systemId]
                self.add_connection(space, temperature_controller, "indoorTemperature", "actualValue") ###
                self.add_connection(temperature_controller, space, "inputSignal", "valvePosition")

            if "C_C_" + space.systemId in self.component_dict:
                co2_controller = self.component_dict["C_C_" + space.systemId]
                self.add_connection(space, co2_controller, "indoorCo2Concentration", "actualValue") ###
                self.add_connection(co2_controller, space, "inputSignal", "supplyDamperPosition") ###
                self.add_connection(co2_controller, space, "inputSignal", "returnDamperPosition")

            if "D_S_" + space.systemId in self.component_dict:
                damper = self.component_dict["D_S_" + space.systemId]
                self.add_connection(damper, space, "airFlowRate", "supplyAirFlowRate")
                ventilation_system = damper.subSystemOf[0]
                node = [v for v in ventilation_system.hasSubSystem if isinstance(v, Node) and v.systemId[0:4] == "N_S_"][0]
                self.add_connection(damper, node, "airFlowRate", "flowRate_" + space.systemId) ###
                self.add_connection(space, node, "indoorTemperature", "flowTemperatureIn_" + space.systemId) ###
                
            if "D_E_" + space.systemId in self.component_dict:
                damper = self.component_dict["D_E_" + space.systemId]
                self.add_connection(damper, space, "airFlowRate", "returnAirFlowRate")
                ventilation_system = damper.subSystemOf[0]
                node = [v for v in ventilation_system.hasSubSystem if isinstance(v, Node) and v.systemId[0:4] == "N_E_"][0]
                self.add_connection(damper, node, "airFlowRate", "flowRate_" + space.systemId) ###
                self.add_connection(space, node, "indoorTemperature", "flowTemperatureIn_" + space.systemId) ###

            self.add_connection(weather_station, space, "shortwaveRadiation", "shortwaveRadiation")
            self.add_connection(weather_station, space, "longwaveRadiation", "longwaveRadiation")
            self.add_connection(weather_station, space, "outdoorTemperature", "outdoorTemperature")
            self.add_connection(occupancy_schedule, space, "scheduleValue", "numberOfPeople")

        for damper in damper_instances:
            if "C_C_" + damper.systemId[4:] in self.component_dict:
                co2_controller = self.component_dict["C_C_" + damper.systemId[4:]]
                self.add_connection(co2_controller, damper, "inputSignal", "damperPosition")

        for space_heater in space_heater_instances:
            space = self.component_dict[space_heater.systemId[3:]]
            valve = self.component_dict["V_" + space_heater.systemId[3:]]
            self.add_connection(space, space_heater, "indoorTemperature", "indoorTemperature") 
            self.add_connection(valve, space_heater, "waterFlowRate", "waterFlowRate")

        for valve in valve_instances:
            if "C_T_" + valve.systemId[2:] in self.component_dict:
                temperature_controller = self.component_dict["C_T_" + valve.systemId[2:]]
                self.add_connection(temperature_controller, valve, "inputSignal", "valvePosition")

        for coil_heating in coil_heating_instances:
            for system in coil_heating.subSystemOf:
                air_to_air_heat_recovery = [v for v in system.hasSubSystem if isinstance(v, AirToAirHeatRecoveryModel)]
                if len(air_to_air_heat_recovery)!=0:
                    air_to_air_heat_recovery = air_to_air_heat_recovery[0]
                    node = [v for v in system.hasSubSystem if isinstance(v, Node) and v.systemId[0:4] == "N_S_"][0]
                    self.add_connection(air_to_air_heat_recovery, coil_heating, "primaryTemperatureOut", "supplyAirTemperature")
                    self.add_connection(node, coil_heating, "flowRate", "airFlowRate")

        for coil_cooling in coil_cooling_instances:
            for system in coil_cooling.subSystemOf:
                air_to_air_heat_recovery = [v for v in system.hasSubSystem if isinstance(v, AirToAirHeatRecoveryModel)]
                if len(air_to_air_heat_recovery)!=0:
                    air_to_air_heat_recovery = air_to_air_heat_recovery[0]
                    node = [v for v in system.hasSubSystem if isinstance(v, Node) and v.systemId[0:4] == "N_S_"][0]
                    self.add_connection(air_to_air_heat_recovery, coil_cooling, "primaryTemperatureOut", "supplyAirTemperature")
                    self.add_connection(node, coil_cooling, "flowRate", "airFlowRate")

        for air_to_air_heat_recovery in air_to_air_heat_recovery_instances:
            ventilation_system = air_to_air_heat_recovery.subSystemOf[0]
            node_S = [v for v in ventilation_system.hasSubSystem if isinstance(v, Node) and v.systemId[0:4] == "N_S_"][0]
            node_E = [v for v in ventilation_system.hasSubSystem if isinstance(v, Node) and v.systemId[0:4] == "N_E_"][0]
            self.add_connection(weather_station, air_to_air_heat_recovery, "outdoorTemperature", "primaryTemperatureIn")
            self.add_connection(node_E, air_to_air_heat_recovery, "flowTemperatureOut", "secondaryTemperatureIn")
            self.add_connection(node_S, air_to_air_heat_recovery, "flowRate", "primaryAirFlowRate")
            self.add_connection(node_E, air_to_air_heat_recovery, "flowRate", "secondaryAirFlowRate")



        for fan in fan_instances:
            ventilation_system = fan.subSystemOf[0]
            if fan.systemId[0:4] == "F_S_":
                node_S = [v for v in ventilation_system.hasSubSystem if isinstance(v, Node) and v.systemId[0:4] == "N_S_"][0]
                self.add_connection(node_S, fan, "flowRate", "airFlowRate")
            elif fan.systemId[0:4] == "F_E_":
                node_E = [v for v in ventilation_system.hasSubSystem if isinstance(v, Node) and v.systemId[0:4] == "N_E_"][0]
                self.add_connection(node_E, fan, "flowRate", "airFlowRate")

        for controller in controller_instances:
            if controller.systemId[0:4] == "C_T_":
                self.add_connection(temperature_setpoint_schedule, controller, "scheduleValue", "setpointValue")
            elif controller.systemId[0:4] == "C_C_":
                self.add_connection(co2_setpoint_schedule, controller, "scheduleValue", "setpointValue")


        self.initComponents = [v for v in self.component_dict.values() if len(v.connectsAt)==0 or isinstance(v, BuildingSpaceModel)]
        

    def load_model_new(self, read_config=True):
        self.add_weather_station()
        self.add_occupancy_schedule()
        self.add_temperature_setpoint_schedule()
        self.add_co2_setpoint_schedule()

        if read_config:
            self.read_config()
            self.connect()

        print("Finished loading model")

    
    def load_model(self):

        hvac_system = DistributionDevice(subSystemOf = [], hasSubSystem = [])
        heating_system = DistributionDevice(subSystemOf = [hvac_system], hasSubSystem = [])
        ventilation_system = DistributionDevice(subSystemOf = [hvac_system], hasSubSystem = [])
        cooling_system = DistributionDevice(subSystemOf = [hvac_system], hasSubSystem = [])

        ventilation_system_dict = {"VentilationSystem1": DistributionDevice(subSystemOf = [hvac_system])}
        heating_system_dict = {"HeatingSystem1": DistributionDevice(subSystemOf = [hvac_system])}
        cooling_system_dict = {"CoolingSystem1": DistributionDevice(subSystemOf = [hvac_system])}

        

        weather_station = WeatherStation(startPeriod = self.startPeriod,
                                                        endPeriod = self.endPeriod,
                                                        input = {},
                                                        output = {},
                                                        savedInput = {},
                                                        savedOutput = {},
                                                        createReport = self.createReport,
                                                        connectedThrough = [],
                                                        connectsAt = [])

        occupancy_schedule = Schedule(startPeriod = self.startPeriod,
                                                timeStep = self.timeStep,
                                                rulesetDict = {
                                                    "ruleset_default_value": 0,
                                                    "ruleset_start_minute": [0,0,0,0,0],
                                                    "ruleset_end_minute": [0,0,0,0,0],
                                                    "ruleset_start_hour": [0,5,8,12,18],
                                                    "ruleset_end_hour": [6,8,12,18,22],
                                                    "ruleset_value": [0,10,30,30,0]}, #35
                                                input = {},
                                                output = {},
                                                savedInput = {},
                                                savedOutput = {},
                                                createReport = False,
                                                connectedThrough = [],
                                                connectsAt = [])

        temperature_setpoint_schedule = Schedule(startPeriod = self.startPeriod,
                                                timeStep = self.timeStep,
                                                rulesetDict = {
                                                    "ruleset_default_value": 20,
                                                    "ruleset_start_minute": [0,0],
                                                    "ruleset_end_minute": [0,0],
                                                    "ruleset_start_hour": [0,6],
                                                    "ruleset_end_hour": [6,18],
                                                    "ruleset_value": [20,24]},
                                                input = {},
                                                output = {},
                                                savedInput = {},
                                                savedOutput = {},
                                                createReport = False,
                                                connectedThrough = [],
                                                connectsAt = [])

        air_to_air_heat_recovery = AirToAirHeatRecoveryModel(
            specificHeatCapacityAir = Measurement(hasValue=1000),
            eps_75_h = Measurement(hasValue=0.8),
            eps_75_c = Measurement(hasValue=0.8),
            eps_100_h = Measurement(hasValue=0.8),
            eps_100_c = Measurement(hasValue=0.8),
            primaryAirFlowRateMax = Measurement(hasValue=1),
            secondaryAirFlowRateMax = Measurement(hasValue=1),
            subSystemOf = [ventilation_system],
            input = {},
            output = {},
            savedInput = {},
            savedOutput = {},
            createReport = self.createReport,
            connectedThrough = [],
            connectsAt = [])
        ventilation_system.hasSubSystem.append(air_to_air_heat_recovery)

        heating_coil = CoilHeatingModel(
            specificHeatCapacityAir = Measurement(hasValue=1000),
            subSystemOf = [heating_system, ventilation_system],
            input = {"supplyAirTemperatureSetpoint": 23},
            output = {},
            savedInput = {},
            savedOutput = {},
            createReport = self.createReport,
            connectedThrough = [],
            connectsAt = [])
        heating_system.hasSubSystem.append(heating_coil)
        ventilation_system.hasSubSystem.append(heating_coil)

        cooling_coil = CoilCoolingModel(
                                        specificHeatCapacityAir = Measurement(hasValue=1000),
                                        subSystemOf = [cooling_system, ventilation_system],
                                        input = {"supplyAirTemperatureSetpoint": 23},
                                        output = {},
                                        savedInput = {},
                                        savedOutput = {},
                                        createReport = self.createReport,
                                        connectedThrough = [],
                                        connectsAt = [])
        heating_system.hasSubSystem.append(cooling_coil)
        ventilation_system.hasSubSystem.append(cooling_coil)

        supply_fan = FanModel(
            c1=Measurement(hasValue=0),
            c2=Measurement(hasValue=0),
            c3=Measurement(hasValue=0),
            c4=Measurement(hasValue=1),
            isSupplyFan = True,
            nominalAirFlowRate = Measurement(hasValue=250/3600*1.225),
            nominalPowerRate = Measurement(hasValue=10000),
            subSystemOf = [ventilation_system],
            input = {},
            output = {},
            savedInput = {},
            savedOutput = {},
            createReport = self.createReport,
            connectedThrough = [],
            connectsAt = [])
        ventilation_system.hasSubSystem.append(supply_fan)

        return_fan = FanModel(
            c1=Measurement(hasValue=0),
            c2=Measurement(hasValue=0),
            c3=Measurement(hasValue=0),
            c4=Measurement(hasValue=1),
            nominalAirFlowRate = Measurement(hasValue=250/3600*1.225),
            nominalPowerRate = Measurement(hasValue=10000),
            subSystemOf = [ventilation_system],
            input = {},
            output = {},
            savedInput = {},
            savedOutput = {},
            createReport = self.createReport,
            connectedThrough = [],
            connectsAt = [])
        ventilation_system.hasSubSystem.append(return_fan)

        supply_flowmeter = FlowMeterModel(
            isSupplyFlowMeter = True, 
            subSystemOf = [ventilation_system],
            input = {},
            output = {},
            connectedThrough = [],
            connectsAt = [])
        ventilation_system.hasSubSystem.append(supply_flowmeter)

        return_flowmeter = FlowMeterModel(
            isReturnFlowMeter = True, 
            subSystemOf = [ventilation_system],
            input = {},
            output = {},
            savedInput = {},
            savedOutput = {},
            connectedThrough = [],
            connectsAt = [])
        ventilation_system.hasSubSystem.append(return_flowmeter)


        self.initComponents.append(temperature_setpoint_schedule)
        self.initComponents.append(weather_station)
        self.initComponents.append(occupancy_schedule)
        for i in range(1):
            space_heater = SpaceHeaterModel(
                outputCapacity = Measurement(hasValue=1000),
                thermalMassHeatCapacity = Measurement(hasValue=30000),
                specificHeatCapacityWater = Measurement(hasValue=4180),
                timeStep = self.timeStep, 
                subSystemOf = [heating_system],
                input = {"supplyWaterTemperature": 60},
                output = {"radiatorOutletTemperature": 22,
                            "Energy": 0},
                savedInput = {},
                savedOutput = {},
                createReport = self.createReport,
                connectedThrough = [],
                connectsAt = [])
            heating_system.hasSubSystem.append(space_heater)

            valve = ValveModel(
                waterFlowRateMax=Measurement(hasValue=0.1),
                valveAuthority = Measurement(hasValue=0.8),
                subSystemOf = [heating_system],
                input = {},
                output = {},
                savedInput = {},
                savedOutput = {},
                createReport = self.createReport,
                connectedThrough = [],
                connectsAt = [])
            heating_system.hasSubSystem.append(valve)

            temperature_controller = ControllerModel(
                k_p = 1,
                k_i = 0,
                k_d = 0,
                subSystemOf = [heating_system],
                input = {},
                output = {"inputSignal": 0},
                savedInput = {},
                savedOutput = {},
                createReport = self.createReport,
                connectedThrough = [],
                connectsAt = [])
            heating_system.hasSubSystem.append(temperature_controller)

            co2_controller = ControllerModel(
                k_p = 0.01, #0.01
                k_i = 0,
                k_d = 0,
                subSystemOf = [ventilation_system],
                input = {"actualValue": 600,
                            "setpointValue": 600},
                output = {"inputSignal": 0},
                savedInput = {},
                savedOutput = {},
                connectedThrough = [],
                connectsAt = [])
            ventilation_system.hasSubSystem.append(co2_controller)

            supply_damper = DamperModel(isSupplyDamper = True,
                nominalAirFlowRate = Measurement(hasValue=250/3600*1.225),
                subSystemOf = [ventilation_system],
                input = {},
                output = {},
                savedInput = {},
                savedOutput = {},
                createReport = self.createReport,
                connectedThrough = [],
                connectsAt = [])
            ventilation_system.hasSubSystem.append(supply_damper)
            supply_damper.output["airFlowRate"] = 0 ########################

            return_damper = DamperModel(isReturnDamper = True,
                nominalAirFlowRate = Measurement(hasValue=250/3600*1.225),
                subSystemOf = [ventilation_system],
                input = {},
                output = {},
                savedInput = {},
                savedOutput = {},
                connectedThrough = [],
                connectsAt = [])
            ventilation_system.hasSubSystem.append(return_damper)
            return_damper.output["airFlowRate"] = 0 ########################

            space = BuildingSpaceModel(densityAir = 1.225,
                airVolume = 50,
                startPeriod = self.startPeriod,
                timeStep = self.timeStep,
                input = {"generationCo2Concentration": 0.06,
                        "outdoorCo2Concentration": 500,
                        "shadesPosition": 0},
                output = {"indoorTemperature": 21.5,
                        "indoorCo2Concentration": 500},
                savedInput = {},
                savedOutput = {},
                createReport = self.createReport,
                connectedThrough = [],
                connectsAt = [])



            
            self.add_connection(temperature_setpoint_schedule, temperature_controller, "scheduleValue", "setpointValue")
            self.add_connection(space, temperature_controller, "indoorTemperature", "actualValue")
            self.add_connection(space, co2_controller, "indoorCo2Concentration", "actualValue")

            self.add_connection(supply_damper, supply_flowmeter, "airFlowRate", "airFlowRate" + str(supply_damper.systemId))
            self.add_connection(return_damper, return_flowmeter, "airFlowRate", "airFlowRate" + str(return_damper.systemId))
        
            self.add_connection(space, space_heater, "indoorTemperature", "indoorTemperature")
            self.add_connection(valve, space_heater, "waterFlowRate", "waterFlowRate")

            self.add_connection(temperature_controller, valve, "inputSignal", "valvePosition")
            self.add_connection(temperature_controller, space, "inputSignal", "valvePosition")

            self.add_connection(co2_controller, space, "inputSignal", "supplyDamperPosition")
            self.add_connection(co2_controller, space, "inputSignal", "returnDamperPosition")

            self.add_connection(supply_damper, space, "airFlowRate", "supplyAirFlowRate")
            self.add_connection(return_damper, space, "airFlowRate", "returnAirFlowRate")
            self.add_connection(occupancy_schedule, space, "scheduleValue", "numberOfPeople")

            self.add_connection(weather_station, space, "shortwaveRadiation", "shortwaveRadiation")
            self.add_connection(weather_station, space, "longwaveRadiation", "longwaveRadiation")
            self.add_connection(weather_station, space, "outdoorTemperature", "outdoorTemperature")


            self.add_connection(co2_controller, supply_damper, "inputSignal", "damperPosition")
            self.add_connection(co2_controller, return_damper, "inputSignal", "damperPosition")

            self.initComponents.append(space)

        self.add_connection(weather_station, air_to_air_heat_recovery, "outdoorTemperature", "outdoorTemperature")

        self.add_connection(space, air_to_air_heat_recovery, "indoorTemperature", "indoorTemperature") #########################
        self.add_connection(supply_flowmeter, air_to_air_heat_recovery, "airFlowRate", "supplyAirFlowRate")
        self.add_connection(return_flowmeter, air_to_air_heat_recovery, "airFlowRate", "returnAirFlowRate")
        
        self.add_connection(air_to_air_heat_recovery, heating_coil, "supplyAirTemperature", "supplyAirTemperature")
        self.add_connection(air_to_air_heat_recovery, cooling_coil, "supplyAirTemperature", "supplyAirTemperature")

        self.add_connection(supply_flowmeter, heating_coil, "airFlowRate", "airFlowRate")
        self.add_connection(supply_flowmeter, cooling_coil, "airFlowRate", "airFlowRate")

        self.add_connection(supply_flowmeter, supply_fan, "airFlowRate", "airFlowRate")
        self.add_connection(return_flowmeter, return_fan, "airFlowRate", "airFlowRate")


        import jsonpickle
        import json
        # jsonpickle.set_decoder_options('json', indent=4)
        encoded = json.dumps(json.loads(jsonpickle.encode(supply_flowmeter, make_refs=False)), indent=4)
        decoded = jsonpickle.decode(encoded)
        # Writing to sample.json
        with open("test.json", "w") as outfile:
            outfile.write(encoded)

        
        self.activeComponents = self.initComponents

    def show_system_graph(self):
        min_fontsize = 14
        max_fontsize = 18

        min_width = 1.2
        max_width = 3

        degree_list = [self.system_graph.degree(node) for node in self.system_graph.nodes]
        min_deg = min(degree_list)
        max_deg = max(degree_list)

        a_fontsize = (max_fontsize-min_fontsize)/(max_deg-min_deg)
        b_fontsize = max_fontsize-a_fontsize*max_deg

        a_width = (max_width-min_width)/(max_deg-min_deg)
        b_width = max_width-a_width*max_deg
        for node in self.system_graph.nodes:
            deg = self.system_graph.degree(node)
            fontsize = a_fontsize*deg + b_fontsize
            width = a_width*deg + b_width
            

            if node not in self.system_graph_node_attribute_dict:
                self.system_graph_node_attribute_dict[node] = {"fontsize": fontsize, "width": width}
            else:
                self.system_graph_node_attribute_dict[node]["fontsize"] = fontsize
                self.system_graph_node_attribute_dict[node]["width"] = width


        #get all node names
        # name_list = []
        # for node in self.system_graph.nodes:
        #     name_list.append(node.name)
        # name_list = list(set(name_list))

        


        nx.set_node_attributes(self.system_graph, values=self.system_graph_node_attribute_dict)

        # print(self.system_graph.nodes)
        # print(self.system_graph.nodes[9]["labels"])

        #####################################################
        # pos = nx.kamada_kawai_layout(self.system_graph)   
        # nx.draw_networkx_nodes(self.system_graph, pos=pos)
        # nx.draw_networkx_labels(self.system_graph, pos, labels=self.system_graph_node_label_dict)

        # nx.draw_networkx_edge_labels(self.system_graph, pos)

        # for edge in self.system_graph.edges(data=True):
        #     nx.draw_networkx_edges(self.system_graph, pos, edgelist=[(edge[0],edge[1])], connectionstyle=f'arc3, rad = {edge[2]["rad"]}')
        ###################################

        graph = nx.drawing.nx_pydot.to_pydot(self.system_graph)

        

        # graph.set_graph_defaults(pack="true", rankdir="TB", bgcolor="transparent", fontname="Helvetica", fontcolor="blue", fontsize=10, dpi=500, splines="ortho")
        graph.set_node_defaults(shape="circle", width=0.8, fixedsize="shape", margin=0, style="filled", fontname="Helvetica", color="#23a6db66", fontsize=10, colorscheme="oranges9")
        graph.set_edge_defaults(fontname="Helvetica", penwidth=2, color="#999999", fontcolor="#999999", fontsize=10, weight=3, minlen=1)

        self.system_graph = nx.drawing.nx_pydot.from_pydot(graph)

        nx.drawing.nx_pydot.write_dot(self.system_graph, 'system_graph.dot')
        # graph = nx.drawing.nx_pydot.to_pydot(self.system_graph)
        cmd_string = "\"C:/Program Files/Graphviz/bin/dot.exe\" -Tpng -Kdot -Grankdir=LR -o system_graph.png system_graph.dot"
        os.system(cmd_string)

        
        # graph = nx.from_edgelist(self.system_graph_edge_label_dict, nx.DiGraph())
        # Graph(graph, node_layout='dot', node_shape = "o", node_size=6, ax=ax,
        #         edge_width=1, edge_label_rotate=True, edge_layout="curved", arrows=True,
        #         node_labels=self.system_graph_node_label_dict, node_label_fontdict=dict(size=14),
        #         edge_labels=self.system_graph_edge_label_dict, edge_label_fontdict=dict(size=8), 
        #     )

# dot ###
# random
# circular
# spring ###
# community
# bipartite
        # plt.show()


    def show_execution_graph(self):
        self.execution_graph = nx.MultiDiGraph() ###
        self.execution_graph_node_attribute_dict = {}


        n = len(self.component_order)
        for i in range(n-1):
            sender_component = self.component_order[i]
            reciever_component = self.component_order[i+1]
            self.execution_graph.add_edge(sender_component.systemId, reciever_component.systemId) 

            self.execution_graph_node_attribute_dict[sender_component.systemId] = {"label": sender_component.__class__.__name__}
            self.execution_graph_node_attribute_dict[reciever_component.systemId] = {"label": reciever_component.__class__.__name__}



        min_fontsize = 14
        max_fontsize = 18

        min_width = 1.2
        max_width = 3

        degree_list = [self.execution_graph.degree(node) for node in self.execution_graph.nodes]
        min_deg = min(degree_list)
        max_deg = max(degree_list)

        a_fontsize = (max_fontsize-min_fontsize)/(max_deg-min_deg)
        b_fontsize = max_fontsize-a_fontsize*max_deg

        a_width = (max_width-min_width)/(max_deg-min_deg)
        b_width = max_width-a_width*max_deg
        for node in self.execution_graph.nodes:
            deg = self.execution_graph.degree(node)
            fontsize = a_fontsize*deg + b_fontsize
            width = a_width*deg + b_width
            

            if node not in self.execution_graph_node_attribute_dict:
                self.execution_graph_node_attribute_dict[node] = {"fontsize": fontsize, "width": width}
            else:
                self.execution_graph_node_attribute_dict[node]["fontsize"] = fontsize
                self.execution_graph_node_attribute_dict[node]["width"] = width



        nx.set_node_attributes(self.execution_graph, values=self.execution_graph_node_attribute_dict)

        graph = nx.drawing.nx_pydot.to_pydot(self.execution_graph)
        graph.set_node_defaults(shape="circle", width=0.8, fixedsize="shape", margin=0, style="filled", fontname="Helvetica", color="#23a6db66", fontsize=10, colorscheme="oranges9")
        graph.set_edge_defaults(fontname="Helvetica", penwidth=2, color="#999999", fontcolor="#999999", fontsize=10, weight=3, minlen=1)

        self.execution_graph = nx.drawing.nx_pydot.from_pydot(graph)

        nx.drawing.nx_pydot.write_dot(self.execution_graph, 'execution_graph.dot')
        cmd_string = "\"C:/Program Files/Graphviz/bin/dot.exe\" -Tpng -Kdot -Grankdir=LR -o execution_graph.png execution_graph.dot"
        os.system(cmd_string)

        

    def simulate(self):
        

        time = self.startPeriod
        time_list = []
        while time < self.endPeriod:
            for component in self.component_order:
                # print("----")
                # print(component.__class__.__name__)
                #Gather all needed inputs for the component through all ingoing connections
                for connection_point in component.connectsAt:
                    connection = connection_point.connectsSystemThrough
                    connected_component = connection.connectsSystem
                    # print("--------------------------------")
                    # print("------")
                    # print(component.__class__.__name__)
                    # print(connected_component.__class__.__name__)
                    # print("------")
                    # print(connection.senderPropertyName)
                    # print(connection_point.recieverPropertyName)
                    # print("------")
                    # print(component.input)
                    # print(connected_component.output)
                    component.input[connection_point.recieverPropertyName] = connected_component.output[connection.senderPropertyName]

                component.update_output()
                component.update_report()

            time_list.append(time)
            time += datetime.timedelta(seconds=self.timeStep)
            print(time)
            


        print("-------")
        for component in self.component_order:
            if component.createReport:
                component.plot_report(time_list)
        plt.show()


        
    def find_path(self):
        self.activeComponents = self.initComponents
        self.visitCount = {}
        self.component_order = []
        self.component_order.extend(self.initComponents)
        while len(self.activeComponents)>0:
            print("YYYYYYYYYYYY")
            self.traverse()
        
        # print(System.id_iter)
        # print(len(self.component_order))
        # aa
        assert len(self.component_order)==len(self.component_dict)


    def traverse(self):
        activeComponentsNew = []
        for component in self.activeComponents:
            for connection in component.connectedThrough:
                connection_point = connection.connectsSystemAt
                connected_component = connection_point.connectionPointOf
                if connected_component.connectionVisits is None:
                    connected_component.connectionVisits = [connection_point.recieverPropertyName]
                else:
                    connected_component.connectionVisits.append(connection_point.recieverPropertyName)

                has_connections = True
                for ingoing_connection_point in connected_component.connectsAt:
                    if ingoing_connection_point.recieverPropertyName not in connected_component.connectionVisits or isinstance(connected_component, BuildingSpace):
                        has_connections = False
                        break
                
                if has_connections:
                    self.component_order.append(connected_component)

                    if connected_component.connectedThrough is not None:
                        activeComponentsNew.append(connected_component)

                # print("---")
                # print(component.__class__.__name__)
                # print(connected_component.__class__.__name__)
                # print(connection.connectionType)

                # print(connected_component.connectionVisits)

        

        self.activeComponents = activeComponentsNew



    def get_leaf_subsystems(self, system):
        for sub_system in system.hasSubSystem:
            if sub_system.hasSubSystem is None:
                self.leaf_subsystems.append(sub_system)
            else:
                self.get_leaf_subsystems(sub_system)

    # def get_system_count():



def run():
    createReport = False
    timeStep = 600
    startPeriod = datetime.datetime(year=2019, month=12, day=8, hour=0, minute=0, second=0, tzinfo=tzutc())
    endPeriod = datetime.datetime(year=2019, month=12, day=20, hour=0, minute=0, second=0, tzinfo=tzutc())
    model = BuildingEnergyModel(timeStep = timeStep,
                                startPeriod = startPeriod,
                                endPeriod = endPeriod,
                                createReport = createReport)

    model.load_model_new()
    model.find_path()
    model.show_execution_graph()
    model.show_system_graph()
    model.simulate()


import cProfile
import pstats
import io

pr = cProfile.Profile()
pr.enable()

my_result = run()

pr.disable()
s = io.StringIO()
ps = pstats.Stats(pr, stream=s).sort_stats('tottime')
ps.print_stats()

with open('test.txt', 'w+') as f:
    f.write(s.getvalue())

# run()