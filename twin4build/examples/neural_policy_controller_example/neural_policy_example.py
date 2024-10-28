#Only code version


import twin4build as tb
import datetime
import twin4build.examples.utils as utils
import torch.nn as nn
import torch
import json
from dateutil.tz import gettz 
import twin4build.utils.plot.plot as plot
import twin4build.utils.input_output_types as tps



def insert_neural_policy_in_fcn(self:tb.Model, input_output_dictionary, policy_path=None):
        """
        The input/output dictionary contains information on the input and output signals of the controller.
        These signals must match the component and signal keys to replace in the model
        The input dictionary will have items like this:
            "component_key": {
                "component_output_signal_key": {
                    "min": 0,
                    "max": 1,
                    "description": "Description of the signal"
                }
            }
        Whilst the output items will have a similar structure but for the output signals:
            "component_key": {
                "component_input_signal_key": {
                    "min": 0,
                    "max": 1,
                    "description": "Description of the signal"
                }
            }
        Note that the input signals must contain the key for the output compoenent signal and the output signals must contain the key for the input component signal

        This function instantiates the controller and adds it to the model.
        Then it goes through the input dictionary adding connection to the input signals
        Then it goes through the output dictionary finding the corresponding existing connections, deleting the existing connections and adding the new connections
        """
        try:
            utils.validate_schema(input_output_dictionary)
        except Exception as e:
            print("Validation error:", e)
            return

        #Create the controller
        input_size = len(input_output_dictionary["input"])
        output_size = len(input_output_dictionary["output"])

        policy = nn.Sequential(
            nn.Linear(input_size, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, output_size),
            nn.Sigmoid()
        )

        #Load the policy model
        if policy_path is not None:
            policy.load_state_dict(torch.load(policy_path))

        neural_policy_controller = tb.NeuralPolicyControllerSystem(
            input_size = input_size,
            output_size = output_size,
            input_output_schema = input_output_dictionary,
            policy_model = policy,
            saveSimulationResult = True,
            id = "neural_controller"
        )

        #Find and remove the existing output connections and components
        for output_component_key in input_output_dictionary["output"]:
            receiving_component = self.component_dict[output_component_key]
            found = False  
            for connection_point in receiving_component.connectsAt:
                if connection_point.receiverPropertyName == input_output_dictionary["output"][output_component_key]["signal_key"]:
                    #Remove the connection(s) to the receiving component
                    for incoming_connection in connection_point.connectsSystemThrough:
                        sender_component = incoming_connection.connectsSystem
                        #If the sender component is not connected to any other component, remove it
                        if len(sender_component.connectedThrough) == 1:
                            self.remove_component(sender_component)
                        else:
                            self.remove_connection(sender_component, receiving_component, incoming_connection.senderPropertyName, connection_point.receiverPropertyName)
                    found = True 
                    break
            if not found:
                print(f"Could not find connection for {output_component_key} and {input_output_dictionary['output'][output_component_key]['signal_key']}")

       
        #Add the input connections
        
        for component_key in input_output_dictionary["input"]:
            try:
                sender_component = self.component_dict[component_key]
            except KeyError:
                print(f"Could not find component {component_key}")
                continue
            receiving_component = neural_policy_controller
            self.add_connection(
                sender_component,
                receiving_component,
                input_output_dictionary["input"][component_key]["signal_key"],
                "actualValue"
            )


        
        # Define the output dictionary for the NeuralController using a dictionary comprehension
        neural_policy_controller.output = {
            f"{component_key}_input_signal": tps.Scalar()
            for component_key in input_output_dictionary["output"]
        }

        # Loop through the components and add connections
        for component_key, output_info in input_output_dictionary["output"].items():
            output_key = f"{component_key}_input_signal"
            receiver_component = self.component_dict.get(component_key)
            if receiver_component is None:
                print(f"Could not find component {component_key}")
                continue
            self.add_connection(
                neural_policy_controller,
                receiver_component,
                output_key,  # Use the unique output_key for each connection
                output_info["signal_key"]
            )
        # Define a custom initial dictionary for the NeuralController outputs:
        custom_initial = {"neural_controller": {f"{component_key}_input_signal": tps.Scalar(0) for component_key in input_output_dictionary["output"]}}
        self.set_custom_initial_dict(custom_initial)


def fcn(self):
    supply_water_schedule = tb.ScheduleSystem(
    weekDayRulesetDict = {
        "ruleset_default_value": 60,
        "ruleset_start_minute": [],
        "ruleset_end_minute": [],
        "ruleset_start_hour": [],
        "ruleset_end_hour": [],
        "ruleset_value": []
    },
    id="supply_water_schedule"
    )
    self.add_connection(supply_water_schedule, self.component_dict["[020B][020B_space_heater]"], "scheduleValue", "supplyWaterTemperature") # Add missing input
    self.component_dict["020B_temperature_sensor"].filename = utils.get_path(["parameter_estimation_example", "temperature_sensor.csv"])
    self.component_dict["020B_co2_sensor"].filename = utils.get_path(["parameter_estimation_example", "co2_sensor.csv"])
    self.component_dict["020B_valve_position_sensor"].filename = utils.get_path(["parameter_estimation_example", "valve_position_sensor.csv"])
    self.component_dict["020B_damper_position_sensor"].filename = utils.get_path(["parameter_estimation_example", "damper_position_sensor.csv"])
    self.component_dict["BTA004"].filename = utils.get_path(["parameter_estimation_example", "supply_air_temperature.csv"])
    self.component_dict["020B_co2_setpoint"].weekDayRulesetDict = {"ruleset_default_value": 900,
                                                                    "ruleset_start_minute": [],
                                                                    "ruleset_end_minute": [],
                                                                    "ruleset_start_hour": [],
                                                                    "ruleset_end_hour": [],
                                                                    "ruleset_value": []}
    self.component_dict["020B_temperature_heating_setpoint"].useFile = True
    self.component_dict["020B_temperature_heating_setpoint"].filename = utils.get_path(["parameter_estimation_example", "temperature_heating_setpoint.csv"])
    self.component_dict["outdoor_environment"].filename = utils.get_path(["parameter_estimation_example", "outdoor_environment.csv"])
    #Load the input/output dictionary from the file policy_input_output.json
    with open(utils.get_path(["neural_policy_controller_example", "policy_input_output.json"])) as f:
        input_output_dictionary = json.load(f)
    insert_neural_policy_in_fcn(self, input_output_dictionary)

if __name__ == "__main__":
    # Create a new model
    model = tb.Model(id="neural_policy_example")
    filename = utils.get_path(["parameter_estimation_example", "one_room_example_model.xlsm"])

    model.load(semantic_model_filename=filename, fcn=fcn, verbose=False)
    
    
    """
    #Load the input/output dictionary from the file policy_input_output.json
    with open(utils.get_path(["neural_policy_controller_example", "policy_input_output.json"])) as f:
        input_output_dictionary = json.load(f)

    model = insert_neural_policy(model, input_output_dictionary)

    
    #Visualize the model
    import matplotlib.pyplot as plt
    import os
    system_graph = os.path.join(model.graph_path, "system_graph.png")
    image = plt.imread(system_graph)
    plt.figure(figsize=(12,12))
    plt.imshow(image)
    plt.axis('off')
    plt.show()
    """

    #Run a simulation


    stepSize = 600  # Seconds
    startTime = datetime.datetime(year=2023, month=11, day=27, hour=0, minute=0, second=0,
                                    tzinfo=gettz("Europe/Copenhagen"))
    endTime = datetime.datetime(year=2023, month=12, day=7, hour=0, minute=0, second=0,
                                tzinfo=gettz("Europe/Copenhagen"))

    simulator = tb.Simulator()
    simulator.simulate(model, startTime=startTime, endTime=endTime, stepSize=stepSize)
    print("Simulation completed successfully!")

    #Plot the results

    plot.plot_space_temperature_fmu(model, simulator, space_id='[020B][020B_space_heater]',show=True)    