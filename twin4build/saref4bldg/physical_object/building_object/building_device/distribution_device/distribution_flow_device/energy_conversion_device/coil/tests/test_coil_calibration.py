import os
import sys
import datetime
from dateutil.tz import tzutc
import pandas as pd
import matplotlib.pyplot as plt
import json
import numpy as np
import seaborn as sns
###Only for testing before distributing package
if __name__ == '__main__':
    uppath = lambda _path,n: os.sep.join(_path.split(os.sep)[:-n])
    file_path = uppath(os.path.abspath(__file__), 9)
    sys.path.append(file_path)

from twin4build.utils.data_loaders.load_from_file import load_from_file
from twin4build.utils.preprocessing.data_collection import DataCollection
from twin4build.utils.preprocessing.data_preparation import sample_data
from twin4build.saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_flow_device.energy_conversion_device.coil.coil_FMUmodel import CoilModel
from twin4build.saref.measurement.measurement import Measurement
from twin4build.utils.constants import Constants
from twin4build.utils.preprocessing.get_measuring_device_from_df import get_measuring_device_from_df
from twin4build.utils.preprocessing.get_measuring_device_error import get_measuring_device_error
from twin4build.utils.plot.plot import get_fig_axes, load_params


def valve_model(u, waterFlowRateMax):
    valve_authority = 1
    u_norm = u/(u**2*(1-valve_authority)+valve_authority)**(0.5)
    m_w = u_norm*waterFlowRateMax
    return m_w


def test():
    colors = sns.color_palette("deep")
    blue = colors[0]
    orange = colors[1]
    green = colors[2]
    red = colors[3]
    purple = colors[4]
    brown = colors[5]
    pink = colors[6]
    grey = colors[7]
    beis = colors[8]
    sky_blue = colors[9]
    load_params()
    stepSize = 60
    coil = CoilModel(
                    airFlowRateMax=None,
                    airFlowRateMin=None,
                    nominalLatentCapacity=None,
                    nominalSensibleCapacity=Measurement(hasValue=96000),
                    nominalUa=None,
                    operationTemperatureMax=None,
                    operationTemperatureMin=None,
                    placementType=None,
                    operationMode=None,
                    saveSimulationResult = True,
                    id = "coil")

    # waterFlowRateMax = abs(space_heater.outputCapacity.hasValue/Constants.specificHeatCapacity["water"]/(space_heater.nominalSupplyTemperature-space_heater.nominalReturnTemperature))
    waterFlowRateMax = 2
    input = pd.DataFrame()

    # startPeriod = datetime.datetime(year=2022, month=2, day=3, hour=0, minute=0, second=0)
    # startPeriod = datetime.datetime(year=2022, month=1, day=1, hour=8, minute=0, second=0) 
    # endPeriod = datetime.datetime(year=2022, month=1, day=1, hour=16, minute=0, second=0)
    # endPeriod = datetime.datetime(year=2022, month=2, day=4, hour=0, minute=0, second=0)
    startPeriod = datetime.datetime(year=2022, month=2, day=1, hour=8, minute=0, second=0)
    endPeriod = datetime.datetime(year=2022, month=2, day=1, hour=21, minute=0, second=0)
    format = "%m/%d/%Y %I:%M:%S %p"


    response_filename = os.path.join(uppath(os.path.abspath(__file__), 10), "test", "data", "time_series_data", "OE20-601b-2_kafka_temperature.txt")
    data = [json.loads(line) for line in open(response_filename, 'rb')]
    data = data[1:] #remove header information
    data = np.array([row[0][0] for row in data])
    data = data[data[:, 0].argsort()]
    constructed_time_list,constructed_value_list,got_data = sample_data(data=data, stepSize=stepSize, start_time=startPeriod, end_time=endPeriod, dt_limit=1200)


    filename = os.path.join(os.path.abspath(uppath(os.path.abspath(__file__), 10)), "test", "data", "time_series_data", "VE02.csv")
    VE02 = load_from_file(filename=filename, stepSize=stepSize, start_time=startPeriod, end_time=endPeriod, format=format, dt_limit=9999)

    filename = os.path.join(os.path.abspath(uppath(os.path.abspath(__file__), 10)), "test", "data", "time_series_data", "VE02_airflowrate_supply_kg_s.csv")
    VE02_supply_air = load_from_file(filename=filename, stepSize=stepSize, start_time=startPeriod, end_time=endPeriod, format=format, dt_limit=9999)

    filename = os.path.join(os.path.abspath(uppath(os.path.abspath(__file__), 10)), "test", "data", "time_series_data", "VE02_FTF1.csv")
    VE02_FTF1 = load_from_file(filename=filename, stepSize=stepSize, start_time=startPeriod, end_time=endPeriod, format=format, dt_limit=9999)

    filename = os.path.join(os.path.abspath(uppath(os.path.abspath(__file__), 10)), "test", "data", "time_series_data", "VE02_FTT1.csv")
    VE02_FTT1 = load_from_file(filename=filename, stepSize=stepSize, start_time=startPeriod, end_time=endPeriod, format=format, dt_limit=9999)

    filename = os.path.join(os.path.abspath(uppath(os.path.abspath(__file__), 10)), "test", "data", "time_series_data", "VE02_FTG_MIDDEL.csv")
    VE02_FTG_MIDDEL = load_from_file(filename=filename, stepSize=stepSize, start_time=startPeriod, end_time=endPeriod, format=format, dt_limit=9999)

    filename = os.path.join(os.path.abspath(uppath(os.path.abspath(__file__), 10)), "test", "data", "time_series_data", "VE02_FTI1.csv")
    VE02_FTI1 = load_from_file(filename=filename, stepSize=stepSize, start_time=startPeriod, end_time=endPeriod, format=format, dt_limit=9999)

    filename = os.path.join(os.path.abspath(uppath(os.path.abspath(__file__), 10)), "test", "data", "time_series_data", "VA01_FTF1_SV.csv")
    VA01_FTF1_SV = load_from_file(filename=filename, stepSize=stepSize, start_time=startPeriod, end_time=endPeriod, format=format, dt_limit=9999)

    filename = os.path.join(os.path.abspath(uppath(os.path.abspath(__file__), 10)), "test", "data", "time_series_data", "VE02_coil.csv")
    VE02_coil = load_from_file(filename=filename, stepSize=stepSize, start_time=startPeriod, end_time=endPeriod, format=format, dt_limit=9999)

    x = VE02["MVV1_S"]
    x[x<1] = 0
    input.insert(0, "time", VE02["Time stamp"])
    input.insert(0, "airFlowRate", VE02_supply_air["primaryAirFlowRate"])
    input.insert(0, "waterFlowRate", valve_model(VE02["MVV1_S"]/100, waterFlowRateMax))
    input.insert(0, "inletWaterTemperature", VE02_FTF1["FTF1"])
    input.insert(0, "outletWaterTemperature", VE02_FTT1["FTT1"])
    input.insert(0, "inletAirTemperature", VE02_FTG_MIDDEL["FTG_MIDDEL"])
    input.insert(0, "outletAirTemperature", VE02_FTI1["FTI1"])

    # input.insert(0, "time", VE02["Time stamp"])
    # input.insert(0, "airFlowRate", VE02_supply_air["primaryAirFlowRate"])
    # input.insert(0, "waterFlowRate", VE02_coil["Max flow [m3/t]"]/3.6)
    # input.insert(0, "inletWaterTemperature", VE02_coil["Temperatur"])
    # input.insert(0, "outletWaterTemperature", VE02_coil["Returloeb"])
    # input.insert(0, "inletAirTemperature", VE02_FTG_MIDDEL["FTG_MIDDEL"])
    # input.insert(0, "outletAirTemperature", VE02_FTI1["FTI1"])

    input.replace([np.inf, -np.inf], np.nan, inplace=True)



    input.set_index("time").plot()
    plt.show()

    colors = sns.color_palette("deep")
    fig, ax = plt.subplots(4, sharex=True)
    ax[0].plot(input["inletAirTemperature"]-273.15, color="black", label="air in")
    ax[0].plot(input["outletAirTemperature"]-273.15, color=colors[0], label="air out")
    ax[0].plot((VE02["FTI_KALK_SV"]-32)*5/9, color=colors[0], label="air out SET")
    
    ax[1].plot(input["outletWaterTemperature"]-273.15, color=colors[0], label="water out")
    ax[1].plot(input["inletWaterTemperature"]-273.15, color="black", label="water in")
    
    ax[2].plot(input["outletAirTemperature"]-273.15, color=colors[0], label="water out")
    ax[2].plot(input["inletWaterTemperature"]-273.15, color="black", label="air out")
    # ax[2].plot(VE02_coil["Temperatur"]-273.15, color=colors[1], label="air out")#

    ax[3].plot(input["waterFlowRate"], color="black", label="waterflow")
    # ax[3].plot(VE02_coil["Max flow [m3/t]"]/3.6, color=colors[1], label="waterflow EnergyKey") #
    

    # for ax_i in ax[:-1]:
    #     ax_i.set_ylim([15,24])
    fig.legend()

    shift = int(1*3600/stepSize)
    fig, ax = plt.subplots()
    
    # ax.plot(VE02_coil["Temperatur"].shift(-shift), color=colors[0], label="EnergyKey water in") #
    # ax.plot(VE02_coil["Returloeb"].shift(-shift), color=colors[1], label="EnergyKey water out") #
    ax.plot(input["inletWaterTemperature"], color=colors[2], label="BMS water in")
    ax.plot(input["outletWaterTemperature"], color=colors[3], label="BMS water out")
    ax.plot(input["inletAirTemperature"], color=colors[4], label="BMS air in")
    ax.plot(input["outletAirTemperature"], color=colors[5], label="BMS air out")
    ax.plot(VA01_FTF1_SV["FTF1_SV"], color=colors[6], label="BMS water setpoint")
    # ax.set_xlim([11800,13200])
    fig.legend()

    air_delta = input["outletAirTemperature"]-input["inletAirTemperature"]
    air_energy = air_delta*1000*input["airFlowRate"]
    water_delta = input["inletWaterTemperature"]-input["outletWaterTemperature"]
    resulting_water_flow = air_energy/(water_delta*4180)
    resulting_water_flow[water_delta<0] = np.nan
    fig, ax = plt.subplots(3, sharex=True)
    ax[0].plot(water_delta, color=colors[1], label="water delta")
    ax[0].plot(air_delta, color=colors[2], label="air delta")
    ax[1].plot(air_energy, color=colors[3], label="air energy")
    ax[2].plot(resulting_water_flow, color=colors[4], label="resulting water flow")
    ax[2].hlines(y=waterFlowRateMax, xmin=11800, xmax=13200, color=colors[5], label="reference water flow")
    # ax.hlines(y=0.2, xmin=4, xmax=20, linewidth=2, color='r')
    # for axx in ax:
    #     axx.set_xlim([11800,13200])
    fig.legend()



    # plt.show()

    fig, ax = plt.subplots(6, sharex=True)
    input.set_index("time").plot(subplots=True, ax=ax)
    for i in range(0,5,1):
        ax[i].set_xlabel("")
        for tick in ax[i].xaxis.get_minor_ticks():
            tick.tick1line.set_visible(False)
            tick.tick2line.set_visible(False)
            tick.label1.set_visible(False)
            tick.label2.set_visible(False)
        for tick in ax[i].xaxis.get_major_ticks():
            tick.tick1line.set_visible(False)
            tick.tick2line.set_visible(False)
            tick.label1.set_visible(False)
            tick.label2.set_visible(False)
    for a in ax:
        a.legend(prop={'size': 14})

    facecolor = tuple(list(beis)+[0.5])
    edgecolor = tuple(list((0,0,0))+[0.5])

    for i, key in enumerate(input.set_index("time")):
        if i<4:
            df = pd.DataFrame(input["time"]).set_index("time")
            df["value"] = input[key].values
            measuring_device = get_measuring_device_from_df(df, "Sensor", "Temperature", "TemperatureUnit", key)
            df = get_measuring_device_error(measuring_device)
            ax[i].fill_between(df.index, y1=df["lower_value"], y2=df["upper_value"], facecolor=facecolor, edgecolor=edgecolor, label=f"error band")
        else:
            df = pd.DataFrame(input["time"]).set_index("time")
            df["value"] = input[key].values
            
            measuring_device = get_measuring_device_from_df(df, "Meter", "Flow", "FlowUnit", key)
            df = get_measuring_device_error(measuring_device)
            ax[i].fill_between(df.index, y1=df["lower_value"], y2=df["upper_value"], facecolor=facecolor, edgecolor=edgecolor, label=f"error band")

    plt.show()
    output = input["outletAirTemperature"].to_numpy()
    input.drop(columns=["outletAirTemperature"], inplace=True)
    input.drop(columns=["outletWaterTemperature"], inplace=True)
    # input = input.iloc[0:-6]
    # output = output[6:]
    input = input.set_index("time")

    coil.initialize()
    measuring_device_types = ["Temperature", "Temperature", "Flow", "Flow"]
    start_pred = coil.do_period(input, stepSize=stepSize, measuring_device_types=measuring_device_types) ####

    res = np.array(coil.output_range)
    
    fig, ax = plt.subplots()
    ax.plot(input.index, start_pred, color="black", linestyle="dashed", label="predicted")
    ax.plot(input.index, output, color=colors[0], label="Measured")
    ax.fill_between(input.index, y1=start_pred-res[:,0], y2=start_pred+res[:,0], facecolor=facecolor, edgecolor=edgecolor, label=f"error band")
    ax.set_title('Using mapped nominal conditions')
    ax.set_xlabel("Timestep (10 min)")
    ax.set_ylabel("outletAirTemperature [C]")
    ax.legend(loc="upper left")
    plt.show()

    fig, ax = plt.subplots(2)
    ax[0].plot(start_pred, color="black", linestyle="dashed", label="predicted")
    ax[0].plot(output, color="blue", label="Measured")
    ax[0].set_title('Using mapped nominal conditions')
    fig.legend()
    # plt.show()
    # input = input.set_index("time")
    # input.plot(subplots=True)
    coil.calibrate(input=input, output=output, stepSize=stepSize)

    end_pred = coil.do_period(input, stepSize=stepSize)
    ax[1].plot(end_pred, color="black", linestyle="dashed", label="predicted")
    ax[1].plot(output, color="blue", label="Measured")
    ax[1].set_title('After calibration')


    box = ax[0].get_position()
    ax[0].set_position([box.x0, box.y0, box.width, box.height-0.05])

    box = ax[1].get_position()
    ax[1].set_position([box.x0, box.y0, box.width, box.height-0.05])

    fig.text(0.5, 0.01, "Timestep (10 min)", va='center', ha='center', rotation='horizontal', fontsize=25, color="black")
    fig.text(0.05, 0.5, "Power [kW]", va='center', ha='center', rotation='vertical', fontsize=25, color="black")

    cumsum_output_meas = np.cumsum(output*stepSize/3600/1000)
    cumsum_output_pred = np.cumsum(start_pred*stepSize/3600/1000)
    fig, ax = plt.subplots(1)
    ax.plot(cumsum_output_meas, color="blue")
    ax.plot(cumsum_output_pred, color="black")

    plt.show()



if __name__ == '__main__':
    test()