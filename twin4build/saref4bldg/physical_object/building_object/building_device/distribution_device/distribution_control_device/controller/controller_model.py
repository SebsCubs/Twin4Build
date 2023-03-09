from .controller import Controller
import torch
from scipy.optimize import least_squares
import numpy as np
class ControllerModel(Controller):
    def __init__(self, 
                # isTemperatureController=None,
                # isCo2Controller=None,
                K_p=None,
                K_i=None,
                K_d=None,
                **kwargs):
        Controller.__init__(self, **kwargs)
        self.K_p = K_p
        self.K_i = K_i
        self.K_d = K_d

        self.input = {"actualValue": None, 
                    "setpointValue": None}
        self.output = {"inputSignal": None}

    def initialize(self,
                    startPeriod=None,
                    endPeriod=None,
                    stepSize=None):
        self.acc_err = 0
        self.prev_err = 0

    def do_step(self, secondTime=None, dateTime=None, stepSize=None):
        err = self.input["setpointValue"]-self.input["actualValue"]
        p = err*self.K_p
        i = self.acc_err*self.K_i
        d = (err-self.prev_err)*self.K_d
        signal_value = p + i + d
        if signal_value>1:
            signal_value = 1
            self.acc_err = 1/self.K_i
            self.prev_err = 0
        elif signal_value<0:
            signal_value = 0
            self.acc_err = 0
            self.prev_err = 0
        else:
            self.acc_err += err
            self.prev_err = err

        self.output["inputSignal"] = signal_value

    def do_period(self, input):
        self.clear_report()
        self.acc_err = 0
        self.prev_err = 0
        for index, row in input.iterrows():
            for key in input:
                self.input[key] = row[key]
            self.do_step()
            self.update_report()
        output_predicted = np.array(self.savedOutput["inputSignal"])
        return output_predicted

    def obj_fun(self, x, input, output):
        self.K_p = x[0]
        self.K_i = x[1]
        self.K_d = x[2]
        output_predicted = self.do_period(input)
        res = output_predicted-output #residual of predicted vs measured
        print(f"Loss: {np.sum(res**2)}")
        return res

    def calibrate(self, input=None, output=None):
        x0 = np.array([self.K_p,self.K_i,self.K_d])
        lw = [0, 0.01, 0]
        up = [1, 1, 1]
        bounds = (lw,up)
        sol = least_squares(self.obj_fun, x0=x0, bounds=bounds, args=(input, output))
        self.K_p,self.K_i,self.K_d = sol.x
        print(self.K_p,self.K_i,self.K_d)
        #print(sol)
        return(self.K_p,self.K_i,self.K_d)