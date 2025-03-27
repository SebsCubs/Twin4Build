import twin4build.base as base
import numpy as np
import twin4build.utils.input_output_types as tps


class VAVReheatControllerSystem(base.SetpointController):
    def __init__(self, 
                 rat_v_flo_min=0.3,  # Minimum airflow ratio
                 rat_v_flo_hea=0.3,  # Heating airflow ratio
                 k_coo=0.1,          # Cooling controller gain
                 k_hea=0.1,          # Heating controller gain
                 ti_coo=120,         # Cooling integral time constant
                 ti_hea=120,         # Heating integral time constant
                 dt_hys=0.5,         # Hysteresis width
                 t_sup_min=5.0,      # Minimum supply air temperature
                 t_sup_max=30.0,
                 **kwargs):    # Maximum supply air temperature
        super().__init__(**kwargs)
        # Parameters
        self.rat_v_flo_min = rat_v_flo_min
        self.rat_v_flo_hea = rat_v_flo_hea
        self.dt_hys = dt_hys
        
        # Supply air temperature limits
        self.t_sup_min = t_sup_min
        self.t_sup_max = t_sup_max
        
        # Controller gains and time constants
        self.k_coo = k_coo
        self.k_hea = k_hea
        self.ti_coo = ti_coo
        self.ti_hea = ti_hea
        
        # State variables for integral control
        self.i_coo = 0
        self.i_hea = 0
        
        self.input = {"roomTemp": tps.Scalar(),
                    "heatingsetpointValue": tps.Scalar(),
                    "coolingsetpointValue": tps.Scalar()}
        self.output = {"y_dam": tps.Scalar(),
                       "supplyAirTemp": tps.Scalar()}
        self._config = {"parameters": ["k_coo",
                                       "k_hea",
                                       "ti_coo",
                                       "ti_hea",]}

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
        self.acc_err = 0
        self.prev_err = 0

    def do_step(self, secondTime=None, dateTime=None, stepSize=None):
        """
        Compute damper and valve signals
        
        Parameters:
        - t_roo_hea_set: Heating setpoint temperature
        - t_roo_coo_set: Cooling setpoint temperature
        - t_roo: Current room temperature
        - dt: Time step for integral calculation
        
        Returns:
        Tuple of (y_dam, y_val)
        """
        # Compute temperature differences
        t_roo_hea_set = self.input["heatingsetpointValue"]
        t_roo_coo_set = self.input["coolingsetpointValue"]
        t_roo = self.input["roomTemp"]
        dt = stepSize

        err_hea = t_roo_hea_set - t_roo
        err_coo = t_roo - t_roo_coo_set
        
        # Determine operating mode
        in_heating_mode = err_hea > self.dt_hys
        in_cooling_mode = err_coo > self.dt_hys
        
        # Cooling controller (PI)
        if in_cooling_mode:
            # Proportional term
            p_coo = self.k_coo * err_coo
            
            # Integral term
            self.i_coo += (self.k_coo / self.ti_coo) * err_coo * dt
            
            # Constrain integral term
            self.i_coo = np.clip(self.i_coo, 0, 1)
            
            # Compute damper signal (between min flow and max flow)
            y_dam = np.clip(p_coo + self.i_coo, self.rat_v_flo_min, 1.0)
            
            # Supply air temperature for cooling (interpolate between min and ambient)
            # Assuming room temperature as ambient for simplicity
            
            t_sup = np.clip(t_roo - (p_coo + self.i_coo), self.t_sup_min, t_roo)
        
        # Heating controller (PI)
        elif in_heating_mode:
            # Proportional term
            p_hea = self.k_hea * err_hea
            
            # Integral term
            self.i_hea += (self.k_hea / self.ti_hea) * err_hea * dt
            
            # Constrain integral term
            self.i_hea = np.clip(self.i_hea, 0, 1)
            
            # Compute valve signal and fix damper at heating minimum
            y_dam = self.rat_v_flo_hea
            
            # Supply air temperature for heating 
            # Interpolate between room temp and max heating temp
            t_sup = np.clip(t_roo + (p_hea + self.i_hea), t_roo, self.t_sup_max)
        
        
        # Dead band
        else:
            y_dam = self.rat_v_flo_min
            # Neutral supply temperature during dead band
            t_sup = t_roo
        
        self.output["y_dam"].set(y_dam)
        self.output["supplyAirTemp"].set(t_sup)




