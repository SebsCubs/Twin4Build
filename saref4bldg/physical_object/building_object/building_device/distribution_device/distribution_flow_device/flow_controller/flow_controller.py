import saref4bldg.physical_object.building_object.building_device.distribution_device.distribution_flow_device.distribution_flow_device as distribution_flow_device
class FlowController(distribution_flow_device.DistributionFlowDevice):
    def __init__(self,
                **kwargs):
        super().__init__(**kwargs)