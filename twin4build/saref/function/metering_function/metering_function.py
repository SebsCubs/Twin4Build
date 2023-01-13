from __future__ import annotations
from typing import Union
import twin4build.saref.function.function as function
import twin4build.saref.commodity.commodity as commodity
import twin4build.saref.property_.property_ as property_
class MeteringFunction(function.Function):
    def __init__(self,
                hasMeterReading: Union[list, None] = None,
                hasMeterReadingType: Union[commodity.Commodity, property_.Property, None] = None,
                **kwargs):
        super().__init__(**kwargs)
        assert isinstance(hasMeterReading, list) or hasMeterReading is None, "Attribute \"hasMeterReading\" is of type \"" + str(type(hasMeterReading)) + "\" but must be of type \"" + str(list) + "\""
        assert isinstance(hasMeterReadingType, commodity.Commodity) or isinstance(hasMeterReadingType, property_.Property) or hasMeterReadingType is None, "Attribute \"hasMeterReadingType\" is of type \"" + str(type(hasMeterReadingType)) + "\" but must be of type \"" + str(property_.Property) + "\" or \"" + str(property_.Property) + "\""
