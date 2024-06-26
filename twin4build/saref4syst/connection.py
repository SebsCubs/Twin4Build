from __future__ import annotations
from typing import Union
from .system import System
from twin4build.logger.Logging import Logging
logger = Logging.get_logger("ai_logfile")

class Connection:
    def __init__(self,
                connectsSystem: Union[System ,None]=None,
                connectsSystemAt: Union[list ,None]=None,
                senderPropertyName=None):
        logger.info("[Connection Class] : Entered in __init__ Function")
        assert isinstance(connectsSystem, System) or connectsSystem is None, "Attribute \"connectsSystem\" is of type \"" + str(type(connectsSystem)) + "\" but must be of type \"" + str(System) + "\""
        assert isinstance(connectsSystemAt, list) or connectsSystemAt is None, "Attribute \"connectsSystemAt\" is of type \"" + str(type(connectsSystemAt)) + "\" but must be of type \"" + str(list) + "\""
        if connectsSystemAt is None:
            connectsSystemAt = []
        self.connectsSystem = connectsSystem
        self.connectsSystemAt = connectsSystemAt
        self.senderPropertyName = senderPropertyName
        logger.info("[Connection Class] : Exited from __init__ Function")
        
        
