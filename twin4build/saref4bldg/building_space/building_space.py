import twin4build.saref4syst.system as system
import twin4build.saref.feature_of_interest.feature_of_interest as feature_of_interest
class BuildingSpace(feature_of_interest.FeatureOfInterest, system.System):
    def __init__(self,
                hasSpace = None,
                isSpaceOf = None,
                contains = None,
                airVolume = None,
                **kwargs):
        super().__init__(**kwargs)
        self.hasSpace = hasSpace
        self.isSpaceOf = isSpaceOf
        self.contains = contains
        self.airVolume = airVolume