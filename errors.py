# Failure cases - data related
class ShortGroundTruthError(Exception):
    pass
class DataValidityError(Exception):
    pass

# Failure cases - route related
class NoRoutesFoundError(Exception):
    pass
class NoLinksInQueriedAreaError(Exception):
    pass
class FuturePathSimplificationError(Exception):
    pass