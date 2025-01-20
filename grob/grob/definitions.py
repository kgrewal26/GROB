from enum import Enum

class States_E(Enum):
    SCANNING = 1
    PLANNING = 2
    NAVIGATING = 3
    WAITING = 4
    EMPTYING = 5

    def __int__(self):
        return self.value