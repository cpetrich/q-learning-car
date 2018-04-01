import math as _math
import geometry as _g

import sys as _sys
if _sys.version_info.major != 2: reload = lambda x: 0

reload(_g)

# Author: Chris Petrich
# Date: 2016
#   Apr 2018: modified for Python 3

class Rewards(object):
    def __init__(self,init=None):
        self.init(init)

    def init(self, init):

        self.odometer_tick = 4. # every 4 meters emit a reward tick

        if init is not None:
            for variable in init:
                self.__setattr__(variable,init[variable])

    def reward(self,originating_state=None, action=None, new_state=None):

        if new_state.status == 'terminated':
            return 'fail'

        if (originating_state is not None):
            if (int(new_state.odometer_reading/self.odometer_tick) !=
                int(originating_state.odometer_reading/self.odometer_tick) ):
                # car managed to advance
                return 'tick'
        if (action[0] != 0) or (action[1] != 0):
            # activity
            return 'action'

        # simple step
        return 'step'

