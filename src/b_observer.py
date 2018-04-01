import math as _math

import sys as _sys
if _sys.version_info.major != 2: reload, xrange = lambda x: 0, range

import a1_process_model as process_model
reload(process_model)
import geometry as _g
reload(_g)

# Author: Chris Petrich
# Date: 2016
#   Apr 2018: modified for Python 3

class Observer(object):
    def __init__(self, init=None):
        self.init(init)

    def init(self, init):
        m = _math.pi/180.

        self._reward_fail = -10. / (1-.95) *10
        self._reward_action = -10.
        self._reward_step = -10.
        self._reward_tick = -1.

        self.look_ahead_gamma = 0.95

        # physical interpretation of actions
        # speed: defines acceleration, and wheel positions: define rate of rotation
        self.distance_steps = (-.1, 0, .1, .3, 1., 3.)
        half_wheel_positions = (0, 2.5, 5, 10, 20, 35)
        wheel_positions = tuple([-p*m for p in half_wheel_positions[-1:0:-1]]+[p*m for p in half_wheel_positions])
        self.wheel_positions = wheel_positions
        # numerical actions
        wheel = (-1, 1, 0) # should these numbers be moved into the car model??
        speed = (-1, 1, 0)
        self._actions = tuple([(w,s) for w in wheel for s in speed])


        # size of impact detection zones
        # front, right, back, left
        # --> the car will try to find a safe driving
        #  distance from the sides
        self.zone_distances=[(.1, .3, 1., 2., 3., 10.),
                             (.1, .3, 1., 2., 3., 10.),
                              (.1,.3,),(.1,),
                              (.1,),
                              (.1,),(.1,.3)]

        self.car = None # observer needs access to car dimensions
        self.environment = None # and domain boundaries

        if init is not None:
            for variable in init:
                self.__setattr__(variable,init[variable])

        self._distance_bin_edges = [.5*(a+b) for a,b in zip(self.distance_steps[:-1],self.distance_steps[1:])]
        self._distance_bin_edges.append(1e10) # really fast

        self._wheel_bin_edges = [.5*(a+b) for a,b in zip(self.wheel_positions[:-1],self.wheel_positions[1:])]
        self._wheel_bin_edges.append(1e10) # really fast

    def reward_for_learner(self, reward_token):
        if reward_token == 'fail': return self._reward_fail
        if reward_token == 'action': return self._reward_action
        if reward_token == 'step': return self._reward_step
        if reward_token == 'tick': return self._reward_tick


    def action_for_model(self, model_state, numerical_state, numerical_action):
        """Translate numerical action into model action, taking into account
           the (old) state of the model."""
        # move wheel to new, discrete value
        # change speed to new, discrete value
        wheel_rotation_direction, speed_change_direction = self._actions[numerical_action]

        wheel_step_idx = wheel_rotation_direction + self.wheel_positions.index(model_state.center_wheel_dir)
        wheel_direction = self.wheel_positions[ max((0,min((len(self.wheel_positions)-1,wheel_step_idx)))) ]

        distance_step_idx = speed_change_direction + self.distance_steps.index(model_state.distance_step)
        distance_step = self.distance_steps[ max((0,min((len(self.distance_steps)-1,distance_step_idx)))) ]

        # action as understood by model
        world_action = wheel_direction, distance_step
        return  world_action

    def state_for_learner(self, model_state, return_intermediate_state=False):
        """Translate state into numerical, discrete state"""
        # NB: we ignore terminated state so the control flow can pass
        #   through to Q-learner. Termination is addressed through the
        #   reward system.
        # NB: this is not a stateful function since the environment may
        #   have changed

        # calculate distance to the sides of the car
        corners=process_model.chassis_outline(model_state.p_front_right,model_state.axis_dir,
                                         self.car.wheelbase,self.car.track,
                                         self.car.overhang_front, self.car.overhang_rear,
                                         self.car.body_width)
        _c = lambda i,j: (.5*(corners[i][0]+corners[j][0]), .5*(corners[i][1]+corners[j][1]))

        # front-right, clockwise to front-left
        zone_points = [_c(0,0),_c(0,1),_c(1,1),_c(2,2),_c(2,3),_c(3,3),_c(3,0)]
        # counter-clockwise outline: Front, Right, Back, Left
        car_outline = _g._lines_from_points(zone_points)
        car_outline = car_outline[-2:]+car_outline[:-2]
        car_outline_h = _g._lines_to_homogenous(car_outline)
        distances = self.zone_distances

        zone_triggered = []
        for side, zone_d in zip(car_outline, distances):
            for distance in zone_d:
                # positive distance is to the left of line --> to the outside
                head_line = _g._offset_line(side, distance)
                # instead of testing whether something is in the zone we test
                #  whether something intersects one of the zone lines
                #  (may want to add cross lines through the zone if we do detection of
                #   obstacles smaller than any of the zones)
                zone_lines = [(side[0], head_line[0]), head_line, (head_line[1],side[1])]
                env_lines, env_lines_h = self.environment.get_domain_lines_that_may_intersect(zone_lines)
                if len(env_lines)>0:
                    obstacle = _g._any_intersect(env_lines, zone_lines, env_lines_h)
                else:
                    obstacle = False
                if obstacle:
                    # found obstacle, stop looking in wider zones
                    break

            zone_triggered.append(distance if obstacle else 1e10)

        front_l,front_r, right_f,right_b, back, left_b,left_f = zone_triggered

        raw_state = (model_state.distance_step, model_state.center_wheel_dir,
                     front_l,front_r, left_f,left_b, right_f,right_b, back)

        if return_intermediate_state:
            return self._state_index(*raw_state), raw_state

        return self._state_index(*raw_state)


    def _state_index(self, distance_step, wheel_direction, front_l,front_r, left_f,left_b, right_f,right_b, back):
        # we can increase learning speed by exploiting state symmetry in wheel_direction
        #  and left/right:
        # merge wheel_direction with left/right
        #  naming them by abs(wheel_direction) and sides toward/agains wheel direction
        #  plus one set with wheel_direction==0 where the sides are
        #  actually left and right


        # distance_step
        speed_idx = min([idx for idx, edge in enumerate(self._distance_bin_edges) if distance_step <= edge])
        wheel_idx = min([idx for idx, edge in enumerate(self._wheel_bin_edges) if wheel_direction <= edge])

        # discretize distance to the sides of the car
        front_l_idx = self.zone_distances[0].index(front_l) if front_l in self.zone_distances[0] else len(self.zone_distances[0])
        front_r_idx = self.zone_distances[1].index(front_r) if front_r in self.zone_distances[0] else len(self.zone_distances[1])
        right_f_idx = self.zone_distances[2].index(right_f) if right_f in self.zone_distances[1] else len(self.zone_distances[2])
        right_b_idx = self.zone_distances[3].index(right_b) if right_b in self.zone_distances[1] else len(self.zone_distances[3])
        back_idx = self.zone_distances[4].index(back) if back in self.zone_distances[2] else len(self.zone_distances[4])
        left_b_idx = self.zone_distances[5].index(left_b) if left_b in self.zone_distances[3] else len(self.zone_distances[5])
        left_f_idx = self.zone_distances[6].index(left_f) if left_f in self.zone_distances[3] else len(self.zone_distances[6])

        return speed_idx, wheel_idx, front_l_idx, front_r_idx, left_f_idx, left_b_idx, right_f_idx, right_b_idx, back_idx

    def N_states_for_learner(self):
        """Number of discrete, numerical states in each dimension"""
        idx_max = []
        limits = 50, 2*_math.pi, 50, 50, 50, 50, 50, 50, 50
        for idx, limit in enumerate(limits):
            test = [0 for i in xrange(len(limits))]
            check = _arange(-limit,limit,limit/1000.)
            maxi = 0
            for v in check:
                test[idx]=v
                ret = self._state_index(*test)
                maxi = max((maxi, ret[idx]))
            idx_max.append(maxi)

        return tuple([idx+1 for idx in idx_max])

    def N_actions_for_learner(self):
        return len(self._actions)

def _arange(start,stop,step):
    v=[]
    while start < stop:
        v.append(start)
        start += step
    return v

def _confined_angle_pi(a):
    """Inclusive -pi, exclusive +pi"""
    while a<-_math.pi:
        a+=2*_math.pi
    while a>=_math.pi:
        a-=2*_math.pi
    return a

def _confined_angle_0(a):
    """inclusive 0, exclusive 2 pi"""
    while a < 0:
        a += 2*_math.pi
    while a >= 2*_math.pi:
        a -= 2*_math.pi
    return a
