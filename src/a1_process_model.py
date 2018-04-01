import math as _math
import geometry as _g

import sys as _sys
if _sys.version_info.major != 2: reload = lambda x: 0

reload(_g)

# Author: Chris Petrich
# Date: 2016
#   Apr 2018: modified for Python 3

class ModelState(object):
    # This class is as fast for member access by name
    #   as tuples are for member access by index. The
    #   purpose is to reduce errors when modifying the
    #   state.
    # However, creating a new instance takes 8
    #   times as long as creating a tuple.
    __slots__ = ('p_front_right',
                 'axis_dir',
                 'center_wheel_dir',
                 'distance_step',
                 'odometer_reading',
                 'status') # status in progression: running or terminated
    def __init__(self, p_front_right, axis_dir, center_wheel_dir, distance_step, odometer_reading, status):
        self.p_front_right = tuple(p_front_right)
        self.axis_dir = axis_dir
        self.center_wheel_dir = center_wheel_dir
        self.distance_step = distance_step # distance rotated by the 'center' wheel
        self.odometer_reading = odometer_reading
        self.status = status

    def __repr__(self):
        return ('%s(p_front_right=%r, axis_dir=%r, center_wheel_dir=%r, distance_step=%r, status=%r)' %
                (self.__class__.__name__,
                 self.p_front_right, self.axis_dir,
                 self.center_wheel_dir, self.distance_step,
                 self.status))

    def __eq__(self, other):
        """Equal compares by value"""
        return ((self.p_front_right[0] == other.p_front_right[0]) and
                (self.p_front_right[1] == other.p_front_right[1]) and
                (self.axis_dir == other.axis_dir) and
                (self.center_wheel_dir == other.center_wheel_dir) and
                (self.distance_step == other.distance_step))
    def __ne__(self, other):
        return not self.__eq__(other)
    def __copy__(self):
        """Return a shallow copy of this instance."""
        return ModelState(self.p_front_right, self.axis_dir, self.center_wheel_dir, self.distance_step)
    def __lt__(self, other):
        raise NotImplementedError('< comparison') # to catch typos
    def __gt__(self, other):
        raise NotImplementedError('> comparison') # to catch typos
    def __le__(self, other):
        raise NotImplementedError('<= comparison') # to catch typos
    def __ge__(self, other):
        raise NotImplementedError('>= comparison') # to catch typos
    def __nonzero__(self, other):
        raise NotImplementedError('bool conversion') # to catch typos



class Car(object):
    _instatiation_counter = [0]
    agent_ID = None
    def __init__(self, init=None):
        """Define static parameters outside the scope of model state"""
        self._instatiation_counter[0]+=1
        self.agent_ID = hash('Car-%i' %self._instatiation_counter[0])

        m=_math.pi/180.
        # dimensions affecting the low-speed movement
        self.wheelbase = 2.5
        self.track = 1.5
        self.center_angle_limits = 30*m
        # dimensions used for plotting
        self.overhang_front = .9
        self.overhang_rear = .7
        self.body_width = 2.

        self.environment = None

        if init is not None:
            # overwrite with dictionary
            for variable in init:
                self.__setattr__(variable,init[variable])

    def act(self, state, action):
        """Perform action on state, return new state."""

        if state.status == 'terminated':
            return state

        if action is not None:
            center_wheel_direction, center_distance_step = action

            p_front_right, axis_dir = _move(self.wheelbase,self.track,
                              state.p_front_right,
                              state.axis_dir,
                              center_wheel_direction,
                              center_distance_step)

        else:
            # just check for crash
            p_front_right = state.p_front_right
            axis_dir = state.axis_dir
            center_wheel_direction = state.center_wheel_dir
            center_distance_step= 0. # we're actually not moving the car --> don't change odometer

        new_status = self._check_crash(ModelState(p_front_right=p_front_right,
                          axis_dir=axis_dir,
                          center_wheel_dir=center_wheel_direction,
                          distance_step=state.distance_step,
                          odometer_reading=state.odometer_reading,
                          status=state.status))

        return ModelState(p_front_right=p_front_right,
                          axis_dir=axis_dir,
                          center_wheel_dir=center_wheel_direction,
                          distance_step=center_distance_step,
                          odometer_reading=state.odometer_reading + abs(center_distance_step),
                          status=new_status)

    def _check_crash(self, state):

        corners=chassis_outline(state.p_front_right,state.axis_dir,
                                      self.wheelbase,self.track,
                                      self.overhang_front, self.overhang_rear,
                                      self.body_width)
        car_outline = _g._lines_from_points(corners)
        car_outline_h = _g._lines_to_homogenous(car_outline)

        self.environment.set_agent_properties(self, car_outline, car_outline_h)

        env_lines, env_lines_h = self.environment.get_domain_lines_that_may_intersect(car_outline)

        # car leaving the domain?
        out_of_bounds = _g._any_intersect(car_outline, env_lines,
                                       car_outline_h, env_lines_h)

        if out_of_bounds:
            return 'terminated'

        if False:
            # check for collision with other agents
            for agent in self.environment.agents:
                if agent == self: continue

                collision = _g._any_intersect(car_outline, self.environment.agents[agent][0],
                                               car_outline_h, self.environment.agents[agent][0])

                if collision:
                    return 'terminated'

        return 'running'

###################################################################

def _front_right_pos_from_center(wheelbase, track, p_center, axis_direction):
    """Specify center of car and orientation, return position of front right wheel"""
    rot=lambda pt, a: (_math.cos(a)*pt[0]+ -_math.sin(a)*pt[1],
                    _math.sin(a)*pt[0]+  _math.cos(a)*pt[1])
    vector=(wheelbase/2., -track/2.)
    return _add(rot(vector, axis_direction), p_center)


def _move(wheelbase, track, p_front_right,
         axis_direction, center_wheel_direction, distance_center_wheel):
    """Move car by moving front right wheel forward by 'distance'.
       Specify the center wheel position."""
    front_right_wheel_direction = wheel_angles_from_track_angle(wheelbase, track, center_wheel_direction)[0]
    # convey reverse driving to move_on_curve() in a consistent way
    used_wheel_direction = _confined_angle_pi(front_right_wheel_direction-_math.pi) if distance_center_wheel < 0 else front_right_wheel_direction
    # since the left and right wheels follow different tracks,
    #   we use the "center" wheel to define distance (and speed)
    #  --> ensures that tracks are symmetric
    R_right, R_center, R_left = absolute_curve_radii_from_center_angle(wheelbase, track, center_wheel_direction)

    # use if/else construct since we cannot divide inf/inf when going straight
    abs_right_front_wheel_distance = abs(distance_center_wheel)*R_right/R_center if R_right != R_center else abs(distance_center_wheel)

    p_front_right, axis_direction = _move_on_curve(p_front_right, axis_direction, used_wheel_direction, wheelbase, abs_right_front_wheel_distance)

    return p_front_right, axis_direction

def _move_on_curve(p_front,car_axis_dir,front_right_wheel_dir,axes_separation,distance):

    wheel_dir = front_right_wheel_dir

    if (abs(wheel_dir)<1e-6): # 1e-6 is the limit
        # straight forward
        return _move_straight(p_front,car_axis_dir,distance)
    if abs(abs(wheel_dir)-_math.pi)<1e-6: # 1e-6 is the limit
        # straight back
        # change the "direction" rather than angle since the angle will be passed back to loop
        return _move_straight(p_front,car_axis_dir,-distance)

    curve_radius_front_wheel=axes_separation/_math.sin(wheel_dir)

    #wheel_angle = car_axis_dir+wheel_dir

    # The curve_angle applies to all wheels:
    curve_angle = 2*_math.pi* distance/(2*_math.pi*curve_radius_front_wheel)

    new_axis = car_axis_dir + curve_angle
    while new_axis>=_math.pi:
        new_axis-=2*_math.pi
    while new_axis<-_math.pi:
        new_axis+=2*_math.pi

    if True:
        # calculate based on front wheel
        #  (same result as back wheel)
        dx = curve_radius_front_wheel * _math.cos(_math.pi/2+car_axis_dir+wheel_dir)
        dy = curve_radius_front_wheel * _math.sin(_math.pi/2+car_axis_dir+wheel_dir)

        turning_center= _add(p_front, (dx,dy))
        start = _sub(p_front, turning_center)
        a = curve_angle
        end=[_math.cos(a)*start[0]+ -_math.sin(a)*start[1],
             _math.sin(a)*start[0]+  _math.cos(a)*start[1]]

        new_front = _add(end, turning_center)
    else:
        # calculate based on back wheel
        #  (same result as front wheel)
        curve_radius_back_wheel=axes_separation/_math.tan(wheel_dir)

        p_rear = _rear_wheel_pos(p_front,car_axis_dir,axes_separation)
        dx = curve_radius_back_wheel * _math.cos(_math.pi/2+car_axis_dir)
        dy = curve_radius_back_wheel * _math.sin(_math.pi/2+car_axis_dir)
        turning_center= _add(p_rear, (dx,dy))

        start = _sub(p_rear, turning_center)
        a = curve_angle
        end=[_math.cos(a)*start[0]+ -_math.sin(a)*start[1],
             _math.sin(a)*start[0]+  _math.cos(a)*start[1]]

        new_rear = _add(end, turning_center)
        new_front__from_back = _rear_wheel_pos(new_rear,new_axis,-axes_separation)
        #print 'back connected', new_front__from_back
        new_front = new_front__from_back

    return new_front, new_axis

def _move_straight(p_front,car_axis_dir,distance):
    dx = distance * _math.cos(car_axis_dir)
    dy = distance * _math.sin(car_axis_dir)
    return _add(p_front, (dx,dy)), car_axis_dir

def _distance(p1,p2):
    return ((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)**.5

def _vector(p1,p2):
    return _math.atan2(p2[1]-p1[1],p2[0]-p1[0])

def _add(p1,p2):
    return (p1[0]+p2[0],p1[1]+p2[1])

def _sub(p1,p2):
    return (p1[0]-p2[0],p1[1]-p2[1])

def _rear_wheel_pos(p_front,car_axis_dir,axes_separation):
    x=p_front[0]-axes_separation*_math.cos(car_axis_dir)
    y=p_front[1]-axes_separation*_math.sin(car_axis_dir)
    return x,y


def _new_wheel_angle(start_angle, movement, max_deflection=None):
    # TODO: implement different limits for in and out
    # we assume that movement is increments |movement| < pi/2
    # or that |movement| = +- pi (i.e., reversing direction)
    # smooth transition between forward and backward is only supported
    # if max_deflection==pi/2
    epsilon = 1e-12
    max_deflection=max_deflection if max_deflection is not None else 20*_math.pi/180.
    ang=_confined_angle_pi(start_angle + movement)
    if abs(max_deflection-_math.pi/2.)<epsilon: return ang

    a=max_deflection
    b=_math.pi-max_deflection
    a_min,a_max = -a, a
    b_neg,b_pos = -b, b
    if (a_min<=ang<=a_max) or (ang<=b_neg) or (ang>=b_pos): return ang
    reversing = abs(abs(movement)-_math.pi) < epsilon
    s= -1 if reversing else 1

    if (s*abs(start_angle)<=s*_math.pi/2):
        # supposed to go forward after move
        return min((a_max,max((a_min,ang))))
    else:
        # supposed to go backward after move
        if (ang<0): return min((ang, b_neg))
        return max((ang, b_pos))


def _confined_angle_pi(a):
    """Inclusive -pi, exclusive +pi"""
    while a<-_math.pi:
        a+=2*_math.pi
    while a>=_math.pi:
        a-=2*_math.pi
    return a


#####################################
#
#  helper functions to plot the state
#  or caclulate collisions
#

def chassis_outline(p_front_right,car_axis_dir,wheelbase,track,
                overhang_front, overhang_rear, car_width):

    chassis_lr = (car_width-track)/2.

    # positions wrt to front right corner

    corners=[[0,0],
             [-(overhang_front+overhang_rear+wheelbase),0],
             [-(overhang_front+overhang_rear+wheelbase),car_width],
             [0,car_width]]
    # translate so
    # positions wrt to front right wheel
    corners = [_add(c,(overhang_front,-chassis_lr)) for c in corners]
    # rotate according to car_axis_dir
    a = car_axis_dir
    rot=lambda pt: [_math.cos(a)*pt[0]+ -_math.sin(a)*pt[1],
                    _math.sin(a)*pt[0]+  _math.cos(a)*pt[1]]
    chassis=[rot(p) for p in corners]

    chassis = [_add(c,p_front_right) for c in chassis]

    return chassis


def wheel_outlines(p_front_right,car_axis_dir,
                   center_wheel_dir,
                   wheelbase,track,
                   wheel_size = None):

    rot=lambda pt, a: [_math.cos(a)*pt[0]+ -_math.sin(a)*pt[1],
                    _math.sin(a)*pt[0]+  _math.cos(a)*pt[1]]
    rotate=lambda points, a: [rot(p,a) for p in points]
    add = lambda many, offset: [_add(p, offset) for p in many]
    sub = lambda many, offset: [_sub(p, offset) for p in many]

    right_wheel_dir, left_wheel_dir = wheel_angles_from_center_angle(wheelbase, track, center_wheel_dir)

    # center position of wheels wrt to front right wheel
    wheel_centers=((0,0),(-wheelbase,0),(-wheelbase,track),(0,track))

    wheel_pos = rotate(wheel_centers, car_axis_dir)
    wheel_pos = add(wheel_pos, p_front_right)

    # rotated wheels at their respective positions
    wheels = []

    # calculate outlines of wheels
    # wheel, origin at center
    if wheel_size is None:
        wheel_size = '195/55R16'
        width=195/1000.
        diameter=2*width*.55+16*.0254

    wh_width,wh_rubber_scale,wh_base = float(wheel_size.split('/')[0]),float(wheel_size.split('/')[1].split('R')[0])*.01,float(wheel_size.split('R')[1])
    width=wh_width/1000.
    diameter=2*width*wh_rubber_scale+wh_base*.0254
    wheel=sub(((0,0),(0,width),(diameter,width),(diameter,0)), (diameter/2.,width/2.))

    # rear wheels
    corners = rotate(wheel, car_axis_dir)
    wheels.append(add(corners,wheel_pos[1]))
    wheels.append(add(corners,wheel_pos[2]))

    # front right wheel
    corners = rotate(wheel, car_axis_dir+right_wheel_dir)
    wheels.append(add(corners,wheel_pos[0]))

    # front left wheel
    corners = rotate(wheel, car_axis_dir+left_wheel_dir)
    wheels.append(add(corners,wheel_pos[3]))

    return wheels

def front_left_wheel_angle(wheelbase, track, right_wheel_dir):
    """Geometric calculation valid for low speeds.
    Returns angle of front left tire and curve radii front right and front left."""
    # cf. https://en.wikipedia.org/wiki/Ackermann_steering_geometry
    if abs(right_wheel_dir)<1e-6:
        return right_wheel_dir, (float('inf'),float('inf'))

    if True:
        self = front_left_wheel_angle
        params = wheelbase, track, right_wheel_dir
        cache=self.cache=self.__dict__.get('cache',{})
        if params in cache:
            return cache[params]

    inner=right_wheel_dir if right_wheel_dir >0 else _math.pi-right_wheel_dir
    curve_radius_front_right=wheelbase/_math.sin(right_wheel_dir)
    curve_radius_front_left=(track**2+curve_radius_front_right**2
                             -2*track*abs(curve_radius_front_right)*
                              _math.cos(inner))**.5
    curve_radius_front_left=abs(curve_radius_front_left)*(1 if right_wheel_dir>=0 else -1)


    left_wheel_dir=_math.acos((track-abs(curve_radius_front_right)*_math.cos(inner))/
                             abs(curve_radius_front_left))
    left_wheel_dir=_math.pi-left_wheel_dir if right_wheel_dir >0 else -left_wheel_dir

    result = left_wheel_dir, (curve_radius_front_right, curve_radius_front_left)
    cache[params]=result
    return result

def wheel_angles_from_center_angle(wheelbase, track, center_wheel_angle):
    if True:
        self = wheel_angles_from_center_angle
        params = wheelbase, track, center_wheel_angle # this assignment takes about 0.15 us (about the same as the overhead of a function call)
        cache=self.cache=self.__dict__.get('cache',{})
        if params in cache:
            return cache[params]

    if abs(center_wheel_angle)<1e-6:
        result = 0., 0.
    else:
        R_right, R_center, R_left = absolute_curve_radii_from_center_angle(wheelbase, track, center_wheel_angle)
        sig = 1. if center_wheel_angle>0 else -1.
        result = sig*_math.asin(wheelbase/R_right), sig*_math.asin(wheelbase/R_left)

    cache[params]=result
    return result


def wheel_angles_from_track_angle(wheelbase, track, center_wheel_angle):
    """Given the track angle, i.e. the direction of driving (a virtual wheel at the center of the car),
       what are the angles of the front wheels?
       Returning: angle of right-hand-side wheel, angle of left-hand-side wheel.
       Tested only for angles -pi/2 < track_angle < pi/2."""
    if abs(center_wheel_angle)<1e-6:
        return center_wheel_angle, center_wheel_angle

    if True:
        self = wheel_angles_from_track_angle
        params = wheelbase, track, center_wheel_angle
        cache=self.cache=self.__dict__.get('cache',{})
        if params in cache:
            return cache[params]

    calc_track_angle = abs(center_wheel_angle) # calculation uses magnitudes of wheel direction
    t2w = track/(2.*wheelbase)
    sin, cos = _math.sin(calc_track_angle), _math.cos(calc_track_angle)
    sin_t2w = sin*t2w
    sin_t2w_2 = sin_t2w**2
    rho = 2*sin*cos*t2w

    gamma_inner, gamma_outer = (1-rho+sin_t2w_2)**(-.5), (1+rho+sin_t2w_2)**(-.5)
    delta_inner = _math.asin(sin_t2w*sin*gamma_inner)
    delta_outer = _math.asin(sin_t2w*sin*gamma_outer)

    if center_wheel_angle>0:
        # turning left
        # right wheel, left wheel
        result = (calc_track_angle-delta_outer), (calc_track_angle+delta_inner)
    else:
        # turning right
        # right wheel, left wheel
        result = -(calc_track_angle+delta_inner), -(calc_track_angle-delta_outer)

    cache[params] = result
    return result

def track_angle_from_right_wheel_angle(wheelbase, track, right_wheel_dir):
    """Given the angle of the right front wheel,
       what is the direction at the front center of the car, i.e. the track angle?
       Returning: track angle.
       Tested only for angles -pi/2 < right_wheel_dir < pi/2.
       NB: for left wheel input, supply -left_wheel_dir instead."""
    if abs(right_wheel_dir)<1e-6:
        return right_wheel_dir

    if True:
        self = track_angle_from_right_wheel_angle
        params = wheelbase, track, right_wheel_dir
        cache=self.cache=self.__dict__.get('cache',{})
        if params in cache:
            return cache[params]

    calc_right_wheel_dir = abs(right_wheel_dir) # calculation uses magnitudes of wheel direction

    t2w = track/(2.*wheelbase)
    sin, cos = _math.sin(calc_right_wheel_dir), _math.cos(calc_right_wheel_dir)
    sin_t2w = sin*t2w
    sin_t2w_2 = sin_t2w**2
    rho = 2*sin*cos*t2w

    if right_wheel_dir > 0:
        # turning left
        gamma_outer = (1-rho+sin_t2w_2)**(-.5) # NB: inner/outer signs reversed wrt to conversion center-->sides
        delta_outer = _math.asin(sin_t2w*sin*gamma_outer)
        result = calc_right_wheel_dir+delta_outer
    else:
        # turning right
        gamma_inner = (1+rho+sin_t2w_2)**(-.5) # NB: inner/outer signs reversed wrt to conversion center-->sides
        delta_inner = _math.asin(sin_t2w*sin*gamma_inner)
        result = -(calc_right_wheel_dir-delta_inner)

    cache[params]=result
    return result

def minimum_road_width_and_road_center_curve_radius(wheelbase, overhang_front, overhang_rear, body_width,
                                                    center_wheel_angle):
    """Helper function to check track design.
       Include any safety distance from the chassis in body_width for realistic estimate,
       or use car's track for body_width to get the absolute minimum values."""
    if abs(center_wheel_angle) < 1e-6:
        return body_width, float('inf')

    if True:
        self = minimum_road_width_and_road_center_curve_radius
        params = wheelbase, overhang_front, overhang_rear, body_width, center_wheel_angle
        cache=self.cache=self.__dict__.get('cache',{})
        if params in cache:
            return cache[params]

    s = wheelbase/_math.tan(abs(center_wheel_angle))
    R_rear  = ( (s-body_width/2.)**2+overhang_rear**2 )**.5
    R_front = ( (s+body_width/2.)**2+(overhang_front+wheelbase)**2 )**.5
    result = R_front-R_rear, .5*(R_front+R_rear)

    cache[params]=result
    return result

def minimum_road_width_for_point_corners(wheelbase, overhang_front, overhang_rear, body_width,
                                                    center_wheel_angle, corner_angle):
    """Minimum width of the road, i.e. square track envelope of a 90 degree curve.
       Add any lateral safety to body width. Note: this assumes the perfect track through
       the curve."""
    if True:
        params = wheelbase, overhang_front, overhang_rear, body_width, center_wheel_angle, corner_angle
        cache=minimum_road_width_for_point_corners.cache=minimum_road_width_for_point_corners.__dict__.get('cache',{})
        if params in cache:
            return cache[params]

    # this is the minimum road with plus 0.3 times the INNER road radius (90 deg corners)
    s = wheelbase/_math.tan(abs(center_wheel_angle))
    R_rear  = ( (s-body_width/2.)**2+overhang_rear**2 )**.5
    R_front = ( (s+body_width/2.)**2+(overhang_front+wheelbase)**2 )**.5
    result = R_front-R_rear + R_rear*(1.-_math.cos(.5*corner_angle))

    if True:
        cache[params]=result
    return result

def absolute_curve_radii_from_center_angle(wheelbase, track, center_wheel_angle):
    """Return absolute values of curve radii of right, center, and left wheel based on center angle."""
    if True:
        self = absolute_curve_radii_from_center_angle
        params = wheelbase, track, center_wheel_angle # this assignment takes about 0.15 us (about the same as the overhead of a function call)
        cache=self.cache=self.__dict__.get('cache',{})
        if params in cache:
            return cache[params]

    abs_center_wheel_angle = abs(center_wheel_angle)
    if abs_center_wheel_angle<1e-6:
        result = float('inf'), float('inf'), float('inf')
    else:
        R_center = wheelbase / _math.sin(abs_center_wheel_angle)
        s = wheelbase / _math.tan(abs_center_wheel_angle)
        R_outer = (wheelbase**2 + (s+track/2.)**2)**.5
        R_inner = (wheelbase**2 + (s-track/2.)**2)**.5
        if center_wheel_angle > 0.: # turning left
            # radii: right, center, left
            result = R_outer, R_center, R_inner
        else:
            # radii: right, center, left
            result = R_inner, R_center, R_outer

    cache[params]=result
    return result
