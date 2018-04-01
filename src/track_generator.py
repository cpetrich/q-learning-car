import math as _math
import random as _random

import sys as _sys
if _sys.version_info.major != 2: xrange = range

# Author: Chris Petrich
# Date: 2016
#   Apr 2018: modified for Python 3

def _homogenous_line(A,B):
    """Return line through A and B in homogenous coordinates (a,b,c) with ax+by+c=0."""
    if A==B: raise ValueError('Degenerate line through %s and %s' % (repr(A),repr(B)))

    Ax,Ay=A
    Bx,By=B
    # keep magnitude of coefficients as close to unity as possible
    if abs(Bx-Ax)>=abs(By-Ay):
        a,b=float(By-Ay)/float(Bx-Ax), -1
        c=Ay-a*Ax
    else:
        a,b=-1, float(Bx-Ax)/float(By-Ay)
        c=Ax-b*Ay

    return a,b,c

def _intersection_homogenous(homog_line_0, homog_line_1):
    """Find point of intersection of two lines in homogenous coordinates"""
    eps = 1e-13
    a,b,c=homog_line_0
    u,v,w=homog_line_1
    D=float(b*u-v*a)
    if abs(D)<eps:
        # parallel lines
        return float('nan'), float('nan')
    xp=-(w*b-c*v)/D
    yp= (w*a-c*u)/D

    return xp, yp

def _intersection(line_points_0, line_points_1):
    """Interection point of infinitely long lines"""
    u,v = line_points_0,line_points_1
    (A,B),(C,D) = line_points_0,line_points_1
    h1 = _homogenous_line(A,B)
    h2 = _homogenous_line(C,D)
    P = _intersection_homogenous(h1, h2)
    return P

def _offset_line(line, distance):
    # positive distance is to the left of the line
    rotate=lambda pt, a: (_math.cos(a)*pt[0]+ -_math.sin(a)*pt[1],
                    _math.sin(a)*pt[0]+  _math.cos(a)*pt[1])
    add=lambda p0, p1: (p0[0]+p1[0], p0[1]+p1[1])

    A, B = line
    (Ax,Ay),(Bx,By) = A, B

    angle = _math.atan2(By-Ay,Bx-Ax)
    C=add(rotate((0,distance), angle), A)
    D=add(rotate((0,distance), angle), B)
    return C, D

def _lines_from_points(points):
    """Make line segments of closed polygon defined by points"""
    return zip(points, tuple(points[1:])+(points[0],))

def _make_offset_lines(track, offset):
    left, right = [], []
    for d, side in ((offset, left), (-offset, right)):
        for idx, (c_now, c_next) in enumerate(zip(track[:-1],track[1:])):
            l1 = _offset_line(c_now, d)
            l2 = _offset_line(c_next, d)
            P = _intersection(l1, l2)
            side.append((l1[0], P))
        side.append((P, l2[1]))

    return left, right

def _make_offset_path(points, offset):
    left, right = [], []
    closed = points[0]==points[-1]
    if closed:
        points = tuple(points)+(points[1],points[2])

    for d, side in ((offset, left), (-offset, right)):
        for idx, (p_now, p_next, p_after) in enumerate(zip(points[:-2],points[1:-1],points[2:])):
            l1 = _offset_line((p_now, p_next), d)
            l2 = _offset_line((p_next, p_after), d)
            P = _intersection(l1, l2)
            if (side == []) and not closed:
                side.append(l1[0])
            side.append(P)

        if not closed: side.append(l2[-1])

    return left, right

def _ellipse_points(a, b, N):
    tp = 2*_math.pi
    ang = [tp/N*i for i in xrange(N)]
    path = []
    for angle in ang:
        path.append((a*_math.cos(angle),b*_math.sin(angle)))
    return path


def _angle_difference(a,b):
    """ return smallest difference between angles a and b"""
    d1=a-b
    d2=2*_math.pi+a-b
    d3=a-b-2*_math.pi
    d=(d1,d2,d3)
    idx=sorted(zip(d, (0,1,2)), key=lambda x: abs(x[0]))[0][1]
    return d[idx]


def _corner_angles(track, closed=False):
    if closed:
        track = tuple(track)+(track[0],track[1])

    lines = _lines_from_points(track)
    angles = [_math.atan2(b[1]-a[1],b[0]-a[0]) for a,b in lines]
    diff = [_angle_difference(a,b) for a,b in zip(angles[1:],angles[:-1])]

    return diff

def _round_corner_points(segment1_angle, segment2_angle, intersection_point, Radius, step_degrees=11.25, debug=False):
    """Find curve points for rounded curve. Need at least N=2 points."""
    # R = 5.5 m at 15 km/h
    # R = 30 m at 30 km/h
    gamma = _confined_angle_pi(segment1_angle-segment2_angle)
    d = Radius*_math.tan(abs(gamma)/2.)

    # number of points along the curve (need at least N=2 to make a cut)
    N=1+int(abs(gamma*180./_math.pi / step_degrees))
    if N<2:
        # the angle between the segments is less than what we would interpolate
        #   anyways, don't do anything
        return [intersection_point]

    pi2 = _math.pi/2.
    P = intersection_point

    # start angle from O to cut_seg1
    angle0 = pi2+segment1_angle


    # this is where we have to cut segment 1
    cut_seg1 = P[0]-d*_math.cos(segment1_angle), P[1]-d*_math.sin(segment1_angle)
    # this is the center of the curve circle
    if gamma > 0:
        # check positive gamma
        v=-1
        O = cut_seg1[0]+Radius*_math.cos(pi2-segment1_angle),cut_seg1[1]+v*Radius*_math.sin(pi2-segment1_angle)
        angles = [angle0+v*gamma*f/(N-1.) for f in xrange(N)]
        curve_points = [(O[0]+Radius*_math.cos(a), O[1]-v*Radius*_math.sin(a)) for a in angles]
    else:
        O = cut_seg1[0]-Radius*_math.cos(pi2-segment1_angle),cut_seg1[1]+Radius*_math.sin(pi2-segment1_angle)
        angles = [angle0-gamma*f/(N-1.) for f in xrange(N)]
        curve_points = [(O[0]-Radius*_math.cos(a), O[1]-Radius*_math.sin(a)) for a in angles]

    if debug:
        return curve_points, O, cut_seg1
    return curve_points

def _round_corners(path_points, target_radius, step_degrees = 11.25):
    path_closed= path_points[0]==path_points[-1]
    if path_closed:
        path_points = tuple(path_points)+(path_points[1],)
    # angles of path segements
    angles = [_math.atan2(p1[1]-p0[1], p1[0]-p0[0]) for p0, p1 in zip(path_points[:-1],path_points[1:])]

    out_path = []
    if not path_closed:
        out_path.append(path_points[0])
    # in a closed cicuit we'll replace the first points with the rounded points
    #   we calculate at the end
    # so anyways, remove the first entry
    path_points=path_points[1:]

    if isinstance(target_radius, float):
        target_radius = [target_radius for i in xrange(len(path_points)-1)]

    target_radius = target_radius[1:]+[target_radius[0]]

    for ang1, ang2, p, R in zip(angles[:-1], angles[1:], path_points[:-1], target_radius):
        if R != 0:
            p_sub = _round_corner_points(ang1, ang2, p, R,step_degrees=step_degrees)
        else:
            p_sub = [p]
        out_path += p_sub

    if not path_closed:
        out_path.append(path_points[-1])
    else:
        # close path again
        # note that origin of path has moved (!)
        out_path.append(out_path[0])

    return out_path


def circuit(R_max, R_min, width, corner_radius=5.5):
    while True:
        a = (R_max-R_min)*_random.random()+R_min
        b = (R_max-a)*_random.random()+a
        N = _random.randint(3, 9)
        if N == 4:
            N = 36
        elif N==3:
            return circuit_straight(400,40,width-.2)

        track = _ellipse_points(b,a,N)
        track+=[track[0]]
        #track = _round_corners(track, corner_radius)

        # none of the angles should be <= 90 degrees
        diff = _corner_angles(track, closed=True)
        min_abs = min([abs(d) for d in diff])
        if  min_abs <= _math.pi/2.+1e-10:
            break
        print('right angle or tighter')

    inner, outer = _make_offset_path(track, width / 2.)
    return inner, outer

def _confined_angle_pi(a):
    """Inclusive -pi, exclusive +pi"""
    while a<-_math.pi:
        a+=2*_math.pi
    while a>=_math.pi:
        a-=2*_math.pi
    return a

def circuit_straight(L,W, width):
    track=[(-L/2., -W/2.), (L/2., -W/2.), (L/2., W/2.), (-L/2., W/2.)]
    track+=[track[0]]
    track = _round_corners(track,W/4., step_degrees=40.)
    inner,outer = _make_offset_path(track, width / 2.)
    return inner,outer

def circuit_1(width):
    # mostly works with 3.5 m width
    # to make sure the r5 turn is passable, use a track width of 3.8 m.
    r1=100.
    r2=50.
    r3=80.
    r4=160.
    r5=35. # here the track may have to be wider than 3.5 m at this point
    # NB: the car cannot go through curve radii this small unless the
    #  center steering angle can be 42 degrees (i.e. inner and outer wheel
    #  angles are 51 and 36 degrees, respectively). The minimum road width
    #  would be 3.2 m.
    # Realistically, the max 'center' wheel angle is 35 degrees,
    #  resulting in left and right angles of 42 and 30 degrees, respectively.
    #  At 35 degrees, the minimum curve width is 3.21 m (incl 10 cm either side)
    #    and the minimum curve radius for passage without backing is 4.2 m.
    p=[(400,0,r1),(1470,0,r1),(860,560,r4),(590,350,r2),(640,290,r2),
       (1070,290,r3),(1070,110,r3),(500,110,r1),(475,250,r1),(580,560,r1),
       (640,640,r2),(530,720,r2),(470,630,r5),
       (360,700,r1),(0,200,r3),(120,120,r2),(300,410,r2),(390,370,r3)]
    # NB: the absolute smallest scale factor is 0.12 in order to let a
    #   normal car pass the r5 curve. We aim for a 5 m curve radius which
    #   would allow a center angle of 30 degrees to pass just (and require
    #   a track width of 3.8 m, incl. safety) --> scale
    #scale = .12
    scale = .15

    p=[(u[0]*scale,u[1]*scale,u[2]*scale) for u in p]

    R=[u[2] for u in p]
    track=[u[:2] for u in p]
    track+=[track[0]]
    track = _round_corners(track, R)

    inner,outer = _make_offset_path(track, width / 2.)

    return inner,outer


if __name__=='__main__':
    import matplotlib.pyplot as plt

    # vehicle turning radius is 11 m
    #i, o = circuit(20, 11., 3.)
    i, o = circuit_1(3.5)

    if False:
        o=[(0,0),(0,30),(30,30)]
        o=[(0,0),(-30,0),(-30,30)]
        o=[(0,0),(30,0),(30,-30)]
        o=[(0,0),(0,-30),(-30,-30)]

        x,y = tuple(zip(*o))
        plt.plot(x,y, 'r-')

        plt.plot(x[:3],y[:3], 'ro')
        segment1_angle = _math.atan2(y[1]-y[0],x[1]-x[0])
        segment2_angle = _math.atan2(y[2]-y[1],x[2]-x[1])
        intersection_point = x[1],y[1]
        Radius = 8.
        pts, O, P = _round_corner_points(segment1_angle, segment2_angle, intersection_point, Radius, debug=True)
        plt.plot(O[0],O[1], 'ko')
        plt.plot(P[0],P[1], 'gs')
        for p in pts:
            plt.plot(p[0],p[1], 'k.')
        plt.gca().set_aspect(1)
        plt.show()
        sdfds

    x,y = tuple(zip(*o))
    plt.plot(x,y, 'r.-')

    x,y = tuple(zip(*i))
    plt.plot(x,y, 'b-')

    plt.gca().set_aspect(1)
    plt.show()
