import math as _math

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
    # NB: renamed from '_intersection'
    eps = 1e-13
    a,b,c=homog_line_0
    u,v,w=homog_line_1
    D=float(b*u-v*a)
    if abs(D)<eps:
        # parallel lines
        return None, None
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

def _do_intersect(line_points_0, line_points_1):
    """function name was: _intersect"""
    (A,B),(C,D) = line_points_0,line_points_1
    h1 = _homogenous_line(A,B)
    h2 = _homogenous_line(C,D)
    P = _intersection_homogenous(h1, h2)
    if P==(None, None): return False # exactly parallel
    return (_point_within_bounds(line_points_0,P) and _point_within_bounds(line_points_1,P))

def _point_within_bounds(bounds, p):
    """Check if point is within bounds, end points inclusive"""
    A, B = bounds
    # we have to add epsilon since test against horizontal or vertical
    #  lines may fail if the point is off by numerical precision
    eps = 1e-10
    (Ax,Ay), (Bx,By), (px,py)=A,B,p
    return (
        (min((Ax,Bx))-eps<=px<=max((Ax,Bx))+eps) and
        (min((Ay,By))-eps<=py<=max((Ay,By))+eps)
        )

def _any_intersect(line_set_1, line_set_2, h_set_1=None, h_set_2=None):
    """Does any of the lines in lines_ref intersect any of the lines in lines_test?"""
    # try to speed this one up by reducing the number of comparisons
    #  necessary. About half the time is spent in here, incl. subroutines.
    h_set_1 = [_homogenous_line(*segment) for segment in line_set_1] if h_set_1 is None else h_set_1
    h_set_2 = [_homogenous_line(*segment) for segment in line_set_2] if h_set_2 is None else h_set_2
    for h1, l1 in zip(h_set_1, line_set_1):
        for h2, l2 in zip(h_set_2, line_set_2):
            P = _intersection_homogenous(h1, h2)
            if P==(None, None): continue
            if _point_within_bounds(l1,P) and _point_within_bounds(l2,P):
                return True
    return False

def _lines_to_homogenous(line_set):
    return [_homogenous_line(*segment) for segment in line_set]

def _lines_from_points(points, points_form_closed_loop=False):
    """Make line segments of closed polygon defined by points"""
    if points_form_closed_loop:
        return list(zip(points[:-1],points[1:]))
    return list(zip(points, tuple(points[1:])+(points[0],)))

def _point_in_bbox(p, bbox):
    return (bbox[0]<=p[0]<=bbox[2]) and (bbox[1]<=p[1]<=bbox[3])

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

def _distance2_point_to_h_line(point, h_line):
    """Return distance from point to infinite line and
       location of closest proximity"""
    a,b,c = h_line
    x0,y0 = point
    # solve for equality
    # r^2 = (x-x0)^2 + (y-y0)^2
    # ax + by + c = 0
    # --> 2nd order polynomial
    # --> find place of exactly one solution, i.e.
    #     radicant of p-q formula is identical zero
    # if radicant is zero, then
    ys = ((a*x0-c)*b + a**2*y0)/(a**2+b**2)
    # or
    xs = ((b*y0-c)*a + b**2*x0)/(a**2+b**2)
    # for a != 0
    if abs(a)>=abs(b):
        R2 = (x0-c/a)**2+y0**2 - (1.+(b/a)**2)*ys**2
    else:
        R2 = (y0-c/b)**2+x0**2 - (1.+(a/b)**2)*xs**2
    R2 = R2 if abs(R2)>1e-13 else 0.
    return R2, (xs, ys)

def _distance2_line_endpoints(line1, line2):
    """Find the minimum distance of the line endpoints from each other,
       and corresponding points of lines 1 and 2, respectively."""
    (A,B),(C,D) = line1, line2
    R2=lambda u,v: (u[0]-v[0])**2+(u[1]-v[1])**2
    pairs = zip((A,A,B,B),(C,D,C,D))
    r2 = [R2(pair[0],pair[1]) for pair in pairs]
    mini=sorted(zip(r2,pairs),key=lambda a,b: a)[0]
    #R2_min = min((R2(A,C), R2(A,D), R2(B,C), R2(B,D)))
    return  mini[0], mini[1][0], mini[1][1]

def _intersection_forward_line_segment(semiinf_line1, line2, semiinf_h_line1=None, h_line2=None):
    """Where does one line segment intersect a semi-infinite line "segment"?
       At what distance from the origin of the semi-infinite line?
       RETURNS DISTANCE SQUARED or None"""

    semiinf_h_line1 = _homogenous_line(*semiinf_line1) if semiinf_h_line1 is None else semiinf_h_line1
    h_line2 = _homogenous_line(*line2) if h_line2 is None else h_line2

    P = _intersection_homogenous(semiinf_h_line1, h_line2)
    if not _point_within_bounds(line2,P):
        # semi-infinite line does not intersect the particular SEGMENT of line2
        return None, P

    A,B = semiinf_line1
    if abs(B[1]-A[1])>=abs(B[0]-A[0]):
        t = (P[1]-A[1])/(B[1]-A[1])
    else:
        t = (P[0]-A[0])/(B[0]-A[0])

    if t>0: # intersection lies behind A, i.e. toward or beyond B
        return None, P

    return (P[0]-A[0])**2+(P[1]-A[1])**2, P

def _distance2_line_segments(line1, line2, h_line1=None, h_line2=None):
    """How close does one line segment approach another? And which line
       segement or points are affected?
       Note: returns distance SQUARED."""
    h_line1 = _homogenous_line(*line1) if h_line1 is None else h_line1
    h_line2 = _homogenous_line(*line2) if h_line2 is None else h_line2

    r_11 = _distance2_point_to_h_line(line1[0], h_line2), line2
    r_12 = _distance2_point_to_h_line(line1[1], h_line2), line2
    r_21 = _distance2_point_to_h_line(line2[0], h_line1), line1
    r_22 = _distance2_point_to_h_line(line2[1], h_line1), line1

    tests = sorted((r_11,r_12,r_21,r_22), key=lambda x: x[0][0])
    # check for validity starting with the closest point
    for (r2, ps), line in tests:
        if _point_within_bounds(line,ps):
            return r2, ps, line #0 if line==line1 else 1

    # none of the corner points is close to any of the line
    # --> line separation is simply the closest distance of
    #     corner points

    r2, p1, p2 = _distance2_line_endpoints(line1, line2)

    return r2, p1, p2

def _distance2_outline_from_lines(outline, lines, h_outline=None, h_lines=None):
    """Return distance squared and approach information for each line in 'outline'"""
    h_outline = [_homogenous_line(*segment) for segment in outline] if h_outline is None else h_outline
    h_lines = [_homogenous_line(*segment) for segment in lines] if h_lines is None else h_lines

    approaches=[]
    for out_idx, (out, h_out) in enumerate(zip(outline, h_outline)):
        data = [(_distance2_line_segments(out, line, h_out, h_line),line_idx) for line_idx, (line, h_line) in enumerate(zip(lines, h_lines))]
        closest = sorted(data, key=lambda x, idx: x[0])[0]
        # distance2, line segement index involved, point involved, line/other point involved
        approaches.append((closest[0][0], closest[1], closest[0][1], closest[0][2]))

    return approaches

