import matplotlib.pyplot as plt
import matplotlib.patches as mp
from matplotlib import animation
import numpy as np
import glob, random, math, os
import matplotlib
#matplotlib.rc('text', usetex=True)

import sys as _sys
if _sys.version_info.major != 2:
    reload, xrange = lambda x: 0, range
    def zip(*argv):
        return list(__builtins__.zip(*argv))

# Author: Chris Petrich
# Date: 2016
#   Apr 2018: modified for Python 3
#   Apr 2018: modified for matplotlib 2

do_write_mp4 = False # works with Py 2.7 & matplotlib 1 but not Py 3.6 & matplotlib 2. TODO: Need to figure out why.

def draw_domain(ax, p_left, p_right):
    if not hasattr(ax,'keepers'): ax.keepers = set([])

    x,y = zip(*(p_left+p_right[-1::-1]))
    patches = ax.fill(x,y, color=(.8,.8,.8),zorder=1)
    for patch in patches:
        ax.keepers.add( patch )

    x,y = zip(*p_left)
    ax.keepers.add( ax.plot(x,y,'k-',zorder=2)[0] )
    x,y = zip(*p_right)
    ax.keepers.add( ax.plot(x,y,'k-',zorder=2)[0] )


def draw_car(ax, outline, wheels):

    car_base=mp.Polygon(outline, closed=True,
               fill=True,facecolor=(1,1,1),edgecolor='k',lw=.5,zorder=5)
    ax.add_patch(car_base)

    car=mp.Polygon(outline, closed=True,
               fill=True,facecolor=(1,.3,.2),edgecolor='k',alpha=.5,zorder=7)
    ax.add_patch(car)

    for wheel in wheels:
        tyre = mp.Polygon(wheel, closed=True,
               fill=True,facecolor=(.3,.3,.3),edgecolor='k',lw=.5,zorder=6)
        ax.add_patch(tyre)

def get_track(state, max_steps=300, with_rewards=True):
    actions=[]
    num_actions=[]
    states = []
    num_states = []
    rewards = []
    while (state.status!='terminated') and ((max_steps is None) or (len(states)<max_steps)):
        numerical_state = observer.state_for_learner(state)
        numerical_action = q_learner.policy(numerical_state, epsilon=policy_epsilon)
        action = observer.action_for_model(state, numerical_state, numerical_action)

        states.append(state)
        actions.append(action)
        num_states.append(numerical_state)
        num_actions.append(numerical_action)

        new_state = car.act(state, action)
        if with_rewards:
            reward_token = reward.reward(state, action, new_state)
            score = observer.reward_for_learner(reward_token)
            rewards.append(score)

        state=new_state

    # add the state where the termination condition occurred
    states.append(state)
    num_states.append(observer.state_for_learner(state))
    actions.append(None)
    num_actions.append(None)
    rewards.append(float('nan'))

    return np.array(states), np.array(actions), np.array(rewards), np.array(num_states), np.array(num_actions)

def plot_car(ax,car,state):
    import a1_process_model as process_model
    reload(process_model)

    car_points = process_model.chassis_outline(state.p_front_right,state.axis_dir,
                                      car.wheelbase, car.track,
                                      car.overhang_front, car.overhang_rear,
                                      car.body_width)
    wheel_points = process_model.wheel_outlines(state.p_front_right,state.axis_dir,state.center_wheel_dir,
                                    car.wheelbase, car.track)

    draw_car(ax, car_points, wheel_points)

def plot_num_state(ax, car, state, num_state, num_action = None):
    import a1_process_model as process_model
    reload(process_model)
    _g = g

    num_state=tuple(num_state)
    rot = math.pi/2.

    scale_model = 3

    s=scale_model
    wheelbase = s*car.wheelbase
    track = s*car.track
    overhang_front = s*car.overhang_front
    overhang_rear = s*car.overhang_rear
    body_width = s*car.body_width
    wheel_size = '195/55R16'
    wheel_size = '%.0f/%.0fR%.0f' % (195*s, 55, 32*s)

    x0,y0=160,72. # get away from race track
    P0 = (x0+.5*track,y0+.5*wheelbase)

    dir_front_center = state.center_wheel_dir
    speed = state.distance_step

    car_points = process_model.chassis_outline(P0,rot,
                                      wheelbase, track,
                                      overhang_front, overhang_rear,
                                      body_width)
    wheel_points = process_model.wheel_outlines(P0,rot,dir_front_center,
                                    wheelbase, track, wheel_size = wheel_size)

    draw_car(ax, car_points, wheel_points)

    corners = car_points
    _c = lambda i,j: (.5*(corners[i][0]+corners[j][0]), .5*(corners[i][1]+corners[j][1]))
    zone_points = [_c(0,0),_c(0,1),_c(1,1),_c(2,2),_c(2,3),_c(3,3),_c(3,0)]

    car_points = None # as a debug flag
    car_outline = None

    car_model_outline = g._lines_from_points(zone_points)
    car_model_outline = car_model_outline[-2:]+car_model_outline[:-2]

    speed_idx, wheel_idx, front_l_idx, front_r_idx, left_f_idx, left_b_idx, right_f_idx, right_b_idx, back_idx =num_state

    if True:
        e=zone_points
        avg=lambda a,b: (.5*(a[0]+b[0]),.5*(a[1]+b[1]))
        ax.annotate('%i'%front_l_idx, avg(e[-2],e[-1]),(0,3),
                    textcoords='offset points',ha='center',va='bottom')
        ax.annotate('%i'%front_r_idx, avg(e[-1],e[0]),(0,3),
                    textcoords='offset points',ha='center',va='bottom')

        ax.annotate('%i'%right_f_idx, avg(e[0],e[1]),(3,0),
                    textcoords='offset points',va='center',ha='left')
        ax.annotate('%i'%right_b_idx, avg(e[1],e[2]),(3,0),
                    textcoords='offset points',va='center',ha='left')

        ax.annotate('%i'%back_idx, avg(e[2],e[3]),(0,-3),
                    textcoords='offset points',ha='center',va='top')

        ax.annotate('%i'%left_b_idx, avg(e[3],e[4]),(-3,0),
                    textcoords='offset points',va='center',ha='right')
        ax.annotate('%i'%left_f_idx, avg(e[4],e[5]),(-3,0),
                    textcoords='offset points',va='center',ha='right')

        ax.annotate('%i'%wheel_idx, avg(e[-2],e[0]),(0,-10),
                    textcoords='offset points',ha='center',va='top',zorder=10)

        if True:
            add=lambda a,b: (a[0]+b[0],a[1]+b[1])
            # show speed
            speed_up = 1. # reciprocal of time step (s) --> 1/s
            unit = 1. # 3.6 for km/h, 1. for m/s
            # speed is mostly determined by how fast one can reasonably turn
            #  the wheel: half a turn per second is probably the most (i.e. 10 deg)
            speed = ('% 4.1f' % (round((state.distance_step*speed_up*unit)/.1,0)*.1)).replace(' ',u'\u00a0').replace('-',u'\u2013')
            p=add(e[0],(1.5*scale_model,.25*scale_model))
            ax.text(p[0],p[1],
                    speed, color='k',
                    ha='center',va='bottom',
                    bbox=dict(facecolor='none', edgecolor='k', boxstyle='round,pad=.2'))

            ax.annotate('%i'%speed_idx, p,(0,-5),
                    textcoords='offset points',ha='center',va='top',zorder=10)
        else:
            ax.annotate('%i'%speed_idx, avg(avg(e[-2],e[0]),avg(e[2],e[3])),(0,0),
                        textcoords='offset points',ha='center',va='center',zorder=10)


    if True:
        if num_action is None:
            policy = q_learner.policy(num_state, epsilon=policy_epsilon)
        else:
            policy = num_action
        action_values = q_learner.Q[num_state]
        dirs, accs=[],[]
        ID = []
        for num_pol in xrange(len(action_values)):
            d,a = observer._actions[num_pol]
            dirs.append('<' if d>=1 else '>' if d<=-1 else '|')
            accs.append('+' if a>=1 else u'\u2013' if a<=-1 else '.')
            ID.append(num_pol)

        tab = sorted(zip(action_values, dirs, accs, ID), key=lambda x: x[0])[-1::-1]
        text = '\n'.join([u'\u00a0 %s %s % 5.0f' % (v[2],v[1],v[0]) if v[3]!=policy else
                          '* %s %s % 5.0f' % (v[2],v[1],v[0]) for v in tab])

        ax.annotate(text, (180,65), (0,0),
                    xycoords='data', textcoords='offset points',
                    ha='left', va='bottom',family='monospace',
                    bbox=dict(fc=".9"))
    else:
        # drawing a table is SUPER-SLOW compared to using annotate()
        if num_action is None:
            policy = q_learner.policy(num_state, epsilon=policy_epsilon)
        else:
            policy = num_action
        action_values = q_learner.Q[num_state]
        dirs, accs=[],[]
        ID = []
        for num_pol in xrange(len(action_values)):
            d,a = observer._actions[num_pol]
            dirs.append('<' if d>=1 else '>' if d<=-1 else '|')
            accs.append('+' if a>=1 else '-' if a<=-1 else 'o')
            ID.append(num_pol)
        tab = sorted(zip(action_values, dirs, accs, ID), key=lambda x: x[0])[-1::-1]
        cell_text = [(' ','%s'%v[2],'%s'%v[1],('% 5.0f'%v[0]).replace(' ',u'\u00a0')) if v[3]!=policy else
                     ('*','%s'%v[2],'%s'%v[1],('% 5.0f'%v[0]).replace(' ',u'\u00a0')) for v in tab]

        w=.01
        tab=ax.table(cellText=cell_text, loc='right',
                     colWidths=[w,w,w,5*w],cellLoc='center')
        tab.auto_set_font_size(False)
        tab.set_fontsize(12)
        for key, cell in tab.get_celld().items():
            cell.set_linewidth(0)

    if True:
        # calculate distance to the sides of the car
        corners=process_model.chassis_outline(state.p_front_right,state.axis_dir,
                                         car.wheelbase,car.track,
                                         car.overhang_front, car.overhang_rear,
                                         car.body_width)
        _c = lambda i,j: (.5*(corners[i][0]+corners[j][0]), .5*(corners[i][1]+corners[j][1]))
        zone_points = [_c(0,0),_c(0,1),_c(1,1),_c(2,2),_c(2,3),_c(3,3),_c(3,0)]

        # counter-clockwise outline: Front, Right, Back, Left
        car_outline = _g._lines_from_points(zone_points)
        car_outline = car_outline[-2:]+car_outline[:-2]
        car_outline_h = _g._lines_to_homogenous(car_outline)

        if True:
            # illustrate clear zones
            for dir_idx, zone_idx in enumerate((front_l_idx,front_r_idx,right_f_idx,right_b_idx,back_idx,left_b_idx,left_f_idx)):
                more_than = observer.zone_distances[dir_idx][zone_idx-1] if zone_idx>0 else 0.
                draw_zone(ax, car_outline[dir_idx], more_than)
                draw_zone(ax, car_model_outline[dir_idx], more_than*scale_model,zorder=0)


def draw_zone(ax, edge_points, distance_to_right, zorder=3):
    rotate=lambda pt, a: (math.cos(a)*pt[0]+ -math.sin(a)*pt[1],
                    math.sin(a)*pt[0]+  math.cos(a)*pt[1])
    add=lambda p0, p1: (p0[0]+p1[0], p0[1]+p1[1])

    P0, P1 = edge_points
    L = ((P0[0]-P1[0])**2+(P0[1]-P1[1])**2)**.5
    d = distance_to_right
    angle = math.atan2(P1[1]-P0[1],P1[0]-P0[0])
    base = ((0,0), (L,0), (L,d), (0,d))
    outline = [add(rotate(o, angle),P0) for o in base]

    zone=mp.Polygon(outline, closed=True,
               fill=True,facecolor=(1,.9,.5),edgecolor='k',lw=.25,zorder=zorder)

    ax.add_patch(zone)



def line_from_hom(h,x0=-15,x1=15):
    a,b,c = h
    if abs(b)>=1.:
        x = np.linspace(x0,x1)
        y = (-c-a*x)/b
    else:
        y = np.linspace(x0,x1)
        x = (-c-b*y)/a
    return x,y


def update_plot(index, ax, car, params):
    print(index)

    for a in ax.lines[:]: # iterate over copy of list
        if a not in ax.keepers: a.remove() # .remove() modifies the list
    for a in ax.patches[:]:
        if a not in ax.keepers: a.remove()
    for a in ax.texts[:]:
        if a not in ax.keepers: a.remove()

    states, actions, reward_scores, num_states, num_actions = params

    reward_score = reward_scores[index] if len(reward_scores)>0 else 0.

    plot_car(ax, car, states[index])
    plot_num_state(ax, car, states[index],num_states[index],num_action=num_actions[index])


    tag = []
    tag.append('Frame: %i' % index)
    if len(reward_scores)>0:
        reward_score = reward_scores[index]
        tag.append('Reward: %.0f' % reward_score)

    distance = [v.distance_step if v.distance_step==v.distance_step else 0. for v in states[:index+1]]
    tag.append('Distance: %.0f m' % sum(distance))
    if False:
        ax.annotate('\n'.join(tag), (1,1),(-4,-4),
                    xycoords='axes fraction',textcoords='offset points',
                    fontsize=13, ha='right', va='top')
    elif True:
        ax.annotate('\n'.join(tag), (165,110),(-4,-4),
                    xycoords='data',textcoords='offset points',
                    fontsize=13, ha='left', va='top')

    ll, all_lines = [], []
    for l1, l2, p in ll:
        l=l1
        ax.plot([l[0][0],l[1][0]],[l[0][1],l[1][1]],'b-',lw=3,zorder=10)
        l=l2
        ax.plot([l[0][0],l[1][0]],[l[0][1],l[1][1]],'r-',lw=3,zorder=3)
        ax.plot(p[0],p[1],'ko',lw=3,zorder=10)

    for l1, l2 in all_lines:
        h1,h2=g._lines_to_homogenous([l1,l2])
        x,y=line_from_hom(h1)
        ax.plot(x,y,'b-',zorder=10)
        x,y=line_from_hom(h2)
        ax.plot(x,y,'r-',zorder=3)


def find_intersecting_lines(state):
    line_set_1 = reward._car_outline_from_state(car, state)
    h_set_1 = g._lines_to_homogenous(line_set_1)

    line_set_2 = reward.parking_boundaries
    h_set_2 = reward._parking_boundaries_h

    intersect=[]

    for i1, (h1, l1) in enumerate(zip(h_set_1, line_set_1)):
        for i2, (h2, l2) in enumerate(zip(h_set_2, line_set_2)):
            P = reward_system._intersection(h1, h2)
            if P==(None, None): continue
            if reward_system._point_within_bounds(l1,P) and reward_system._point_within_bounds(l2,P):
                intersect.append((l1, l2,P))
    return intersect, zip(line_set_1, line_set_2)


def plot_state(p_left, p_right, car, state):
    fig = plt.figure()
    ax= fig.add_subplot(111)

    draw_domain(ax, p_left, p_right)

    ax.set_xlim(-35,35)
    ax.set_ylim(-35,35)
    ax.set_aspect(1)
    ax.set_facecolor((.97,1,.95))

    plot_car(ax, car, state)

    return fig, ax

if __name__=='__main__':

    if True:
        import a1_process_model as process_model
        reload(process_model)
        import a2_reward_system as reward_system
        reload(reward_system)
        import a3_environment_context as environment_context
        reload(environment_context)
        import b_observer as observer_system
        reload(observer_system)
        import c_q_learner as q_learner_system
        reload(q_learner_system)
        import track_generator
        reload(track_generator)
        import geometry as g
        reload(g)

        p_left, p_right = track_generator.circuit(30., 20., 4.)

        left = g._lines_from_points(p_left,points_form_closed_loop=True)
        right = g._lines_from_points(p_right,points_form_closed_loop=True)
        track_lines = left+right

        environment = environment_context.EnvironmentState()

        car = process_model.Car({'environment':environment})
        reward = reward_system.Rewards()
        observer = observer_system.Observer({'car':car,
                                             'environment':environment})
        numerical_dimension_sizes=observer.N_states_for_learner(),observer.N_actions_for_learner()
        q_learner = q_learner_system.Q_learner(*numerical_dimension_sizes, init={'gamma':observer.look_ahead_gamma})

    if True:
        fns = glob.glob('learned_result_*.npy')
        fns=[]
        if len(fns)==0:
            fns = glob.glob('learned_result_*.pickle.zip')
            if len(fns)==0:
                #fns = glob.glob('learned_result_*.py')
                fns = glob.glob('learned_result_*.*')
        fn = sorted(fns,key=lambda x: int(x.split('_')[-1].split('.')[0]))[-1]
        print('loading',fn)
        if fn.endswith('.npy'):
            data = np.load(fn).item()
        elif fn.endswith('.pickle.zip'):
            import zipfile, cPickle
            with zipfile.ZipFile(fn,'r') as f:
                data = cPickle.loads(f.read('data.pickle'))
        else:
            import save_load
            reload(save_load)
            data=save_load.load(fn)['data']

        if 'type' not in data: data['type'] = 3

        if data['type']==1:
            Q=data['Q']
        elif data['type']==2:
            Q=data['Q']
            Q.shape=data['Q.shape']
        elif data['type']==3:
            if isinstance(data['Q'], dict):
                # q_learner.Q is a DefaultZeroArray --> just push data without overwriting the instance type
                Q = q_learner_system.DefaultZeroArray(data['Q.shape'])
                Q.update( data['Q'] )
            else:
                Q=data['Q']
            Q.shape=data['Q.shape']
        elif data['type'] is None:
            Q=data['Q']
            Q.shape=data['Q.shape']

    try:

        policy_epsilon = -1

        while True:
            if False:
                # upscaling the track shows how important it is
                #   to train on tracks with relevant dimensions
                track_scale = 3.
                track_width_scale = 2.
            else:
                track_scale = track_width_scale = 1.

            p_left, p_right = track_generator.circuit_1(4.*track_width_scale)
            p_left = [(p[0]*track_scale, p[1]*track_scale) for p in p_left]
            p_right = [(p[0]*track_scale, p[1]*track_scale) for p in p_right]
            #p_left, p_right = track_generator.circuit_straight(400,40,2*4.)
            max_test_time = 2000

            #p_left=[(19.54023585232318, 17.829119764252358), (0.0, 25.288403124504686), (-19.540235852323175, 17.82911976425236), (-27.553532635790525, -0.0), (-19.540235852323182, -17.829119764252358), (-4.653319886562684e-15, -25.288403124504683), (19.54023585232317, -17.829119764252358), (27.55353263579052, -0.0), (19.54023585232318, 17.829119764252358)]
            #p_right = [(22.139696257097444, 20.583151938363695), (0.0, 29.034752478895204), (-22.13969625709744, 20.5831519383637), (-31.390792632141956, 3.952290759910167e-15), (-22.139696257097444, -20.583151938363695), (-4.6533198865626834e-15, -29.034752478895204), (22.13969625709744, -20.583151938363695), (31.390792632141956, -0.0), (22.139696257097444, 20.583151938363695)]

            left = g._lines_from_points(p_left,points_form_closed_loop=True)
            right = g._lines_from_points(p_right,points_form_closed_loop=True)
            track_lines = left+right


            environment.init_state(None)
            environment.add_domain_lines(track_lines)

            q_learner.Q=Q

            # NB: the track is always wider in the corners, so this is an upper limit
            track_width = ((right[0][0][0]-left[0][0][0])**2+(right[0][0][1]-left[0][0][1])**2)**.5

            p_center = [(.5*(l[0]+r[0]), .5*(l[1]+r[1])) for l,r in zip(p_left, p_right)]
            center = g._lines_from_points(p_center,points_form_closed_loop=True)

            lengths = [((a[0]-b[0])**2+(a[1]-b[1])**2)**.5 for a, b in center]
            track_length = sum(lengths)
            print('track length: %.0f m' % track_length)
            breaks=[0]
            for L in lengths:
                breaks.append(breaks[-1]+L)
            breaks=breaks[1:]

            print('finding state')
            while True:
                print('.')
                while True:
                    # find a random place along the center of the track
                    start = track_length * random.random()
                    line_segment_idx = max([idx for idx, brk in enumerate([0]+breaks) if brk<=start])
                    offset = start - ([0]+breaks)[line_segment_idx]
                    t = offset / lengths[line_segment_idx]
                    l = center[line_segment_idx]
                    x0 = t*(l[1][0]- l[0][0])+l[0][0]
                    y0 = t*(l[1][1]- l[0][1])+l[0][1]

                    d = .2*(track_width-car.body_width)/2.
                    dx, dy = 2*d*random.random()-d, 2*d*random.random()-d
                    rot = 2*math.pi*random.random()-math.pi
                    X,Y=x0+dx, y0+dy

                    model_state = process_model.ModelState((X,Y), rot, 0., 0., 0., None)
                    model_state = car.act(model_state, None)
                    if model_state.status != 'terminated':
                        break

                initial_state = model_state
                t_states, t_actions, t_reward_scores, t_num_states, t_num_actions = get_track(initial_state,
                                                                                              max_steps=max_test_time,
                                                                                              with_rewards=True)
                if len(t_states)>0:
                    this_track_length = sum([s.distance_step for s in t_states])
                    print(len(t_states), this_track_length)

                if abs(this_track_length)>20.:
                    break


            states, actions = t_states, t_actions
            reward_scores = t_reward_scores
            num_states, num_actions = t_num_states, t_num_actions
            print(len(states))

            if True:
                fig = plt.figure(figsize=(16.,9.2))
                ax= fig.add_subplot(111)

                draw_domain(ax, p_left, p_right)

                ax.set_xlim(0,200)
                ax.set_ylim(-5,110)
                ax.set_aspect(1)
                ax.set_position([.025,.02,.96,.97])
                ax.set_facecolor((.97,1,.95))


            if True:
                print('animating')
                ani = animation.FuncAnimation(fig,
                                                  update_plot,
                                                  fargs=(ax,car,[states,actions,reward_scores, num_states, num_actions]),
                                                  frames=len(states) -1,
                                                  blit=False,
                                                  interval=100,
                                                  repeat=True,
                                                  repeat_delay=1000,
                                                  )
                if do_write_mp4:
                    # TODO: fixme: this doesn't work with python 3.6 & matplotlib 2
                    ani_fns=sorted(glob.glob('ani_*.mp4'),key=lambda x: int(os.path.basename(x).split('_')[1].split('.')[0]),reverse=True)
                    if len(ani_fns)>0:
                        latest_idx = int(os.path.basename(ani_fns[0]).split('_')[1].split('.')[0])
                    else:
                        latest_idx = 0
                    fn_ani = 'ani_%i.mp4' % (latest_idx+1)
                    print('saving animation %s' % fn_ani)
                    import datetime
                    # all metadata, except "Lyrics" have a 255 byte limit (UTF-8)
                    ani.save(fn_ani,fps=10,bitrate=None,dpi=100,
                         extra_args=['-vcodec','libx264'], # automatically saves as "cartoon" in a very small file
                         metadata={'Copyright':'Chris Petrich',
                                   'Year': datetime.datetime.now().strftime('%Y-%m-%d'),
                                   'Lyrics': '%r' % {
                                   'Model initial state': '%r' % initial_state,
                                   'Track left': '%r' % p_left,
                                   'Track right': '%r' % p_right}})
                else:
                    plt.show()
                plt.close('all')

    except KeyboardInterrupt:
        plt.close('all')
