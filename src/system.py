import random, math, time

import sys as _sys
if _sys.version_info.major != 2: reload = lambda x: 0
else: range = xrange

# Author: Chris Petrich
# Date: 2016
#   Apr 2018: modified for Python 3

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
import save_load
reload(save_load)

def get_example_boundaries_and_state(params=None):
    p_left, p_right = track_generator.circuit_1(4.)
    left = g._lines_from_points(p_left,points_form_closed_loop=True)
    right = g._lines_from_points(p_right,points_form_closed_loop=True)
    boundaries = left+right

    environment_state = environment_context.EnvironmentState()
    model = process_model.Car({'environment':environment_state})

    p_center = .5*(p_left[0][0]+p_right[0][0]),.5*(p_left[0][1]+p_right[0][1])
    rot = 0.

    p_front_right = process_model._front_right_pos_from_center(model.wheelbase, model.track, p_center, rot)
    state = process_model.ModelState(p_front_right, rot, 0., 0., 0., None)
    state=model.act(state, None)

    return boundaries, state


def get_path(boundaries, start_state, data, max_steps=300):
    environment_state = environment_context.EnvironmentState()
    environment_state.add_domain_lines(boundaries)

    model = process_model.Car({'environment':environment_state})
    observer = observer_system.Observer({'car':model,
                                         'environment':environment_state})
    reward = reward_system.Rewards()
    numerical_dimension_sizes=observer.N_states_for_learner(),observer.N_actions_for_learner()

    q_learner = q_learner_system.Q_learner(*numerical_dimension_sizes, init={'gamma':observer.look_ahead_gamma})
    if data is not None:
        if isinstance(data['Q'], dict):
            q_learner.Q = q_learner_system.DefaultZeroArray(data['Q.shape'])
            q_learner.Q.update( data['Q'] )
        else:
            q_learner.Q = data['Q']
        q_learner.Q.shape = data['Q.shape']

    model_state = start_state
    states,actions,rewards = [],[],[]
    num_states,num_actions,num_rewards=[],[],[]
    epsilon = 0
    while (model_state.status != 'terminated') and (len(states)<max_steps):
        # observer.state_for_learner() is slow
        #  will have to be re-calculated here in case environment changes
        #  due to several agents active
        numerical_state = observer.state_for_learner(model_state)
        numerical_action = q_learner.policy(numerical_state, epsilon=epsilon)
        model_action = observer.action_for_model(model_state, numerical_state, numerical_action)
        new_model_state = model.act(model_state, model_action)
        model_reward = reward.reward(model_state, model_action, new_model_state)
        new_numerical_state = observer.state_for_learner(new_model_state)
        numerical_reward = observer.reward_for_learner(model_reward)

        states.append(model_state)
        num_states.append(numerical_state)
        actions.append(model_action)
        num_actions.append(numerical_action)
        rewards.append(model_reward)
        num_rewards.append(numerical_reward)

        model_state = new_model_state

    states.append(new_model_state)
    num_states.append(new_numerical_state)
    actions.append(None)
    num_actions.append(None)
    rewards.append(None)
    num_rewards.append(None)

    return states, actions, rewards, num_states, num_actions, num_rewards

if __name__=='__main__':
    # domain dimensions etc that are not part of the state
    if True:
        print('generating track lines')
        track_set = []
        for run in range(10000):
            # new track every couple of tries
            # our car could make an n-point turn on roads wider than 4.5 m

            road_width = 3.8+1.2*random.random()
            #p_left, p_right = track_generator.circuit_1(road_width)
            p_left, p_right = track_generator.circuit(35.,15.,road_width)

            left = g._lines_from_points(p_left,points_form_closed_loop=True)
            right = g._lines_from_points(p_right,points_form_closed_loop=True)
            track_lines = left+right

            track_width = ((right[0][0][0]-left[0][0][0])**2+(right[0][0][1]-left[0][0][1])**2)**.5

            p_center = [(.5*(l[0]+r[0]), .5*(l[1]+r[1])) for l,r in zip(p_left, p_right)]
            center = g._lines_from_points(p_center,points_form_closed_loop=True)

            lengths = [((a[0]-b[0])**2+(a[1]-b[1])**2)**.5 for a, b in center]
            track_length = sum(lengths)
            breaks=[0]
            for L in lengths:
                breaks.append(breaks[-1]+L)
            breaks=breaks[1:]

            track_set.append((track_lines, left, right, center, lengths, track_width, track_length, breaks, p_left, p_right, p_center))

        print('done')

        environment_state = environment_context.EnvironmentState()
        model = process_model.Car({'environment':environment_state})
        observer = observer_system.Observer({'car':model,
                                             'environment':environment_state})
        reward = reward_system.Rewards()
        numerical_dimension_sizes=observer.N_states_for_learner(),observer.N_actions_for_learner()

        q_learner = q_learner_system.Q_learner(*numerical_dimension_sizes, init={'gamma':observer.look_ahead_gamma})


    simulation_last_saved = time.time()

    # car tries to maximize points for limited period of time
    #   goal of this execise is "survival"
    max_test_time = 300

    save_at = 1000
    count = 0
    visits = 0
    avg_length = 100.
    while True:
        if count % 20 == 0:
            idx = random.randint(0,len(track_set)-1) # NB: numpy is last one exclusive
            track_lines, left, right, center, lengths, track_width, track_length, breaks, p_left, p_right, p_center = track_set[idx]

            environment_state.init_state(None)
            environment_state.add_domain_lines(track_lines)

        while True:
            # find a random place along the center of the track
            start = track_length * random.random()
            line_segment_idx = max([idx for idx, brk in enumerate([0]+breaks) if brk<=start])
            offset = start - ([0]+breaks)[line_segment_idx]
            t = offset / lengths[line_segment_idx]
            l = center[line_segment_idx]
            x0 = t*(l[1][0]- l[0][0])+l[0][0]
            y0 = t*(l[1][1]- l[0][1])+l[0][1]

            d = .2*(track_width-model.body_width)/2.
            dx, dy = 2*d*random.random()-d, 2*d*random.random()-d
            rot = 2*math.pi*random.random()-math.pi
            p_center = (x0+dx, y0+dy)

            wheel_dir = random.choice(observer.wheel_positions)
            if random.random()<.01:
                # this will often lead to problems, in particular if
                #   speed is high. --> minimize their occurence but
                #   make sure these states are visited
                car_speed = random.choice(observer.distance_steps)
            else:
                car_speed = 0.

            fits = False
            for r_test in range(8):
                rot += math.pi/2.*1.0189 # rotate by 91.7 degrees on each attempt
                p_front_right = process_model._front_right_pos_from_center(model.wheelbase, model.track, p_center, rot)
                model_state = process_model.ModelState(p_front_right, rot, wheel_dir, car_speed, 0., None)
                # todo: test if car intersects the center line.
                #  i.e. is on track

                model_state=model.act(model_state, None)

                if model_state.status != 'terminated':
                    fits = True
                    break
            if fits: break

        start_model_state = model_state

        count += 1
        # generic loop
        track = [model_state]
        alpha = 1./(10.+5e-6*count)  # use starting value of 0.1, then start decreasing beyond 20,000 runs
        epsilon = 1./avg_length
        while (model_state.status != 'terminated') and (len(track)<max_test_time):
            # observer.state_for_learner() is slow
            #  will have to be re-calculated here in case environment changes
            #  due to several agents active
            numerical_state = observer.state_for_learner(model_state)
            numerical_action = q_learner.policy(numerical_state, epsilon=epsilon)
            model_action = observer.action_for_model(model_state, numerical_state, numerical_action)
            new_model_state = model.act(model_state, model_action)
            model_reward = reward.reward(model_state, model_action, new_model_state)
            # observer.state_for_learner() is slow
            new_numerical_state = observer.state_for_learner(new_model_state)
            numerical_reward = observer.reward_for_learner(model_reward)
            q_learner.learn(numerical_state, numerical_action, numerical_reward, new_numerical_state, alpha=alpha)
            model_state = new_model_state

            visits += 1
            track.append(model_state)

        avg_length = .9*avg_length + .1*len(track)

        q_learner.learn(alpha=alpha) # indicate end of a logical set
        if count % 10 == 0:
            print(count, '*' if len(track)==max_test_time else '.', len(track), '%.1f' % sum([s.distance_step for s in track]), '%.3f' % alpha, visits)

        if (count==100) or (count % save_at == 0) or (time.time()-simulation_last_saved>=3600*6):
            simulation_last_saved = time.time() # save at least once every 6 hours
            out={'type':None,'Q':q_learner.Q,'Q.shape':q_learner.Q.shape,
                 'runs':count,'steps':visits}
            if True:
                import os
                if True and isinstance(q_learner.Q,q_learner_system.DefaultZeroArray):
                    # store data as ordinary dict so we can unpickle it without importing c_q_learner first
                    d = dict()
                    d.update(q_learner.Q)
                    out['Q']=d
                    out['type']=3
                save_load.save('learned_result_%i.py'%count, os.path.basename(__file__), out)
                d=None
            else:
                if isinstance(q_learner.Q,q_learner_system.DefaultZeroArray):
                    import cPickle, zipfile
                    out['type']=2
                    with zipfile.ZipFile('learned_result_%i.pickle.zip'%count,'w',zipfile.ZIP_DEFLATED) as f:
                        f.writestr('data.pickle',cPickle.dumps(out,-1))
                else:
                    import numpy as np
                    out['type']=1
                    np.save('learned_result_%i.npy'%count,out)

        if count > save_at*10:
            save_at *= 10
