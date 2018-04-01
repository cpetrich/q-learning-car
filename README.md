# Q-Learning a Car to Drive on a Narrow Track

This is the code behind https://youtu.be/o7VYaKHq9jw, *Q-learning: car driving narrow track (4)*.
[![Model building blocks](/images/screenshot_480px.png)](https://youtu.be/o7VYaKHq9jw)

The code is provided as per request in the comments section.

## System requirements
  * **Simulation:** Python 2.7 or Python 3. Use of [PyPy](https://pypy.org/) or [PyPy3](https://pypy.org/) implementation is strongly recommended. No external dependencies.
  * **Plotting:** Python 2.7 or Python 3, numpy, matplotlib.
    *(known limitation: it appears that generating mp4 files of the animation may not work reliably under Python 3.6 with matplotlib 2.x. It seems to work fine under Python 2.7 with matplotlib 1.x.)*

## Overview
A Q-learner has been implemented in Python with the Q-function stored in a table. The problem has been split into three elements:
  1. A physical model of the world
  2. An interface layer between the world and the numerical Q-learner
  3. The numerical Q-learner

The flow of information is illustrated in this diagram:
![Model building blocks](/images/RL_system_160713.png)

The individual building blocks of the **simulation** are implemented in the following modules:
  * **Current Model State:** Part of the main loop in `system.py`
  * **Process Model:** `a1_process_model.py`
  * **Reward Function:** `a2_reward_system.py`
  * **Environment State:** `a3_environment_context.py`
  * **Observer/Actor:** `b_observer.py`
  * **Q-learner:** `c_q_learner.py`

In addition,
  * **Track generator:** Routines for generating the train tracks (**simulation**) and the test track (**plotting**) are in `track_generator.py`.
  * **Helper functions:** for the physical model of the world `geometry.py`
  * **Input/Output** `save_load.py` imports or exports learned data as `json`

Exported simulations can be visualized with
  * **Plotting** `show.py`

A learned model is provided as `data/learned_result_11040091.json` (based on 11,040,091 iterations), which has been used to generate `data/sample_run_11040091.mp4` (i.e. the same simulation seen on youtube).

### Program Flow
The main loop in `system.py` is approximately

```python
model_state = process_model.ModelState(position, rotation, wheel_direction, car_speed, 0., None)
while model_state.status != 'terminated':
    numerical_state = observer.state_for_learner(model_state)
    numerical_action = q_learner.policy(numerical_state, epsilon=epsilon)
    model_action = observer.action_for_model(model_state, numerical_state, numerical_action)
    new_model_state = model.act(model_state, model_action)
    model_reward = reward.reward(model_state, model_action, new_model_state)
    new_numerical_state = observer.state_for_learner(new_model_state)
    numerical_reward = observer.reward_for_learner(model_reward)
    q_learner.learn(numerical_state, numerical_action, numerical_reward, new_numerical_state, alpha=alpha)
    model_state = new_model_state
```

## Installation
Copy all files of the `src` directory to a local directory. To use the simulation results provided, copy json data from `data/` into the same directory.

## Execution
### Simulation
To start learning from a blank slate, run `pypy system.py` or `pypy3 system.py`. The standard CPython implementations can be used for testing but run too slow to be of practical use. Learned data are written to the current directory in regular intervals.
### Plotting
To generate an animation from the learned data run `python show.py` or `python3 show.py`. The latest simulation data in the current directory will be used for plotting (based on filename).

## License
MIT
