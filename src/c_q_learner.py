import platform as _platform

if _platform.python_version_tuple()[0] == '2': range = xrange

# Author: Chris Petrich
# Date: 2016
#   Apr 2018: modified for Python 3

_always_ignore_numpy = True

class DefaultZeroArray(dict):
    # http://stackoverflow.com/questions/4126348/how-do-i-rewrite-this-function-to-implement-ordereddict/4127426#4127426
    # http://stackoverflow.com/questions/6190331/can-i-do-an-ordered-default-dict-in-python
    def __init__(self, *args, **kwargs):
        self.shape = tuple([int('%g'%i,10) for i in args[0]])
        args = args[1:]
        super(DefaultZeroArray, self).__init__(*args, **kwargs)

    def __missing__ (self, key):
        self[key] = default = [0. for i in range(self.shape[-1])] #self.default_factory()
        return default

    def __reduce__(self):  # optional, for pickle support
        args = ([0. for i in range(self.shape[-1])],)
        return self.__class__, args, None, None, self.iteritems()

    def copy(self):
        return self.__copy__()

    def __copy__(self):
        return type(self)(self.shape, self)

    def __deepcopy__(self, memo):
        import copy
        return type(self)(self.shape,
                          copy.deepcopy(self.items()))


if _platform.python_implementation()=='PyPy' or _always_ignore_numpy:
    import random as _random
    _randint = _random.randrange # NB: random's randint has a different calling convention
    _rand = _random.random
    _choice = _random.choice
    _zero_array = DefaultZeroArray
else:
    import numpy as _np
    _randint = _np.random.randint
    _rand = _np.random.rand
    _choice = _np.random.choice
    _zero_array = _np.zeros

class Q_learner(object):
    def __init__(self, N_states, N_actions, init=None):
        try:
            N_states = tuple(N_states)
        except TypeError:
            N_states = (N_states,)
        N_actions = int(N_actions)

        self.gamma = None # get this from observer in conjunction with rewards (e.g., .997)

        self._memory=[]
        self._last_replay=len(self._memory)
        self._N_learn = 0
        self._N_memory = 0

        if init is not None:
            for variable in init:
                self.__setattr__(variable,init[variable])

        # 0 is a good value for neutral contributions
        #  from cells that are terminal
        self.Q=_zero_array(N_states+(N_actions,))


    def policy(self, state, epsilon=0.):
        if (epsilon!=0.) and (_rand()<epsilon):
            return _randint(0,self.Q.shape[-1])
        return _argmax_a_Q(self.Q, state, strict=epsilon == 0.)

    def value(self, state):
        return _max_a_Q(self.Q, state)

    def learn(self, state=None, action=None, reward=None, new_state=None, alpha=None):
        """Call: learn(state, action, reward, new_state) or learn()"""
        if True:
            # this seems to be the algorithm that leads to the
            #   most robust results early on
            if state is not None:
                # this helps avoid tracks of excessive
                #   length when they first start to appear often
                _update_Q(self.Q, (state, action, reward, new_state), alpha=alpha, gamma=self.gamma)
                self._N_learn += 1
        if False: # and (self._N_learn>100000):
            # this promotes earlier development of successful tracks
            if state is None:
                gamma = self.gamma
                self._N_learn += _experience_replay(self.Q, self._memory, self._last_replay, alpha=alpha, gamma=gamma)
                self._last_replay = len(self._memory)
                # erase memory periodically so we won't replay out-of-date
                # (state,action)--> new state combinations??
            else:
                self._memory.append((state, action, reward, new_state))
                self._N_memory += 1


def _argmax_a_Q(Q,state, strict=True):
    """Creates the policy"""
    #if len(set(Q[state]))!=Q.shape[-1]:
    if False and (not strict):
        # This seems to work. And it seems to help, at least early on.
        values = Q[state]
        # choose one of the highest-scoring options with probability
        #   based on Q(s,a)
        v_half = min(values) # consider all values
        idx_out, values = zip(*[(i,v) for i,v in enumerate(values) if v>=v_half])

        upper_limit=max(values)
        lower_limit=min(values)
        if upper_limit != lower_limit:
            # square the value to emphasize the top choice a bit more
            prob = [((value-lower_limit)/(upper_limit-lower_limit))**2 for value in values]
        else:
            return _randint(0,len(values))
        total = sum(prob)
        start=[0]
        for v in prob:
            start.append(start[-1]+v)
        thr=_rand()*total
        return idx_out[max([idx for idx, v in enumerate(start) if v<=thr])]

    # defaut: choose highest
    return sorted([(i,q) for i,q in enumerate(Q[state])],key=lambda x: x[1])[-1][0]


def _max_a_Q(Q,state):
    """Creates the value function"""
    return sorted([(i,q) for i,q in enumerate(Q[state])],key=lambda x: x[1])[-1][1]

def _experience_replay(Q, memory, last_replay=None, alpha=None, gamma=None, N_random_samples=None):
    count = 0
    if True:
        if last_replay is not None:
            # replay last episode backward leads to higher
            #   rates of success early on
            for replay in memory[-1:max((0,last_replay-1)):-1]:
                _update_Q(Q, replay, alpha=alpha, gamma=gamma)
                count += 1

    if False:
        # I suppose this makes really only sense once the association
        # (state,action)-->new state is reasonably stable
        # as it is too perturbig otherwise
        N_random_samples = 1000 if N_random_samples is None else N_random_samples
        if len(memory)>10*N_random_samples:
            # random replay -- rather detrimental early on and should
            #   be augmented by regular forward propagation for
            #   robustness
            for idx in _randint(0, len(memory), N_random_samples):
                _update_Q(Q, memory[idx], alpha=alpha, gamma=gamma)
                count += 1
    return count

def _update_Q(Q, sars, alpha=None, gamma=None):
    # this will never be called from a terminal state
    state, action, reward, new_state = sars
    Q[state][action]=((1-alpha)*Q[state][action]+
                      alpha* (reward + gamma * _max_a_Q(Q, new_state)))
