import geometry as _g

import sys as _sys
if _sys.version_info.major != 2: reload, xrange = lambda x: 0, range

reload(_g)

# Author: Chris Petrich
# Date: 2016
#   Apr 2018: modified for Python 3

# this is the only class that is purpousefully stateful
#  (i.e., beyond keeping model parameters, this keeps and updates
#         lists or dictionaries in-place)

class EnvironmentState(object):
    def __init__(self,init=None):
        self.init_state(init)

    def init_state(self, init):
        """Initialize the in-place mutable state of the environment."""
        # static "domain" definition
        #  use add_domain_lines() to add to this
        self._domain_lines = []
        self._domain_lines_h = []
        self._domain_cell_refs={}
        self._domain_cell_bbox=[None,None,None,None]

        self.agents = {} # dict of agents, starts empty

        if init is not None:
            for variable in init:
                self.__setattr__(variable,init[variable])

    def get_domain_lines_that_may_intersect_infinite_line(self, inf_line_h):
        """This doesn't make sense as implemented as it identify too many cell indices"""
        raise NotImplementedError('TODO: revise implementation!')
        # what bbox-cells are relevant?
        #  we limit the infinte line to the domain size to find a bounding box:
        a,b,c = inf_line_h
        if abs(a)>abs(b):
            y_min = self._domain_cell_bbox[1]-1.
            y_max = self._domain_cell_bbox[3]+1.
            # with x: min and max correspond to y-designation
            x_min = -(b*y_min+c)/a
            x_max = -(b*y_max+c)/a
        else:
            x_min = self._domain_cell_bbox[0]-1.
            x_max = self._domain_cell_bbox[2]+1.
            # with y: min and max correspond to x-designation
            y_min = -(a*x_min+c)/b
            y_max = -(a*x_max+c)/b

        line = ((x_min,y_min),(x_max,y_max))

        bbox = self._get_discrete_bbox_of_line(line)
        cells=set(self._get_potential_cell_list(bbox))

        return self._get_domain_lines_from_cells(cells)

    def get_domain_lines_that_may_intersect(self, lines):
        # what bbox-cells are relevant?
        cells = set({})
        for line in lines:
            bbox = self._get_discrete_bbox_of_line(line)
            cells.update(set(self._get_potential_cell_list(bbox)))

        return self._get_domain_lines_from_cells(cells)

    def _get_domain_lines_from_cells(self, cells):
        # which line segments are in these bbox-cells
        idx_list = []
        for cell in cells:
            idx_list += self._domain_cell_refs.get(cell,[])
        idx_list = set(idx_list) # make entries unique

        subset = lambda lst, idxs: [lst[i] for i in idxs]

        # pull out those lines
        return subset(self._domain_lines, idx_list), subset(self._domain_lines_h, idx_list)

    def add_domain_lines(self, lines, lines_h=None):
        if (lines_h is None) or (len(lines_h)==0):
            lines_h = _g._lines_to_homogenous(lines)
        elif len(lines_h) != len(lines):
            raise ValueError('%i line segements but %i homogenous line segements' % (len(lines), len(lines_h)))

        self._add_lines_to_dict_and_list(self._domain_cell_refs, self._domain_cell_bbox, lines, lines_h, self._domain_lines, self._domain_lines_h)

    def _add_lines_to_dict_and_list(self, cell_ref_dict, bbox_bbox, lines, lines_h, list_lines, list_lines_h):
        for line, line_h in zip(lines, lines_h):
            idx = len(list_lines)
            list_lines.append(line)
            list_lines_h.append(line_h)

            bbox = self._get_discrete_bbox_of_line(line)
            if bbox_bbox[0] is not None:
                bbox_bbox[0]=min((bbox_bbox[0], bbox[0]))
                bbox_bbox[2]=max((bbox_bbox[2], bbox[2]))
                bbox_bbox[1]=min((bbox_bbox[1], bbox[1]))
                bbox_bbox[3]=max((bbox_bbox[3], bbox[3]))
            else:
                bbox_bbox[0]=bbox[0]
                bbox_bbox[2]=bbox[2]
                bbox_bbox[1]=bbox[1]
                bbox_bbox[3]=bbox[3]

            cells = self._get_potential_cell_list(bbox)
            for cell in cells:
                cell_ref_dict.setdefault(cell,[]).append(idx)

    def _get_discrete_bbox_of_line(self, line):
        Xs,Ys = zip(*line)
        bbox = (int(min(Xs)),int(min(Ys)),int(max(Xs)),int(max(Ys)))
        return bbox

    def _get_potential_cell_list(self, bbox):
        return [(x,y) for x in xrange(bbox[0], bbox[2]+1) for y in xrange(bbox[1], bbox[3]+1)]

    def set_agent_properties(self, agent, outline, outline_h=None):
        # set outline to [] to remove data
        props = self.agents.get(agent.agent_ID,[[] for i in (1,2)])
        props[0] = [tuple(o) for o in outline] # store copy
        outline_h = g._lines_to_homogenous(outline) if outline_h is None else outline_h
        props[1] = outline_h

    def get_all_agent_lines(self, except_agents):
        """Return a list of all agent outlines, and a second list
        with homogenous line definitions."""
        lines, lines_h = [],[]
        for agent in self.agents:
            if agent in except_agents: continue
            l,lh = self.agents[agent]
            lines.append(l)
            lines_h.append(lh)
        return lines, lines_h
