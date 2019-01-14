#!/usr/bin/python

from collections import namedtuple
from heuristics import manhattan, euclidean

from models.thing import Thing
from models.robot import Robot

import networkx as nx
import sympy as sp
import numpy as np

class Move(object):
    def __init__(self, name, change, move_type):
        self.name = name
        self.change = np.array( change )
        # self.cost = cost
        self.type = move_type
        self.counter_action = -self.change

class Planner(object):

    TYPES = namedtuple("PlanType", "land both")(0, 1)
    INF = 10000000000000

    def __init__(self, move_size=1, plan_type = TYPES.both, heuristic = manhattan, config = None):

        self.move_size = move_size
        self.type = plan_type
        self.heuristic = heuristic

        self.distance_factor = 1

        self.time_factor = 0.5
        self.energy_factor = 0.5

        if config:
            self.time_factor = config['time_factor']
            self.energy_factor = config['energy_factor']

        # print self.time_factor
        # print self.energy_factor

        self._moves = [
            Move("FORWARD",      [0, +self.move_size, 0], [Robot.TYPES.land]),
            Move("BACKWARD",    [0, -self.move_size, 0], [Robot.TYPES.land]),
            Move("LEFT",        [-self.move_size, 0, 0], [Robot.TYPES.land]),
            Move("RIGHT",       [+self.move_size, 0, 0], [Robot.TYPES.land]),
            Move("RISE",        [0, 0, +self.move_size], [Robot.TYPES.aerial, Robot.TYPES.land]),
            Move("DESCEND",     [0, 0, -self.move_size], [Robot.TYPES.aerial, Robot.TYPES.land]),
            Move("FORWARD-AERIAL",   [0, +self.move_size, 0], [Robot.TYPES.aerial]),
            Move("BACKWARD-AERIAL", [0, -self.move_size, 0], [Robot.TYPES.aerial]),
            Move("LEFT-AERIAL",     [-self.move_size, 0, 0], [Robot.TYPES.aerial]),
            Move("RIGHT-AERIAL",    [+self.move_size, 0, 0], [Robot.TYPES.aerial]),
            # Move("FOWARD",   [0, +self.move_size, 0], [Planner.TYPES.land, Planner.TYPES.both]),
            # Move("BACKWARD", [0, -self.move_size, 0], [Planner.TYPES.land, Planner.TYPES.both]),
            # Move("LEFT",     [-self.move_size, 0, 0], [Planner.TYPES.land, Planner.TYPES.both]),
            # Move("RIGHT",    [+self.move_size, 0, 0], [Planner.TYPES.land, Planner.TYPES.both]),
            # Move("RISE",     [0, 0, +self.move_size], [Planner.TYPES.both]),
            # Move("DESCEND",  [0, 0, -self.move_size], [Planner.TYPES.both]),
        ]

        self._test_case = None
        self._search_graph = nx.DiGraph()

        # self._create_utility_function()

    # def _create_utility_function(self):
    #     time_factor_s = sp.Symbol('time_factor')
    #     distance_factor_s = sp.Symbol('distance_factor')
    #     energy_factor_s = sp.Symbol('energy_factor')

    #     self._distance_s = sp.Symbol('distance')
    #     self._time_s = sp.Symbol('time')
    #     self._energy_s = sp.Symbol('energy')

    #     uf = sp.sympify('time_factor * time + distance_factor * distance + energy_factor * energy')

    #     self._utility_function = uf.subs({
    #         time_factor_s: self.time_factor,
    #         distance_factor_s: self.distance_factor,
    #         energy_factor_s: self.energy_factor
    #     })

    def plan(self, test_case):
        self.valid_test_case(test_case)

        self._item_type = type(test_case["start_position"])
        self._test_case = test_case

        return self._search()

    def valid_test_case(self, test_case):
        workspace = test_case["workspace"]
        start_position = test_case["start_position"]
        end_position = test_case["end_position"]

        workspace.valid_state( start_position )
        workspace.valid_state( end_position )

    # Utils

    @staticmethod
    def _create_backward_position( state, action, mult = 1 ):
        new_position = state.copy()
        new_position += (action.counter_action * mult)

        return new_position

    @staticmethod
    def create_robot_position( node, mult = 1 ):
        return Planner._create_backward_position( node['state'], node['action'], mult )

    # Search Utils

    def _distance_to_goal(self, node):
        goal = self._test_case["end_position"].position
        current = node["state"].position

        return self.heuristic( current, goal )

    def _assign_utility(self, node, direction_change):

        if self._item_type is Thing:

            for robot_type in self._test_case['robot_types']:
                robot_type_data = Robot.TYPES_DATA[robot_type]

                if node["state"].in_air() and not robot_type_data['fly']:
                    result = Planner.INF

                elif not node["state"].in_air() and node['action'] and node['action'].name not in ['RISE', 'DESCEND'] and robot_type_data['fly']:
                    result = Planner.INF

                else:
                    distance = self._distance_to_goal( node )
                    time = self.time_factor * robot_type_data['time']
                    energy = self.energy_factor * robot_type_data['energy']

                    result = distance + time + energy

                    # result = self._utility_function.subs({
                        # self._distance_s: self._distance_to_goal( node ),
                        # self._time_s: robot_type_data['time'],
                        # self._energy_s: robot_type_data['energy']
                    # })

                # parent_utility = node['utility_parent_type'][robot_type] if robot_type in node['utility_parent_type'] else 0

                node["utility_type"][robot_type] = float( result ) + node['depth']
                # + node['utility_parent']
                # + parent_utility

                # node['utility_parent']

                # print robot_type, result
                # node["utility_type"][robot_type]
                # print '--------'

                if robot_type == Robot.TYPES.aerial:
                    node["utility_type"][robot_type] += 1

                if direction_change:
                    node["utility_type"][robot_type] += 1

        else:
            # result = self._utility_function.subs({
            #     self._distance_s: self._distance_to_goal( node ),
            #     self._time_s: 1,
            #     self._energy_s: 1
            # })

            distance = self._distance_to_goal( node )

            result = distance

            node["utility"] = float( result ) + node['depth']

    # Graph Utils

    def _create_node(self, state, action = None, parent = None):
        node_id = hash( state )

        self._search_graph.add_node(node_id, {
            "id": node_id,
            "state": state,
            "depth": 0,
            "utility": 0,
            "action": action,
            "utility_type": {},
            # "utility_parent": 0,
            # "utility_parent_type": {},
            "robot_type": None
        })

        node = self._search_graph.node[ node_id ]

        direction_change = False

        if parent:
            self._search_graph.add_edge( parent["id"], node_id )
            node['depth'] = parent['depth'] + 1

            if self._item_type is Thing:
            #     node['utility_parent'] = parent['utility_type'][parent['robot_type']]
            #     node['utility_parent_type'] = parent['utility_type']

                if action and parent['action'] and (action.name != parent['action'].name):
                    direction_change = True
                    # node['utility_parent'] += 1

        # direction_change = False

        self._assign_utility( node, direction_change )

        # if parent and True:
        #     for robot_type in self._test_case['robot_types']:
        #         if action and parent['action'] and (action.name != parent['action'].name):
        #             node["utility_type"][robot_type] += 0

        return node

    # A* Search

    def _select_node(self, fringe, first = False):

        if self._item_type is Thing:

            if first:
                return fringe.pop()

            best_node = None
            best_utility = 0
            best_type = -1

            for robot_type in self._test_case['robot_types']:

                fringe.sort( key = lambda node: node["utility_type"][robot_type] )
                temp_node = fringe[0]

                if best_node is None or temp_node['utility_type'][robot_type] < best_utility:
                    best_node = temp_node
                    best_utility = temp_node['utility_type'][robot_type]
                    best_type = robot_type

            fringe.remove(best_node)
            best_node['robot_type'] = best_type

            return best_node

        else:
            fringe.sort( key = lambda node: node["utility"] )

            best_node = fringe.pop(0)
            best_node['robot_type'] = self._test_case['robot_type']

            return best_node

    def _expand(self, node):
        result_list = []

        base_state = node['state']
        workspace = self._test_case["workspace"]

        for action in self._moves:

            if node['robot_type'] is not None:
                if node['robot_type'] not in action.type:
                    continue

                if node['robot_type'] is Robot.TYPES.land and self._item_type is Robot and action.name in ['RISE', 'DESCEND']:
                    continue

            if '-AERIAL' in action.name and not node['state'].in_air():
                continue

            test_state = base_state.copy()
            test_state += action.change

            try:
                workspace.valid_state( test_state )

                if hash( test_state ) not in self._search_graph:

                    if self._item_type is Thing and Robot.TYPES.land in action.type and action.name not in ['RISE', 'DESCEND']:
                        backward_position = Planner._create_backward_position( test_state, action, 2 )
                        workspace.valid_state( backward_position )

                    expanded_node = self._create_node( test_state, action, node )

                    result_list.append( expanded_node )

            except Exception, e:
                pass
                # print e
                # print "Invalid new state: ", e.message

        return result_list

    def _search(self):
        initial_node = self._create_node( self._test_case['start_position'] )

        fringe = [ initial_node ]
        first = True

        while True:
            if len(fringe) == 0:
                raise Exception("Plan not found")

            current_node = self._select_node( fringe, first )
            first = False

            if hash(current_node['state']) == hash( self._test_case['end_position'] ):
                return self._path_from( current_node, initial_node )

            fringe.extend( self._expand( current_node ) )

    def _path_from(self, node, initial_node):
        base_graph = self._search_graph.reverse()
        path = nx.shortest_path( base_graph, node['id'], initial_node['id'] )

        out_graph = nx.DiGraph( base_graph.subgraph( path ) ).reverse()

        out_graph.graph['start_id'] = initial_node['id']
        out_graph.graph['end_id'] = node['id']

        successors = out_graph.successors( out_graph.graph['start_id'] )

        if len(successors) != 0:
            first_node = out_graph.node[ out_graph.graph['start_id'] ]
            first_node['action'] = out_graph.node[ successors[0] ]['action']
            first_node['robot_type'] = out_graph.node[ successors[0] ]['robot_type']

        # Refine path
        # Insert nodes on direction change

        # print "before:", len(out_graph)

        # current_id = out_graph.graph['start_id']
        # while True:
        #     successors = out_graph.successors( current_id )

        #     if len(successors) == 0:
        #         break

        #     next_id = successors[0]

        #     curr_action = out_graph.node[ current_id ]['action'].name
        #     next_action = out_graph.node[ next_id ]['action'].name

        #     # Insert node
        #     if curr_action != next_action:

        #         out_graph.remove_edge( current_id, next_id )

        #         new_node = self._create_node(
        #             out_graph.node[ current_id ]['state'],
        #             out_graph.node[ next_id ]['action'],
        #             out_graph.node[ current_id ]
        #         )

        #         out_graph.add_edge( new_node['id'], next_id )

        #     current_id = next_id

        # print "after:", len(out_graph)

        # return self._refine_path( out_graph )
        return out_graph
