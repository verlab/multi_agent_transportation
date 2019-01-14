#!/usr/bin/python

import networkx as nx

class Analyser(object):

    @staticmethod
    def slipt( path ):
        start_id = path.graph['start_id']

        segments = []
        segment = []
        current_id = start_id

        def create_sub_graph(segment):
            sub = nx.DiGraph( path.subgraph( segment ) )
            sub.graph['start_id'] = segment[0]
            sub.graph['end_id'] = segment[-1]
            sub.graph['type'] = sub.node[ segment[0] ]['robot_type']

            return sub

        while True:
            segment.append( current_id )
            neighbor = path.neighbors( current_id )

            if len(neighbor) == 0:
                segments.append( create_sub_graph( segment ) )
                break

            next_id = neighbor[0]

            curr_action = path.node[ current_id ][ 'action' ].name
            next_action = path.node[ next_id ][ 'action' ].name

            curr_air = path.node[ current_id ]['state'].in_air()
            next_air = path.node[ next_id ]['state'].in_air() or next_action is 'DESCEND'

            current_id = next_id

            if curr_air and next_air:
                continue

            if curr_action is not next_action:
                segments.append( create_sub_graph( segment ) )
                segment = []

        return segments

