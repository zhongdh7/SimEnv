#!/usr/bin/env python3
"""Small, deterministic graph model for floor exploration tasks.

The graph stores only nodes discovered by navigation and does not depend on
the building metadata.  Motion planning remains the responsibility of the
navigation node; this module only handles topology and task ordering.
"""

from __future__ import print_function

import math


class GraphNode(object):
    """A discovered navigation node."""

    def __init__(self, node_id, node_type, floor, x, y, yaw=0.0,
                 corridor_s=None, confidence=0.0, label=""):
        self.node_id = int(node_id)
        self.node_type = str(node_type)
        self.floor = int(floor)
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.corridor_s = None if corridor_s is None else float(corridor_s)
        self.confidence = float(confidence)
        self.label = str(label)
        self.visited = False
        self.entered = False
        self.interior_observation = False
        self.coverage = 0.0
        self.completed = False
        self.blocked = False

    @property
    def position(self):
        return self.x, self.y

    def distance_to(self, x, y):
        return math.hypot(self.x - float(x), self.y - float(y))

    def as_dict(self):
        return {
            "id": self.node_id,
            "type": self.node_type,
            "floor": self.floor,
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "corridor_s": self.corridor_s,
            "confidence": self.confidence,
            "label": self.label,
            "visited": self.visited,
            "entered": self.entered,
            "interior_observation": self.interior_observation,
            "coverage": self.coverage,
            "completed": self.completed,
            "blocked": self.blocked,
        }


class GraphEdge(object):
    """An undirected traversable connection between two nodes."""

    def __init__(self, edge_id, start, end, length=None):
        self.edge_id = int(edge_id)
        self.start = int(start)
        self.end = int(end)
        self.length = None if length is None else float(length)
        self.visited = False
        self.blocked = False

    def other(self, node_id):
        node_id = int(node_id)
        if node_id == self.start:
            return self.end
        if node_id == self.end:
            return self.start
        raise ValueError("node %d is not incident to edge %d" %
                         (node_id, self.edge_id))


class CorridorGraph(object):
    """Mutable floor graph with deterministic DFS task ordering."""

    def __init__(self, merge_distance=0.5):
        self.merge_distance = float(merge_distance)
        self.nodes = {}
        self.edges = {}
        self._next_node_id = 0
        self._next_edge_id = 0

    def add_node(self, node_type, floor, x, y, yaw=0.0, corridor_s=None,
                 confidence=0.0, label="", merge=True):
        """Add a node, merging nearby same-floor observations when requested."""
        if merge:
            existing = self._nearest_node(node_type, floor, x, y)
            if existing is not None:
                existing.confidence = max(existing.confidence, float(confidence))
                if existing.corridor_s is None and corridor_s is not None:
                    existing.corridor_s = float(corridor_s)
                if label and not existing.label:
                    existing.label = str(label)
                return existing.node_id, False

        node = GraphNode(
            self._next_node_id,
            node_type,
            floor,
            x,
            y,
            yaw,
            corridor_s,
            confidence,
            label,
        )
        self.nodes[node.node_id] = node
        self._next_node_id += 1
        return node.node_id, True

    def add_room_entry(self, floor, x, y, yaw=0.0, corridor_s=None,
                       confidence=0.0, label=""):
        return self.add_node(
            "room_entry", floor, x, y, yaw, corridor_s, confidence, label
        )

    def add_edge(self, start, end, length=None):
        """Add an undirected edge and return its stable id."""
        start, end = int(start), int(end)
        if start == end:
            raise ValueError("self-loop is not a corridor edge")
        if start not in self.nodes or end not in self.nodes:
            raise KeyError("edge endpoint is not in graph")
        for edge in self.edges.values():
            if {edge.start, edge.end} == {start, end}:
                return edge.edge_id, False
        if length is None:
            length = math.hypot(
                self.nodes[start].x - self.nodes[end].x,
                self.nodes[start].y - self.nodes[end].y,
            )
        edge = GraphEdge(self._next_edge_id, start, end, length)
        self.edges[edge.edge_id] = edge
        self._next_edge_id += 1
        return edge.edge_id, True

    def mark_node(self, node_id, visited=None, completed=None, blocked=None):
        node = self.nodes[int(node_id)]
        if visited is not None:
            node.visited = bool(visited)
        if completed is not None:
            node.completed = bool(completed)
        if blocked is not None:
            node.blocked = bool(blocked)

    def mark_edge(self, edge_id, visited=None, blocked=None):
        edge = self.edges[int(edge_id)]
        if visited is not None:
            edge.visited = bool(visited)
        if blocked is not None:
            edge.blocked = bool(blocked)

    def neighbors(self, node_id, include_blocked=False):
        node_id = int(node_id)
        result = []
        for edge in self.edges.values():
            if edge.blocked and not include_blocked:
                continue
            if edge.start == node_id or edge.end == node_id:
                result.append((edge, self.nodes[edge.other(node_id)]))
        return result

    def pending_rooms(self, floor, descending=True):
        rooms = [
            node for node in self.nodes.values()
            if (node.floor == int(floor) and node.node_type == "room_entry" and
                not node.completed and not node.blocked)
        ]
        return sorted(
            rooms,
            key=lambda node: (
                float("-inf") if node.corridor_s is None else node.corridor_s,
                node.node_id,
            ),
            reverse=bool(descending),
        )

    def dfs_order(self, start_node_id, floor=None):
        """Return a deterministic DFS node order without changing node state."""
        start_node_id = int(start_node_id)
        if start_node_id not in self.nodes:
            raise KeyError("DFS start node is not in graph")
        visited = set()
        order = []

        def sort_key(item):
            edge, node = item
            type_priority = {
                "room_entry": 0,
                "corridor_end": 1,
                "junction": 2,
                "stair": 3,
                "entrance": 4,
            }.get(node.node_type, 5)
            distance = (
                float("-inf") if node.corridor_s is None else node.corridor_s
            )
            return type_priority, -distance, edge.edge_id

        def visit(node_id):
            if node_id in visited:
                return
            visited.add(node_id)
            node = self.nodes[node_id]
            if floor is None or node.floor == int(floor):
                order.append(node_id)
            for edge, neighbor in sorted(self.neighbors(node_id), key=sort_key):
                if neighbor.node_id in visited:
                    continue
                if floor is not None and neighbor.floor != int(floor):
                    continue
                edge.visited = True
                visit(neighbor.node_id)

        visit(start_node_id)
        return order

    def _nearest_node(self, node_type, floor, x, y):
        candidates = [
            node for node in self.nodes.values()
            if node.node_type == str(node_type) and node.floor == int(floor)
        ]
        if not candidates:
            return None
        nearest = min(candidates, key=lambda node: node.distance_to(x, y))
        return nearest if nearest.distance_to(x, y) <= self.merge_distance else None
