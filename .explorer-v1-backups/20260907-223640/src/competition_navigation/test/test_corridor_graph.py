#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "scripts"))

from corridor_graph import CorridorGraph


class CorridorGraphTest(unittest.TestCase):
    def test_room_entry_observations_are_deduplicated(self):
        graph = CorridorGraph(merge_distance=0.5)
        first, created = graph.add_room_entry(
            floor=0, x=1.0, y=10.0, corridor_s=2.0, confidence=0.4
        )
        second, merged = graph.add_room_entry(
            floor=0, x=1.2, y=10.1, corridor_s=2.1, confidence=0.8
        )

        self.assertTrue(created)
        self.assertFalse(merged)
        self.assertEqual(first, second)
        self.assertEqual(len(graph.nodes), 1)
        self.assertAlmostEqual(graph.nodes[first].confidence, 0.8)

    def test_pending_rooms_are_ordered_from_corridor_end(self):
        graph = CorridorGraph()
        graph.add_room_entry(0, -1.1, 10.0, corridor_s=2.0)
        graph.add_room_entry(0, 1.1, 20.0, corridor_s=12.0)
        graph.add_room_entry(0, -1.1, 30.0, corridor_s=22.0)

        rooms = graph.pending_rooms(0)

        self.assertEqual([node.corridor_s for node in rooms], [22.0, 12.0, 2.0])

    def test_dfs_visits_branch_and_returns_through_shared_junction(self):
        graph = CorridorGraph()
        root, _ = graph.add_node("junction", 0, 0.0, 0.0)
        left, _ = graph.add_node("room_entry", 0, -1.0, 2.0, corridor_s=2.0)
        right, _ = graph.add_node("room_entry", 0, 1.0, 2.0, corridor_s=3.0)
        end, _ = graph.add_node("corridor_end", 0, 0.0, 4.0)
        graph.add_edge(root, left)
        graph.add_edge(root, right)
        graph.add_edge(root, end)

        order = graph.dfs_order(root, floor=0)

        self.assertEqual(order[0], root)
        self.assertEqual(set(order), {root, left, right, end})
        self.assertEqual(len(order), 4)
        self.assertTrue(all(edge.visited for edge in graph.edges.values()))

    def test_dfs_from_corridor_end_processes_far_rooms_first(self):
        graph = CorridorGraph()
        root, _ = graph.add_node("entrance", 0, 0.0, 0.0, corridor_s=0.0)
        near, _ = graph.add_node("junction", 0, 0.0, 2.0, corridor_s=2.0)
        far, _ = graph.add_node("junction", 0, 0.0, 4.0, corridor_s=4.0)
        end, _ = graph.add_node("corridor_end", 0, 0.0, 5.0, corridor_s=5.0)
        near_room, _ = graph.add_room_entry(0, 1.0, 2.0, corridor_s=2.0)
        far_room, _ = graph.add_room_entry(0, -1.0, 4.0, corridor_s=4.0)
        graph.add_edge(root, near)
        graph.add_edge(near, far)
        graph.add_edge(far, end)
        graph.add_edge(near, near_room)
        graph.add_edge(far, far_room)

        order = graph.dfs_order(end, floor=0)

        self.assertEqual(order[0], end)
        self.assertLess(order.index(far_room), order.index(near_room))
        self.assertLess(order.index(far), order.index(near))

    def test_blocked_room_is_not_a_pending_task(self):
        graph = CorridorGraph()
        room, _ = graph.add_room_entry(0, 0.0, 1.0, corridor_s=1.0)
        graph.mark_node(room, blocked=True)

        self.assertEqual(graph.pending_rooms(0), [])


if __name__ == "__main__":
    unittest.main()
