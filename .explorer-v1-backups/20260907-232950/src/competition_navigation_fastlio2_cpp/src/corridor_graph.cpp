// C++ port of corridor_graph.py.  See corridor_graph.h for the contract.
#include "competition_navigation_fastlio2_cpp/corridor_graph.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace competition_navigation_fastlio2_cpp {

CorridorGraph::CorridorGraph(double merge_distance)
    : merge_distance_(merge_distance) {}

std::pair<int, bool> CorridorGraph::add_node(
    const std::string& node_type, int floor, double x, double y, double yaw,
    std::optional<double> corridor_s, double confidence,
    const std::string& label, bool merge) {
  if (merge) {
    int existing = nearest_node(node_type, floor, x, y);
    if (existing >= 0) {
      GraphNode& node = nodes.at(existing);
      node.confidence = std::max(node.confidence, confidence);
      if (!node.corridor_s.has_value() && corridor_s.has_value()) {
        node.corridor_s = corridor_s;
      }
      if (!label.empty() && node.label.empty()) {
        node.label = label;
      }
      return {node.node_id, false};
    }
  }

  GraphNode node;
  node.node_id = next_node_id_++;
  node.node_type = node_type;
  node.floor = floor;
  node.x = x;
  node.y = y;
  node.yaw = yaw;
  node.corridor_s = corridor_s;
  node.confidence = confidence;
  node.label = label;
  nodes[node.node_id] = node;
  return {node.node_id, true};
}

std::pair<int, bool> CorridorGraph::add_room_entry(
    int floor, double x, double y, double yaw, std::optional<double> corridor_s,
    double confidence, const std::string& label) {
  return add_node("room_entry", floor, x, y, yaw, corridor_s, confidence, label);
}

std::pair<int, bool> CorridorGraph::add_edge(int start, int end,
                                             std::optional<double> length) {
  if (start == end) {
    // Python raised ValueError here; navigation never creates a self-loop.
    return {-1, false};
  }
  if (nodes.find(start) == nodes.end() || nodes.find(end) == nodes.end()) {
    return {-1, false};
  }
  for (const auto& kv : edges) {
    const GraphEdge& edge = kv.second;
    if ((edge.start == start && edge.end == end) ||
        (edge.start == end && edge.end == start)) {
      return {edge.edge_id, false};
    }
  }
  double resolved = length.has_value()
                        ? length.value()
                        : std::hypot(nodes.at(start).x - nodes.at(end).x,
                                     nodes.at(start).y - nodes.at(end).y);
  GraphEdge edge;
  edge.edge_id = next_edge_id_++;
  edge.start = start;
  edge.end = end;
  edge.length = resolved;
  edges[edge.edge_id] = edge;
  return {edge.edge_id, true};
}

void CorridorGraph::mark_node(int node_id, std::optional<bool> visited,
                              std::optional<bool> completed,
                              std::optional<bool> blocked) {
  GraphNode& node = nodes.at(node_id);
  if (visited.has_value()) node.visited = visited.value();
  if (completed.has_value()) node.completed = completed.value();
  if (blocked.has_value()) node.blocked = blocked.value();
}

void CorridorGraph::mark_edge(int edge_id, std::optional<bool> visited,
                              std::optional<bool> blocked) {
  GraphEdge& edge = edges.at(edge_id);
  if (visited.has_value()) edge.visited = visited.value();
  if (blocked.has_value()) edge.blocked = blocked.value();
}

std::vector<std::pair<GraphEdge, GraphNode>> CorridorGraph::neighbors(
    int node_id, bool include_blocked) const {
  std::vector<std::pair<GraphEdge, GraphNode>> result;
  for (const auto& kv : edges) {
    const GraphEdge& edge = kv.second;
    if (edge.blocked && !include_blocked) continue;
    if (edge.start == node_id || edge.end == node_id) {
      result.emplace_back(edge, nodes.at(edge.other(node_id)));
    }
  }
  return result;
}

std::vector<GraphNode> CorridorGraph::pending_rooms(int floor,
                                                   bool descending) const {
  std::vector<GraphNode> rooms;
  for (const auto& kv : nodes) {
    const GraphNode& node = kv.second;
    if (node.floor == floor && node.node_type == "room_entry" &&
        !node.completed && !node.blocked) {
      rooms.push_back(node);
    }
  }
  std::sort(rooms.begin(), rooms.end(),
            [](const GraphNode& a, const GraphNode& b) {
              double a_s = a.corridor_s.has_value()
                               ? a.corridor_s.value()
                               : -std::numeric_limits<double>::infinity();
              double b_s = b.corridor_s.has_value()
                               ? b.corridor_s.value()
                               : -std::numeric_limits<double>::infinity();
              if (a_s != b_s) return a_s < b_s;
              return a.node_id < b.node_id;
            });
  if (descending) std::reverse(rooms.begin(), rooms.end());
  return rooms;
}

std::vector<int> CorridorGraph::dfs_order(int start_node_id,
                                          std::optional<int> floor) {
  std::vector<int> order;
  std::map<int, bool> visited;

  // Matches the Python sort_key: (type_priority, -distance, edge_id).
  auto type_priority = [](const std::string& t) {
    if (t == "room_entry") return 0;
    if (t == "corridor_end") return 1;
    if (t == "junction") return 2;
    if (t == "stair") return 3;
    if (t == "entrance") return 4;
    return 5;
  };

  std::function<void(int)> visit = [&](int node_id) {
    if (visited[node_id]) return;
    visited[node_id] = true;
    const GraphNode& node = nodes.at(node_id);
    if (!floor.has_value() || node.floor == floor.value()) {
      order.push_back(node_id);
    }
    // Collect incident (edge_id, other_node) pairs, sorted by sort_key.
    std::vector<std::pair<int, int>> incident;  // (edge_id, other_node_id)
    for (const auto& kv : edges) {
      const GraphEdge& edge = kv.second;
      if (edge.blocked) continue;
      if (edge.start == node_id || edge.end == node_id) {
        incident.emplace_back(edge.edge_id, edge.other(node_id));
      }
    }
    std::sort(incident.begin(), incident.end(),
              [&](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                const GraphNode& na = nodes.at(a.second);
                const GraphNode& nb = nodes.at(b.second);
                int pa = type_priority(na.node_type);
                int pb = type_priority(nb.node_type);
                if (pa != pb) return pa < pb;
                double da = na.corridor_s.has_value()
                                ? na.corridor_s.value()
                                : -std::numeric_limits<double>::infinity();
                double db = nb.corridor_s.has_value()
                                ? nb.corridor_s.value()
                                : -std::numeric_limits<double>::infinity();
                if (da != db) return -da < -db;  // -distance ascending
                return a.first < b.first;        // edge_id
              });
    for (const auto& inc : incident) {
      int edge_id = inc.first;
      int neighbor_id = inc.second;
      if (visited.find(neighbor_id) != visited.end()) continue;
      if (floor.has_value() && nodes.at(neighbor_id).floor != floor.value())
        continue;
      edges.at(edge_id).visited = true;
      visit(neighbor_id);
    }
  };

  if (nodes.find(start_node_id) == nodes.end()) {
    return order;  // Python raised KeyError; navigation passes a valid id.
  }
  visit(start_node_id);
  return order;
}

int CorridorGraph::nearest_node(const std::string& node_type, int floor,
                                double x, double y) const {
  int nearest = -1;
  double nearest_dist = 0.0;
  for (const auto& kv : nodes) {
    const GraphNode& node = kv.second;
    if (node.node_type != node_type || node.floor != floor) continue;
    double d = node.distance_to(x, y);
    if (nearest < 0 || d < nearest_dist) {
      nearest = node.node_id;
      nearest_dist = d;
    }
  }
  if (nearest < 0) return -1;
  return nearest_dist <= merge_distance_ ? nearest : -1;
}

}  // namespace competition_navigation_fastlio2_cpp
