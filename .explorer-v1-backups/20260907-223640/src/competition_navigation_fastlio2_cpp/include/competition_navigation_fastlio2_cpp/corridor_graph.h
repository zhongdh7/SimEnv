// C++ port of corridor_graph.py -- a small, deterministic graph model for
// floor exploration tasks.  The graph stores only nodes discovered by
// navigation and does not depend on the building metadata.  Motion planning
// remains the responsibility of the navigation node; this module only handles
// topology and task ordering.
#pragma once

#include <cmath>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace competition_navigation_fastlio2_cpp {

// A discovered navigation node.  Mirrors GraphNode in corridor_graph.py.
struct GraphNode {
  int node_id = 0;
  std::string node_type;
  int floor = 0;
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  std::optional<double> corridor_s;
  double confidence = 0.0;
  std::string label;
  bool visited = false;
  bool entered = false;
  bool interior_observation = false;
  double coverage = 0.0;
  bool completed = false;
  bool blocked = false;

  std::pair<double, double> position() const { return {x, y}; }
  double distance_to(double px, double py) const {
    return std::hypot(x - px, y - py);
  }
};

// An undirected traversable connection between two nodes.
struct GraphEdge {
  int edge_id = 0;
  int start = 0;
  int end = 0;
  std::optional<double> length;
  bool visited = false;
  bool blocked = false;

  int other(int node_id) const {
    if (node_id == start) return end;
    if (node_id == end) return start;
    return -1;  // Python raised ValueError here; navigation never hits it.
  }
};

// Mutable floor graph with deterministic DFS task ordering.
class CorridorGraph {
 public:
  explicit CorridorGraph(double merge_distance = 0.5);

  // Returns (node_id, created).  When merge=true and a same-floor same-type
  // node lies within merge_distance, an existing node is reused (created=false).
  std::pair<int, bool> add_node(const std::string& node_type, int floor,
                                double x, double y, double yaw = 0.0,
                                std::optional<double> corridor_s = std::nullopt,
                                double confidence = 0.0,
                                const std::string& label = "",
                                bool merge = true);

  std::pair<int, bool> add_room_entry(int floor, double x, double y,
                                      double yaw = 0.0,
                                      std::optional<double> corridor_s =
                                          std::nullopt,
                                      double confidence = 0.0,
                                      const std::string& label = "");

  // Returns (edge_id, created).
  std::pair<int, bool> add_edge(int start, int end,
                                std::optional<double> length = std::nullopt);

  void mark_node(int node_id, std::optional<bool> visited = std::nullopt,
                 std::optional<bool> completed = std::nullopt,
                 std::optional<bool> blocked = std::nullopt);

  void mark_edge(int edge_id, std::optional<bool> visited = std::nullopt,
                 std::optional<bool> blocked = std::nullopt);

  // Incident (edge, other_node) pairs, in edge-insertion order.
  std::vector<std::pair<GraphEdge, GraphNode>> neighbors(
      int node_id, bool include_blocked = false) const;

  std::vector<GraphNode> pending_rooms(int floor, bool descending = true) const;

  // Deterministic DFS node order without changing node state.
  std::vector<int> dfs_order(int start_node_id,
                             std::optional<int> floor = std::nullopt);

  std::map<int, GraphNode> nodes;
  std::map<int, GraphEdge> edges;

 private:
  // Nearest same-type/same-floor node within merge_distance, or -1.
  int nearest_node(const std::string& node_type, int floor, double x,
                   double y) const;

  double merge_distance_;
  int next_node_id_ = 0;
  int next_edge_id_ = 0;
};

}  // namespace competition_navigation_fastlio2_cpp
