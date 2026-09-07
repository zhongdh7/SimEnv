// C++ port of the HDPlannerPolicy class in competition_navigation_node.py.
// Runs the published HDPlanner PolicyNet through the same LibTorch C bridge
// (libhdplanner_inference.so) that the Python version loaded via ctypes.CDLL.
// Here it is loaded at runtime with dlopen/dlsym from the ``library_path``
// roslaunch param, so the .so is still produced by the competition_navigation
// package (no CUDA/libtorch rebuild, no symbol collision).
#pragma once

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace competition_navigation_fastlio2_cpp {

class HdplannerPolicy {
 public:
  static constexpr int64_t NODE_COUNT = 360;
  static constexpr int64_t FEATURE_COUNT = 7;
  static constexpr int64_t K_SIZE = 25;

  HdplannerPolicy(const std::string& library_path,
                  const std::string& model_path);
  ~HdplannerPolicy();

  HdplannerPolicy(const HdplannerPolicy&) = delete;
  HdplannerPolicy& operator=(const HdplannerPolicy&) = delete;

  void close();

  bool ready = false;
  std::string reason = "disabled";
  int inference_count = 0;
  int64_t last_selected_center = -1;
  double last_action_logp = std::numeric_limits<double>::quiet_NaN();

  // Runs the policy and returns the selected neighbor node index, or -1 when
  // the policy is unavailable / returns no valid neighbor (Python returned
  // None).  ``nodes`` is an Nx2 list of (x, y) positions.
  int64_t select(const std::vector<std::pair<double, double>>& nodes,
                 const std::vector<int>& utility,
                 const std::vector<int>& visited, int64_t current_index,
                 int64_t target_index, const std::vector<int64_t>& neighbors,
                 const std::vector<int64_t>& centers,
                 const std::vector<std::vector<int64_t>>& adjacency);

 private:
  using CreateFn = void* (*)(const char*, char*, int64_t);
  using DestroyFn = void (*)(void*);
  using SelectFn = int (*)(void*, const float*, const int64_t*, int64_t,
                           int64_t, const int64_t*, const int16_t*,
                           const int16_t*, const int64_t*, const int64_t*,
                           int64_t*, int64_t*, float*, char*, int64_t);

  void* library_ = nullptr;
  void* handle_ = nullptr;
  CreateFn create_ = nullptr;
  DestroyFn destroy_ = nullptr;
  SelectFn select_ = nullptr;
};

}  // namespace competition_navigation_fastlio2_cpp
