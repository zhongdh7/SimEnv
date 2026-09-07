// C++ port of HDPlannerPolicy (ctypes) using dlopen/dlsym.
#include "competition_navigation_fastlio2_cpp/hdplanner_policy.h"

#include <dlfcn.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#include <ros/console.h>

namespace competition_navigation_fastlio2_cpp {

namespace {
constexpr int64_t kErrorCapacity = 4096;

void set_error(char* buffer, int64_t capacity, const std::string& message) {
  if (buffer == nullptr || capacity <= 0) return;
  const int64_t count =
      std::min<int64_t>(capacity - 1, static_cast<int64_t>(message.size()));
  std::memcpy(buffer, message.data(), static_cast<size_t>(count));
  buffer[count] = '\0';
}
}  // namespace

HdplannerPolicy::HdplannerPolicy(const std::string& library_path,
                                 const std::string& model_path) {
  try {
    library_ = dlopen(library_path.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (library_ == nullptr) {
      throw std::runtime_error(dlerror() ? dlerror() : "dlopen failed");
    }
    create_ = reinterpret_cast<CreateFn>(dlsym(library_, "hdplanner_create"));
    destroy_ = reinterpret_cast<DestroyFn>(dlsym(library_, "hdplanner_destroy"));
    select_ = reinterpret_cast<SelectFn>(dlsym(library_, "hdplanner_select"));
    if (create_ == nullptr || destroy_ == nullptr || select_ == nullptr) {
      throw std::runtime_error("missing HDPlanner symbol(s)");
    }
    char error[kErrorCapacity];
    error[0] = '\0';
    handle_ = create_(model_path.c_str(), error, kErrorCapacity);
    if (handle_ == nullptr) {
      throw std::runtime_error(error[0] ? error : "hdplanner_create failed");
    }
    ready = true;
    reason = "torchscript_checkpoint_loaded";
    ROS_INFO("HDPlanner TorchScript policy loaded from %s", model_path.c_str());
  } catch (const std::exception& exc) {
    close();
    reason = std::string(typeid(exc).name()) + ": " + exc.what();
    ROS_ERROR("HDPlanner policy unavailable: %s", reason.c_str());
  }
}

HdplannerPolicy::~HdplannerPolicy() { close(); }

void HdplannerPolicy::close() {
  if (destroy_ != nullptr && handle_ != nullptr) {
    destroy_(handle_);
  }
  handle_ = nullptr;
  if (library_ != nullptr) {
    dlclose(library_);
    library_ = nullptr;
  }
  create_ = nullptr;
  destroy_ = nullptr;
  select_ = nullptr;
  ready = false;
}

int64_t HdplannerPolicy::select(
    const std::vector<std::pair<double, double>>& nodes,
    const std::vector<int>& utility, const std::vector<int>& visited,
    int64_t current_index, int64_t target_index,
    const std::vector<int64_t>& neighbors, const std::vector<int64_t>& centers,
    const std::vector<std::vector<int64_t>>& adjacency) {
  const int64_t node_count = static_cast<int64_t>(nodes.size());
  if (!ready || neighbors.empty() || node_count >= NODE_COUNT) {
    return -1;
  }

  std::pair<double, double> current = nodes[current_index];
  std::pair<double, double> target = nodes[target_index];

  // features: NODE_COUNT x FEATURE_COUNT, float32.
  std::vector<float> features(NODE_COUNT * FEATURE_COUNT, 0.0f);
  std::vector<int64_t> center_set = centers;
  for (int64_t index = 0; index < node_count; ++index) {
    const std::pair<double, double>& node = nodes[index];
    features[index * FEATURE_COUNT + 0] =
        static_cast<float>((node.first - current.first) / 60.0);
    features[index * FEATURE_COUNT + 1] =
        static_cast<float>((node.second - current.second) / 60.0);
    features[index * FEATURE_COUNT + 2] = utility[index] > 0 ? 1.0f : 0.0f;
    features[index * FEATURE_COUNT + 3] = visited[index] ? 1.0f : 0.0f;
    features[index * FEATURE_COUNT + 4] =
        static_cast<float>((target.first - current.first) / 60.0);
    features[index * FEATURE_COUNT + 5] =
        static_cast<float>((target.second - current.second) / 60.0);
    features[index * FEATURE_COUNT + 6] =
        std::find(center_set.begin(), center_set.end(), index) !=
                center_set.end()
            ? 1.0f
            : 0.0f;
  }

  // edge_inputs / edge_padding (K_SIZE each).
  std::vector<int64_t> edge_inputs(K_SIZE, current_index);
  std::vector<int16_t> edge_padding(K_SIZE, 1);
  std::vector<int64_t> valid_neighbors;
  for (int64_t index : neighbors) {
    if (index != current_index) valid_neighbors.push_back(index);
  }
  for (size_t i = 0;
       i < valid_neighbors.size() && i + 1 < static_cast<size_t>(K_SIZE);
       ++i) {
    edge_inputs[i + 1] = valid_neighbors[i];
    edge_padding[i + 1] = 0;
  }
  edge_padding[0] = 1;

  // center_inputs / center_padding (K_SIZE each).
  std::vector<int64_t> center_indices;
  for (int64_t index : centers) {
    if (center_indices.size() >= static_cast<size_t>(K_SIZE)) break;
    center_indices.push_back(index);
  }
  if (center_indices.empty()) center_indices.push_back(target_index);
  std::vector<int64_t> center_inputs(K_SIZE, NODE_COUNT - 1);
  std::vector<int64_t> center_padding(K_SIZE, 1);
  for (size_t offset = 0; offset < center_indices.size(); ++offset) {
    center_inputs[offset] = center_indices[offset];
    center_padding[offset] = 0;
  }

  // node_padding (NODE_COUNT).
  std::vector<int16_t> node_padding(NODE_COUNT, 1);
  for (int64_t i = 0; i < node_count; ++i) node_padding[i] = 0;

  // edge_mask (NODE_COUNT x NODE_COUNT).
  std::vector<int64_t> edge_mask(NODE_COUNT * NODE_COUNT, 1);
  for (int64_t index = 0; index < static_cast<int64_t>(adjacency.size());
       ++index) {
    edge_mask[index * NODE_COUNT + index] = 0;
    for (int64_t neighbor : adjacency[index]) {
      edge_mask[index * NODE_COUNT + neighbor] = 0;
    }
  }

  // NaN/Inf guard on features (Python checked np.isfinite(features)).
  for (float value : features) {
    if (!std::isfinite(value)) {
      ROS_ERROR_THROTTLE(5.0, "HDPlanner observation contains NaN/Inf");
      return -1;
    }
  }

  int64_t selected = -1;
  int64_t selected_center = -1;
  float action_logp = std::numeric_limits<float>::quiet_NaN();
  char error[kErrorCapacity];
  error[0] = '\0';
  int result = select_(handle_, features.data(), edge_inputs.data(),
                       current_index, target_index, center_inputs.data(),
                       node_padding.data(), edge_padding.data(),
                       edge_mask.data(), center_padding.data(), &selected,
                       &selected_center, &action_logp, error, kErrorCapacity);
  if (result != 0) {
    ROS_ERROR_THROTTLE(5.0, "HDPlanner inference failed (%d): %s", result,
                       error[0] ? error : "");
    return -1;
  }
  const int64_t selected_index = selected;
  if (std::find(valid_neighbors.begin(), valid_neighbors.end(),
                selected_index) == valid_neighbors.end()) {
    ROS_ERROR_THROTTLE(5.0, "HDPlanner selected non-neighbor node %ld",
                       static_cast<long>(selected_index));
    return -1;
  }
  if (!std::isfinite(action_logp)) {
    ROS_ERROR_THROTTLE(5.0, "HDPlanner action contains NaN/Inf");
    return -1;
  }
  inference_count += 1;
  last_selected_center = selected_center;
  last_action_logp = static_cast<double>(action_logp);
  return selected_index;
}

}  // namespace competition_navigation_fastlio2_cpp
