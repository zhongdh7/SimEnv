#include <torch/script.h>
#include <torch/torch.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace {
constexpr int64_t kNodeCount = 360;
constexpr int64_t kNodeFeatures = 7;
constexpr int64_t kNeighborCount = 25;

struct HDPlannerHandle {
  explicit HDPlannerHandle(const std::string& path)
      : module(torch::jit::load(path, torch::kCPU)) {
    module.eval();
  }

  torch::jit::Module module;
  std::mutex mutex;
};

void setError(char* buffer, int64_t capacity, const std::string& message) {
  if (buffer == nullptr || capacity <= 0) {
    return;
  }
  const auto count = std::min<int64_t>(capacity - 1, message.size());
  std::memcpy(buffer, message.data(), static_cast<size_t>(count));
  buffer[count] = '\0';
}

template <typename T>
bool required(const T* pointer, const char* name, char* error,
              int64_t errorCapacity) {
  if (pointer != nullptr) {
    return true;
  }
  setError(error, errorCapacity, std::string(name) + " is null");
  return false;
}
}  // namespace

extern "C" void* hdplanner_create(const char* modelPath, char* error,
                                    int64_t errorCapacity) noexcept {
  try {
    if (!required(modelPath, "modelPath", error, errorCapacity)) {
      return nullptr;
    }
    torch::set_num_threads(1);
    torch::set_num_interop_threads(1);
    auto handle = std::make_unique<HDPlannerHandle>(modelPath);
    setError(error, errorCapacity, "");
    return handle.release();
  } catch (const c10::Error& exception) {
    setError(error, errorCapacity, exception.what_without_backtrace());
  } catch (const std::exception& exception) {
    setError(error, errorCapacity, exception.what());
  } catch (...) {
    setError(error, errorCapacity, "unknown model load error");
  }
  return nullptr;
}

extern "C" void hdplanner_destroy(void* opaqueHandle) noexcept {
  delete static_cast<HDPlannerHandle*>(opaqueHandle);
}

extern "C" int hdplanner_select(
    void* opaqueHandle, const float* nodeInputs, const int64_t* edgeInputs,
    int64_t currentIndex, int64_t targetIndex, const int64_t* centerInputs,
    const int16_t* nodePadding, const int16_t* edgePadding,
    const int64_t* edgeMask, const int64_t* centerPadding,
    int64_t* selectedNode, int64_t* selectedCenter, float* actionLogProbability,
    char* error, int64_t errorCapacity) noexcept {
  try {
    if (!required(opaqueHandle, "handle", error, errorCapacity) ||
        !required(nodeInputs, "nodeInputs", error, errorCapacity) ||
        !required(edgeInputs, "edgeInputs", error, errorCapacity) ||
        !required(centerInputs, "centerInputs", error, errorCapacity) ||
        !required(nodePadding, "nodePadding", error, errorCapacity) ||
        !required(edgePadding, "edgePadding", error, errorCapacity) ||
        !required(edgeMask, "edgeMask", error, errorCapacity) ||
        !required(centerPadding, "centerPadding", error, errorCapacity) ||
        !required(selectedNode, "selectedNode", error, errorCapacity) ||
        !required(selectedCenter, "selectedCenter", error, errorCapacity) ||
        !required(actionLogProbability, "actionLogProbability", error,
                  errorCapacity)) {
      return -1;
    }
    if (currentIndex < 0 || currentIndex >= kNodeCount || targetIndex < 0 ||
        targetIndex >= kNodeCount) {
      setError(error, errorCapacity, "current/target index is out of range");
      return -2;
    }

    auto* handle = static_cast<HDPlannerHandle*>(opaqueHandle);
    std::lock_guard<std::mutex> lock(handle->mutex);
    c10::InferenceMode inferenceGuard;

    const auto floatOptions = torch::TensorOptions().dtype(torch::kFloat32);
    const auto longOptions = torch::TensorOptions().dtype(torch::kInt64);
    const auto shortOptions = torch::TensorOptions().dtype(torch::kInt16);
    auto nodes = torch::from_blob(const_cast<float*>(nodeInputs),
                                  {1, kNodeCount, kNodeFeatures}, floatOptions);
    auto edges = torch::from_blob(const_cast<int64_t*>(edgeInputs),
                                  {1, kNeighborCount, 1}, longOptions);
    auto current = torch::tensor({{{currentIndex}}}, longOptions);
    auto target = torch::tensor({{{targetIndex}}}, longOptions);
    auto centers = torch::from_blob(const_cast<int64_t*>(centerInputs),
                                    {1, 1, kNeighborCount}, longOptions);
    auto nodePad = torch::from_blob(const_cast<int16_t*>(nodePadding),
                                    {1, 1, kNodeCount}, shortOptions);
    auto edgePad = torch::from_blob(const_cast<int16_t*>(edgePadding),
                                    {1, 1, kNeighborCount}, shortOptions)
                       .clone();
    auto graphMask = torch::from_blob(const_cast<int64_t*>(edgeMask),
                                      {1, kNodeCount, kNodeCount}, longOptions);
    auto centerPad = torch::from_blob(const_cast<int64_t*>(centerPadding),
                                      {1, 1, kNeighborCount}, longOptions);

    std::vector<torch::jit::IValue> inputs{
        nodes, edges, current, target, centers,
        nodePad, edgePad, graphMask, centerPad,
    };
    auto output = handle->module.forward(inputs).toTuple();
    const auto& values = output->elements();
    if (values.size() != 8) {
      setError(error, errorCapacity, "unexpected HDPlanner output tuple size");
      return -3;
    }
    const auto actionLogp = values.at(1).toTensor();
    const auto centerIndex = values.at(2).toTensor();
    const auto actionIndex = values.at(3).toTensor();
    if (!torch::isfinite(actionLogp).all().item<bool>()) {
      setError(error, errorCapacity, "HDPlanner output contains NaN/Inf");
      return -4;
    }

    *selectedNode = actionIndex.item<int64_t>();
    *selectedCenter = centerIndex.item<int64_t>();
    *actionLogProbability = actionLogp.max().item<float>();
    if (*selectedNode < 0 || *selectedNode >= kNodeCount ||
        *selectedCenter < 0 || *selectedCenter >= kNodeCount) {
      setError(error, errorCapacity, "HDPlanner selected an invalid node");
      return -5;
    }
    setError(error, errorCapacity, "");
    return 0;
  } catch (const c10::Error& exception) {
    setError(error, errorCapacity, exception.what_without_backtrace());
  } catch (const std::exception& exception) {
    setError(error, errorCapacity, exception.what());
  } catch (...) {
    setError(error, errorCapacity, "unknown inference error");
  }
  return -10;
}
