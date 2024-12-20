// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/profiler_metrics_ios.h"

#import <Foundation/Foundation.h>

#import "flutter/fml/platform/darwin/cf_utils.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/IOKit.h"

FLUTTER_ASSERT_ARC

namespace {

// RAII holder for `thread_array_t` this is so any early returns in
// `ProfilerMetricsIOS::CpuUsage` don't leak them.
class MachThreads {
 public:
  thread_array_t threads = NULL;
  mach_msg_type_number_t thread_count = 0;

  MachThreads() = default;

  ~MachThreads() {
    [[maybe_unused]] kern_return_t kernel_return_code = vm_deallocate(
        mach_task_self(), reinterpret_cast<vm_offset_t>(threads), thread_count * sizeof(thread_t));
    FML_DCHECK(kernel_return_code == KERN_SUCCESS) << "Failed to deallocate thread infos.";
  }

 private:
  FML_DISALLOW_COPY_AND_ASSIGN(MachThreads);
};

}  // namespace

#if FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG || \
    FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_PROFILE

namespace fml {

/// fml::CFRef retain and release implementations for io_object_t and related types.
template <>
struct CFRefTraits<io_object_t> {
  static constexpr io_object_t kNullValue = 0;
  static void Retain(io_object_t instance) { IOObjectRetain(instance); }
  static void Release(io_object_t instance) { IOObjectRelease(instance); }
};

}  // namespace fml

#endif

namespace flutter {
namespace {

#if FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG || \
    FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_PROFILE

std::optional<GpuUsageInfo> FindGpuUsageInfo(io_iterator_t iterator) {
  for (fml::CFRef<io_registry_entry_t> reg_entry(IOIteratorNext(iterator)); reg_entry.Get();
       reg_entry.Reset(IOIteratorNext(iterator))) {
    CFMutableDictionaryRef cf_service_dictionary;
    if (IORegistryEntryCreateCFProperties(reg_entry.Get(), &cf_service_dictionary,
                                          kCFAllocatorDefault, kNilOptions) != kIOReturnSuccess) {
      continue;
    }
    // Transfer ownership to ARC-managed pointer.
    NSDictionary* service_dictionary = (__bridge_transfer NSDictionary*)cf_service_dictionary;
    cf_service_dictionary = nullptr;
    NSDictionary* performanceStatistics = service_dictionary[@"PerformanceStatistics"];
    NSNumber* utilization = performanceStatistics[@"Device Utilization %"];
    if (utilization) {
      return (GpuUsageInfo){.percent_usage = [utilization doubleValue]};
    }
  }
  return std::nullopt;
}

[[maybe_unused]] std::optional<GpuUsageInfo> FindSimulatorGpuUsageInfo() {
  io_iterator_t io_iterator;
  if (IOServiceGetMatchingServices(kIOMasterPortDefault, IOServiceNameMatching("IntelAccelerator"),
                                   &io_iterator) == kIOReturnSuccess) {
    fml::CFRef<io_iterator_t> iterator(io_iterator);
    return FindGpuUsageInfo(iterator.Get());
  }
  return std::nullopt;
}

[[maybe_unused]] std::optional<GpuUsageInfo> FindDeviceGpuUsageInfo() {
  io_iterator_t io_iterator;
  if (IOServiceGetMatchingServices(kIOMasterPortDefault, IOServiceNameMatching("sgx"),
                                   &io_iterator) == kIOReturnSuccess) {
    fml::CFRef<io_iterator_t> iterator(io_iterator);
    for (fml::CFRef<io_registry_entry_t> reg_entry(IOIteratorNext(iterator.Get())); reg_entry.Get();
         reg_entry.Reset(IOIteratorNext(iterator.Get()))) {
      io_iterator_t io_inner_iterator;
      if (IORegistryEntryGetChildIterator(reg_entry.Get(), kIOServicePlane, &io_inner_iterator) ==
          kIOReturnSuccess) {
        fml::CFRef<io_iterator_t> inner_iterator(io_inner_iterator);
        std::optional<GpuUsageInfo> result = FindGpuUsageInfo(inner_iterator.Get());
        if (result.has_value()) {
          return result;
        }
      }
    }
  }
  return std::nullopt;
}

#endif  // FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_DEBUG ||
        // FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_PROFILE

std::optional<GpuUsageInfo> PollGpuUsage() {
#if (FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_RELEASE || \
     FLUTTER_RUNTIME_MODE == FLUTTER_RUNTIME_MODE_JIT_RELEASE)
  return std::nullopt;
#elif TARGET_IPHONE_SIMULATOR
  return FindSimulatorGpuUsageInfo();
#elif TARGET_OS_IOS
  return FindDeviceGpuUsageInfo();
#endif  // TARGET_IPHONE_SIMULATOR
}
}  // namespace

ProfileSample ProfilerMetricsIOS::GenerateSample() {
  return {.cpu_usage = CpuUsage(), .memory_usage = MemoryUsage(), .gpu_usage = PollGpuUsage()};
}

std::optional<CpuUsageInfo> ProfilerMetricsIOS::CpuUsage() {
  kern_return_t kernel_return_code;
  MachThreads mach_threads = MachThreads();

  // Get threads in the task
  kernel_return_code =
      task_threads(mach_task_self(), &mach_threads.threads, &mach_threads.thread_count);
  if (kernel_return_code != KERN_SUCCESS) {
    return std::nullopt;
  }

  double total_cpu_usage = 0.0;
  uint32_t num_threads = mach_threads.thread_count;

  // Add the CPU usage for each thread. It should be noted that there may be some CPU usage missing
  // from this calculation. If a thread ends between calls to this routine, then its info will be
  // lost. We could solve this by installing a callback using pthread_key_create. The callback would
  // report the thread is ending and allow the code to get the CPU usage. But we need to call
  // pthread_setspecific in each thread to set the key's value to a non-null value for the callback
  // to work. If we really need this information and if we have a good mechanism for calling
  // pthread_setspecific in every thread, then we can include that value in the CPU usage.
  for (mach_msg_type_number_t i = 0; i < mach_threads.thread_count; i++) {
    thread_basic_info_data_t basic_thread_info;
    mach_msg_type_number_t thread_info_count = THREAD_BASIC_INFO_COUNT;
    kernel_return_code =
        thread_info(mach_threads.threads[i], THREAD_BASIC_INFO,
                    reinterpret_cast<thread_info_t>(&basic_thread_info), &thread_info_count);
    switch (kernel_return_code) {
      case KERN_SUCCESS: {
        const double current_thread_cpu_usage =
            basic_thread_info.cpu_usage / static_cast<float>(TH_USAGE_SCALE);
        total_cpu_usage += current_thread_cpu_usage;
        break;
      }
      case MACH_SEND_TIMEOUT:
      case MACH_SEND_TIMED_OUT:
      case MACH_SEND_INVALID_DEST:
        // Ignore as this thread been destroyed. The possible return codes are not really well
        // documented. This handling is inspired from the following sources:
        // - https://opensource.apple.com/source/xnu/xnu-4903.221.2/tests/task_inspect.c.auto.html
        // - https://github.com/apple/swift-corelibs-libdispatch/blob/main/src/queue.c#L6617
        num_threads--;
        break;
      default:
        return std::nullopt;
    }
  }

  flutter::CpuUsageInfo cpu_usage_info = {.num_threads = num_threads,
                                          .total_cpu_usage = total_cpu_usage * 100.0};
  return cpu_usage_info;
}

std::optional<MemoryUsageInfo> ProfilerMetricsIOS::MemoryUsage() {
  kern_return_t kernel_return_code;
  task_vm_info_data_t task_memory_info;
  mach_msg_type_number_t task_memory_info_count = TASK_VM_INFO_COUNT;

  kernel_return_code =
      task_info(mach_task_self(), TASK_VM_INFO, reinterpret_cast<task_info_t>(&task_memory_info),
                &task_memory_info_count);
  if (kernel_return_code != KERN_SUCCESS) {
    return std::nullopt;
  }

  // `phys_footprint` is Apple's recommended way to measure app's memory usage. It provides the
  // best approximate to xcode memory gauge. According to its source code explanation, the physical
  // footprint mainly consists of app's internal memory data and IOKit mappings. `resident_size`
  // is the total physical memory used by the app, so we simply do `resident_size - phys_footprint`
  // to obtain the shared memory usage.
  const double dirty_memory_usage =
      static_cast<double>(task_memory_info.phys_footprint) / 1024.0 / 1024.0;
  const double owned_shared_memory_usage =
      static_cast<double>(task_memory_info.resident_size) / 1024.0 / 1024.0 - dirty_memory_usage;
  flutter::MemoryUsageInfo memory_usage_info = {
      .dirty_memory_usage = dirty_memory_usage,
      .owned_shared_memory_usage = owned_shared_memory_usage};
  return memory_usage_info;
}

}  // namespace flutter
