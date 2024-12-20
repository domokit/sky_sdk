// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/common/skia_event_tracer_impl.h"

#define TRACE_EVENT_HIDE_MACROS
#include <map>
#include <set>
#include <vector>

#include "flutter/fml/logging.h"
#include "flutter/fml/posix_wrappers.h"
#include "flutter/fml/trace_event.h"
#include "third_party/dart/runtime/include/dart_tools_api.h"
#include "third_party/skia/include/utils/SkEventTracer.h"
#include "third_party/skia/include/utils/SkTraceEventPhase.h"

#if defined(OS_FUCHSIA)

#include <algorithm>
#include <cstring>

#include <lib/trace-engine/context.h>
#include <lib/trace-engine/instrumentation.h>

// Skia's copy of these flags are defined in a private header, so, as is
// commonly done with "trace_event_common.h" values, copy them inline here (see
// https://cs.chromium.org/chromium/src/base/trace_event/common/trace_event_common.h?l=1102-1110&rcl=239b85aeb3a6c07b33b5f162cd0ae8128eabf44d).
//
// Type values for identifying types in the TraceValue union.
#define TRACE_VALUE_TYPE_BOOL (static_cast<unsigned char>(1))
#define TRACE_VALUE_TYPE_UINT (static_cast<unsigned char>(2))
#define TRACE_VALUE_TYPE_INT (static_cast<unsigned char>(3))
#define TRACE_VALUE_TYPE_DOUBLE (static_cast<unsigned char>(4))
#define TRACE_VALUE_TYPE_POINTER (static_cast<unsigned char>(5))
#define TRACE_VALUE_TYPE_STRING (static_cast<unsigned char>(6))
#define TRACE_VALUE_TYPE_COPY_STRING (static_cast<unsigned char>(7))
#define TRACE_VALUE_TYPE_CONVERTABLE (static_cast<unsigned char>(8))

#endif  // defined(OS_FUCHSIA)

namespace flutter {

namespace {

// Skia prepends this string to the category names of its trace events.
// Defined in Skia's src/core/SkTraceEvent.h.
constexpr std::string_view kTraceCategoryPrefix = "disabled-by-default-";

// Category name used for shader compilation events.
constexpr std::string_view kShaderCategoryName =
    "disabled-by-default-skia.shaders";

#if !defined(OS_FUCHSIA)
// Argument name of the tag used by DevTools.
constexpr char kDevtoolsTagArg[] = "devtoolsTag";

// DevtoolsTag value for shader events.
constexpr char kShadersDevtoolsTag[] = "shaders";
#endif  // !defined(OS_FUCHSIA)

#if defined(OS_FUCHSIA)
template <class T, class U>
inline T BitCast(const U& u) {
  static_assert(sizeof(T) == sizeof(U));

  T t;
  memcpy(&t, &u, sizeof(t));
  return t;
}
#endif  // defined(OS_FUCHSIA)

}  // namespace

class FlutterEventTracer : public SkEventTracer {
 public:
  static constexpr const char* kSkiaTag = "skia";
  static constexpr uint8_t kYes = 1;
  static constexpr uint8_t kNo = 0;

  FlutterEventTracer(bool enabled,
                     const std::optional<std::vector<std::string>>& allowlist)
      : enabled_(enabled ? kYes : kNo) {
    if (allowlist.has_value()) {
      allowlist_.emplace();
      for (const std::string& category : *allowlist) {
        allowlist_->insert(std::string(kTraceCategoryPrefix) + category);
      }
    }
  };

  SkEventTracer::Handle addTraceEvent(char phase,
                                      const uint8_t* category_enabled_flag,
                                      const char* name,
                                      uint64_t id,
                                      int num_args,
                                      const char** p_arg_names,
                                      const uint8_t* p_arg_types,
                                      const uint64_t* p_arg_values,
                                      uint8_t flags) override {
#if defined(OS_FUCHSIA)
    static trace_site_t trace_site;
    trace_string_ref_t category_ref;
    trace_context_t* trace_context = trace_acquire_context_for_category_cached(
        kSkiaTag, &trace_site, &category_ref);

    if (likely(!trace_context)) {
      return 0;
    }

    trace_ticks_t ticks = zx_ticks_get();

    trace_thread_ref_t thread_ref;
    trace_context_register_current_thread(trace_context, &thread_ref);
    trace_string_ref_t name_ref;
    trace_context_register_string_literal(trace_context, name, &name_ref);

    constexpr int kMaxArgs = 2;
    trace_arg_t trace_args[kMaxArgs] = {};
    FML_DCHECK(num_args >= 0);
    int num_trace_args = std::min(kMaxArgs, num_args);

    for (int i = 0; i < num_trace_args; i++) {
      const char* arg_name = p_arg_names[i];
      const uint8_t arg_type = p_arg_types[i];
      const uint64_t arg_value = p_arg_values[i];

      trace_string_ref_t arg_name_string_ref =
          trace_context_make_registered_string_literal(trace_context, arg_name);

      trace_arg_value_t trace_arg_value;
      switch (arg_type) {
        case TRACE_VALUE_TYPE_BOOL: {
          trace_arg_value = trace_make_bool_arg_value(!!arg_value);
          break;
        }
        case TRACE_VALUE_TYPE_UINT:
          trace_arg_value = trace_make_uint64_arg_value(arg_value);
          break;
        case TRACE_VALUE_TYPE_INT:
          trace_arg_value =
              trace_make_int64_arg_value(BitCast<int64_t>(arg_value));
          break;
        case TRACE_VALUE_TYPE_DOUBLE:
          trace_arg_value =
              trace_make_double_arg_value(BitCast<double>(arg_value));
          break;
        case TRACE_VALUE_TYPE_POINTER:
          trace_arg_value =
              trace_make_pointer_arg_value(BitCast<uintptr_t>(arg_value));
          break;
        case TRACE_VALUE_TYPE_STRING: {
          trace_string_ref_t arg_value_string_ref =
              trace_context_make_registered_string_literal(
                  trace_context, reinterpret_cast<const char*>(arg_value));
          trace_arg_value = trace_make_string_arg_value(arg_value_string_ref);
          break;
        }
        case TRACE_VALUE_TYPE_COPY_STRING: {
          const char* arg_value_as_cstring =
              reinterpret_cast<const char*>(arg_value);
          trace_string_ref_t arg_value_string_ref =
              trace_context_make_registered_string_copy(
                  trace_context, arg_value_as_cstring,
                  strlen(arg_value_as_cstring));
          trace_arg_value = trace_make_string_arg_value(arg_value_string_ref);
          break;
        }
        case TRACE_VALUE_TYPE_CONVERTABLE:
          trace_arg_value = trace_make_null_arg_value();
          break;
        default:
          trace_arg_value = trace_make_null_arg_value();
      }

      trace_args[i] = trace_make_arg(arg_name_string_ref, trace_arg_value);
    }

    switch (phase) {
      case TRACE_EVENT_PHASE_BEGIN:
      case TRACE_EVENT_PHASE_COMPLETE:
        trace_context_write_duration_begin_event_record(
            trace_context, ticks, &thread_ref, &category_ref, &name_ref,
            trace_args, num_trace_args);
        break;
      case TRACE_EVENT_PHASE_END:
        trace_context_write_duration_end_event_record(
            trace_context, ticks, &thread_ref, &category_ref, &name_ref,
            trace_args, num_trace_args);
        break;
      case TRACE_EVENT_PHASE_INSTANT:
        trace_context_write_instant_event_record(
            trace_context, ticks, &thread_ref, &category_ref, &name_ref,
            TRACE_SCOPE_THREAD, trace_args, num_trace_args);
        break;
      case TRACE_EVENT_PHASE_ASYNC_BEGIN:
        trace_context_write_async_begin_event_record(
            trace_context, ticks, &thread_ref, &category_ref, &name_ref, id,
            trace_args, num_trace_args);
        break;
      case TRACE_EVENT_PHASE_ASYNC_END:
        trace_context_write_async_end_event_record(
            trace_context, ticks, &thread_ref, &category_ref, &name_ref, id,
            trace_args, num_trace_args);
        break;
      default:
        break;
    }

    trace_release_context(trace_context);

#else   // defined(OS_FUCHSIA)
    const char* devtoolsTag = nullptr;
    if (shaders_category_flag_ &&
        category_enabled_flag == shaders_category_flag_) {
      devtoolsTag = kShadersDevtoolsTag;
    }
    switch (phase) {
      case TRACE_EVENT_PHASE_BEGIN:
      case TRACE_EVENT_PHASE_COMPLETE:
        if (devtoolsTag) {
          fml::tracing::TraceEvent1(kSkiaTag, name, /*flow_id_count=*/0,
                                    /*flow_ids=*/nullptr, kDevtoolsTagArg,
                                    devtoolsTag);
        } else {
          fml::tracing::TraceEvent0(kSkiaTag, name, /*flow_id_count=*/0,
                                    /*flow_ids=*/nullptr);
        }
        break;
      case TRACE_EVENT_PHASE_END:
        fml::tracing::TraceEventEnd(name);
        break;
      case TRACE_EVENT_PHASE_INSTANT:
        if (devtoolsTag) {
          fml::tracing::TraceEventInstant1(kSkiaTag, name,
                                           /*flow_id_count=*/0,
                                           /*flow_ids=*/nullptr,
                                           kDevtoolsTagArg, devtoolsTag);
        } else {
          fml::tracing::TraceEventInstant0(kSkiaTag, name,
                                           /*flow_id_count=*/0,
                                           /*flow_ids=*/nullptr);
        }
        break;
      case TRACE_EVENT_PHASE_ASYNC_BEGIN:
        if (devtoolsTag) {
          fml::tracing::TraceEventAsyncBegin1(kSkiaTag, name, id,
                                              /*flow_id_count=*/0,
                                              /*flow_ids=*/nullptr,
                                              kDevtoolsTagArg, devtoolsTag);
        } else {
          fml::tracing::TraceEventAsyncBegin0(kSkiaTag, name, id,
                                              /*flow_id_count=*/0,
                                              /*flow_ids=*/nullptr);
        }
        break;
      case TRACE_EVENT_PHASE_ASYNC_END:
        if (devtoolsTag) {
          fml::tracing::TraceEventAsyncEnd1(kSkiaTag, name, id, kDevtoolsTagArg,
                                            devtoolsTag);
        } else {
          fml::tracing::TraceEventAsyncEnd0(kSkiaTag, name, id);
        }
        break;
      default:
        break;
    }
#endif  // defined(OS_FUCHSIA)
    return 0;
  }

  void updateTraceEventDuration(const uint8_t* category_enabled_flag,
                                const char* name,
                                SkEventTracer::Handle handle) override {
    // This is only ever called from a scoped trace event so we will just end
    // the section.
#if defined(OS_FUCHSIA)
    TRACE_DURATION_END(kSkiaTag, name);
#else
    fml::tracing::TraceEventEnd(name);
#endif
  }

  const uint8_t* getCategoryGroupEnabled(const char* name) override {
    // Skia will only use long-lived string literals as event names.
    std::lock_guard<std::mutex> lock(flag_map_mutex_);
    auto flag_it = category_flag_map_.find(name);
    if (flag_it == category_flag_map_.end()) {
      bool allowed;
      if (enabled_) {
        allowed = !allowlist_.has_value() ||
                  allowlist_->find(name) != allowlist_->end();
      } else {
        allowed = false;
      }
      flag_it = category_flag_map_.insert(std::make_pair(name, allowed)).first;
      const uint8_t* flag = &flag_it->second;
      reverse_flag_map_.insert(std::make_pair(flag, name));
      if (kShaderCategoryName == name) {
        shaders_category_flag_ = flag;
      }
    }
    return &flag_it->second;
  }

  const char* getCategoryGroupName(
      const uint8_t* category_enabled_flag) override {
    std::lock_guard<std::mutex> lock(flag_map_mutex_);
    auto reverse_it = reverse_flag_map_.find(category_enabled_flag);
    if (reverse_it != reverse_flag_map_.end()) {
      return reverse_it->second;
    } else {
      return kSkiaTag;
    }
  }

 private:
  uint8_t enabled_;
  std::optional<std::set<std::string>> allowlist_;
  std::mutex flag_map_mutex_;
  std::map<const char*, uint8_t> category_flag_map_;
  std::map<const uint8_t*, const char*> reverse_flag_map_;
  const uint8_t* shaders_category_flag_ = nullptr;
  FML_DISALLOW_COPY_AND_ASSIGN(FlutterEventTracer);
};

void InitSkiaEventTracer(
    bool enabled,
    const std::optional<std::vector<std::string>>& allowlist) {
  auto tracer = new FlutterEventTracer(enabled, allowlist);
  // Initialize the binding to Skia's tracing events. Skia will
  // take ownership of and clean up the memory allocated here.
  SkEventTracer::SetInstance(tracer);
}

}  // namespace flutter
