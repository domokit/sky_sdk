// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_TRACE_EVENT_H_
#define FLUTTER_FML_TRACE_EVENT_H_

#include <functional>

#include "flutter/fml/build_config.h"

#if defined(OS_FUCHSIA)

// Forward to the system tracing mechanism on Fuchsia.

#include <lib/trace/event.h>

// TODO(DNO-448): This is disabled because the Fuchsia counter id json parsing
// only handles ints whereas this can produce ints or strings.
#define FML_TRACE_COUNTER(a, b, c, arg1, ...) \
  ::fml::tracing::TraceCounterNopHACK((a), (b), (c), (arg1), __VA_ARGS__);

#define FML_TRACE_EVENT(a, b, args...) TRACE_DURATION(a, b)
// On Fuchsia, the flow_id arguments to this macro are ignored.
#define FML_TRACE_EVENT_WITH_FLOW_IDS(category_group, name, flow_id_count, \
                                      flow_ids, ...)                       \
  FML_TRACE_EVENT(category_group, name)

#define TRACE_EVENT0(a, b) TRACE_DURATION(a, b)
// On Fuchsia, the flow_id arguments to this macro are ignored.
#define TRACE_EVENT0_WITH_FLOW_IDS(category_group, name, flow_id_count, \
                                   flow_ids)                            \
  TRACE_EVENT0(category_group, name)
#define TRACE_EVENT1(a, b, c, d) TRACE_DURATION(a, b, c, d)
// On Fuchsia, the flow_id arguments to this macro are ignored.
#define TRACE_EVENT1_WITH_FLOW_IDS(category_group, name, flow_id_count, \
                                   flow_ids, arg1_name, arg1_val)       \
  TRACE_EVENT1(category_group, name, arg1_name, arg1_val)
#define TRACE_EVENT2(a, b, c, d, e, f) TRACE_DURATION(a, b, c, d, e, f)
#define TRACE_EVENT_ASYNC_BEGIN0(a, b, c) TRACE_ASYNC_BEGIN(a, b, c)
// On Fuchsia, the flow_id arguments to this macro are ignored.
#define TRACE_EVENT_ASYNC_BEGIN0_WITH_FLOW_IDS(category_group, name, id, \
                                               flow_id_count, flow_ids)  \
  TRACE_EVENT_ASYNC_BEGIN0(category_group, name, id)
#define TRACE_EVENT_ASYNC_END0(a, b, c) TRACE_ASYNC_END(a, b, c)
#define TRACE_EVENT_ASYNC_BEGIN1(a, b, c, d, e) TRACE_ASYNC_BEGIN(a, b, c, d, e)
#define TRACE_EVENT_ASYNC_END1(a, b, c, d, e) TRACE_ASYNC_END(a, b, c, d, e)
#define TRACE_EVENT_INSTANT0(a, b) TRACE_INSTANT(a, b, TRACE_SCOPE_THREAD)
#define TRACE_EVENT_INSTANT1(a, b, k1, v1) \
  TRACE_INSTANT(a, b, TRACE_SCOPE_THREAD, k1, v1)
#define TRACE_EVENT_INSTANT2(a, b, k1, v1, k2, v2) \
  TRACE_INSTANT(a, b, TRACE_SCOPE_THREAD, k1, v1, k2, v2)

#endif  //  defined(OS_FUCHSIA)

#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <vector>

#include "flutter/fml/macros.h"
#include "flutter/fml/time/time_point.h"
#include "third_party/dart/runtime/include/dart_tools_api.h"

#if (FLUTTER_RELEASE && !defined(OS_FUCHSIA) && !defined(FML_OS_ANDROID))
#define FLUTTER_TIMELINE_ENABLED 0
#else
#define FLUTTER_TIMELINE_ENABLED 1
#endif

#if !defined(OS_FUCHSIA)
#ifndef TRACE_EVENT_HIDE_MACROS

#define __FML__TOKEN_CAT__(x, y) x##y
#define __FML__TOKEN_CAT__2(x, y) __FML__TOKEN_CAT__(x, y)
#define __FML__AUTO_TRACE_END(name)                                  \
  ::fml::tracing::ScopedInstantEnd __FML__TOKEN_CAT__2(__trace_end_, \
                                                       __LINE__)(name);

// This macro has the FML_ prefix so that it does not collide with the macros
// from lib/trace/event.h on Fuchsia.
//
// TODO(chinmaygarde): All macros here should have the FML prefix.
#define FML_TRACE_COUNTER(category_group, name, counter_id, arg1, ...)         \
  ::fml::tracing::TraceCounter((category_group), (name), (counter_id), (arg1), \
                               __VA_ARGS__);

// Avoid using the same `name` and `argX_name` for nested traces, which can
// lead to double free errors. E.g. the following code should be avoided:
//
// ```cpp
// {
//    TRACE_EVENT1("flutter", "Foo::Bar", "count", "initial_count_value");
//    ...
//    TRACE_EVENT_INSTANT1("flutter", "Foo::Bar",
//                         "count", "updated_count_value");
// }
// ```
//
// Instead, either use different `name` or `arg1` parameter names.
#define FML_TRACE_EVENT_WITH_FLOW_IDS(category_group, name, flow_id_count, \
                                      flow_ids, ...)                       \
  ::fml::tracing::TraceEvent((category_group), (name), (flow_id_count),    \
                             (flow_ids), __VA_ARGS__);                     \
  __FML__AUTO_TRACE_END(name)

// Avoid using the same `name` and `argX_name` for nested traces, which can
// lead to double free errors. E.g. the following code should be avoided:
//
// ```cpp
// {
//    TRACE_EVENT1("flutter", "Foo::Bar", "count", "initial_count_value");
//    ...
//    TRACE_EVENT_INSTANT1("flutter", "Foo::Bar",
//                         "count", "updated_count_value");
// }
// ```
//
// Instead, either use different `name` or `arg1` parameter names.
#define FML_TRACE_EVENT(category_group, name, ...)                             \
  FML_TRACE_EVENT_WITH_FLOW_IDS((category_group), (name),                      \
                                /*flow_id_count=*/(0), /*flow_ids=*/(nullptr), \
                                __VA_ARGS__)

#define TRACE_EVENT0_WITH_FLOW_IDS(category_group, name, flow_id_count,       \
                                   flow_ids)                                  \
  ::fml::tracing::TraceEvent0(category_group, name, flow_id_count, flow_ids); \
  __FML__AUTO_TRACE_END(name)

#define TRACE_EVENT0(category_group, name)                              \
  TRACE_EVENT0_WITH_FLOW_IDS(category_group, name, /*flow_id_count=*/0, \
                             /*flow_ids=*/nullptr)

#define TRACE_EVENT1_WITH_FLOW_IDS(category_group, name, flow_id_count,      \
                                   flow_ids, arg1_name, arg1_val)            \
  ::fml::tracing::TraceEvent1(category_group, name, flow_id_count, flow_ids, \
                              arg1_name, arg1_val);                          \
  __FML__AUTO_TRACE_END(name)

#define TRACE_EVENT1(category_group, name, arg1_name, arg1_val)         \
  TRACE_EVENT1_WITH_FLOW_IDS(category_group, name, /*flow_id_count=*/0, \
                             /*flow_ids=*/nullptr, arg1_name, arg1_val)

#define TRACE_EVENT2(category_group, name, arg1_name, arg1_val, arg2_name, \
                     arg2_val)                                             \
  ::fml::tracing::TraceEvent2(category_group, name, /*flow_id_count=*/0,   \
                              /*flow_ids=*/nullptr, arg1_name, arg1_val,   \
                              arg2_name, arg2_val);                        \
  __FML__AUTO_TRACE_END(name)

#define TRACE_EVENT_ASYNC_BEGIN0_WITH_FLOW_IDS(category_group, name, id, \
                                               flow_id_count, flow_ids)  \
  ::fml::tracing::TraceEventAsyncBegin0(category_group, name, id,        \
                                        flow_id_count, flow_ids);

#define TRACE_EVENT_ASYNC_BEGIN0(category_group, name, id) \
  TRACE_EVENT_ASYNC_BEGIN0_WITH_FLOW_IDS(                  \
      category_group, name, id, /*flow_id_count=*/0, /*flow_ids=*/nullptr)

#define TRACE_EVENT_ASYNC_END0(category_group, name, id) \
  ::fml::tracing::TraceEventAsyncEnd0(category_group, name, id);

#define TRACE_EVENT_ASYNC_BEGIN1(category_group, name, id, arg1_name,      \
                                 arg1_val)                                 \
  ::fml::tracing::TraceEventAsyncBegin1(                                   \
      category_group, name, id, /*flow_id_count=*/0, /*flow_ids=*/nullptr, \
      arg1_name, arg1_val);

#define TRACE_EVENT_ASYNC_END1(category_group, name, id, arg1_name, arg1_val) \
  ::fml::tracing::TraceEventAsyncEnd1(                                        \
      category_group, name, id, /*flow_id_count=*/0, /*flow_ids=*/nullptr,    \
      arg1_name, arg1_val);

#define TRACE_EVENT_INSTANT0(category_group, name) \
  ::fml::tracing::TraceEventInstant0(              \
      category_group, name, /*flow_id_count=*/0, /*flow_ids=*/nullptr);

#define TRACE_EVENT_INSTANT1(category_group, name, arg1_name, arg1_val) \
  ::fml::tracing::TraceEventInstant1(                                   \
      category_group, name, /*flow_id_count=*/0, /*flow_ids=*/nullptr,  \
      arg1_name, arg1_val);

#define TRACE_EVENT_INSTANT2(category_group, name, arg1_name, arg1_val, \
                             arg2_name, arg2_val)                       \
  ::fml::tracing::TraceEventInstant2(                                   \
      category_group, name, /*flow_id_count=*/0, /*flow_ids=*/nullptr,  \
      arg1_name, arg1_val, arg2_name, arg2_val);

#define TRACE_FLOW_BEGIN(category, name, id) \
  ::fml::tracing::TraceEventFlowBegin0(category, name, id);

#define TRACE_FLOW_STEP(category, name, id) \
  ::fml::tracing::TraceEventFlowStep0(category, name, id);

#define TRACE_FLOW_END(category, name, id) \
  ::fml::tracing::TraceEventFlowEnd0(category, name, id);

#endif  // TRACE_EVENT_HIDE_MACROS
#endif  // !defined(OS_FUCHSIA)

#define TRACE_EVENT2_INT(category_group, name, arg1_name, arg1_val, arg2_name, \
                         arg2_val)                                             \
  const auto __arg1_val_str = std::to_string(arg1_val);                        \
  const auto __arg2_val_str = std::to_string(arg2_val);                        \
  TRACE_EVENT2(category_group, name, arg1_name, __arg1_val_str.c_str(),        \
               arg2_name, __arg2_val_str.c_str());

namespace fml {
namespace tracing {

using TraceArg = const char*;
using TraceIDArg = int64_t;

void TraceSetAllowlist(const std::vector<std::string>& allowlist);

typedef void (*TimelineEventHandler)(const char*,
                                     int64_t,
                                     int64_t,
                                     intptr_t,
                                     const int64_t*,
                                     Dart_Timeline_Event_Type,
                                     intptr_t,
                                     const char**,
                                     const char**);

using TimelineMicrosSource = int64_t (*)();

void TraceSetTimelineEventHandler(TimelineEventHandler handler);

bool TraceHasTimelineEventHandler();

void TraceSetTimelineMicrosSource(TimelineMicrosSource source);

int64_t TraceGetTimelineMicros();

void TraceTimelineEvent(TraceArg category_group,
                        TraceArg name,
                        int64_t timestamp_micros,
                        TraceIDArg id,
                        size_t flow_id_count,
                        const uint64_t* flow_ids,
                        Dart_Timeline_Event_Type type,
                        const std::vector<const char*>& names,
                        const std::vector<std::string>& values);

void TraceTimelineEvent(TraceArg category_group,
                        TraceArg name,
                        TraceIDArg id,
                        size_t flow_id_count,
                        const uint64_t* flow_ids,
                        Dart_Timeline_Event_Type type,
                        const std::vector<const char*>& names,
                        const std::vector<std::string>& values);

inline std::string TraceToString(const char* string) {
  return std::string{string};
}

inline std::string TraceToString(std::string string) {
  return string;
}

inline std::string TraceToString(TimePoint point) {
  return std::to_string(point.ToEpochDelta().ToNanoseconds());
}

template <typename T, typename = std::enable_if_t<std::is_arithmetic<T>::value>>
std::string TraceToString(T string) {
  return std::to_string(string);
}

inline void SplitArgumentsCollect(std::vector<const char*>& keys,
                                  std::vector<std::string>& values) {}

template <typename Key, typename Value, typename... Args>
void SplitArgumentsCollect(std::vector<const char*>& keys,
                           std::vector<std::string>& values,
                           Key key,
                           Value value,
                           Args... args) {
  keys.emplace_back(key);
  values.emplace_back(TraceToString(value));
  SplitArgumentsCollect(keys, values, args...);
}

inline std::pair<std::vector<const char*>, std::vector<std::string>>
SplitArguments() {
  return {};
}

template <typename Key, typename Value, typename... Args>
std::pair<std::vector<const char*>, std::vector<std::string>>
SplitArguments(Key key, Value value, Args... args) {
  std::vector<const char*> keys;
  std::vector<std::string> values;
  SplitArgumentsCollect(keys, values, key, value, args...);
  return std::make_pair(std::move(keys), std::move(values));
}

size_t TraceNonce();

template <typename... Args>
void TraceCounter(TraceArg category,
                  TraceArg name,
                  TraceIDArg identifier,
                  Args... args) {
#if FLUTTER_TIMELINE_ENABLED
  auto split = SplitArguments(args...);
  TraceTimelineEvent(category, name, identifier, /*flow_id_count=*/0,
                     /*flow_ids=*/nullptr, Dart_Timeline_Event_Counter,
                     split.first, split.second);
#endif  // FLUTTER_TIMELINE_ENABLED
}

// HACK: Used to NOP FML_TRACE_COUNTER macro without triggering unused var
// warnings at usage sites.
template <typename... Args>
void TraceCounterNopHACK(TraceArg category,
                         TraceArg name,
                         TraceIDArg identifier,
                         Args... args) {}

template <typename... Args>
void TraceEvent(TraceArg category,
                TraceArg name,
                size_t flow_id_count,
                const uint64_t* flow_ids,
                Args... args) {
#if FLUTTER_TIMELINE_ENABLED
  auto split = SplitArguments(args...);
  TraceTimelineEvent(category, name, 0, flow_id_count, flow_ids,
                     Dart_Timeline_Event_Begin, split.first, split.second);
#endif  // FLUTTER_TIMELINE_ENABLED
}

void TraceEvent0(TraceArg category_group,
                 TraceArg name,
                 size_t flow_id_count,
                 const uint64_t* flow_ids);

void TraceEvent1(TraceArg category_group,
                 TraceArg name,
                 size_t flow_id_count,
                 const uint64_t* flow_ids,
                 TraceArg arg1_name,
                 TraceArg arg1_val);

void TraceEvent2(TraceArg category_group,
                 TraceArg name,
                 size_t flow_id_count,
                 const uint64_t* flow_ids,
                 TraceArg arg1_name,
                 TraceArg arg1_val,
                 TraceArg arg2_name,
                 TraceArg arg2_val);

void TraceEventEnd(TraceArg name);

template <typename... Args>
void TraceEventAsyncComplete(TraceArg category_group,
                             TraceArg name,
                             TimePoint begin,
                             TimePoint end,
                             Args... args) {
#if FLUTTER_TIMELINE_ENABLED
  auto identifier = TraceNonce();
  const auto split = SplitArguments(args...);

  if (begin > end) {
    std::swap(begin, end);
  }

  const int64_t begin_micros = begin.ToEpochDelta().ToMicroseconds();
  const int64_t end_micros = end.ToEpochDelta().ToMicroseconds();

  TraceTimelineEvent(category_group,                   // group
                     name,                             // name
                     begin_micros,                     // timestamp_micros
                     identifier,                       // identifier
                     0,                                // flow_id_count
                     nullptr,                          // flow_ids
                     Dart_Timeline_Event_Async_Begin,  // type
                     split.first,                      // names
                     split.second                      // values
  );

  TraceTimelineEvent(category_group,                 // group
                     name,                           // name
                     end_micros,                     // timestamp_micros
                     identifier,                     // identifier
                     0,                              // flow_id_count
                     nullptr,                        // flow_ids
                     Dart_Timeline_Event_Async_End,  // type
                     split.first,                    // names
                     split.second                    // values
  );
#endif  // FLUTTER_TIMELINE_ENABLED
}

void TraceEventAsyncBegin0(TraceArg category_group,
                           TraceArg name,
                           TraceIDArg id,
                           size_t flow_id_count,
                           const uint64_t* flow_ids);

void TraceEventAsyncEnd0(TraceArg category_group, TraceArg name, TraceIDArg id);

void TraceEventAsyncBegin1(TraceArg category_group,
                           TraceArg name,
                           TraceIDArg id,
                           size_t flow_id_count,
                           const uint64_t* flow_ids,
                           TraceArg arg1_name,
                           TraceArg arg1_val);

void TraceEventAsyncEnd1(TraceArg category_group,
                         TraceArg name,
                         TraceIDArg id,
                         TraceArg arg1_name,
                         TraceArg arg1_val);

void TraceEventInstant0(TraceArg category_group,
                        TraceArg name,
                        size_t flow_id_count,
                        const uint64_t* flow_ids);

void TraceEventInstant1(TraceArg category_group,
                        TraceArg name,
                        size_t flow_id_count,
                        const uint64_t* flow_ids,
                        TraceArg arg1_name,
                        TraceArg arg1_val);

void TraceEventInstant2(TraceArg category_group,
                        TraceArg name,
                        size_t flow_id_count,
                        const uint64_t* flow_ids,
                        TraceArg arg1_name,
                        TraceArg arg1_val,
                        TraceArg arg2_name,
                        TraceArg arg2_val);

void TraceEventFlowBegin0(TraceArg category_group,
                          TraceArg name,
                          TraceIDArg id);

void TraceEventFlowStep0(TraceArg category_group, TraceArg name, TraceIDArg id);

void TraceEventFlowEnd0(TraceArg category_group, TraceArg name, TraceIDArg id);

class ScopedInstantEnd {
 public:
  explicit ScopedInstantEnd(const char* str) : label_(str) {}

  ~ScopedInstantEnd() { TraceEventEnd(label_); }

 private:
  const char* label_;

  FML_DISALLOW_COPY_AND_ASSIGN(ScopedInstantEnd);
};

// A move-only utility object that creates a new flow with a unique ID and
// automatically ends it when it goes out of scope. When tracing using multiple
// overlapping flows, it often gets hard to make sure to end the flow
// (especially with early returns), or, end/step on the wrong flow. This
// leads to corrupted or missing traces in the UI.
class TraceFlow {
 public:
  explicit TraceFlow(const char* label) : label_(label), nonce_(TraceNonce()) {
    TraceEvent0("flutter", label_, /*flow_id_count=*/1,
                /*flow_ids=*/&nonce_);
    TraceEventFlowBegin0("flutter", label_, nonce_);
    TraceEventEnd(label_);
  }

  ~TraceFlow() { End(label_); }

  TraceFlow(TraceFlow&& other) : label_(other.label_), nonce_(other.nonce_) {
    other.nonce_ = 0;
  }

  void Step(const char* label = nullptr) const {
    TraceEvent0("flutter", label ? label : label_, /*flow_id_count=*/1,
                /*flow_ids=*/&nonce_);
    TraceEventFlowStep0("flutter", label ? label : label_, nonce_);
    TraceEventEnd(label ? label : label_);
  }

  void End(const char* label = nullptr) {
    if (nonce_ != 0) {
      TraceEvent0("flutter", label ? label : label_, /*flow_id_count=*/1,
                  /*flow_ids=*/&nonce_);
      TraceEventFlowEnd0("flutter", label ? label : label_, nonce_);
      TraceEventEnd(label ? label : label_);
      nonce_ = 0;
    }
  }

 private:
  const char* label_;
  uint64_t nonce_;

  FML_DISALLOW_COPY_AND_ASSIGN(TraceFlow);
};

}  // namespace tracing
}  // namespace fml

#endif  // FLUTTER_FML_TRACE_EVENT_H_
