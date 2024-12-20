// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/trace_event.h"

#include <algorithm>
#include <atomic>
#include <utility>

#include "flutter/fml/ascii_trie.h"
#include "flutter/fml/build_config.h"
#include "flutter/fml/logging.h"

namespace fml {
namespace tracing {

#if FLUTTER_TIMELINE_ENABLED

namespace {

int64_t DefaultMicrosSource() {
  return -1;
}

AsciiTrie gAllowlist;
std::atomic<TimelineEventHandler> gTimelineEventHandler;
std::atomic<TimelineMicrosSource> gTimelineMicrosSource = DefaultMicrosSource;

inline void FlutterTimelineEvent(const char* label,
                                 int64_t timestamp0,
                                 int64_t timestamp1_or_async_id,
                                 intptr_t flow_id_count,
                                 const int64_t* flow_ids,
                                 Dart_Timeline_Event_Type type,
                                 intptr_t argument_count,
                                 const char** argument_names,
                                 const char** argument_values) {
  TimelineEventHandler handler =
      gTimelineEventHandler.load(std::memory_order_relaxed);
  if (handler && gAllowlist.Query(label)) {
    handler(label, timestamp0, timestamp1_or_async_id, flow_id_count, flow_ids,
            type, argument_count, argument_names, argument_values);
  }
}
}  // namespace

void TraceSetAllowlist(const std::vector<std::string>& allowlist) {
  gAllowlist.Fill(allowlist);
}

void TraceSetTimelineEventHandler(TimelineEventHandler handler) {
  gTimelineEventHandler = handler;
}

bool TraceHasTimelineEventHandler() {
  return static_cast<bool>(
      gTimelineEventHandler.load(std::memory_order_relaxed));
}

int64_t TraceGetTimelineMicros() {
  return gTimelineMicrosSource.load()();
}

void TraceSetTimelineMicrosSource(TimelineMicrosSource source) {
  gTimelineMicrosSource = source;
}

size_t TraceNonce() {
  static std::atomic_size_t last_item;
  return ++last_item;
}

void TraceTimelineEvent(TraceArg category_group,
                        TraceArg name,
                        int64_t timestamp_micros,
                        TraceIDArg identifier,
                        size_t flow_id_count,
                        const uint64_t* flow_ids,
                        Dart_Timeline_Event_Type type,
                        const std::vector<const char*>& c_names,
                        const std::vector<std::string>& values) {
  const auto argument_count = std::min(c_names.size(), values.size());

  std::vector<const char*> c_values;
  c_values.resize(argument_count, nullptr);

  for (size_t i = 0; i < argument_count; i++) {
    c_values[i] = values[i].c_str();
  }

  FlutterTimelineEvent(
      name,                                        // label
      timestamp_micros,                            // timestamp0
      identifier,                                  // timestamp1_or_async_id
      flow_id_count,                               // flow_id_count
      reinterpret_cast<const int64_t*>(flow_ids),  // flow_ids
      type,                                        // event type
      argument_count,                              // argument_count
      const_cast<const char**>(c_names.data()),    // argument_names
      c_values.data()                              // argument_values
  );
}

void TraceTimelineEvent(TraceArg category_group,
                        TraceArg name,
                        TraceIDArg identifier,
                        size_t flow_id_count,
                        const uint64_t* flow_ids,
                        Dart_Timeline_Event_Type type,
                        const std::vector<const char*>& c_names,
                        const std::vector<std::string>& values) {
  TraceTimelineEvent(category_group,                  // group
                     name,                            // name
                     gTimelineMicrosSource.load()(),  // timestamp_micros
                     identifier,                      // identifier
                     flow_id_count,                   // flow_id_count
                     flow_ids,                        // flow_ids
                     type,                            // type
                     c_names,                         // names
                     values                           // values
  );
}

void TraceEvent0(TraceArg category_group,
                 TraceArg name,
                 size_t flow_id_count,
                 const uint64_t* flow_ids) {
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       0,              // timestamp1_or_async_id
                       flow_id_count,  // flow_id_count
                       reinterpret_cast<const int64_t*>(flow_ids),  // flow_ids
                       Dart_Timeline_Event_Begin,  // event type
                       0,                          // argument_count
                       nullptr,                    // argument_names
                       nullptr                     // argument_values
  );
}

void TraceEvent1(TraceArg category_group,
                 TraceArg name,
                 size_t flow_id_count,
                 const uint64_t* flow_ids,
                 TraceArg arg1_name,
                 TraceArg arg1_val) {
  const char* arg_names[] = {arg1_name};
  const char* arg_values[] = {arg1_val};
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       0,              // timestamp1_or_async_id
                       flow_id_count,  // flow_id_count
                       reinterpret_cast<const int64_t*>(flow_ids),  // flow_ids
                       Dart_Timeline_Event_Begin,  // event type
                       1,                          // argument_count
                       arg_names,                  // argument_names
                       arg_values                  // argument_values
  );
}

void TraceEvent2(TraceArg category_group,
                 TraceArg name,
                 size_t flow_id_count,
                 const uint64_t* flow_ids,
                 TraceArg arg1_name,
                 TraceArg arg1_val,
                 TraceArg arg2_name,
                 TraceArg arg2_val) {
  const char* arg_names[] = {arg1_name, arg2_name};
  const char* arg_values[] = {arg1_val, arg2_val};
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       0,              // timestamp1_or_async_id
                       flow_id_count,  // flow_id_count
                       reinterpret_cast<const int64_t*>(flow_ids),  // flow_ids
                       Dart_Timeline_Event_Begin,  // event type
                       2,                          // argument_count
                       arg_names,                  // argument_names
                       arg_values                  // argument_values
  );
}

void TraceEventEnd(TraceArg name) {
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       0,                        // timestamp1_or_async_id
                       0,                        // flow_id_count
                       nullptr,                  // flow_ids
                       Dart_Timeline_Event_End,  // event type
                       0,                        // argument_count
                       nullptr,                  // argument_names
                       nullptr                   // argument_values
  );
}

void TraceEventAsyncBegin0(TraceArg category_group,
                           TraceArg name,
                           TraceIDArg id,
                           size_t flow_id_count,
                           const uint64_t* flow_ids) {
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       id,             // timestamp1_or_async_id
                       flow_id_count,  // flow_id_count
                       reinterpret_cast<const int64_t*>(flow_ids),  // flow_ids
                       Dart_Timeline_Event_Async_Begin,  // event type
                       0,                                // argument_count
                       nullptr,                          // argument_names
                       nullptr                           // argument_values
  );
}

void TraceEventAsyncEnd0(TraceArg category_group,
                         TraceArg name,
                         TraceIDArg id) {
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       id,                             // timestamp1_or_async_id
                       0,                              // flow_id_count
                       nullptr,                        // flow_ids
                       Dart_Timeline_Event_Async_End,  // event type
                       0,                              // argument_count
                       nullptr,                        // argument_names
                       nullptr                         // argument_values
  );
}

void TraceEventAsyncBegin1(TraceArg category_group,
                           TraceArg name,
                           TraceIDArg id,
                           size_t flow_id_count,
                           const uint64_t* flow_ids,
                           TraceArg arg1_name,
                           TraceArg arg1_val) {
  const char* arg_names[] = {arg1_name};
  const char* arg_values[] = {arg1_val};
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       id,             // timestamp1_or_async_id
                       flow_id_count,  // flow_id_count
                       reinterpret_cast<const int64_t*>(flow_ids),  // flow_ids
                       Dart_Timeline_Event_Async_Begin,  // event type
                       1,                                // argument_count
                       arg_names,                        // argument_names
                       arg_values                        // argument_values
  );
}

void TraceEventAsyncEnd1(TraceArg category_group,
                         TraceArg name,
                         TraceIDArg id,
                         TraceArg arg1_name,
                         TraceArg arg1_val) {
  const char* arg_names[] = {arg1_name};
  const char* arg_values[] = {arg1_val};
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       id,                             // timestamp1_or_async_id
                       0,                              // flow_id_count
                       nullptr,                        // flow_ids
                       Dart_Timeline_Event_Async_End,  // event type
                       1,                              // argument_count
                       arg_names,                      // argument_names
                       arg_values                      // argument_values
  );
}

void TraceEventInstant0(TraceArg category_group,
                        TraceArg name,
                        size_t flow_id_count,
                        const uint64_t* flow_ids) {
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       0,              // timestamp1_or_async_id
                       flow_id_count,  // flow_id_count
                       reinterpret_cast<const int64_t*>(flow_ids),  // flow_ids
                       Dart_Timeline_Event_Instant,  // event type
                       0,                            // argument_count
                       nullptr,                      // argument_names
                       nullptr                       // argument_values
  );
}

void TraceEventInstant1(TraceArg category_group,
                        TraceArg name,
                        size_t flow_id_count,
                        const uint64_t* flow_ids,
                        TraceArg arg1_name,
                        TraceArg arg1_val) {
  const char* arg_names[] = {arg1_name};
  const char* arg_values[] = {arg1_val};
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       0,              // timestamp1_or_async_id
                       flow_id_count,  // flow_id_count
                       reinterpret_cast<const int64_t*>(flow_ids),  // flow_ids
                       Dart_Timeline_Event_Instant,  // event type
                       1,                            // argument_count
                       arg_names,                    // argument_names
                       arg_values                    // argument_values
  );
}

void TraceEventInstant2(TraceArg category_group,
                        TraceArg name,
                        size_t flow_id_count,
                        const uint64_t* flow_ids,
                        TraceArg arg1_name,
                        TraceArg arg1_val,
                        TraceArg arg2_name,
                        TraceArg arg2_val) {
  const char* arg_names[] = {arg1_name, arg2_name};
  const char* arg_values[] = {arg1_val, arg2_val};
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       0,              // timestamp1_or_async_id
                       flow_id_count,  // flow_id_count
                       reinterpret_cast<const int64_t*>(flow_ids),  // flow_ids
                       Dart_Timeline_Event_Instant,  // event type
                       2,                            // argument_count
                       arg_names,                    // argument_names
                       arg_values                    // argument_values
  );
}

void TraceEventFlowBegin0(TraceArg category_group,
                          TraceArg name,
                          TraceIDArg id) {
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       id,       // timestamp1_or_async_id
                       0,        // flow_id_count
                       nullptr,  // flow_ids
                       Dart_Timeline_Event_Flow_Begin,  // event type
                       0,                               // argument_count
                       nullptr,                         // argument_names
                       nullptr                          // argument_values
  );
}

void TraceEventFlowStep0(TraceArg category_group,
                         TraceArg name,
                         TraceIDArg id) {
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       id,                             // timestamp1_or_async_id
                       0,                              // flow_id_count
                       nullptr,                        // flow_ids
                       Dart_Timeline_Event_Flow_Step,  // event type
                       0,                              // argument_count
                       nullptr,                        // argument_names
                       nullptr                         // argument_values
  );
}

void TraceEventFlowEnd0(TraceArg category_group, TraceArg name, TraceIDArg id) {
  FlutterTimelineEvent(name,                            // label
                       gTimelineMicrosSource.load()(),  // timestamp0
                       id,                            // timestamp1_or_async_id
                       0,                             // flow_id_count
                       nullptr,                       // flow_ids
                       Dart_Timeline_Event_Flow_End,  // event type
                       0,                             // argument_count
                       nullptr,                       // argument_names
                       nullptr                        // argument_values
  );
}

#else  // FLUTTER_TIMELINE_ENABLED

void TraceSetAllowlist(const std::vector<std::string>& allowlist) {}

void TraceSetTimelineEventHandler(TimelineEventHandler handler) {}

bool TraceHasTimelineEventHandler() {
  return false;
}

int64_t TraceGetTimelineMicros() {
  return -1;
}

void TraceSetTimelineMicrosSource(TimelineMicrosSource source) {}

size_t TraceNonce() {
  return 0;
}

void TraceTimelineEvent(TraceArg category_group,
                        TraceArg name,
                        int64_t timestamp_micros,
                        TraceIDArg identifier,
                        size_t flow_id_count,
                        const uint64_t* flow_ids,
                        Dart_Timeline_Event_Type type,
                        const std::vector<const char*>& c_names,
                        const std::vector<std::string>& values) {}

void TraceTimelineEvent(TraceArg category_group,
                        TraceArg name,
                        TraceIDArg identifier,
                        size_t flow_id_count,
                        const uint64_t* flow_ids,
                        Dart_Timeline_Event_Type type,
                        const std::vector<const char*>& c_names,
                        const std::vector<std::string>& values) {}

void TraceEvent0(TraceArg category_group,
                 TraceArg name,
                 size_t flow_id_count,
                 const uint64_t* flow_ids) {}

void TraceEvent1(TraceArg category_group,
                 TraceArg name,
                 size_t flow_id_count,
                 const uint64_t* flow_ids,
                 TraceArg arg1_name,
                 TraceArg arg1_val) {}

void TraceEvent2(TraceArg category_group,
                 TraceArg name,
                 size_t flow_id_count,
                 const uint64_t* flow_ids,
                 TraceArg arg1_name,
                 TraceArg arg1_val,
                 TraceArg arg2_name,
                 TraceArg arg2_val) {}

void TraceEventEnd(TraceArg name) {}

void TraceEventAsyncComplete(TraceArg category_group,
                             TraceArg name,
                             TimePoint begin,
                             TimePoint end) {}

void TraceEventAsyncBegin0(TraceArg category_group,
                           TraceArg name,
                           TraceIDArg id,
                           size_t flow_id_count,
                           const uint64_t* flow_ids) {}

void TraceEventAsyncEnd0(TraceArg category_group,
                         TraceArg name,
                         TraceIDArg id) {}

void TraceEventAsyncBegin1(TraceArg category_group,
                           TraceArg name,
                           TraceIDArg id,
                           size_t flow_id_count,
                           const uint64_t* flow_ids,
                           TraceArg arg1_name,
                           TraceArg arg1_val) {}

void TraceEventAsyncEnd1(TraceArg category_group,
                         TraceArg name,
                         TraceIDArg id,
                         TraceArg arg1_name,
                         TraceArg arg1_val) {}

void TraceEventInstant0(TraceArg category_group,
                        TraceArg name,
                        size_t flow_id_count,
                        const uint64_t* flow_ids) {}

void TraceEventInstant1(TraceArg category_group,
                        TraceArg name,
                        size_t flow_id_count,
                        const uint64_t* flow_ids,
                        TraceArg arg1_name,
                        TraceArg arg1_val) {}

void TraceEventInstant2(TraceArg category_group,
                        TraceArg name,
                        size_t flow_id_count,
                        const uint64_t* flow_ids,
                        TraceArg arg1_name,
                        TraceArg arg1_val,
                        TraceArg arg2_name,
                        TraceArg arg2_val) {}

void TraceEventFlowBegin0(TraceArg category_group,
                          TraceArg name,
                          TraceIDArg id) {}

void TraceEventFlowStep0(TraceArg category_group,
                         TraceArg name,
                         TraceIDArg id) {}

void TraceEventFlowEnd0(TraceArg category_group, TraceArg name, TraceIDArg id) {
}

#endif  // FLUTTER_TIMELINE_ENABLED

}  // namespace tracing
}  // namespace fml
