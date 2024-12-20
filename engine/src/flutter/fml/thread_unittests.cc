// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/build_config.h"
#include "flutter/fml/thread.h"

#if defined(FML_OS_MACOSX) || defined(FML_OS_LINUX) || defined(FML_OS_ANDROID)
#define FLUTTER_PTHREAD_SUPPORTED 1
#else
#define FLUTTER_PTHREAD_SUPPORTED 0
#endif

#if FLUTTER_PTHREAD_SUPPORTED
#include <pthread.h>
#else
#endif

#if defined(FML_OS_WIN)
#include <windows.h>
#endif

#include <algorithm>
#include <memory>
#include "gtest/gtest.h"

TEST(Thread, CanStartAndEnd) {
  fml::Thread thread;
  ASSERT_TRUE(thread.GetTaskRunner());
}

TEST(Thread, CanStartAndEndWithExplicitJoin) {
  fml::Thread thread;
  ASSERT_TRUE(thread.GetTaskRunner());
  thread.Join();
}

TEST(Thread, HasARunningMessageLoop) {
  fml::Thread thread;
  bool done = false;
  thread.GetTaskRunner()->PostTask([&done]() { done = true; });
  thread.Join();
  ASSERT_TRUE(done);
}

TEST(Thread, HasExpectedStackSize) {
  size_t stack_size = 0;
  fml::Thread thread;

  thread.GetTaskRunner()->PostTask([&stack_size]() {
#if defined(FML_OS_WIN)
    ULONG_PTR low_limit;
    ULONG_PTR high_limit;
    GetCurrentThreadStackLimits(&low_limit, &high_limit);
    stack_size = high_limit - low_limit;
#elif defined(FML_OS_MACOSX)
    stack_size = pthread_get_stacksize_np(pthread_self());
#else
    pthread_attr_t attr;
    pthread_getattr_np(pthread_self(), &attr);
    pthread_attr_getstacksize(&attr, &stack_size);
    pthread_attr_destroy(&attr);
#endif
  });
  thread.Join();

  // Actual stack size will be aligned to page size, this assumes no supported
  // platform has a page size larger than 16k. On Linux reducing the default
  // stack size (8MB) does not seem to have any effect.
  const size_t kPageSize = 16384;
  ASSERT_TRUE(stack_size / kPageSize >=
              fml::Thread::GetDefaultStackSize() / kPageSize);
}

#if FLUTTER_PTHREAD_SUPPORTED
TEST(Thread, ThreadNameCreatedWithConfig) {
  const std::string name = "Thread1";
  fml::Thread thread(name);

  bool done = false;
  thread.GetTaskRunner()->PostTask([&done, &name]() {
    done = true;
    char thread_name[16];
    pthread_t current_thread = pthread_self();
    pthread_getname_np(current_thread, thread_name, 16);
    ASSERT_EQ(thread_name, name);
  });
  thread.Join();
  ASSERT_TRUE(done);
}

static int clamp_priority(int priority, int policy) {
  return std::clamp(priority, sched_get_priority_min(policy),
                    sched_get_priority_max(policy));
}

static void MockThreadConfigSetter(const fml::Thread::ThreadConfig& config) {
  // set thread name
  fml::Thread::SetCurrentThreadName(config);

  pthread_t tid = pthread_self();
  struct sched_param param;
  int policy = SCHED_OTHER;
  switch (config.priority) {
    case fml::Thread::ThreadPriority::kDisplay:
      param.sched_priority = clamp_priority(10, policy);
      break;
    default:
      param.sched_priority = clamp_priority(1, policy);
  }
  pthread_setschedparam(tid, policy, &param);
}

TEST(Thread, ThreadPriorityCreatedWithConfig) {
  const std::string thread1_name = "Thread1";
  const std::string thread2_name = "Thread2";

  fml::Thread thread(MockThreadConfigSetter,
                     fml::Thread::ThreadConfig(
                         thread1_name, fml::Thread::ThreadPriority::kNormal));
  bool done = false;

  struct sched_param param;
  int policy;
  thread.GetTaskRunner()->PostTask([&]() {
    done = true;
    char thread_name[16];
    pthread_t current_thread = pthread_self();
    pthread_getname_np(current_thread, thread_name, 16);
    pthread_getschedparam(current_thread, &policy, &param);
    ASSERT_EQ(thread_name, thread1_name);
    ASSERT_EQ(policy, SCHED_OTHER);
    ASSERT_EQ(param.sched_priority, clamp_priority(1, policy));
  });

  fml::Thread thread2(MockThreadConfigSetter,
                      fml::Thread::ThreadConfig(
                          thread2_name, fml::Thread::ThreadPriority::kDisplay));
  thread2.GetTaskRunner()->PostTask([&]() {
    done = true;
    char thread_name[16];
    pthread_t current_thread = pthread_self();
    pthread_getname_np(current_thread, thread_name, 16);
    pthread_getschedparam(current_thread, &policy, &param);
    ASSERT_EQ(thread_name, thread2_name);
    ASSERT_EQ(policy, SCHED_OTHER);
    ASSERT_EQ(param.sched_priority, clamp_priority(10, policy));
  });
  thread.Join();
  ASSERT_TRUE(done);
}
#endif  // FLUTTER_PTHREAD_SUPPORTED

#if defined(FML_OS_LINUX)
TEST(Thread, LinuxLongThreadNameTruncated) {
  const std::string name = "VeryLongThreadNameTest";
  fml::Thread thread(name);

  thread.GetTaskRunner()->PostTask([&name]() {
    constexpr size_t kThreadNameLen = 16;
    char thread_name[kThreadNameLen];
    pthread_getname_np(pthread_self(), thread_name, kThreadNameLen);
    ASSERT_EQ(thread_name, name.substr(0, kThreadNameLen - 1));
  });
  thread.Join();
}
#endif  // FML_OS_LINUX
