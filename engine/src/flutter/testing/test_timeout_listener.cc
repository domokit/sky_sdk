// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/test_timeout_listener.h"

#include <map>
#include <sstream>

namespace flutter::testing {

class PendingTests : public std::enable_shared_from_this<PendingTests> {
 public:
  static std::shared_ptr<PendingTests> Create(
      fml::RefPtr<fml::TaskRunner> host_task_runner,
      fml::TimeDelta timeout) {
    return std::shared_ptr<PendingTests>(
        new PendingTests(std::move(host_task_runner), timeout));
  }

  ~PendingTests() = default;

  void OnTestBegin(const std::string& test_name, fml::TimePoint test_time) {
    FML_CHECK(tests_.find(test_name) == tests_.end())
        << "Attempting to start a test that is already pending.";
    tests_[test_name] = test_time;

    host_task_runner_->PostDelayedTask(
        [weak = weak_from_this()] {
          if (auto strong = weak.lock()) {
            strong->CheckTimedOutTests();
          }
        },
        timeout_);
  }

  void OnTestEnd(const std::string& test_name) { tests_.erase(test_name); }

  void CheckTimedOutTests() const {
    const auto now = fml::TimePoint::Now();

    for (const auto& test : tests_) {
      auto delay = now - test.second;
      FML_CHECK(delay < timeout_)
          << "Test " << test.first << " did not complete in "
          << timeout_.ToSeconds()
          << " seconds and is assumed to be hung. Killing the test harness.";
    }
  }

 private:
  using TestData = std::map<std::string, fml::TimePoint>;

  fml::RefPtr<fml::TaskRunner> host_task_runner_;
  TestData tests_;
  const fml::TimeDelta timeout_;

  PendingTests(fml::RefPtr<fml::TaskRunner> host_task_runner,
               fml::TimeDelta timeout)
      : host_task_runner_(std::move(host_task_runner)), timeout_(timeout) {}

  FML_DISALLOW_COPY_AND_ASSIGN(PendingTests);
};

template <class T>
auto WeakPtr(std::shared_ptr<T> pointer) {
  return std::weak_ptr<T>{pointer};
}

TestTimeoutListener::TestTimeoutListener(fml::TimeDelta timeout)
    : timeout_(timeout),
      listener_thread_("test_timeout_listener"),
      listener_thread_runner_(listener_thread_.GetTaskRunner()),
      pending_tests_(PendingTests::Create(listener_thread_runner_, timeout_)) {
  FML_LOG(INFO) << "Test timeout of " << timeout_.ToSeconds()
                << " seconds per test case will be enforced.";
}

TestTimeoutListener::~TestTimeoutListener() {
  listener_thread_runner_->PostTask(
      [tests = std::move(pending_tests_)]() mutable { tests.reset(); });
  FML_CHECK(pending_tests_ == nullptr);
}

static std::string GetTestNameFromTestInfo(
    const ::testing::TestInfo& test_info) {
  std::stringstream stream;
  stream << test_info.test_suite_name();
  stream << ".";
  stream << test_info.name();
  if (auto type_param = test_info.type_param()) {
    stream << "/" << type_param;
  }
  if (auto value_param = test_info.value_param()) {
    stream << "/" << value_param;
  }
  return stream.str();
}

// |testing::EmptyTestEventListener|
void TestTimeoutListener::OnTestStart(const ::testing::TestInfo& test_info) {
  listener_thread_runner_->PostTask([weak_tests = WeakPtr(pending_tests_),
                                     name = GetTestNameFromTestInfo(test_info),
                                     now = fml::TimePoint::Now()]() {
    if (auto tests = weak_tests.lock()) {
      tests->OnTestBegin(name, now);
    }
  });
}

// |testing::EmptyTestEventListener|
void TestTimeoutListener::OnTestEnd(const ::testing::TestInfo& test_info) {
  listener_thread_runner_->PostTask(
      [weak_tests = WeakPtr(pending_tests_),
       name = GetTestNameFromTestInfo(test_info)]() {
        if (auto tests = weak_tests.lock()) {
          tests->OnTestEnd(name);
        }
      });
}

}  // namespace flutter::testing
