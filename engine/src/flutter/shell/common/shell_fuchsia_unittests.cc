// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// A fuchsia-specific shell test.
//
// This test is only supposed to be ran on Fuchsia OS, as it exercises
// Fuchsia-specific functionality for which no equivalent exists elsewhere.

#define FML_USED_ON_EMBEDDER

#include <time.h>
#include <unistd.h>

#include <memory>

#include <fuchsia/intl/cpp/fidl.h>
#include <fuchsia/settings/cpp/fidl.h>
#include <lib/sys/cpp/component_context.h>

#include "flutter/fml/logging.h"
#include "flutter/fml/synchronization/count_down_latch.h"
#include "flutter/runtime/dart_vm.h"
#include "flutter/shell/common/shell_test.h"

namespace flutter {
namespace testing {

using fuchsia::intl::TimeZoneId;
using fuchsia::settings::Intl_Set_Result;
using fuchsia::settings::IntlSettings;

class FuchsiaShellTest : public ShellTest {
 protected:
  FuchsiaShellTest()
      : ctx_(sys::ComponentContext::CreateAndServeOutgoingDirectory()),
        intl_() {
    ctx_->svc()->Connect(intl_.NewRequest());
  }

  ~FuchsiaShellTest() {
    // Restore the time zone that matches that of the test harness.  This is
    // the default.
    // TODO(https://fxbug.dev/110019): This crashes right now.

    // const std::string local_timezone = GetLocalTimezone();
    // SetTimezone(local_timezone);
    // AssertTimezone(local_timezone, GetSettings());
  }

  // Gets the international settings from this Fuchsia realm.
  IntlSettings GetSettings() {
    IntlSettings settings;
    zx_status_t status = intl_->Watch(&settings);
    EXPECT_EQ(status, ZX_OK);
    return settings;
  }

  // Sets the timezone of this Fuchsia realm to `timezone_name`.
  void SetTimezone(const std::string& timezone_name) {
    fuchsia::settings::IntlSettings settings;
    settings.set_time_zone_id(TimeZoneId{.id = timezone_name});
    Intl_Set_Result result;
    zx_status_t status = intl_->Set(std::move(settings), &result);
    ASSERT_EQ(status, ZX_OK);
  }

  std::string GetLocalTimezone() {
    const time_t timestamp = time(nullptr);
    const struct tm* local_time = localtime(&timestamp);
    EXPECT_NE(local_time, nullptr)
        << "Could not get local time: errno=" << errno << ": "
        << strerror(errno);
    return std::string(local_time->tm_zone);
  }

  std::string GetLocalTime() {
    const time_t timestamp = time(nullptr);
    const struct tm* local_time = localtime(&timestamp);
    EXPECT_NE(local_time, nullptr)
        << "Could not get local time: errno=" << errno << ": "
        << strerror(errno);
    char buffer[sizeof("2020-08-26 14")];
    const size_t written =
        strftime(buffer, sizeof(buffer), "%Y-%m-%d %H", local_time);
    EXPECT_LT(0UL, written);
    return std::string(buffer);
  }

  // Checks that the timezone name in the `settings` matches what is `expected`.
  void AssertTimezone(const std::string& expected,
                      const IntlSettings& settings) {
    ASSERT_EQ(expected, settings.time_zone_id().id);
  }

  std::unique_ptr<sys::ComponentContext> ctx_;
  fuchsia::settings::IntlSyncPtr intl_;

  fuchsia::settings::IntlSettings save_settings_;
};

// These functions are used by tests that are currently disabled.
#if false
static bool ValidateShell(Shell* shell) {
  if (!shell) {
    return false;
  }

  if (!shell->IsSetup()) {
    return false;
  }

  ShellTest::PlatformViewNotifyCreated(shell);

  {
    fml::AutoResetWaitableEvent latch;
    fml::TaskRunner::RunNowOrPostTask(
        shell->GetTaskRunners().GetPlatformTaskRunner(), [shell, &latch]() {
          shell->GetPlatformView()->NotifyDestroyed();
          latch.Signal();
        });
    latch.Wait();
  }

  return true;
}

// Runs the function `f` in lock-step with a Dart isolate until the function
// returns `true`, or until a certain fixed number of retries is exhausted.
// Events `tick` and `tock` are used to synchronize the lock-stepping.  The
// event 'tick' is a signal to the dart isolate to advance a single iteration
// step.  'tock' is used by the dart isolate to signal that it has completed
// its step.
static void RunCoroutineWithRetry(int retries,
                                  fml::AutoResetWaitableEvent* tick,
                                  fml::AutoResetWaitableEvent* tock,
                                  std::function<bool()> f) {
  for (; retries > 0; retries--) {
    // Do a single coroutine step.
    tick->Signal();
    tock->Wait();
    if (f()) {
      break;
    }
    FML_LOG(INFO) << "Retries left: " << retries;
    sleep(1);
  }
}
#endif  // false

// Verifies that changing the Fuchsia settings timezone through the FIDL
// settings interface results in a change of the reported local time in the
// isolate.
//
// The test is as follows:
//
// - Set an initial timezone, then get a timestamp from the isolate rounded down
//   to the nearest hour.  The assumption is as long as this test doesn't run
//   very near the whole hour, which should be very unlikely, the nearest hour
//   will vary depending on the time zone.
// - Set a different timezone.  Get a timestamp from the isolate again and
//   confirm that this time around the timestamps are different.
// - Set the initial timezone again, and get the timestamp.  This time, the
//   timestamp rounded down to whole hour should match the timestamp we got
//   in the initial step.
TEST_F(FuchsiaShellTest, LocaltimesVaryOnTimezoneChanges) {
#if defined(OS_FUCHSIA)
  GTEST_SKIP()
      << "This test fails after the CF V2 migration. https://fxbug.dev/110019 ";
#else

  // See fixtures/shell_test.dart, the callback NotifyLocalTime is declared
  // there.
  fml::AutoResetWaitableEvent latch;
  std::string dart_isolate_time_str;
  AddNativeCallback("NotifyLocalTime", CREATE_NATIVE_ENTRY([&](auto args) {
                      dart_isolate_time_str =
                          tonic::DartConverter<std::string>::FromDart(
                              Dart_GetNativeArgument(args, 0));
                      latch.Signal();
                    }));

  // As long as this is set, the isolate will keep rerunning its only task.
  bool continue_fixture = true;
  fml::AutoResetWaitableEvent fixture_latch;
  AddNativeCallback("WaitFixture", CREATE_NATIVE_ENTRY([&](auto args) {
                      // Wait for the test fixture to advance.
                      fixture_latch.Wait();
                      tonic::DartConverter<bool>::SetReturnValue(
                          args, continue_fixture);
                    }));

  auto settings = CreateSettingsForFixture();
  auto configuration = RunConfiguration::InferFromSettings(settings);
  configuration.SetEntrypoint("timezonesChange");

  std::unique_ptr<Shell> shell = CreateShell(settings);
  ASSERT_NE(shell.get(), nullptr);
  ASSERT_TRUE(ValidateShell(shell.get()));
  RunEngine(shell.get(), std::move(configuration));
  latch.Wait();  // After this point, the fixture is at waitFixture().

  // Start with the local timezone, ensure that the isolate and the test
  // fixture are the same.
  SetTimezone(GetLocalTimezone());
  AssertTimezone(GetLocalTimezone(), GetSettings());
  std::string expected = GetLocalTime();
  std::string actual = "undefined";
  RunCoroutineWithRetry(10, &fixture_latch, &latch, [&]() {
    actual = dart_isolate_time_str;
    FML_LOG(INFO) << "reference: " << expected << ", actual: " << actual;
    return expected == actual;
  });
  ASSERT_EQ(expected, actual)
      << "The Dart isolate was expected to show the same time as the test "
      << "fixture eventually, but that didn't happen after multiple retries.";

  // Set a new timezone, which is hopefully different from the local one.
  SetTimezone("America/New_York");
  AssertTimezone("America/New_York", GetSettings());
  RunCoroutineWithRetry(10, &fixture_latch, &latch, [&]() {
    actual = dart_isolate_time_str;
    FML_LOG(INFO) << "reference: " << expected << ", actual: " << actual;
    return expected != actual;
  });
  ASSERT_NE(expected, actual)
      << "The Dart isolate was expected to show a time different from the test "
      << "fixture eventually, but that didn't happen after multiple retries.";

  // Set a new isolate timezone, and check that the reported time is eventually
  // different from what it used to be prior to the change.
  SetTimezone("Europe/Amsterdam");
  AssertTimezone("Europe/Amsterdam", GetSettings());
  RunCoroutineWithRetry(10, &fixture_latch, &latch, [&]() {
    actual = dart_isolate_time_str;
    FML_LOG(INFO) << "reference: " << expected << ", actual: " << actual;
    return expected != actual;
  });
  ASSERT_NE(expected, actual)
      << "The Dart isolate was expected to show a time different from the "
      << "prior timezone eventually, but that didn't happen after multiple "
      << "retries.";

  // Let's try to bring the timezone back to the old one.
  expected = actual;
  SetTimezone("America/New_York");
  AssertTimezone("America/New_York", GetSettings());
  RunCoroutineWithRetry(10, &fixture_latch, &latch, [&]() {
    actual = dart_isolate_time_str;
    FML_LOG(INFO) << "reference: " << expected << ", actual: " << actual;
    return expected != actual;
  });
  ASSERT_NE(expected, actual)
      << "The Dart isolate was expected to show a time different from the "
      << "prior timezone eventually, but that didn't happen after multiple "
      << "retries.";

  // Tell the isolate to exit its loop.
  ASSERT_FALSE(fixture_latch.IsSignaledForTest());
  continue_fixture = false;
  fixture_latch.Signal();
  DestroyShell(std::move(shell));
#endif  // OS_FUCHSIA
}

}  // namespace testing
}  // namespace flutter
