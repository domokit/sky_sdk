// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_FML_PLATFORM_FUCHSIA_TASK_OBSERVERS_H_
#define FLUTTER_FML_PLATFORM_FUCHSIA_TASK_OBSERVERS_H_

#include <lib/fit/function.h>

namespace fml {

// Executes all closures that were registered via
// `CurrentMessageLoopAddAfterTaskObserver` on this thread.
//
// WARNING(fxbug.dev/77957): These task observers are distinct from the task
// observers that can be specified via `fml::MessageLoop::AddTaskObserver` and
// they behave differently!
//
// Task observers registered via `fml::MessageLoop::AddTaskObserver` only fire
// after work that was posted via the `fml::MessageLoop`'s `TaskRunner`
// completes. Work that is posted directly to the Fuchsia message loop (e.g.
// using `async::PostTask(async_get_default_dispatcher(), ...)`) is invisible to
// `fml::MessageLoop`, so the `fml::MessageLoop`'s task observers don't fire.
//
// The task observers registered with `CurrentMessageLoopAddAfterTaskObserver`,
// however, fire after _every_ work item is completed, regardless of whether it
// was posted to the Fuchsia message loop directly or via `fml::MessageLoop`.
//
// These two mechanisms are redundant and confusing, so we should fix it
// somehow.
void ExecuteAfterTaskObservers();

void CurrentMessageLoopAddAfterTaskObserver(intptr_t key,
                                            fit::closure observer);

void CurrentMessageLoopRemoveAfterTaskObserver(intptr_t key);

}  // namespace fml

#endif  // FLUTTER_FML_PLATFORM_FUCHSIA_TASK_OBSERVERS_H_
