// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_RUNTIME_STAGE_RUNTIME_STAGE_PLAYGROUND_H_
#define FLUTTER_IMPELLER_RUNTIME_STAGE_RUNTIME_STAGE_PLAYGROUND_H_

#include "impeller/playground/playground_test.h"
#include "impeller/runtime_stage/runtime_stage.h"

namespace impeller {

class RuntimeStagePlayground : public PlaygroundTest {
 public:
  RuntimeStagePlayground();

  ~RuntimeStagePlayground();

  bool RegisterStage(const RuntimeStage& stage);

 private:
  RuntimeStagePlayground(const RuntimeStagePlayground&) = delete;

  RuntimeStagePlayground& operator=(const RuntimeStagePlayground&) = delete;
};

}  // namespace impeller

#endif  // FLUTTER_IMPELLER_RUNTIME_STAGE_RUNTIME_STAGE_PLAYGROUND_H_
