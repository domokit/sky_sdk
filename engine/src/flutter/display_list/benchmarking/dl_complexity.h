// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_DISPLAY_LIST_BENCHMARKING_DL_COMPLEXITY_H_
#define FLUTTER_DISPLAY_LIST_BENCHMARKING_DL_COMPLEXITY_H_

#include "flutter/display_list/display_list.h"

#include "third_party/skia/include/gpu/ganesh/GrTypes.h"

namespace flutter {

class DisplayListComplexityCalculator {
 public:
  static DisplayListComplexityCalculator* GetForSoftware();
  static DisplayListComplexityCalculator* GetForBackend(GrBackendApi backend);

  virtual ~DisplayListComplexityCalculator() = default;

  // Returns a calculated complexity score for a given DisplayList object
  virtual unsigned int Compute(const DisplayList* display_list) = 0;

  // Returns whether a given complexity score meets the threshold for
  // cacheability for this particular ComplexityCalculator
  virtual bool ShouldBeCached(unsigned int complexity_score) = 0;

  // Sets a ceiling for the complexity score being calculated. By default
  // this is the largest number representable by an unsigned int.
  //
  // This setting has no effect on non-accumulator based scorers such as
  // the Naive calculator.
  virtual void SetComplexityCeiling(unsigned int ceiling) = 0;
};

class DisplayListNaiveComplexityCalculator
    : public DisplayListComplexityCalculator {
 public:
  static DisplayListComplexityCalculator* GetInstance();

  unsigned int Compute(const DisplayList* display_list) override {
    return display_list->op_count(true);
  }

  bool ShouldBeCached(unsigned int complexity_score) override {
    return complexity_score > 5u;
  }

  void SetComplexityCeiling(unsigned int ceiling) override {}

 private:
  DisplayListNaiveComplexityCalculator() {}
  static DisplayListNaiveComplexityCalculator* instance_;
};

}  // namespace flutter

#endif  // FLUTTER_DISPLAY_LIST_BENCHMARKING_DL_COMPLEXITY_H_
