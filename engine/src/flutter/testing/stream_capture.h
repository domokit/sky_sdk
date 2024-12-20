// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_STREAM_CAPTURE_H_
#define FLUTTER_TESTING_STREAM_CAPTURE_H_

#include <ostream>
#include <sstream>
#include <string>

namespace flutter::testing {

// Temporarily replaces the specified stream's output buffer to capture output.
//
// Example:
// StreamCapture captured_stdout(&std::cout);
// ... code that writest to std::cout ...
// std::string output = captured_stdout.GetCapturedOutput();
class StreamCapture {
 public:
  // Begins capturing output to the specified stream.
  explicit StreamCapture(std::ostream* ostream);

  // Stops capturing output to the specified stream, and restores the original
  // output buffer, if |Stop| has not already been called.
  ~StreamCapture();

  // Stops capturing output to the specified stream, and restores the original
  // output buffer.
  void Stop();

  // Returns any output written to the captured stream between construction and
  // the first call to |Stop|, if any, or now.
  std::string GetOutput() const;

 private:
  std::ostream* ostream_;
  std::stringstream buffer_;
  std::streambuf* old_buffer_;
};

}  // namespace flutter::testing

#endif  // FLUTTER_TESTING_STREAM_CAPTURE_H_
