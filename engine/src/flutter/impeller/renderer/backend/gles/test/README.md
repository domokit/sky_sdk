# `MockGLES`

This directory contains a mock implementation of the GLES backend.

Most functions are implemented as no-ops, have a default implementation that is not configurable, or just record the call. The latter is useful for testing:

```cc
TEST(MockGLES, Example) {
  // Creates a mock GLES implementation and sets it as the current one.
  auto mock_gles = MockGLES::Init();
  auto& gl = mock_gles->GetProcTable();

  // Call the proc table methods as usual, or pass the proc table to a class
  // that needs it.
  gl.PushDebugGroupKHR(GL_DEBUG_SOURCE_APPLICATION_KHR, 0, -1, "test");
  gl.PopDebugGroupKHR();

  // Method names are recorded and can be inspected.
  //
  // Note that many built-ins, like glGetString, are not recorded (otherwise the // logs would be much bigger and less useful).
  auto calls = mock_gles->GetCapturedCalls();
  EXPECT_EQ(calls, std::vector<std::string>(
                       {"PushDebugGroupKHR", "PopDebugGroupKHR"}));
}
```

To add a new function, do the following:

1. Add a new top-level method to [`mock_gles.cc`](mock_gles.cc):

   ```cc
   void glFooBar() {
     recordCall("glFooBar");
   }
   ```

2. Edit the `kMockResolver`, and add a new `else if` clause:

   ```diff
   + else if (strcmp(name, "glFooBar") == 0) {
   +  return reinterpret_cast<void*>(&glFooBar);
     } else {
      return reinterpret_cast<void*>(&glDoNothing);
     }
   ```

It's possible we'll want to add a more sophisticated mechanism for mocking
besides capturing calls, but this is a good start. PRs welcome!
