# The Impeller Playground

An extension of the testing fixtures set, provides utilities for interactive
experimentation with the Impeller rendering subsystem. One the test author is
satisfied with the behavior of component as verified in the playground, pixel
test assertions can be added to before committing the new test case. Meant to
provide a gentle-er on-ramp to testing Impeller components. The WSI in the
playground allows for points at which third-party profiling and instrumentation
tools can be used to examine isolated test cases.
