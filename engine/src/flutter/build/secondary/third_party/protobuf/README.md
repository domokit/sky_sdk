# Protocol Buffers GN Build Support

This repository contains
[GN build system](https://gn.googlesource.com/gn/+/HEAD/) support for
[Protocol Buffers](https://github.com/protocolbuffers/protobuf). It's in its own
repository because it needs to be shared by
[Fuchsia](https://fuchsia.googlesource.com/fuchsia/) and
[Cobalt](https://fuchsia.googlesource.com/cobalt/).

This repo should be checked out such that:

* It is in `//build/secondary/third_party/protobuf`.
* Protobuf is in `//third_party/protobuf`.
* `//.gn` contains `secondary_source = "//build/secondary/"`

See the
[GN documentation on secondary_source](https://gn.googlesource.com/gn/+/master/docs/reference.md#other-help-topics-gn-file-variables).
