# The Impeller Geometry Library

Set of utilities used by most graphics operations. While the utilities
themselves are rendering backend agnostic, the layout and packing of the various
POD structs is arranged such that these can be copied into device memory
directly. The supported operations also mimic GLSL to some extent. For this
reason, the Impeller shader compiler and reflector uses these utilities in
generated code.
