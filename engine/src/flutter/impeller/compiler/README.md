# The Impeller Shader Compiler & Reflector

Host side tooling that consumes [GLSL 4.60 (Core
Profile)](https://www.khronos.org/registry/OpenGL/specs/gl/GLSLangSpec.4.60.pdf)
shaders and generates libraries suitable for consumption by an Impeller backend.
Along with said libraries, the reflector generates code and meta-data to
construct rendering and compute pipelines at runtime.
