# Impeller's Coordinate System

**TL;DR, Impeller uses the Metal coordinate system.**

This document describes the Impeller coordinate system. This is the coordinate
system assumed by all Impeller sub-subsystems and users of Impeller itself.

All sub-systems that deal with interacting with backend client rendering APIs
(like OpenGL, Metal, Direct3D, Dawn, etc..) must reconcile with Impellers
coordinate system.

While the readers familiarity with a particular coordinate system might make
them think otherwise, there is no right or wrong coordinate system. However,
having a consist notion of a coordinate system is essential. Since the Metal
backend was the first Impeller backend, the Metal coordinate system was picked
as the Impeller coordinate system with very little consideration of
alternatives.

The following table describes the Impeller coordinate system along with how it
differs with that of popular client rendering APIs and backends.

| API           | Normalized Device Coordinate                          | Viewport / Framebuffer Coordinate     | Texture Coordinate                   |
|---------------|-------------------------------------------------------|---------------------------------------|--------------------------------------|
| **Impeller**  | `(-1,-1)` Bottom-Left, `(+1,+1)` Top-Right, `+Y` up.  | `(0,0)` Top-Left Origin, `+Y` down.   | `(0,0)` Top-Left Origin, `+Y` down.  |
| **Metal**     | `(-1,-1)` Bottom-Left, `(+1,+1)` Top-Right, `+Y` up.  | `(0,0)` Top-Left Origin, `+Y` down.   | `(0,0)` Top-Left Origin, `+Y` down.  |
| **OpenGL**    | `(-1,-1)` Bottom-Left, `(+1,+1)` Top-Right, `+Y` up.  | `(0,0)` Bottom-Left Origin, `+Y` up.  | `(0,0)` Bottom-Left Origin, `+Y` up. |
| **OpenGL ES** | `(-1,-1)` Bottom-Left, `(+1,+1)` Top-Right, `+Y` up.  | `(0,0)` Bottom-Left Origin, `+Y` up.  | `(0,0)` Bottom-Left Origin, `+Y` up. |
| **WebGL**     | `(-1,-1)` Bottom-Left, `(+1,+1)` Top-Right, `+Y` up.  | `(0,0)` Bottom-Left Origin, `+Y` up.  | `(0,0)` Bottom-Left Origin, `+Y` up. |
| **Direct 3D** | `(-1,-1)` Bottom-Left, `(+1,+1)` Top-Right, `+Y` up.  | `(0,0)` Top-Left Origin, `+Y` down.   | `(0,0)` Top-Left Origin, `+Y` down.  |
| **Vulkan**    | `(-1,-1)` Top-Left, `(+1,+1)` Bottom-Right, `+Y` down.| `(0,0)` Top-Left Origin, `+Y` down.   | `(0,0)` Top-Left Origin, `+Y` down.  |
| **WebGPU**    | `(-1,-1)` Bottom-Left, `(+1,+1)` Top-Right, `+Y` up.  | `(0,0)` Top-Left Origin, `+Y` down.   | `(0,0)` Top-Left Origin, `+Y` down.  |
