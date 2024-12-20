# Flutter Embedder Engine GLFW Example
## Description
This is an example of how to use Flutter Engine Embedder in order to get a
Flutter project rendering in a new host environment. The intended audience is
people who need to support host environment other than the ones already provided
by Flutter.  This is an advanced topic and not intended for beginners.

In this example we are demonstrating rendering a Flutter project inside of the GUI
library [GLFW](https://www.glfw.org/).  For more information about using the
embedder you can read the wiki article [Custom Flutter Engine Embedders](https://github.com/flutter/flutter/wiki/Custom-Flutter-Engine-Embedders).

The key difference between this example and the other GLFW example in this
folder is that the present example implements dirty region management (i.e.
rendering only the regions that have changed between frames as opposed to always
rendering the entire frame). For more information on the implementation of
dirty region management within the Embedder API, see [DRM Embedder](https://flutter.dev/go/drm-embedder).

## Running Instructions
The following example was tested on Linux but with a bit of tweaking should be
able to run on other *nix platforms and Windows. However, because this example
uses the EGL library, it is not compatible with MacOS platforms.

The example has the following dependencies:
 * [GLFW](https://www.glfw.org/) - This can be installed by running `sudo apt-get install libglfw3`
 * [CMake](https://cmake.org/) - This can be installed by running `sudo apt-get install cmake`
 * [EGL](https://docs.mesa3d.org/egl.html) - This can be installed by running  `sudo apt-get install libglfw3-dev`
 * [Flutter](https://flutter.dev/) - This can be installed from the [Flutter webpage](https://flutter.dev/docs/get-started/install)
 * [Flutter Engine](https://flutter.dev) - This can be built or downloaded, see [Custom Flutter Engine Embedders](https://github.com/flutter/flutter/wiki/Custom-Flutter-Engine-Embedders) for more information.

In order to **build** and **run** the example you should be able to go into this directory and run
`./run.sh`.

## Troubleshooting
There are a few things you might have to tweak in order to get your build working:
 * Flutter Engine Location - Inside the `CMakeList.txt` file you will see that it is set up to search for the header and library for the Flutter Engine in specific locations, those might not be the location of your Flutter Engine.
 * Pixel Ratio - If the project runs but is drawing at the wrong scale you may have to tweak the `kPixelRatio` variable in `FlutterEmbedderGLFW.cc` file.
 * GLFW Location - Inside the `CMakeLists.txt` we are searching for the GLFW library, if CMake can't find it you may have to edit that.
