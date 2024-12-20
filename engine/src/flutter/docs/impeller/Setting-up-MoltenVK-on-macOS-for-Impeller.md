- Get the MoltenVK SDK from https://vulkan.lunarg.com/sdk/home#mac.
- Make sure to check off `System Global Installation`:
<img width="855" alt="image" src="https://user-images.githubusercontent.com/8620741/236010259-ae68283a-0a9e-4f85-9513-bc6ba199c351.png">

- When running `flutter/tools/gn`, add the `--impeller-enable-vulkan` flag, e.g. `./flutter/tools/gn --impeller-enable-vulkan --unopt --mac-cpu arm64`

You should now be able to build and run the Vulkan host tests, e.g. `out/host_debug_unopt_arm64/impeller_unittests --gtest_filter="*Vulkan*"`.
