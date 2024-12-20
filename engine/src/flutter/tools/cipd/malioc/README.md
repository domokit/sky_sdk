# Updating malioc

Flutter uses `malioc` from the Arm Mobile Studio to statically analyze
Impeller's shaders. See `impeller/tools/malioc_diff.py` for more information
about that.

The files in this directory are instructions for updating the `flutter_internal`
CIPD package that CI uses to obtain `malioc`.

## Steps

1. Download the Arm Mobile Studio from [Arm](https://developer.arm.com/Tools%20and%20Software/Arm%20Mobile%20Studio)
2. Extract the `.zip` archive into a directory called `arm-tools` sibling to
   this file.
3. Check that the `arm-tools` directory contains a child directory
   `mali_offline_compiler`. If it instead contains a child directory which
   is something like `Arm_Mobile_Studio_2022.4`, then copy the contents of that
   directory to `arm-tools` instead.
4. Run the `generate.sh` script in this directory.
5. Verify that the file appears [here](https://chrome-infra-packages.appspot.com/p/flutter_internal/tools).
6. Update `.ci.yaml` to refer to the new version tag echoed by `generate.sh`.
7. Delete the `arm-tools` directory
