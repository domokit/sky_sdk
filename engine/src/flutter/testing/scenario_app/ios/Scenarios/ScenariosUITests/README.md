#  Golden UI Tests

This folder contains golden image tests. It renders UI (for instance, a platform
view) and does a screen shot comparison against a known good configuration.

The screen shots are named `golden_[scenario name]_[MODEL]`, with `_simulator`
appended for simulators. The model numbers for physical devices correspond
to the `hw.model` sys call, and will represent the model numbers. Simulator
names are taken from the environment.

New devices require running the test on the device, gathering the attachment
from the test result and verifying it manually. Then adding an appropriately
named file to this folder.

If the test is attempted on a new device, the log will contain a message
indicating the file name it expected to find. The test will continue and fail,
but will contain an attachment with the expected screen shot. If the screen
shot looks good, add it with the correct name to the project and run the test
again - it should pass this time.
