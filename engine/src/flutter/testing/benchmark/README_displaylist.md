# Running and Processing DisplayList Benchmarks

The DisplayList benchmarks for Flutter are used to determine the relative
cost of operations in order to assign scores to each op for the raster 
cache’s cache admission algorithm.

Due to the nature of benchmarking, these need to be run on actual devices in
order to get representative results, and because of the more locked-down
nature of the iOS and Android platforms it’s a little involved getting the
benchmark suite to run.

This document will detail the steps involved in getting the benchmarks to run
on both iOS and Android, and how to process the resulting data.

## iOS

iOS does not allow you to run unsigned code or arbitrary executables, so the
approach here is to build a dylib that contains the benchmarking code which
will then be linked to a skeleton test app in Xcode. The dylib contains an
exported C function, void RunBenchmarks(int argc, char **argv) that should
be called from the skeleton test app to run the benchmarks.

The dylib is not built by default and it will need to be specified as a
target manually when calling ninja.

The target name is ios_display_list_benchmarks, e.g.:

    $ ninja -C out/ios_profile ios_display_list_benchmarks

Once that dylib exists, the IosBenchmarks test app in flutter/testing/ios can
be loaded in Xcode. Ensure that the team is set appropriately so the code can
be signed and that FLUTTER_ENGINE matches the Flutter Engine build you wish to
use (e.g. ios_profile).

Once that is done, you can just hit the Run button and the JSON output will be
sent to the Xcode console. Copy that elsewhere and save it as a .json file.

Note: you may need to delete some errors from the console output that are
unrelated to the JSON output.

## Android

On Android, even on non-rooted devices, it is possible to execute unsigned
binaries using adb. As a result, there is a build target that will build a
binary that can be pushed to device using adb and executed using adb shell.
The only caveat is that the binary needs to be on a volume that isn’t mounted
as noexec, which typically rules out the sd card. /data/local/tmp seems like
an option that is typically available.

The build target is called display_list_benchmarks and will create a binary
called display_list_benchmarks in the root output directory
(e.g. android_profile_arm64).

    $ adb push out/android_profile_arm64/display_list_benchmarks /data/local/tmp/display_list_benchmarks
    $ adb shell /data/local/tmp/display_list_benchmarks --benchmark_format=json | tee android-results.json


The results in android-results.json can then be processed.

## Processing Results

There is a script in flutter/testing/benchmark called
displaylist_benchmark_parser.py which will take the JSON file and output a PDF
with graphs of all the benchmark series, as well as a CSV that can be imported
into a spreadsheet for further analysis.

This can then be manually analysed to determine the relative weightings for the
raster cache’s cache admission algorithm.