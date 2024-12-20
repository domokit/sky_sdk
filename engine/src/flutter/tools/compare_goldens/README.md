# Compare Goldens

This is a script that will let you check golden image diffs locally.

The directories are scanned for png files that match in name, then the diff
is written to `diff_<name of file>` in the CWD. This allows you to get
results quicker than having to upload to skia gold.  By default it uses fuzzy
RMSE to compare.

## Usage

```sh
dart run compare_goldens <dir path> <dir path>
```

Here's the steps for using this with something like impeller golden tests:

1) Checkout a base revision
2) Build impeller_golden_tests
3) Execute `impeller_golden_tests --working_dir=\<path a\>
4) Checkout test revision
5) Build impeller_golden_tests
6) Execute `impeller_golden_tests --working_dir=\<path b\>
7) Execute `compare_goldens \<path a\> \<path b\>

## Requirements

- ImageMagick is installed on $PATH

## Testing

To run the tests:

```sh
dart pub get
find . -name "*_test.dart" | xargs -n 1 dart --enable-asserts
```
