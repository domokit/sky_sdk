# dir_contents_diff

This tool will compare the contents of a directory to a file that lists the
contents of the directory, printing out a patch to apply if they differ.

The exit code is 0 if there is no difference.

## Usage

```sh
dart run ./bin/dir_contents_diff.dart <golden file path> <dir path>
```
