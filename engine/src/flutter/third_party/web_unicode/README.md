## Usage ##

To generate code for line/word break properties, follow these steps:

### 1. **<u>Download the unicode files:</u>**

The properties files can be found on the unicode.org website, for example [LineBreak.txt](https://www.unicode.org/Public/13.0.0/ucd/LineBreak.txt) and [WordBreakProperty.txt](https://www.unicode.org/Public/13.0.0/ucd/auxiliary/WordBreakProperty.txt). The codegen script expects the files to be located at `third_party/web_unicode/properties/`.

### 2. **<u>Run the codegen script:</u>**

Inside the `third_party/web_unicode` directory:
```
dart tool/unicode_sync_script.dart
```

## Check Mode ##

If you don't want to generate code, but you want to make sure that the properties files and the codegen files are still in sync, you can run the codegen script in "check mode".

Inside the `third_party/web_unicode` directory:
```
dart tool/unicode_sync_script.dart --check
```

This command won't overwite the existing codegen files. It only checks whether they are still in sync with the properties files or not. If not, it exits with a non-zero exit code.
