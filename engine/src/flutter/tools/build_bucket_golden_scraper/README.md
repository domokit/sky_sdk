# `build_bucket_golden_scraper`

Given logging on Flutter's CI, scrapes the log for golden file changes.

```shell
$ dart bin/main.dart <path to log file, which can be http or a file>

Wrote 3 golden file changes:
  testing/resources/performance_overlay_gold_60fps.png
  testing/resources/performance_overlay_gold_90fps.png
  testing/resources/performance_overlay_gold_120fps.png
```

It can also be run with `--dry-run` to just print what it _would_ do:

```shell
$ dart bin/main.dart --dry-run <path to log file, which can be http or a file>

Found 3 golden file changes:
  testing/resources/performance_overlay_gold_60fps.png
  testing/resources/performance_overlay_gold_90fps.png
  testing/resources/performance_overlay_gold_120fps.png

Run again without --dry-run to apply these changes.
```

You're recommended to still use `git diff` to verify the changes look good.

## Upgrading `git diff`

By default, `git diff` is not very helpful for binary files. You can install
[`imagemagick`](https://imagemagick.org/) and configure your local git client
to make `git diff` show a PNG diff:

```shell
# On MacOS.
$ brew install imagemagick

# Create a comparison script.
$ cat > ~/bin/git-imgdiff <<EOF
#!/bin/sh
echo "Comparing $2 and $5"

# Find a temporary directory to store the diff.
if [ -z "$TMPDIR" ]; then
  TMPDIR=/tmp
fi

compare \
  "$2" "$5" \
  /tmp/git-imgdiff-diff.png

# Display the diff.
open /tmp/git-imgdiff-diff.png
EOF

# Setup git.
git config --global core.attributesfile '~/.gitattributes'

# Add the following to ~/.gitattributes.
cat >> ~/.gitattributes <<EOF
*.png diff=imgdiff
*.jpg diff=imgdiff
*.gif diff=imgdiff
EOF

git config --global diff.imgdiff.command '~/bin/git-imgdiff'
```

## Motivation

Due to <https://github.com/flutter/flutter/issues/53784>, on non-Linux OSes
there is no way to get golden-file changes locally for a variety of engine
tests.

This tool, given log output from a Flutter CI run, will scrape the log for:

```txt
Golden file mismatch. Please check the difference between /b/s/w/ir/cache/builder/src/flutter/testing/resources/performance_overlay_gold_90fps.png and /b/s/w/ir/cache/builder/src/flutter/testing/resources/performance_overlay_gold_90fps_new.png, and  replace the former with the latter if the difference looks good.
S
See also the base64 encoded /b/s/w/ir/cache/builder/src/flutter/testing/resources/performance_overlay_gold_90fps_new.png:
iVBORw0KGgoAAAANSUhEUgAAA+gAAAPoCAYAAABNo9TkAAAABHNCSVQICAgIfAhkiAAAIABJREFUeJzs3elzFWeeJ/rnHB3tSEILktgEBrPvYBbbUF4K24X3t (...omitted)
```

And convert the base64 encoded image into a PNG file, and overwrite the old
golden file with the new one.
