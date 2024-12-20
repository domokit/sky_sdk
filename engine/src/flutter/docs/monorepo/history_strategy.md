# Monorepo History Pruning

These are the steps we will/did follow to prune excessive history from the [flutter/engine](https://github.com/flutter/engine) repository when we merged with [flutter/flutter](https://github.com/flutter/flutter). The idea was to retain as much useful history as possible without blowing up the footprint of the framework's `.git` folder. The history that should get merged should be as relative and useful to currently engine development.

The engine `.git` folder is ~780MB of history.

* Binary files were checked in that are not used anymore.
* Third party librariers were checked in and removed nearly a decade ago.
* Examples were created and later moved elsewhere.

## Step 1: Fresh Clone + Safety

Do not start with your working tree.
Remove the origin so we don't mess with the `flutter/engine`.

```shell
##############################################
## Do some cleanup work on the engine and get
## the folder structure right.
##############################################

# clone the repo to a fresh working folder
git clone git@github.com:flutter/engine.git engine_prep
cd engine_prep

# for saftey - remove the remote - we're going to edit history
git remote remove origin
```

### Optional - Analyze the repo

If you want to analyze the repository, you should intall [git filter-repo](http://github.com/newren/git-filter-repo) on your path and then run:

```shell
# Analyze if you want, just remember to remove .git/filter-repo
git filter-repo --analyze --force
```

The output is stored in `.git/filter-repo`.

## Step 2: Prune the History

The following table is pulled from git-filter-repo's analsis.  The `Packed Size` due to cross referencing. In general; we looked at large files that are not referenced any more and folders older than 2016.

| Packed Size | Deleted Date | Path                                    | Notes                        |
|-------------|:-------------|:----------------------------------------|:-----------------------------|
| 112784745   | 2024-05-13   | ci/licenses_golden/licenses_third_party |                              |
| 27531902    | ~2021        | *.jar                                   | binary                       |
| 27379931    | 2016-08-09   | third_party/android_platform            | android_platform and webview |
| 27000000    | 2024-07-15   | impeller/docs/assets/*.(png|gif)        | moved to another repository  |
| 15121375    | 2023-02-13   | *.ttc                                   | font files                   |
| 10104182    | 2023-02-13   | */SourceHanSerifCN*                     |                              |
| 7985682     | 2018-08-08   | travis                                  | old ci                       |
| 6315637     | 2015-11-07   | examples/game                           |                              |
| 3939429     | 2015-07-28   | sky/sdk                                 |                              |
| 3939429     | 2015-07-28   | sky/packages/sky                        |                              |
| 3903787     | 2016-08-09   | mojo                                    |                              |
| 3686830     | 2022-06-14   | testing/scenario_app/android/reports    |                              |
| 3188930     | 2015-06-30   | tests/fast                              |                              |
| 3173966     | 2015-08-07   | */example/game*                         |                              |
| 2018961     | 2016-08-09   | third_party/libxml                      |                              |
| 1804199     | 2016-08-09   | third_party/tcmalloc                    |                              |
| 1393936     | ~2016        | *.dll                                   | binary                       |
| 1373740     | 2017-07-06   | tests/data                              |                              |
| 1100665     | 2015-06-27   | benchmarks/parser/resources/html5.html  |                              |
| 1059673     | 2015-07-20   | third_party/protobuf                    |                              |
| 978870      | 2022-04-27   | impeller/third_party                    |                              |
| 798852      | 2015-07-20   | third_party/cython                      |                              |
| 778560      | 2022-01-24   | lib/web_ui/test/golden_files            |                              |
| 634455      | 2016-08-09   | third_party/libpng                      |                              |
| 610751      | 2024-05-13   | .golden                                 |                              |
| 550475      | 2024-09-17   | impeller/fixtures/flutter_logo_baked.*  |                              |
| 526837      | 2016-08-09   | third_party/libevent                    |                              |
| 523436      | 2015-07-20   | third_party/boringssl                   |                              |
| 514968      | 2022-04-27   | impeller/fixtures/image.png             |                              |
| 461527      | 2015-12-11   | third_party/re2                         |                              |
| 418122      | 2015-10-12   | examples/demo_launcher                  |                              |
| 413787      | 2015-11-07   | .aac                                    |                              |
| 362787      | 2016-08-09   | third_party/glfw                        |                              |
| 349604      | 2016-08-09   | third_party/harfbuzz-ng                 |                              |
| 340869      | 2016-08-09   | third_party/okhttp                      |                              |
| 321659      | 2016-08-09   | .S                                      |                              |
| 300824      | 2016-08-09   | .so                                     |                              |
| 257633      | 2016-08-09   | third_party/libjpeg                     |                              |
| 257519      | 2016-08-09   | third_party/jinja2                      |                              |
| 249618      | 2016-08-09   | third_party/zlib                        |                              |
| 218643      | 2015-12-11   | third_party/brotli                      |                              |
| 188622      | 2021-01-06   | .idl                                    |                              |
| 184593      | 2015-09-02   | third_party/khronos                     |                              |
| 173210      | 2016-08-09   | .gypi                                   |                              |
| 170484      | 2016-08-09   | third_party/expat                       |                              |
| 169578      | 2016-08-09   | .asm                                    |                              |
| 161360      | 2016-08-09   | .m4                                     |                              |
| 142670      | 2018-05-10   | .in                                     |                              |
| 140364      | 2015-12-11   | third_party/ots                         |                              |
| 137270      | 2016-08-09   | .hh                                     |                              |
| 136787      | 2016-08-09   | .gyp                                    |                              |
| 99503       | 2016-08-09   | third_party/qcms                        |                              |
| 91730       | 2015-08-21   | .pxd                                    |                              |
| 84850       | 2016-08-09   | third_party/yasm                        |                              |

The following command will remove files and foldes from the checkout history. Since this is a destructive edit, the SHA1 git hashes will be changed in the process. At the end, the `.git` history will be 74 MB of object files.

```shell
# Lets do some heavy filtering;
# .git starts out at ~780MB and ends up at ~110MB
git filter-repo  --force --invert-paths \
--path-glob 'impeller/docs/assets/*.png' \
--path-glob 'impeller/docs/assets/*.gif' \
--path-glob '*/example/game/*' \
--path-glob 'benchmarks/parser/resources/html5.html' \
--path-glob '*.dll' \
--path-glob '*.jar' \
--path-glob '*/SourceHanSerifCN*' \
--path-glob 'third_party/txt/third_party/fonts/NotoSansCJK-Regular.ttc' \
--path-glob 'impeller/fixtures/flutter_logo_baked.*' \
--path-glob 'impeller/fixtures/image.png' \
--path-glob '*.golden' \
--path-glob '*.aac' \
--path-glob '*.S' \
--path-glob '*.so' \
--path-glob '*.idl' \
--path-glob '*.gpy' \
--path-glob '*.gypi' \
--path-glob '*.asm' \
--path-glob '*.m4' \
--path-glob '*.in' \
--path-glob '*.pxd' \
--path-glob '*.hh' \
--path 'ci/licenses_golden/licenses_third_party' \
--path 'testing/scenario_app/android/reports' \
--path 'impeller/third_party' \
--path 'mojo/public/third_party' \
--path 'tests/data' \
--path 'tests/fast' \
--path 'tests/framework' \
--path 'travis' \
--path 'mojo' \
--path 'sky/sdk' \
--path 'sky/engine' \
--path 'sky/tools/webkitpy' \
--path 'sky/shell' \
--path 'sky/packages/sky' \
--path 'sky/tests' \
--path 'sky/unit' \
--path 'sky/services' \
--path 'sky/compositor' \
--path 'sky/build' \
--path 'sky/specs' \
--path 'skysprites' \
--path 'examples/demo_launcher' \
--path 'examples/game' \
--path 'third_party/qcms' \
--path 'third_party/libevent' \
--path 'third_party/boringssl' \
--path 'third_party/tcmalloc' \
--path 'third_party/cython' \
--path 'third_party/protobuf' \
--path 'third_party/libpng' \
--path 'third_party/re2' \
--path 'third_party/harfbuzz-ng' \
--path 'third_party/jinja2' \
--path 'third_party/libjpeg' \
--path 'third_party/glfw' \
--path 'third_party/zlib' \
--path 'third_party/android_platform' \
--path 'third_party/expat' \
--path 'third_party/brotli' \
--path 'third_party/yasm' \
--path 'third_party/khronos' \
--path 'third_party/okhttp' \
--path 'third_party/libxml' \
--path 'third_party/ots' \
--path 'third_party/libXNVCtrl' \
--path 'lib/web_ui/test/golden_files' \
--path 'apk' \
--path 'flutter' \
--path 'base' \
--path 'sdk' \
--path 'gpu' \
--path 'engine' \
--path 'tools/webkitpy' \
--path 'tools/valgrind' \
--path 'tools/clang' \
--path 'tools/android' \
--path 'build/linux' \
--path 'build/win' \
--path 'build/mac' \
--path 'ui' \
--path 'examples/stocks' \
--path 'examples/stocks2' \
--path 'examples/stocks-fn' \
--path 'examples/data' \
--path 'examples/fitness' \
--path 'examples/city-list' \
--path 'examples/widgets' \
--path 'examples/raw' \
--path 'examples/color' \
--path 'examples/flights' \
--path 'examples/rendering' \
--path 'examples/fn' \
--path 'specs' \
--path 'url' \
--path 'services' \
--path 'framework' \
--path 'crypto' \
--path 'skia/ext' \
--path 'e2etests' \
--path 'tests/resources' \
--path 'viewer' \
--path 'lib/stub_ui' \
--path 'content_handler'

# Garbage collect!
git reflog expire --expire=now --all && git gc --prune=now --aggressive
```

## Step 3 - Rewirte directories

The final destination for the engine source code will be in the directory `engine/src/flutter` *except* for `DEPS` which remains at the root. Using `git mv` only affects HEAD and can have some problems when logging. Instead we'll re-write history so it makes sense in the new world.

```shell
# Move files to engine/src/flutter, update tags so they don't collide, and move DEPS back to root.
git filter-repo  --to-subdirectory-filter engine/src/flutter --tag-rename '':'engine-' --force
git filter-repo --path-rename engine/src/flutter/DEPS:DEPS
```

## Step 4 - Rewrite the PR links

The PR link in the first line of the comment message will be wrong; `flutter/flutter` doesn't have the same history. To make history a little bit better, we only want to edit the first line. This must be done before we merge with the `flutter/flutter` repo so as not to step on their commit lines.

```shell
git filter-repo --force --message-callback '
    return re.sub(br"^(.*)\((#\d+)\)\n(.*)", br"\1(flutter/engine\2)\n\3", message, 1)
    '
```

## Execute Order 42: Merge The Repositories

```shell
##############################################
## Now handle merging into flutter/flutter
##############################################

git clone git@github.com:flutter/flutter.git flutter_merge
cd flutter_merge

# add the other tree as remote
git remote add -f engine-upstream ~/src/engine_prep

# --no-commit is important because we want to look around
git merge --no-commit --allow-unrelated-histories engine-upstream/main

# You're a wizard, Harry
git commit -m "Merge flutter/engine into framework"

# Garbage collect!
# Now at 234MB .git
git reflog expire --expire=now --all && git gc --prune=now --aggressive
```
