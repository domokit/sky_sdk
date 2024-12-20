// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'regexp_debug.dart';

// Directories or files that are not included in the final binaries.
//
// These are absolute paths relative to the root of the buildroot repository
// as checked out after gclient imports all the dependencies.
//
// Do not provide trailing slashes for directories.
//
// Including a directory in this list excludes the entire subtree rooted at
// that directory.
//
// Keep this list in lexical order.
final Set<String> skippedPaths = <String>{
  r'README.md',
  r'build', // only used by build
  r'build_overrides', // only used by build
  r'buildtools', // only used by build
  r'flutter/build',
  r'flutter/build_overrides', // only used by build
  r'flutter/buildtools', // only used by build
  r'flutter/ci',
  r'flutter/docs',
  r'flutter/engine.code-workspace',
  r'flutter/flutter_frontend_server',
  r'flutter/impeller/docs',
  r'flutter/lib/web_ui/build', // this is compiler-generated output
  r'flutter/lib/web_ui/dev', // these are build tools; they do not end up in Engine artifacts
  r'flutter/prebuilts',
  r'flutter/sky/packages/sky_engine/LICENSE',
  r'flutter/third_party/android_embedding_dependencies', // Not shipped. Used only for the build-time classpath, and for the in-tree testing framework for Android
  r'flutter/third_party/android_tools', // excluded on advice
  r'flutter/third_party/angle/android',
  r'flutter/third_party/angle/doc',
  r'flutter/third_party/angle/extensions',
  r'flutter/third_party/angle/infra',
  r'flutter/third_party/angle/scripts',
  r'flutter/third_party/angle/src/libANGLE/renderer/metal/doc',
  r'flutter/third_party/angle/src/libANGLE/renderer/vulkan/doc',
  r'flutter/third_party/angle/src/third_party/volk', // We don't use Vulkan in our ANGLE build.
  r'flutter/third_party/angle/third_party', // Unused by Flutter: BUILD files with forwarding targets (but no code).
  r'flutter/third_party/angle/tools', // These are build-time tools, and aren't shipped.
  r'flutter/third_party/angle/util',
  r'flutter/third_party/benchmark', // only used by tests
  r'flutter/third_party/boringssl/src/crypto/err/err_data_generate.go',
  r'flutter/third_party/boringssl/src/fuzz', // testing tools, not shipped
  r'flutter/third_party/boringssl/src/rust', // rust-related code is not shipped
  r'flutter/third_party/boringssl/src/util', // code generators, not shipped
  r'flutter/third_party/brotli/csharp', // C# code, not shipped.
  r'flutter/third_party/brotli/docs', // Docs, not shipped.
  r'flutter/third_party/brotli/go', // Go code, not shipped.
  r'flutter/third_party/brotli/java', // Java code, not shipped.
  r'flutter/third_party/brotli/js', // JavaScript code, not shipped.
  r'flutter/third_party/brotli/python', // Python code, not shipped.
  r'flutter/third_party/brotli/research', // Research tools, not shipped.
  r'flutter/third_party/brotli/scripts', // Scripts, not shipped.
  r'flutter/third_party/brotli/tests', // Tests, not shipped.
  r'flutter/third_party/dart/benchmarks', // not shipped in binary
  r'flutter/third_party/dart/build', // not shipped in binary
  r'flutter/third_party/dart/docs', // not shipped in binary
  r'flutter/third_party/dart/pkg', // packages that don't become part of the binary (e.g. the analyzer)
  r'flutter/third_party/dart/runtime/bin/ffi_test',
  r'flutter/third_party/dart/runtime/docs',
  r'flutter/third_party/dart/runtime/vm/service',
  r'flutter/third_party/dart/sdk/lib/html/doc',
  r'flutter/third_party/dart/sdk/lib/svg/dart2js', // generated from other sources
  r'flutter/third_party/dart/third_party/binary_size', // not linked in
  r'flutter/third_party/dart/third_party/binaryen', // not linked in
  r'flutter/third_party/dart/third_party/d3', // Siva says "that is the charting library used by the binary size tool"
  r'flutter/third_party/dart/third_party/d8', // testing tool for dart2js
  r'flutter/third_party/dart/third_party/devtools', // not linked in
  r'flutter/third_party/dart/third_party/fallback_root_certificates/certdata.pem',
  r'flutter/third_party/dart/third_party/fallback_root_certificates/certdata.txt',
  r'flutter/third_party/dart/third_party/firefox_jsshell', // testing tool for dart2js
  r'flutter/third_party/dart/third_party/pkg',
  r'flutter/third_party/dart/third_party/pkg_tested',
  r'flutter/third_party/dart/third_party/requirejs', // only used by DDC
  r'flutter/third_party/dart/tools', // not shipped in binary
  r'flutter/third_party/depot_tools', // only used for the Windows build system.
  r'flutter/third_party/expat/expat/doc',
  r'flutter/third_party/expat/expat/win32/expat.iss',
  r'flutter/third_party/flatbuffers/android',
  r'flutter/third_party/flatbuffers/benchmarks',
  r'flutter/third_party/flatbuffers/docs',
  r'flutter/third_party/flatbuffers/go',
  r'flutter/third_party/flatbuffers/net',
  r'flutter/third_party/flatbuffers/php',
  r'flutter/third_party/flatbuffers/python',
  r'flutter/third_party/flatbuffers/rust',
  r'flutter/third_party/flatbuffers/ts',
  r'flutter/third_party/freetype2/builds',
  r'flutter/third_party/freetype2/src/tools',
  r'flutter/third_party/glfw/deps', // Only used by examples and tests; not linked in build.
  r'flutter/third_party/glfw/docs',
  r'flutter/third_party/gn',
  r'flutter/third_party/google_fonts_for_unit_tests', // only used in web unit tests
  r'flutter/third_party/gradle',
  r'flutter/third_party/harfbuzz/docs',
  r'flutter/third_party/harfbuzz/util', // utils are command line tools that do not end up in the binary
  r'flutter/third_party/icu/filters',
  r'flutter/third_party/icu/fuzzers',
  r'flutter/third_party/icu/patches', // patches added by Chromium that are already applied to the code base
  r'flutter/third_party/icu/scripts',
  r'flutter/third_party/icu/source/common/unicode/uvernum.h', // this file contains strings that confuse the analysis
  r'flutter/third_party/icu/source/config',
  r'flutter/third_party/icu/source/data/brkitr/dictionaries/burmesedict.txt', // explicitly handled by ICU license
  r'flutter/third_party/icu/source/data/brkitr/dictionaries/cjdict.txt', // explicitly handled by ICU license
  r'flutter/third_party/icu/source/data/brkitr/dictionaries/laodict.txt', // explicitly handled by ICU license
  r'flutter/third_party/icu/source/data/dtd',
  r'flutter/third_party/imgui',
  r'flutter/third_party/inja/doc', // documentation
  r'flutter/third_party/inja/third_party/amalgamate', // only used at build time
  r'flutter/third_party/inja/third_party/include/doctest', // seems to be a unit test library
  r'flutter/third_party/java', // only used for Android builds
  r'flutter/third_party/json/docs',
  r'flutter/third_party/libcxx/benchmarks',
  r'flutter/third_party/libcxx/docs',
  r'flutter/third_party/libcxx/src/support/solaris',
  r'flutter/third_party/libcxx/utils',
  r'flutter/third_party/libcxxabi/www',
  r'flutter/third_party/libpng/ci', // not linked in
  r'flutter/third_party/libpng/contrib', // not linked in
  r'flutter/third_party/libpng/loongarch', // not linked in
  r'flutter/third_party/libpng/mips', // not linked in
  r'flutter/third_party/libpng/powerpc', // not linked in
  r'flutter/third_party/libpng/projects', // not linked in
  r'flutter/third_party/libpng/scripts', // not linked in
  r'flutter/third_party/libtess2/Contrib/nanosvg.c', // only used by the ../Example
  r'flutter/third_party/libtess2/Contrib/nanosvg.h', // only used by the ../Example
  r'flutter/third_party/libtess2/Example',
  r'flutter/third_party/libwebp/doc',
  r'flutter/third_party/libwebp/gradle', // not included in our build
  r'flutter/third_party/libwebp/swig', // not included in our build
  r'flutter/third_party/libwebp/webp_js',
  r'flutter/third_party/ninja', // build system
  r'flutter/third_party/ocmock', // only used for tests
  r'flutter/third_party/perfetto/debian', // contains nothing that ends up in the binary executable
  r'flutter/third_party/perfetto/infra', // contains nothing that ends up in the binary executable
  r'flutter/third_party/perfetto/protos', // contains nothing that ends up in the binary executable
  r'flutter/third_party/perfetto/python/perfetto/trace_processor', // contains nothing that ends up in the binary executable
  r'flutter/third_party/perfetto/src/ipc', // contains nothing that ends up in the binary executable
  r'flutter/third_party/perfetto/src/profiling/memory', // contains nothing that ends up in the binary executable
  r'flutter/third_party/perfetto/src/tools', // contains nothing that ends up in the binary executable
  r'flutter/third_party/perfetto/src/trace_processor', // contains nothing that ends up in the binary executable
  r'flutter/third_party/perfetto/src/traced', // contains nothing that ends up in the binary executable
  r'flutter/third_party/perfetto/src/tracing', // contains nothing that ends up in the binary executable
  r'flutter/third_party/pkg/archive', // contains nothing that ends up in the binary executable
  r'flutter/third_party/pkg/equatable',
  r'flutter/third_party/pkg/flutter_packages',
  r'flutter/third_party/pkg/gcloud',
  r'flutter/third_party/pkg/googleapis',
  r'flutter/third_party/pkg/process',
  r'flutter/third_party/pkg/process_runner',
  r'flutter/third_party/pkg/vector_math',
  r'flutter/third_party/protobuf', // build-time dependency only
  r'flutter/third_party/pyyaml', // build-time dependency only
  r'flutter/third_party/rapidjson/contrib', // contains nothing that ends up in the binary executable
  r'flutter/third_party/rapidjson/doc', // documentation
  r'flutter/third_party/shaderc/third_party/LICENSE.glslang', // unclear what the purpose of this file is
  r'flutter/third_party/shaderc/third_party/LICENSE.spirv-tools', // unclear what the purpose of this file is
  r'flutter/third_party/skia/bazel', // contains nothing that ends up in the binary executable
  r'flutter/third_party/skia/bench',
  r'flutter/third_party/skia/demos.skia.org',
  r'flutter/third_party/skia/docs',
  r'flutter/third_party/skia/experimental',
  r'flutter/third_party/skia/infra', // contains nothing that ends up in the binary executable
  r'flutter/third_party/skia/modules/canvaskit/go/gold_test_env',
  r'flutter/third_party/skia/platform_tools', // contains nothing that ends up in the binary executable
  r'flutter/third_party/skia/resources', // contains nothing that ends up in the binary executable
  r'flutter/third_party/skia/samplecode',
  r'flutter/third_party/skia/site',
  r'flutter/third_party/skia/specs',
  r'flutter/third_party/skia/third_party/freetype2', // we use our own version
  r'flutter/third_party/skia/third_party/icu', // we use our own version
  r'flutter/third_party/skia/third_party/libjpeg-turbo', // we use our own version
  r'flutter/third_party/skia/third_party/libpng', // we use our own version
  r'flutter/third_party/skia/third_party/lua', // not linked in
  r'flutter/third_party/skia/third_party/vello', // not linked in
  r'flutter/third_party/skia/tools', // contains nothing that ends up in the binary executable
  r'flutter/third_party/stb',
  r'flutter/third_party/swiftshader/third_party', // See: https://github.com/flutter/flutter/issues/145726
  r'flutter/third_party/test_shaders', // for tests only
  r'flutter/third_party/txt/third_party/fonts',
  r'flutter/third_party/vulkan-deps/glslang/LICENSE', // excluded to make sure we don't accidentally apply it as a default license
  r'flutter/third_party/vulkan-deps/glslang/src/LICENSE.txt', // redundant with licenses inside files
  r'flutter/third_party/vulkan-deps/glslang/src/glslang/OSDependent/Web', // we only use glslang in impellerc, not in web apps
  r'flutter/third_party/vulkan-deps/glslang/src/kokoro', // only build files
  r'flutter/third_party/vulkan-deps/spirv-cross/src/LICENSES', // directory with license templates
  r'flutter/third_party/vulkan-deps/spirv-cross/src/shaders', // used by regression tests
  r'flutter/third_party/vulkan-deps/spirv-cross/src/shaders-hlsl', // used by regression tests
  r'flutter/third_party/vulkan-deps/spirv-cross/src/shaders-hlsl-no-opt', // used by regression tests
  r'flutter/third_party/vulkan-deps/spirv-cross/src/shaders-msl', // used by regression tests
  r'flutter/third_party/vulkan-deps/spirv-cross/src/shaders-msl-no-opt', // used by regression tests
  r'flutter/third_party/vulkan-deps/spirv-cross/src/shaders-no-opt', // used by regression tests
  r'flutter/third_party/vulkan-deps/spirv-cross/src/shaders-other', // used by regression tests
  r'flutter/third_party/vulkan-deps/spirv-cross/src/shaders-reflection', // used by regression tests
  r'flutter/third_party/vulkan-deps/spirv-cross/src/shaders-ue4', // used by regression tests
  r'flutter/third_party/vulkan-deps/spirv-cross/src/shaders-ue4-no-opt', // used by regression tests
  r'flutter/third_party/vulkan-deps/spirv-headers', // only used on hosts for tests
  r'flutter/third_party/vulkan-deps/spirv-tools', // only used on hosts for tests
  r'flutter/third_party/vulkan-deps/vulkan-headers/src/LICENSE.md', // redundant with licenses inside files
  r'flutter/third_party/vulkan-deps/vulkan-headers/src/registry',
  r'flutter/third_party/vulkan-deps/vulkan-loader', // on hosts for tests
  r'flutter/third_party/vulkan-deps/vulkan-tools', // on hosts for tests
  r'flutter/third_party/vulkan-deps/vulkan-utility-libraries/src/LICENSE.md', // redundant with licenses inside files
  r'flutter/third_party/vulkan-deps/vulkan-validation-layers/src/docs',
  r'flutter/third_party/vulkan_memory_allocator/bin',
  r'flutter/third_party/vulkan_memory_allocator/docs',
  r'flutter/third_party/vulkan_memory_allocator/media',
  r'flutter/third_party/vulkan_memory_allocator/src',
  r'flutter/third_party/vulkan_memory_allocator/tools',
  r'flutter/third_party/wuffs/docs',
  r'flutter/third_party/wuffs/script',
  r'flutter/third_party/yapf', // only used for code formatting
  r'flutter/third_party/zlib/contrib/minizip/miniunz.c', // sample file
  r'flutter/third_party/zlib/contrib/minizip/minizip.c', // sample file
  r'flutter/tools',
  r'flutter/web_sdk', // this code is not linked into Flutter apps; it's only used by engine tests and tools
  r'fuchsia/sdk/linux/docs',
  r'fuchsia/sdk/linux/meta',
  r'fuchsia/sdk/linux/NOTICE.fuchsia', // covers things that contribute to the Fuchsia SDK; see fxb/94240
  r'fuchsia/sdk/linux/packages/blobs', // See https://github.com/flutter/flutter/issues/134042.
  r'fuchsia/sdk/linux/tools',
  r'fuchsia/sdk/mac/docs',
  r'fuchsia/sdk/mac/meta',
  r'fuchsia/sdk/mac/NOTICE.fuchsia',
  r'fuchsia/sdk/mac/tools',
  r'out', // output of build
  r'third_party/android_tools', // excluded on advice
  r'third_party/libxml', // dependency of the testing system that we don't actually use
  r'tools', // not distributed in binary
};

// Directories that should be skipped anywhere we find them.
//
// Any directory whose name matches one of these strings is skipped, including
// its entire subtree.
//
// Keep this list in lexical order.
final Set<String> skippedCommonDirectories = <String>{
  r'.bazelci',
  r'.build-id',
  r'.ccls-cache',
  r'.cipd',
  r'.dart_tool',
  r'.git',
  r'.github',
  r'.reuse',
  r'.versions',
  r'.vscode',
  r'CMake',
  r'Test',
  r'cmake',
  r'example',
  r'examples',
  r'fixtures',
  r'googletest',
  r'javatests',
  r'jvmTest',
  r'playground',
  r'samples',
  r'test',
  r'test.disabled',
  r'test_runner',
  r'test_support',
  r'testdata',
  r'testing',
  r'tests',
  r'unit_tests',
};

// Filenames of files we never look at.
//
// Any file with a name in this list is skipped.
//
// Be careful about adding files like "LICENSE" or "README" (and such
// variants) to this list as many packages put important license text in those
// files and we don't want to skip them. Only include files here whose names
// we would want to skip in ANY package.
//
// Keep this list in lexical order.
final Set<String> skippedCommonFiles = <String>{
  r'.DS_Store',
  r'.appveyor.yml',
  r'.bazelrc',
  r'.bazelversion',
  r'.clang-format',
  r'.clang-tidy',
  r'.editorconfig',
  r'.eslintrc.js',
  r'.gitattributes',
  r'.gitconfig',
  r'.gitignore',
  r'.gitlab-ci.yml',
  r'.gitmodules',
  r'.gn',
  r'.lgtm.yml',
  r'.mailmap',
  r'.packages',
  r'.project',
  r'.style.yapf',
  r'.travis.yml',
  r'.vpython',
  r'.vpython3',
  r'.yapfignore',
  r'ABSEIL_ISSUE_TEMPLATE.md',
  r'ANNOUNCE',
  r'API-CONVENTIONS.md',
  r'AUTHORS',
  r'BREAKING-CHANGES.md',
  r'BUILD.bazel',
  r'BUILD.md',
  r'BUILDING.md',
  r'Brewfile',
  r'CHANGES',
  r'CHANGES.md',
  r'CITATION.cff',
  r'CMake.README',
  r'CMakeLists.txt',
  r'CODEOWNERS',
  r'CODE_CONVENTIONS.md',
  r'CODE_OF_CONDUCT.adoc',
  r'CODE_OF_CONDUCT.md',
  r'CONFIG.md',
  r'CONTRIBUTORS',
  r'CONTRIBUTORS.md',
  r'CPPLINT.cfg',
  r'CQ_COMMITTERS',
  r'CREDITS.TXT',
  r'Changes',
  r'CodingStandard.md',
  r'DEPS',
  r'DIR_METADATA',
  r'Dockerfile',
  r'Doxyfile',
  r'FAQ.md',
  r'FIPS.md',
  r'FUZZING.md',
  r'FeatureSupportGL.md',
  r'FormatTables.md',
  r'Formatters.md',
  r'GIT_REVISION',
  r'HACKING.md',
  r'INCORPORATING.md',
  r'LAYER_CONFIGURATION.md',
  r'LOCALE_DEPS.json',
  r'MANIFEST.in',
  r'MANIFEST.txt',
  r'METADATA',
  r'NEWS',
  r'OWNERS',
  r'OWNERS.android',
  r'PATENT_GRANT',
  r'PORTING.md',
  r'README.asciidoc',
  r'RELEASE_NOTES.TXT',
  r'RELEASING.md',
  r'RapidJSON.pc.in',
  r'RapidJSONConfig.cmake.in',
  r'RapidJSONConfigVersion.cmake.in',
  r'SANDBOXING.md',
  r'SECURITY.md',
  r'STYLE.md',
  r'TESTING.md',
  r'THANKS',
  r'TODO',
  r'TODO.TXT',
  r'TRADEMARK',
  r'UPGRADES.md',
  r'UniformBlockToStructuredBufferTranslation.md',
  r'WATCHLISTS',
  r'WORKSPACE',
  r'WORKSPACE.bazel',
  r'_config.yml',
  r'additional_readme_paths.json',
  r'alg_outline.md',
  r'allowed_experiments.json',
  r'amalgamate_config.json',
  r'api_readme.md',
  r'appveyor-reqs-install.cmd',
  r'appveyor.yml',
  r'build.xml',
  r'codereview.settings',
  r'coderules.txt',
  r'configure-ac-style.md',
  r'doxygen.config',
  r'example.html',
  r'gerrit.md',
  r'gradle.properties',
  r'include_dirs.js',
  r'known_good.json',
  r'known_good_khr.json',
  r'libraries.json',
  r'library.json',
  r'license-checker.cfg',
  r'license.html',
  r'memory-sanitizer-blacklist.txt',
  r'meson.build',
  r'meta.json',
  r'minizip.md',
  r'package.json',
  r'pkgdataMakefile.in',
  r'pom.xml',
  r'pubspec.lock',
  r'requirements-dev.txt',
  r'requirements.txt',
  r'sources.txt',
  r'structure.txt',
  r'swift.swiftformat',
  r'sync.txt',
  r'unit_tests.md',
  r'version',
  r'version_history.json',
  r'vms_make.com',
  r'vmservice_libraries.json',
};

// Extensions that we just never look at.
//
// We explicitly do not exclude .txt, .md, .TXT, and .MD files because it is common
// for licenses to be in such files.
//
// This list only works for extensions with a single dot.
//
// Keep this list in lexical order.
final Set<String> skippedCommonExtensions = <String>{
  r'.1',
  r'.3',
  r'.5',
  r'.autopkg',
  r'.build',
  r'.bzl',
  r'.cmake',
  r'.css',
  r'.gn',
  r'.gni',
  r'.gradle',
  r'.log',
  r'.m4',
  r'.mk',
  r'.pl',
  r'.py',
  r'.pyc',
  r'.pylintrc',
  r'.sha1',
  r'.yaml',
  r'.~',
};

// Patterns for files and directories we should skip.
//
// Keep this list to a minimum, preferring all the other lists
// in this file. Testing patterns is more expensive.
//
// Keep this list in lexical order.
final List<Pattern> skippedFilePatterns = <Pattern>[
  RegExp(r'\.[1-8]\.in$'), // man page source files (e.g. foo.1.in)
  RegExp(r'/(?:_|\b)CONTRIBUTING(?:_|\b)[^/]*$'),
  RegExp(r'/(?:_|\b)LAST_UPDATE(?:_|\b)[^/]*$'),
  RegExp(r'/(?:_|\b)PATENTS(?:_|\b)[^/]*$'),
  RegExp(r'/(?:_|\b)README(?!\.IJG)(?:_|\b)[^/]*$', caseSensitive: false),
  RegExp(r'/(?:_|\b)VERSION(?:_|\b)[^/]*$'),
  RegExp(r'/CHANGELOG(?:\.[.A-Z0-9]+)?$', caseSensitive: false),
  RegExp(r'/INSTALL(?:\.[a-zA-Z0-9]+)?$'),
  RegExp(r'/Makefile(?:\.[.A-Z0-9]+)?$', caseSensitive: false),
  RegExp(
    r'\.~[0-9]+~$',
    expectNoMatch: true,
  ), // files that end in ".~1~", a backup convention of some IDEs
  RegExp(r'\bmanual\.txt$'),
  RegExp(r'^flutter/(?:.+/)*[^/]+_unittests?\.[^/]+$'),
  RegExp(
    r'^flutter/lib/web_ui/lib/assets/ahem\.ttf$',
    expectNoMatch: true,
  ), // this gitignored file exists only for testing purposes
  RegExp(r'^flutter/sky/packages/sky_engine/LICENSE$'), // that is the output of this script
  RegExp(r'^flutter/third_party/abseil-cpp/(?:.+/)*[^/]+_test\.[^/]+$'),
  RegExp(r'^flutter/third_party/angle/(?:.+/)*[^/]+_unittest\.[^/]+$'),
  RegExp(r'^flutter/third_party/boringssl/(?:.+/)*[^/]+_test\.[^/]+$'),
  RegExp(r'^flutter/third_party/boringssl/src/crypto/fipsmodule/bn/[^/]+.go$'),
  RegExp(r'^flutter/third_party/boringssl/src/crypto/fipsmodule/ec/[^/]+.go$'),
  RegExp(r'^flutter/third_party/dart/(?:.+/)*[^/]+_test\.[^/]+$'),
  RegExp(
    r'^flutter/third_party/freetype2/docs/(?!FTL\.TXT$).+',
  ), // ignore all documentation except the license
  RegExp(r'^flutter/third_party/zlib/(?:.+/)*[^/]+_unittest\.[^/]+$'),
];
