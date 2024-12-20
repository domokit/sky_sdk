TESTING_DIRECTORY=$(cd $(dirname "${BASH_SOURCE[0]}"); pwd -P)
ENGINE_BUILDROOT=$(cd $TESTING_DIRECTORY/../..; pwd -P)

case "$(uname -s)" in
  Linux)
    BUILDTOOLS_DIRECTORY="${ENGINE_BUILDROOT}/flutter/buildtools/linux-x64"
    ;;
  Darwin)
    BUILDTOOLS_DIRECTORY="${ENGINE_BUILDROOT}/flutter/buildtools/mac-x64"
    ;;
esac

TSAN_SUPPRESSIONS_FILE="${TESTING_DIRECTORY}/tsan_suppressions.txt"
export TSAN_OPTIONS="suppressions=${TSAN_SUPPRESSIONS_FILE}"
echo "Using Thread Sanitizer suppressions in ${TSAN_SUPPRESSIONS_FILE}"

LSAN_SUPPRESSIONS_FILE="${TESTING_DIRECTORY}/lsan_suppressions.txt"
export LSAN_OPTIONS="suppressions=${LSAN_SUPPRESSIONS_FILE}"
echo "Using Leak Sanitizer suppressions in ${LSAN_SUPPRESSIONS_FILE}"

UBSAN_SUPPRESSIONS_FILE="${TESTING_DIRECTORY}/ubsan_suppressions.txt"
export UBSAN_OPTIONS="suppressions=${UBSAN_SUPPRESSIONS_FILE}"
echo "Using Undefined Behavior suppressions in ${UBSAN_SUPPRESSIONS_FILE}"

export ASAN_OPTIONS="symbolize=1:detect_leaks=1:intercept_tls_get_addr=0"
export ASAN_SYMBOLIZER_PATH="${BUILDTOOLS_DIRECTORY}/clang/bin/llvm-symbolizer"
