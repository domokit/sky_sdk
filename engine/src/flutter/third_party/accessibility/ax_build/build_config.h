// Copyright (c) 2012 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file adds defines about the platform we're currently building on.
//
//  Operating System:
//    OS_AIX / OS_ANDROID / OS_ASMJS / OS_FREEBSD / OS_FUCHSIA / OS_IOS /
//    OS_LINUX / OS_MAC / OS_NACL (SFI or NONSFI) / OS_NETBSD / OS_OPENBSD /
//    OS_QNX / OS_SOLARIS / OS_WIN
//  Operating System family:
//    OS_APPLE: IOS or MAC
//    OS_BSD: FREEBSD or NETBSD or OPENBSD
//    OS_POSIX: AIX or ANDROID or ASMJS or CHROMEOS or FREEBSD or IOS or LINUX
//              or MAC or NACL or NETBSD or OPENBSD or QNX or SOLARIS
//
//  /!\ Note: OS_CHROMEOS is set by the build system, not this file
//
//  Compiler:
//    COMPILER_MSVC / COMPILER_GCC
//
//  Processor:
//    ARCH_CPU_ARM64 / ARCH_CPU_ARMEL / ARCH_CPU_MIPS / ARCH_CPU_MIPS64 /
//    ARCH_CPU_MIPS64EL / ARCH_CPU_MIPSEL / ARCH_CPU_PPC64 / ARCH_CPU_S390 /
//    ARCH_CPU_S390X / ARCH_CPU_X86 / ARCH_CPU_X86_64
//  Processor family:
//    ARCH_CPU_ARM_FAMILY: ARMEL or ARM64
//    ARCH_CPU_MIPS_FAMILY: MIPS64EL or MIPSEL or MIPS64 or MIPS
//    ARCH_CPU_PPC64_FAMILY: PPC64
//    ARCH_CPU_S390_FAMILY: S390 or S390X
//    ARCH_CPU_X86_FAMILY: X86 or X86_64
//  Processor features:
//    ARCH_CPU_31_BITS / ARCH_CPU_32_BITS / ARCH_CPU_64_BITS
//    ARCH_CPU_BIG_ENDIAN / ARCH_CPU_LITTLE_ENDIAN

#ifndef UI_ACCESSIBILITY_BUILD_BUILD_CONFIG_H_
#define UI_ACCESSIBILITY_BUILD_BUILD_CONFIG_H_

// A set of macros to use for platform detection.
#if defined(__native_client__)
// __native_client__ must be first, so that other OS_ defines are not set.
#define OS_NACL 1
// OS_NACL comes in two sandboxing technology flavors, SFI or Non-SFI.
// PNaCl toolchain defines __native_client_nonsfi__ macro in Non-SFI build
// mode, while it does not in SFI build mode.
#if defined(__native_client_nonsfi__)
#define OS_NACL_NONSFI
#else
#define OS_NACL_SFI
#endif
#elif defined(ANDROID)
#define OS_ANDROID 1
#elif defined(__APPLE__)
// Only include TargetConditionals after testing ANDROID as some Android builds
// on the Mac have this header available and it's not needed unless the target
// is really an Apple platform.
#include <TargetConditionals.h>
#if defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
#define OS_IOS 1
#else
#define OS_MAC 1
#endif  // defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE
#elif defined(__linux__)
#define OS_LINUX 1
// Include a system header to pull in features.h for glibc/uclibc macros.
#include <unistd.h>
#if defined(__GLIBC__) && !defined(__UCLIBC__)
// We really are using glibc, not uClibc pretending to be glibc.
#define LIBC_GLIBC 1
#endif
#elif defined(_WIN32)
#define OS_WIN 1
#elif defined(__Fuchsia__)
#define OS_FUCHSIA 1
#elif defined(__FreeBSD__)
#define OS_FREEBSD 1
#elif defined(__NetBSD__)
#define OS_NETBSD 1
#elif defined(__OpenBSD__)
#define OS_OPENBSD 1
#elif defined(__sun)
#define OS_SOLARIS 1
#elif defined(__QNXNTO__)
#define OS_QNX 1
#elif defined(_AIX)
#define OS_AIX 1
#elif defined(__asmjs__) || defined(__wasm__)
#define OS_ASMJS 1
#else
#error Please add support for your platform in build/build_config.h
#endif
// NOTE: Adding a new port? Please follow
// https://chromium.googlesource.com/chromium/src/+/master/docs/new_port_policy.md

#if defined(OS_MAC) || defined(OS_IOS)
#define OS_APPLE 1
#endif

// For access to standard BSD features, use OS_BSD instead of a
// more specific macro.
#if defined(OS_FREEBSD) || defined(OS_NETBSD) || defined(OS_OPENBSD)
#define OS_BSD 1
#endif

// For access to standard POSIXish features, use OS_POSIX instead of a
// more specific macro.
#if defined(OS_AIX) || defined(OS_ANDROID) || defined(OS_ASMJS) ||  \
    defined(OS_FREEBSD) || defined(OS_IOS) || defined(OS_LINUX) ||  \
    defined(OS_CHROMEOS) || defined(OS_MAC) || defined(OS_NACL) ||  \
    defined(OS_NETBSD) || defined(OS_OPENBSD) || defined(OS_QNX) || \
    defined(OS_SOLARIS)
#define OS_POSIX 1
#endif

// Compiler detection. Note: clang masquerades as GCC on POSIX and as MSVC on
// Windows.
#if defined(__GNUC__)
#define COMPILER_GCC 1
#elif defined(_MSC_VER)
#define COMPILER_MSVC 1
#else
#error Please add support for your compiler in build/build_config.h
#endif

// Processor architecture detection.  For more info on what's defined, see:
//   http://msdn.microsoft.com/en-us/library/b0084kay.aspx
//   http://www.agner.org/optimize/calling_conventions.pdf
//   or with gcc, run: "echo | gcc -E -dM -"
#if defined(_M_X64) || defined(__x86_64__)
#define ARCH_CPU_X86_FAMILY 1
#define ARCH_CPU_X86_64 1
#define ARCH_CPU_64_BITS 1
#define ARCH_CPU_LITTLE_ENDIAN 1
#elif defined(_M_IX86) || defined(__i386__)
#define ARCH_CPU_X86_FAMILY 1
#define ARCH_CPU_X86 1
#define ARCH_CPU_32_BITS 1
#define ARCH_CPU_LITTLE_ENDIAN 1
#elif defined(__s390x__)
#define ARCH_CPU_S390_FAMILY 1
#define ARCH_CPU_S390X 1
#define ARCH_CPU_64_BITS 1
#define ARCH_CPU_BIG_ENDIAN 1
#elif defined(__s390__)
#define ARCH_CPU_S390_FAMILY 1
#define ARCH_CPU_S390 1
#define ARCH_CPU_31_BITS 1
#define ARCH_CPU_BIG_ENDIAN 1
#elif (defined(__PPC64__) || defined(__PPC__)) && defined(__BIG_ENDIAN__)
#define ARCH_CPU_PPC64_FAMILY 1
#define ARCH_CPU_PPC64 1
#define ARCH_CPU_64_BITS 1
#define ARCH_CPU_BIG_ENDIAN 1
#elif defined(__PPC64__)
#define ARCH_CPU_PPC64_FAMILY 1
#define ARCH_CPU_PPC64 1
#define ARCH_CPU_64_BITS 1
#define ARCH_CPU_LITTLE_ENDIAN 1
#elif defined(__ARMEL__)
#define ARCH_CPU_ARM_FAMILY 1
#define ARCH_CPU_ARMEL 1
#define ARCH_CPU_32_BITS 1
#define ARCH_CPU_LITTLE_ENDIAN 1
#elif defined(__aarch64__) || defined(_M_ARM64)
#define ARCH_CPU_ARM_FAMILY 1
#define ARCH_CPU_ARM64 1
#define ARCH_CPU_64_BITS 1
#define ARCH_CPU_LITTLE_ENDIAN 1
#elif defined(__pnacl__) || defined(__asmjs__) || defined(__wasm__)
#define ARCH_CPU_32_BITS 1
#define ARCH_CPU_LITTLE_ENDIAN 1
#elif defined(__MIPSEL__)
#if defined(__LP64__)
#define ARCH_CPU_MIPS_FAMILY 1
#define ARCH_CPU_MIPS64EL 1
#define ARCH_CPU_64_BITS 1
#define ARCH_CPU_LITTLE_ENDIAN 1
#else
#define ARCH_CPU_MIPS_FAMILY 1
#define ARCH_CPU_MIPSEL 1
#define ARCH_CPU_32_BITS 1
#define ARCH_CPU_LITTLE_ENDIAN 1
#endif
#elif defined(__MIPSEB__)
#if defined(__LP64__)
#define ARCH_CPU_MIPS_FAMILY 1
#define ARCH_CPU_MIPS64 1
#define ARCH_CPU_64_BITS 1
#define ARCH_CPU_BIG_ENDIAN 1
#else
#define ARCH_CPU_MIPS_FAMILY 1
#define ARCH_CPU_MIPS 1
#define ARCH_CPU_32_BITS 1
#define ARCH_CPU_BIG_ENDIAN 1
#endif
#else
#error Please add support for your architecture in build/build_config.h
#endif

// Type detection for wchar_t.
#if defined(OS_WIN)
#define WCHAR_T_IS_UTF16
#elif defined(OS_FUCHSIA)
#define WCHAR_T_IS_UTF32
#elif defined(OS_POSIX) && defined(COMPILER_GCC) && defined(__WCHAR_MAX__) && \
    (__WCHAR_MAX__ == 0x7fffffff || __WCHAR_MAX__ == 0xffffffff)
#define WCHAR_T_IS_UTF32
#elif defined(OS_POSIX) && defined(COMPILER_GCC) && defined(__WCHAR_MAX__) && \
    (__WCHAR_MAX__ == 0x7fff || __WCHAR_MAX__ == 0xffff)
// On Posix, we'll detect short wchar_t, but projects aren't guaranteed to
// compile in this mode (in particular, Chrome doesn't). This is intended for
// other projects using base who manage their own dependencies and make sure
// short wchar works for them.
#define WCHAR_T_IS_UTF16
#else
#error Please add support for your compiler in build/build_config.h
#endif

#if defined(OS_ANDROID)
// The compiler thinks std::string::const_iterator and "const char*" are
// equivalent types.
#define STD_STRING_ITERATOR_IS_CHAR_POINTER
// The compiler thinks std::u16string::const_iterator and "char16*" are
// equivalent types.
#define BASE_STRING16_ITERATOR_IS_CHAR16_POINTER
#endif

#endif  // UI_ACCESSIBILITY_BUILD_BUILD_CONFIG_H_
