// Copyright (c) 2014 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file contains defines and typedefs that allow popular Windows types to
// be used without the overhead of including windows.h.

#ifndef BASE_WIN_WINDOWS_TYPES_H_
#define BASE_WIN_WINDOWS_TYPES_H_

// Needed for function prototypes.
#include <concurrencysal.h>
#include <sal.h>
#include <specstrings.h>

#ifdef __cplusplus
extern "C" {
#endif

// typedef and define the most commonly used Windows integer types.

typedef unsigned long DWORD;
typedef long LONG;
typedef __int64 LONGLONG;
typedef unsigned __int64 ULONGLONG;

#define VOID void
typedef char CHAR;
typedef short SHORT;
typedef long LONG;
typedef int INT;
typedef unsigned int UINT;
typedef unsigned int* PUINT;
typedef void* LPVOID;
typedef void* PVOID;
typedef void* HANDLE;
typedef int BOOL;
typedef unsigned char BYTE;
typedef BYTE BOOLEAN;
typedef DWORD ULONG;
typedef unsigned short WORD;
typedef WORD UWORD;
typedef WORD ATOM;

#if defined(_WIN64)
typedef __int64 INT_PTR, *PINT_PTR;
typedef unsigned __int64 UINT_PTR, *PUINT_PTR;

typedef __int64 LONG_PTR, *PLONG_PTR;
typedef unsigned __int64 ULONG_PTR, *PULONG_PTR;
#else
typedef __w64 int INT_PTR, *PINT_PTR;
typedef __w64 unsigned int UINT_PTR, *PUINT_PTR;

typedef __w64 long LONG_PTR, *PLONG_PTR;
typedef __w64 unsigned long ULONG_PTR, *PULONG_PTR;
#endif

typedef UINT_PTR WPARAM;
typedef LONG_PTR LPARAM;
typedef LONG_PTR LRESULT;
#define LRESULT LONG_PTR
typedef _Return_type_success_(return >= 0) long HRESULT;

typedef ULONG_PTR SIZE_T, *PSIZE_T;
typedef LONG_PTR SSIZE_T, *PSSIZE_T;

typedef DWORD ACCESS_MASK;
typedef ACCESS_MASK REGSAM;

typedef LONG NTSTATUS;

// As defined in guiddef.h.
#ifndef _REFGUID_DEFINED
#define _REFGUID_DEFINED
#define REFGUID const GUID&
#endif

// Forward declare Windows compatible handles.

#define CHROME_DECLARE_HANDLE(name) \
  struct name##__;                  \
  typedef struct name##__* name
CHROME_DECLARE_HANDLE(HDESK);
CHROME_DECLARE_HANDLE(HGLRC);
CHROME_DECLARE_HANDLE(HICON);
CHROME_DECLARE_HANDLE(HINSTANCE);
CHROME_DECLARE_HANDLE(HKEY);
CHROME_DECLARE_HANDLE(HKL);
CHROME_DECLARE_HANDLE(HMENU);
CHROME_DECLARE_HANDLE(HWINSTA);
CHROME_DECLARE_HANDLE(HWND);
#undef CHROME_DECLARE_HANDLE

typedef LPVOID HINTERNET;
typedef HINSTANCE HMODULE;
typedef PVOID LSA_HANDLE;
typedef PVOID HDEVINFO;

// Forward declare some Windows struct/typedef sets.

typedef struct _OVERLAPPED OVERLAPPED;
typedef struct tagMSG MSG, *PMSG, *NPMSG, *LPMSG;

typedef struct _RTL_SRWLOCK RTL_SRWLOCK;
typedef RTL_SRWLOCK SRWLOCK, *PSRWLOCK;

typedef struct _GUID GUID;
typedef GUID CLSID;

typedef struct tagLOGFONTW LOGFONTW, *PLOGFONTW, *NPLOGFONTW, *LPLOGFONTW;
typedef LOGFONTW LOGFONT;

typedef struct _FILETIME FILETIME;

typedef struct tagMENUITEMINFOW MENUITEMINFOW, MENUITEMINFO;

typedef struct tagNMHDR NMHDR;

typedef struct _SP_DEVINFO_DATA SP_DEVINFO_DATA;

typedef PVOID PSID;

// Declare Chrome versions of some Windows structures. These are needed for
// when we need a concrete type but don't want to pull in Windows.h. We can't
// declare the Windows types so we declare our types and cast to the Windows
// types in a few places.

struct CHROME_SRWLOCK {
  PVOID Ptr;
};

struct CHROME_CONDITION_VARIABLE {
  PVOID Ptr;
};

// Define some commonly used Windows constants. Note that the layout of these
// macros - including internal spacing - must be 100% consistent with windows.h.

// clang-format off

#ifndef INVALID_HANDLE_VALUE
// Work around there being two slightly different definitions in the SDK.
#define INVALID_HANDLE_VALUE ((HANDLE)(LONG_PTR)-1)
#endif
#define TLS_OUT_OF_INDEXES ((DWORD)0xFFFFFFFF)
#define HTNOWHERE 0
#define MAX_PATH 260
#define CS_GLOBALCLASS 0x4000

#define ERROR_SUCCESS 0L
#define ERROR_FILE_NOT_FOUND 2L
#define ERROR_ACCESS_DENIED 5L
#define ERROR_INVALID_HANDLE 6L
#define ERROR_SHARING_VIOLATION 32L
#define ERROR_LOCK_VIOLATION 33L
#define REG_BINARY ( 3ul )

#define STATUS_PENDING ((DWORD   )0x00000103L)
#define STILL_ACTIVE STATUS_PENDING
#define SUCCEEDED(hr) (((HRESULT)(hr)) >= 0)
#define FAILED(hr) (((HRESULT)(hr)) < 0)

#define HKEY_CLASSES_ROOT (( HKEY ) (ULONG_PTR)((LONG)0x80000000) )
#define HKEY_LOCAL_MACHINE (( HKEY ) (ULONG_PTR)((LONG)0x80000002) )
#define HKEY_CURRENT_USER (( HKEY ) (ULONG_PTR)((LONG)0x80000001) )
#define KEY_QUERY_VALUE (0x0001)
#define KEY_SET_VALUE (0x0002)
#define KEY_CREATE_SUB_KEY (0x0004)
#define KEY_ENUMERATE_SUB_KEYS (0x0008)
#define KEY_NOTIFY (0x0010)
#define KEY_CREATE_LINK (0x0020)
#define KEY_WOW64_32KEY (0x0200)
#define KEY_WOW64_64KEY (0x0100)
#define KEY_WOW64_RES (0x0300)

#define READ_CONTROL (0x00020000L)
#define SYNCHRONIZE (0x00100000L)

#define STANDARD_RIGHTS_READ (READ_CONTROL)
#define STANDARD_RIGHTS_WRITE (READ_CONTROL)
#define STANDARD_RIGHTS_ALL (0x001F0000L)

#define KEY_READ                ((STANDARD_RIGHTS_READ       |\
                                  KEY_QUERY_VALUE            |\
                                  KEY_ENUMERATE_SUB_KEYS     |\
                                  KEY_NOTIFY)                 \
                                  &                           \
                                 (~SYNCHRONIZE))


#define KEY_WRITE               ((STANDARD_RIGHTS_WRITE      |\
                                  KEY_SET_VALUE              |\
                                  KEY_CREATE_SUB_KEY)         \
                                  &                           \
                                 (~SYNCHRONIZE))

#define KEY_ALL_ACCESS          ((STANDARD_RIGHTS_ALL        |\
                                  KEY_QUERY_VALUE            |\
                                  KEY_SET_VALUE              |\
                                  KEY_CREATE_SUB_KEY         |\
                                  KEY_ENUMERATE_SUB_KEYS     |\
                                  KEY_NOTIFY                 |\
                                  KEY_CREATE_LINK)            \
                                  &                           \
                                 (~SYNCHRONIZE))

// clang-format on

// Define some macros needed when prototyping Windows functions.

#define DECLSPEC_IMPORT __declspec(dllimport)
#define WINBASEAPI DECLSPEC_IMPORT
#define WINUSERAPI DECLSPEC_IMPORT
#define WINAPI __stdcall
#define CALLBACK __stdcall

// Needed for LockImpl.
WINBASEAPI _Releases_exclusive_lock_(*SRWLock) VOID WINAPI
    ReleaseSRWLockExclusive(_Inout_ PSRWLOCK SRWLock);
WINBASEAPI BOOLEAN WINAPI TryAcquireSRWLockExclusive(_Inout_ PSRWLOCK SRWLock);

// Needed to support protobuf's GetMessage macro magic.
WINUSERAPI BOOL WINAPI GetMessageW(_Out_ LPMSG lpMsg,
                                   _In_opt_ HWND hWnd,
                                   _In_ UINT wMsgFilterMin,
                                   _In_ UINT wMsgFilterMax);

// Needed for thread_local_storage.h
WINBASEAPI LPVOID WINAPI TlsGetValue(_In_ DWORD dwTlsIndex);

// Needed for scoped_handle.h
WINBASEAPI _Check_return_ _Post_equals_last_error_ DWORD WINAPI
    GetLastError(VOID);

WINBASEAPI VOID WINAPI SetLastError(_In_ DWORD dwErrCode);

#ifdef __cplusplus
}
#endif

// These macros are all defined by windows.h and are also used as the names of
// functions in the Chromium code base. Add to this list as needed whenever
// there is a Windows macro which causes a function call to be renamed. This
// ensures that the same renaming will happen everywhere. Includes of this file
// can be added wherever needed to ensure this consistent renaming.

#define CopyFile CopyFileW
#define CreateDirectory CreateDirectoryW
#define CreateEvent CreateEventW
#define CreateFile CreateFileW
#define CreateService CreateServiceW
#define DeleteFile DeleteFileW
#define DispatchMessage DispatchMessageW
#define DrawText DrawTextW
#define FindFirstFile FindFirstFileW
#define FindNextFile FindNextFileW
#define GetComputerName GetComputerNameW
#define GetCurrentDirectory GetCurrentDirectoryW
#define GetCurrentTime() GetTickCount()
#define GetFileAttributes GetFileAttributesW
#define GetMessage GetMessageW
#define GetUserName GetUserNameW
#define LoadIcon LoadIconW
#define LoadImage LoadImageW
#define PostMessage PostMessageW
#define RemoveDirectory RemoveDirectoryW
#define ReplaceFile ReplaceFileW
#define ReportEvent ReportEventW
#define SendMessage SendMessageW
#define SendMessageCallback SendMessageCallbackW
#define SetCurrentDirectory SetCurrentDirectoryW
#define StartService StartServiceW
#define UpdateResource UpdateResourceW

#endif  // BASE_WIN_WINDOWS_TYPES_H_
