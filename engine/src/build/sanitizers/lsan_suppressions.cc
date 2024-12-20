// Copyright 2015 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file contains the default suppressions for LeakSanitizer.
// You can also pass additional suppressions via LSAN_OPTIONS:
// LSAN_OPTIONS=suppressions=/path/to/suppressions. Please refer to
// http://dev.chromium.org/developers/testing/leaksanitizer for more info.

#if defined(LEAK_SANITIZER)

// Please make sure the code below declares a single string variable
// kLSanDefaultSuppressions which contains LSan suppressions delimited by
// newlines. See http://dev.chromium.org/developers/testing/leaksanitizer
// for the instructions on writing suppressions.
char kLSanDefaultSuppressions[] =
// Intentional leak used as sanity test for Valgrind/memcheck.
"leak:base::ToolsSanityTest_MemoryLeak_Test::TestBody\n"

// ================ Leaks in third-party code ================

// False positives in libfontconfig. http://crbug.com/39050
"leak:libfontconfig\n"

// Leaks in Nvidia's libGL.
"leak:libGL.so\n"

// A small leak in V8. http://crbug.com/46571#c9
"leak:blink::V8GCController::collectGarbage\n"

// TODO(earthdok): revisit NSS suppressions after the switch to BoringSSL
// NSS leaks in CertDatabaseNSSTest tests. http://crbug.com/51988
"leak:net::NSSCertDatabase::ImportFromPKCS12\n"
"leak:net::NSSCertDatabase::ListCerts\n"
"leak:net::NSSCertDatabase::DeleteCertAndKey\n"
"leak:crypto::ScopedTestNSSDB::ScopedTestNSSDB\n"
// Another leak due to not shutting down NSS properly. http://crbug.com/124445
"leak:error_get_my_stack\n"
// The NSS suppressions above will not fire when the fast stack unwinder is
// used, because it can't unwind through NSS libraries. Apply blanket
// suppressions for now.
"leak:libnssutil3\n"
"leak:libnspr4\n"
"leak:libnss3\n"
"leak:libplds4\n"
"leak:libnssckbi\n"

// XRandR has several one time leaks.
"leak:libxrandr\n"

// xrandr leak. http://crbug.com/119677
"leak:XRRFindDisplay\n"

// Suppressions for objects which can be owned by the V8 heap. This is a
// temporary workaround until LeakSanitizer supports the V8 heap.
// Those should only fire in (browser)tests. If you see one of them in Chrome,
// then it's a real leak.
// http://crbug.com/328552
"leak:WTF::StringImpl::createUninitialized\n"
"leak:WTF::StringImpl::create8BitIfPossible\n"
"leak:blink::MouseEvent::create\n"
"leak:blink::*::*GetterCallback\n"
"leak:blink::CSSComputedStyleDeclaration::create\n"
"leak:blink::V8PerIsolateData::ensureDomInJSContext\n"
"leak:gin/object_template_builder.h\n"
"leak:gin::internal::Dispatcher\n"
"leak:blink::LocalDOMWindow::getComputedStyle\n"
// This should really be RemoteDOMWindow::create, but symbolization is
// weird in release builds. https://crbug.com/484760
"leak:blink::RemoteFrame::create\n"
// Likewise, this should really be blink::WindowProxy::initializeIfNeeded.
// https://crbug.com/484760
"leak:blink::WindowProxy::createContext\n"

// http://crbug.com/356785
"leak:content::RenderViewImplTest_DecideNavigationPolicyForWebUI_Test::TestBody\n"

// ================ Leaks in Chromium code ================
// PLEASE DO NOT ADD SUPPRESSIONS FOR NEW LEAKS.
// Instead, commits that introduce memory leaks should be reverted. Suppressing
// the leak is acceptable in some cases when reverting is impossible, i.e. when
// enabling leak detection for the first time for a test target with
// pre-existing leaks.

// Small test-only leak in ppapi_unittests. http://crbug.com/258113
"leak:ppapi::proxy::PPP_Instance_Private_ProxyTest_PPPInstancePrivate_Test\n"

// http://crbug.com/322671
"leak:content::SpeechRecognitionBrowserTest::SetUpOnMainThread\n"

// http://crbug.com/355641
"leak:TrayAccessibilityTest\n"

// http://crbug.com/354644
"leak:CertificateViewerUITest::ShowModalCertificateViewer\n"

// http://crbug.com/356306
"leak:content::SetProcessTitleFromCommandLine\n"

// http://crbug.com/506433
"leak:blink::ResourceFetcher::garbageCollectDocumentResources\n"

// PLEASE READ ABOVE BEFORE ADDING NEW SUPPRESSIONS.

// End of suppressions.
;  // Please keep this semicolon.

#endif  // LEAK_SANITIZER
