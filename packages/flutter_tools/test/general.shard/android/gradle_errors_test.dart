// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:file_testing/file_testing.dart';
import 'package:flutter_tools/src/android/gradle_errors.dart';
import 'package:flutter_tools/src/android/gradle_utils.dart';
import 'package:flutter_tools/src/android/java.dart';
import 'package:flutter_tools/src/base/bot_detector.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

void main() {
  late FileSystem fileSystem;
  late FakeProcessManager processManager;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    processManager = FakeProcessManager.empty();
  });

  group('gradleErrors', () {
    testWithoutContext('list of errors', () {
      // If you added a new Gradle error, please update this test.
      expect(
        gradleErrors,
        equals(<GradleHandledError>[
          licenseNotAcceptedHandler,
          networkErrorHandler,
          permissionDeniedErrorHandler,
          flavorUndefinedHandler,
          r8DexingBugInAgp73Handler,
          minSdkVersionHandler,
          transformInputIssueHandler,
          lockFileDepMissingHandler,
          minCompileSdkVersionHandler,
          incompatibleJavaAndAgpVersionsHandler,
          outdatedGradleHandler,
          sslExceptionHandler,
          zipExceptionHandler,
          incompatibleJavaAndGradleVersionsHandler,
          remoteTerminatedHandshakeHandler,
          couldNotOpenCacheDirectoryHandler,
          incompatibleCompileSdk35AndAgpVersionHandler,
          jlinkErrorWithJava21AndSourceCompatibility,
          incompatibleKotlinVersionHandler,
        ]),
      );
    });
  });

  group('network errors', () {
    testUsingContext(
      'retries if gradle fails while downloading',
      () async {
        const String errorMessage = r'''
Exception in thread "main" java.io.FileNotFoundException: https://downloads.gradle.org/distributions/gradle-4.1.1-all.zip
at sun.net.www.protocol.http.HttpURLConnection.getInputStream0(HttpURLConnection.java:1872)
at sun.net.www.protocol.http.HttpURLConnection.getInputStream(HttpURLConnection.java:1474)
at sun.net.www.protocol.https.HttpsURLConnectionImpl.getInputStream(HttpsURLConnectionImpl.java:254)
at org.gradle.wrapper.Download.downloadInternal(Download.java:58)
at org.gradle.wrapper.Download.download(Download.java:44)
at org.gradle.wrapper.Install$1.call(Install.java:61)
at org.gradle.wrapper.Install$1.call(Install.java:48)
at org.gradle.wrapper.ExclusiveFileAccessManager.access(ExclusiveFileAccessManager.java:65)
at org.gradle.wrapper.Install.createDist(Install.java:48)
at org.gradle.wrapper.WrapperExecutor.execute(WrapperExecutor.java:128)
at org.gradle.wrapper.GradleWrapperMain.main(GradleWrapperMain.java:61)''';

        expect(formatTestErrorMessage(errorMessage, networkErrorHandler), isTrue);
        expect(
          await networkErrorHandler.handler(
            line: '',
            project: FakeFlutterProject(),
            usesAndroidX: true,
          ),
          equals(GradleBuildStatus.retry),
        );

        expect(
          testLogger.errorText,
          contains('Gradle threw an error while downloading artifacts from the network.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );

    testUsingContext('retries if remote host terminated ssl handshake', () async {
      const String errorMessage = r'''
Exception in thread "main" javax.net.ssl.SSLHandshakeException: Remote host terminated the handshake
	at java.base/sun.security.ssl.SSLSocketImpl.handleEOF(SSLSocketImpl.java:1696)
	at java.base/sun.security.ssl.SSLSocketImpl.decode(SSLSocketImpl.java:1514)
	at java.base/sun.security.ssl.SSLSocketImpl.readHandshakeRecord(SSLSocketImpl.java:1416)
	at java.base/sun.security.ssl.SSLSocketImpl.startHandshake(SSLSocketImpl.java:456)
	at java.base/sun.security.ssl.SSLSocketImpl.startHandshake(SSLSocketImpl.java:427)
	at java.base/sun.net.www.protocol.https.HttpsClient.afterConnect(HttpsClient.java:572)
	at java.base/sun.net.www.protocol.https.AbstractDelegateHttpsURLConnection.connect(AbstractDelegateHttpsURLConnection.java:197)
	at java.base/sun.net.www.protocol.http.HttpURLConnection.followRedirect0(HttpURLConnection.java:2783)
	at java.base/sun.net.www.protocol.http.HttpURLConnection.followRedirect(HttpURLConnection.java:2695)
	at java.base/sun.net.www.protocol.http.HttpURLConnection.getInputStream0(HttpURLConnection.java:1854)
	at java.base/sun.net.www.protocol.http.HttpURLConnection.getInputStream(HttpURLConnection.java:1520)
	at java.base/sun.net.www.protocol.https.HttpsURLConnectionImpl.getInputStream(HttpsURLConnectionImpl.java:250)
	at org.gradle.wrapper.Download.downloadInternal(Download.java:58)
	at org.gradle.wrapper.Download.download(Download.java:44)
	at org.gradle.wrapper.Install$1.call(Install.java:61)
	at org.gradle.wrapper.Install$1.call(Install.java:48)
	at org.gradle.wrapper.ExclusiveFileAccessManager.access(ExclusiveFileAccessManager.java:65)
	at org.gradle.wrapper.Install.createDist(Install.java:48)
	at org.gradle.wrapper.WrapperExecutor.execute(WrapperExecutor.java:128)
	at org.gradle.wrapper.GradleWrapperMain.main(GradleWrapperMain.java:61)
Caused by: java.io.EOFException: SSL peer shut down incorrectly
	at java.base/sun.security.ssl.SSLSocketInputRecord.read(SSLSocketInputRecord.java:483)
	at java.base/sun.security.ssl.SSLSocketInputRecord.readHeader(SSLSocketInputRecord.java:472)
	at java.base/sun.security.ssl.SSLSocketInputRecord.decode(SSLSocketInputRecord.java:160)
	at java.base/sun.security.ssl.SSLTransport.decode(SSLTransport.java:111)
	at java.base/sun.security.ssl.SSLSocketImpl.decode(SSLSocketImpl.java:1506)''';

      expect(formatTestErrorMessage(errorMessage, remoteTerminatedHandshakeHandler), isTrue);
      expect(
        await remoteTerminatedHandshakeHandler.handler(
          line: '',
          project: FakeFlutterProject(),
          usesAndroidX: true,
        ),
        equals(GradleBuildStatus.retry),
      );

      expect(
        testLogger.errorText,
        contains('Gradle threw an error while downloading artifacts from the network.'),
      );
    });

    testUsingContext(
      'retries if gradle fails downloading with proxy error',
      () async {
        const String errorMessage = r'''
Exception in thread "main" java.io.IOException: Unable to tunnel through proxy. Proxy returns "HTTP/1.1 400 Bad Request"
at sun.net.www.protocol.http.HttpURLConnection.doTunneling(HttpURLConnection.java:2124)
at sun.net.www.protocol.https.AbstractDelegateHttpsURLConnection.connect(AbstractDelegateHttpsURLConnection.java:183)
at sun.net.www.protocol.http.HttpURLConnection.getInputStream0(HttpURLConnection.java:1546)
at sun.net.www.protocol.http.HttpURLConnection.getInputStream(HttpURLConnection.java:1474)
at sun.net.www.protocol.https.HttpsURLConnectionImpl.getInputStream(HttpsURLConnectionImpl.java:254)
at org.gradle.wrapper.Download.downloadInternal(Download.java:58)
at org.gradle.wrapper.Download.download(Download.java:44)
at org.gradle.wrapper.Install$1.call(Install.java:61)
at org.gradle.wrapper.Install$1.call(Install.java:48)
at org.gradle.wrapper.ExclusiveFileAccessManager.access(ExclusiveFileAccessManager.java:65)
at org.gradle.wrapper.Install.createDist(Install.java:48)
at org.gradle.wrapper.WrapperExecutor.execute(WrapperExecutor.java:128)
at org.gradle.wrapper.GradleWrapperMain.main(GradleWrapperMain.java:61)''';

        expect(formatTestErrorMessage(errorMessage, networkErrorHandler), isTrue);
        expect(
          await networkErrorHandler.handler(
            line: '',
            project: FakeFlutterProject(),
            usesAndroidX: true,
          ),
          equals(GradleBuildStatus.retry),
        );

        expect(
          testLogger.errorText,
          contains('Gradle threw an error while downloading artifacts from the network.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );

    testUsingContext(
      'retries if gradle fails downloading with bad gateway error',
      () async {
        const String errorMessage = r'''
Exception in thread "main" java.io.IOException: Server returned HTTP response code: 502 for URL: https://objects.githubusercontent.com/github-production-release-asset-2e65be/696192900/1e77bbfb-4cde-4376-92ea-fc4ff57b8362?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=FFFF%2F20231220%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20231220T160553Z&X-Amz-Expires=300&X-Amz-Signature=ffff&X-Amz-SignedHeaders=host&actor_id=0&key_id=0&repo_id=696192900&response-content-disposition=attachment%3B%20filename%3Dgradle-8.2.1-all.zip&response-content-type=application%2Foctet-stream
at java.base/sun.net.www.protocol.http.HttpURLConnection.getInputStream0(HttpURLConnection.java:1997)
at java.base/sun.net.www.protocol.http.HttpURLConnection.getInputStream(HttpURLConnection.java:1589)
at java.base/sun.net.www.protocol.https.HttpsURLConnectionImpl.getInputStream(HttpsURLConnectionImpl.java:224)
at org.gradle.wrapper.Download.downloadInternal(Download.java:58)
at org.gradle.wrapper.Download.download(Download.java:44)
at org.gradle.wrapper.Install$1.call(Install.java:61)
at org.gradle.wrapper.Install$1.call(Install.java:48)
at org.gradle.wrapper.ExclusiveFileAccessManager.access(ExclusiveFileAccessManager.java:65)
at org.gradle.wrapper.Install.createDist(Install.java:48)
at org.gradle.wrapper.WrapperExecutor.execute(WrapperExecutor.java:128)
at org.gradle.wrapper.GradleWrapperMain.main(GradleWrapperMain.java:61)''';

        expect(formatTestErrorMessage(errorMessage, networkErrorHandler), isTrue);
        expect(
          await networkErrorHandler.handler(
            line: '',
            project: FakeFlutterProject(),
            usesAndroidX: true,
          ),
          equals(GradleBuildStatus.retry),
        );

        expect(
          testLogger.errorText,
          contains('Gradle threw an error while downloading artifacts from the network.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );

    testUsingContext(
      'retries if gradle times out waiting for exclusive access to zip',
      () async {
        const String errorMessage = '''
Exception in thread "main" java.lang.RuntimeException: Timeout of 120000 reached waiting for exclusive access to file: /User/documents/gradle-5.6.2-all.zip
	at org.gradle.wrapper.ExclusiveFileAccessManager.access(ExclusiveFileAccessManager.java:61)
	at org.gradle.wrapper.Install.createDist(Install.java:48)
	at org.gradle.wrapper.WrapperExecutor.execute(WrapperExecutor.java:128)
	at org.gradle.wrapper.GradleWrapperMain.main(GradleWrapperMain.java:61)''';

        expect(formatTestErrorMessage(errorMessage, networkErrorHandler), isTrue);
        expect(
          await networkErrorHandler.handler(
            line: '',
            project: FakeFlutterProject(),
            usesAndroidX: true,
          ),
          equals(GradleBuildStatus.retry),
        );

        expect(
          testLogger.errorText,
          contains('Gradle threw an error while downloading artifacts from the network.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );

    testUsingContext(
      'retries if remote host closes connection',
      () async {
        const String errorMessage = r'''
Downloading https://services.gradle.org/distributions/gradle-5.6.2-all.zip
Exception in thread "main" javax.net.ssl.SSLHandshakeException: Remote host closed connection during handshake
	at sun.security.ssl.SSLSocketImpl.readRecord(SSLSocketImpl.java:994)
	at sun.security.ssl.SSLSocketImpl.performInitialHandshake(SSLSocketImpl.java:1367)
	at sun.security.ssl.SSLSocketImpl.startHandshake(SSLSocketImpl.java:1395)
	at sun.security.ssl.SSLSocketImpl.startHandshake(SSLSocketImpl.java:1379)
	at sun.net.www.protocol.https.HttpsClient.afterConnect(HttpsClient.java:559)
	at sun.net.www.protocol.https.AbstractDelegateHttpsURLConnection.connect(AbstractDelegateHttpsURLConnection.java:185)
	at sun.net.www.protocol.http.HttpURLConnection.followRedirect0(HttpURLConnection.java:2729)
	at sun.net.www.protocol.http.HttpURLConnection.followRedirect(HttpURLConnection.java:2641)
	at sun.net.www.protocol.http.HttpURLConnection.getInputStream0(HttpURLConnection.java:1824)
	at sun.net.www.protocol.http.HttpURLConnection.getInputStream(HttpURLConnection.java:1492)
	at sun.net.www.protocol.https.HttpsURLConnectionImpl.getInputStream(HttpsURLConnectionImpl.java:263)
	at org.gradle.wrapper.Download.downloadInternal(Download.java:58)
	at org.gradle.wrapper.Download.download(Download.java:44)
	at org.gradle.wrapper.Install$1.call(Install.java:61)
	at org.gradle.wrapper.Install$1.call(Install.java:48)
	at org.gradle.wrapper.ExclusiveFileAccessManager.access(ExclusiveFileAccessManager.java:65)
	at org.gradle.wrapper.Install.createDist(Install.java:48)
	at org.gradle.wrapper.WrapperExecutor.execute(WrapperExecutor.java:128)
	at org.gradle.wrapper.GradleWrapperMain.main(GradleWrapperMain.java:61)''';

        expect(formatTestErrorMessage(errorMessage, networkErrorHandler), isTrue);
        expect(
          await networkErrorHandler.handler(
            line: '',
            project: FakeFlutterProject(),
            usesAndroidX: true,
          ),
          equals(GradleBuildStatus.retry),
        );

        expect(
          testLogger.errorText,
          contains('Gradle threw an error while downloading artifacts from the network.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );

    testUsingContext(
      'retries if file opening fails',
      () async {
        const String errorMessage = r'''
Downloading https://services.gradle.org/distributions/gradle-3.5.0-all.zip
Exception in thread "main" java.io.FileNotFoundException: https://downloads.gradle-dn.com/distributions/gradle-3.5.0-all.zip
	at sun.net.www.protocol.http.HttpURLConnection.getInputStream0(HttpURLConnection.java:1890)
	at sun.net.www.protocol.http.HttpURLConnection.getInputStream(HttpURLConnection.java:1492)
	at sun.net.www.protocol.https.HttpsURLConnectionImpl.getInputStream(HttpsURLConnectionImpl.java:263)
	at org.gradle.wrapper.Download.downloadInternal(Download.java:58)
	at org.gradle.wrapper.Download.download(Download.java:44)
	at org.gradle.wrapper.Install$1.call(Install.java:61)
	at org.gradle.wrapper.Install$1.call(Install.java:48)
	at org.gradle.wrapper.ExclusiveFileAccessManager.access(ExclusiveFileAccessManager.java:65)
	at org.gradle.wrapper.Install.createDist(Install.java:48)
	at org.gradle.wrapper.WrapperExecutor.execute(WrapperExecutor.java:128)
	at org.gradle.wrapper.GradleWrapperMain.main(GradleWrapperMain.java:61)''';

        expect(formatTestErrorMessage(errorMessage, networkErrorHandler), isTrue);
        expect(
          await networkErrorHandler.handler(
            line: '',
            project: FakeFlutterProject(),
            usesAndroidX: true,
          ),
          equals(GradleBuildStatus.retry),
        );

        expect(
          testLogger.errorText,
          contains('Gradle threw an error while downloading artifacts from the network.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );

    testUsingContext(
      'retries if the connection is reset',
      () async {
        const String errorMessage = r'''
Downloading https://services.gradle.org/distributions/gradle-5.6.2-all.zip
Exception in thread "main" java.net.SocketException: Connection reset
	at java.net.SocketInputStream.read(SocketInputStream.java:210)
	at java.net.SocketInputStream.read(SocketInputStream.java:141)
	at sun.security.ssl.InputRecord.readFully(InputRecord.java:465)
	at sun.security.ssl.InputRecord.readV3Record(InputRecord.java:593)
	at sun.security.ssl.InputRecord.read(InputRecord.java:532)
	at sun.security.ssl.SSLSocketImpl.readRecord(SSLSocketImpl.java:975)
	at sun.security.ssl.SSLSocketImpl.performInitialHandshake(SSLSocketImpl.java:1367)
	at sun.security.ssl.SSLSocketImpl.startHandshake(SSLSocketImpl.java:1395)
	at sun.security.ssl.SSLSocketImpl.startHandshake(SSLSocketImpl.java:1379)
	at sun.net.www.protocol.https.HttpsClient.afterConnect(HttpsClient.java:559)
	at sun.net.www.protocol.https.AbstractDelegateHttpsURLConnection.connect(AbstractDelegateHttpsURLConnection.java:185)
	at sun.net.www.protocol.http.HttpURLConnection.getInputStream0(HttpURLConnection.java:1564)
	at sun.net.www.protocol.http.HttpURLConnection.getInputStream(HttpURLConnection.java:1492)
	at sun.net.www.protocol.https.HttpsURLConnectionImpl.getInputStream(HttpsURLConnectionImpl.java:263)
	at org.gradle.wrapper.Download.downloadInternal(Download.java:58)
	at org.gradle.wrapper.Download.download(Download.java:44)
	at org.gradle.wrapper.Install$1.call(Install.java:61)
	at org.gradle.wrapper.Install$1.call(Install.java:48)
	at org.gradle.wrapper.ExclusiveFileAccessManager.access(ExclusiveFileAccessManager.java:65)
	at org.gradle.wrapper.Install.createDist(Install.java:48)
	at org.gradle.wrapper.WrapperExecutor.execute(WrapperExecutor.java:128)
	at org.gradle.wrapper.GradleWrapperMain.main(GradleWrapperMain.java:61)''';

        expect(formatTestErrorMessage(errorMessage, networkErrorHandler), isTrue);
        expect(
          await networkErrorHandler.handler(
            line: '',
            project: FakeFlutterProject(),
            usesAndroidX: true,
          ),
          equals(GradleBuildStatus.retry),
        );

        expect(
          testLogger.errorText,
          contains('Gradle threw an error while downloading artifacts from the network.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );

    testUsingContext(
      'retries if Gradle could not get a resource',
      () async {
        const String errorMessage = '''
A problem occurred configuring root project 'android'.
> Could not resolve all artifacts for configuration ':classpath'.
   > Could not resolve net.sf.proguard:proguard-gradle:6.0.3.
     Required by:
         project : > com.android.tools.build:gradle:3.3.0
      > Could not resolve net.sf.proguard:proguard-gradle:6.0.3.
         > Could not parse POM https://jcenter.bintray.com/net/sf/proguard/proguard-gradle/6.0.3/proguard-gradle-6.0.3.pom
            > Could not resolve net.sf.proguard:proguard-parent:6.0.3.
               > Could not resolve net.sf.proguard:proguard-parent:6.0.3.
                  > Could not get resource 'https://jcenter.bintray.com/net/sf/proguard/proguard-parent/6.0.3/proguard-parent-6.0.3.pom'.
                     > Could not GET 'https://jcenter.bintray.com/net/sf/proguard/proguard-parent/6.0.3/proguard-parent-6.0.3.pom'. Received status code 504 from server: Gateway Time-out''';

        expect(formatTestErrorMessage(errorMessage, networkErrorHandler), isTrue);
        expect(
          await networkErrorHandler.handler(
            line: '',
            project: FakeFlutterProject(),
            usesAndroidX: true,
          ),
          equals(GradleBuildStatus.retry),
        );

        expect(
          testLogger.errorText,
          contains('Gradle threw an error while downloading artifacts from the network.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );

    testUsingContext(
      'retries if Gradle could not get a resource (non-Gateway)',
      () async {
        const String errorMessage = '''
* Error running Gradle:
Exit code 1 from: /home/travis/build/flutter/flutter sdk/examples/flutter_gallery/android/gradlew app:properties:
Starting a Gradle Daemon (subsequent builds will be faster)
Picked up _JAVA_OPTIONS: -Xmx2048m -Xms512m
FAILURE: Build failed with an exception.
* What went wrong:
A problem occurred configuring root project 'android'.
> Could not resolve all files for configuration ':classpath'.
   > Could not resolve com.android.tools.build:gradle:3.1.2.
     Required by:
         project :
      > Could not resolve com.android.tools.build:gradle:3.1.2.
         > Could not get resource 'https://dl.google.com/dl/android/maven2/com/android/tools/build/gradle/3.1.2/gradle-3.1.2.pom'.
            > Could not GET 'https://dl.google.com/dl/android/maven2/com/android/tools/build/gradle/3.1.2/gradle-3.1.2.pom'.
               > Remote host closed connection during handshake''';

        expect(formatTestErrorMessage(errorMessage, networkErrorHandler), isTrue);
        expect(
          await networkErrorHandler.handler(
            line: '',
            project: FakeFlutterProject(),
            usesAndroidX: true,
          ),
          equals(GradleBuildStatus.retry),
        );

        expect(
          testLogger.errorText,
          contains('Gradle threw an error while downloading artifacts from the network.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );

    testUsingContext(
      'retries if connection times out',
      () async {
        const String errorMessage = r'''
Exception in thread "main" java.net.ConnectException: Connection timed out
java.base/sun.nio.ch.Net.connect0(Native Method)
  at java.base/sun.nio.ch.Net.connect(Net.java:579)
  at java.base/sun.nio.ch.Net.connect(Net.java:568)
  at java.base/sun.nio.ch.NioSocketImpl.connect(NioSocketImpl.java:588)
  at java.base/java.net.SocksSocketImpl.connect(SocksSocketImpl.java:327)
  at java.base/java.net.Socket.connect(Socket.java:633)
  at java.base/sun.security.ssl.SSLSocketImpl.connect(SSLSocketImpl.java:299)
  at java.base/sun.security.ssl.BaseSSLSocketImpl.connect(BaseSSLSocketImpl.java:174)
  at java.base/sun.net.NetworkClient.doConnect(NetworkClient.java:183)
  at java.base/sun.net.www.http.HttpClient.openServer(HttpClient.java:498)
  at java.base/sun.net.www.http.HttpClient.openServer(HttpClient.java:603)
  at java.base/sun.net.www.protocol.https.HttpsClient.<init>(HttpsClient.java:266)
  at java.base/sun.net.www.protocol.https.HttpsClient.New(HttpsClient.java:380)''';

        expect(formatTestErrorMessage(errorMessage, networkErrorHandler), isTrue);
        expect(
          await networkErrorHandler.handler(
            line: '',
            project: FakeFlutterProject(),
            usesAndroidX: true,
          ),
          equals(GradleBuildStatus.retry),
        );

        expect(
          testLogger.errorText,
          contains('Gradle threw an error while downloading artifacts from the network.'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );
  });

  group('permission errors', () {
    testUsingContext('throws toolExit if gradle is missing execute permissions', () async {
      const String errorMessage = '''
Permission denied
Command: /home/android/gradlew assembleRelease
''';
      expect(formatTestErrorMessage(errorMessage, permissionDeniedErrorHandler), isTrue);
      expect(
        await permissionDeniedErrorHandler.handler(
          usesAndroidX: true,
          line: '',
          project: FakeFlutterProject(),
        ),
        equals(GradleBuildStatus.exit),
      );

      expect(testLogger.statusText, contains('Gradle does not have execution permission.'));
      expect(
        testLogger.statusText,
        contains(
          '\n'
          '┌─ Flutter Fix ───────────────────────────────────────────────────────────────────────────────────┐\n'
          '│ [!] Gradle does not have execution permission.                                                  │\n'
          '│ You should change the ownership of the project directory to your user, or move the project to a │\n'
          '│ directory with execute permissions.                                                             │\n'
          '└─────────────────────────────────────────────────────────────────────────────────────────────────┘\n',
        ),
      );
    });

    testUsingContext('pattern', () async {
      const String errorMessage = '''
Permission denied
Command: /home/android/gradlew assembleRelease
''';
      expect(formatTestErrorMessage(errorMessage, permissionDeniedErrorHandler), isTrue);
    });

    testUsingContext('handler', () async {
      expect(
        await permissionDeniedErrorHandler.handler(
          usesAndroidX: true,
          line: '',
          project: FakeFlutterProject(),
        ),
        equals(GradleBuildStatus.exit),
      );

      expect(testLogger.statusText, contains('Gradle does not have execution permission.'));
      expect(
        testLogger.statusText,
        contains(
          '\n'
          '┌─ Flutter Fix ───────────────────────────────────────────────────────────────────────────────────┐\n'
          '│ [!] Gradle does not have execution permission.                                                  │\n'
          '│ You should change the ownership of the project directory to your user, or move the project to a │\n'
          '│ directory with execute permissions.                                                             │\n'
          '└─────────────────────────────────────────────────────────────────────────────────────────────────┘\n',
        ),
      );
    });
  });

  group('license not accepted', () {
    testWithoutContext('pattern', () {
      expect(
        licenseNotAcceptedHandler.test(
          'You have not accepted the license agreements of the following SDK components',
        ),
        isTrue,
      );
    });

    testUsingContext('handler', () async {
      await licenseNotAcceptedHandler.handler(
        line:
            'You have not accepted the license agreements of the following SDK components: [foo, bar]',
        project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        usesAndroidX: true,
      );

      expect(
        testLogger.statusText,
        contains(
          '\n'
          '┌─ Flutter Fix ─────────────────────────────────────────────────────────────────────────────────┐\n'
          '│ [!] Unable to download needed Android SDK components, as the following licenses have not been │\n'
          '│ accepted: foo, bar                                                                            │\n'
          '│                                                                                               │\n'
          '│ To resolve this, please run the following command in a Terminal:                              │\n'
          '│ flutter doctor --android-licenses                                                             │\n'
          '└───────────────────────────────────────────────────────────────────────────────────────────────┘\n',
        ),
      );
    });
  });

  group('flavor undefined', () {
    testWithoutContext('pattern', () {
      expect(
        flavorUndefinedHandler.test('Task assembleFooRelease not found in root project.'),
        isTrue,
      );
      expect(
        flavorUndefinedHandler.test('Task assembleBarRelease not found in root project.'),
        isTrue,
      );
      expect(flavorUndefinedHandler.test('Task assembleBar not found in root project.'), isTrue);
      expect(
        flavorUndefinedHandler.test('Task assembleBar_foo not found in root project.'),
        isTrue,
      );
    });

    testUsingContext(
      'handler - with flavor',
      () async {
        processManager.addCommand(
          const FakeCommand(
            command: <String>['gradlew', 'app:tasks', '--all', '--console=auto'],
            stdout: '''
assembleRelease
assembleFlavor1
assembleFlavor1Release
assembleFlavor_2
assembleFlavor_2Release
assembleDebug
assembleProfile
assembles
assembleFooTest
          ''',
          ),
        );

        await flavorUndefinedHandler.handler(
          project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
          usesAndroidX: true,
          line: '',
        );

        expect(
          testLogger.statusText,
          contains(
            'Gradle project does not define a task suitable '
            'for the requested build.',
          ),
        );
        expect(
          testLogger.statusText,
          contains(
            '\n'
            '┌─ Flutter Fix ───────────────────────────────────────────────────────────────────────────────────┐\n'
            '│ [!]  Gradle project does not define a task suitable for the requested build.                    │\n'
            '│                                                                                                 │\n'
            '│ The /android/app/build.gradle file defines product flavors: flavor1, flavor_2. You must specify │\n'
            '│ a --flavor option to select one of them.                                                        │\n'
            '└─────────────────────────────────────────────────────────────────────────────────────────────────┘\n',
          ),
        );
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        Java: () => FakeJava(),
        GradleUtils: () => FakeGradleUtils(),
        Platform: () => fakePlatform('android'),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );

    testUsingContext(
      'handler - without flavor',
      () async {
        processManager.addCommand(
          const FakeCommand(
            command: <String>['gradlew', 'app:tasks', '--all', '--console=auto'],
            stdout: '''
assembleRelease
assembleDebug
assembleProfile
          ''',
          ),
        );

        await flavorUndefinedHandler.handler(
          project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
          usesAndroidX: true,
          line: '',
        );

        expect(
          testLogger.statusText,
          contains(
            '\n'
            '┌─ Flutter Fix ─────────────────────────────────────────────────────────────────────────────────┐\n'
            '│ [!]  Gradle project does not define a task suitable for the requested build.                  │\n'
            '│                                                                                               │\n'
            '│ The /android/app/build.gradle file does not define any custom product flavors. You cannot use │\n'
            '│ the --flavor option.                                                                          │\n'
            '└───────────────────────────────────────────────────────────────────────────────────────────────┘\n',
          ),
        );
        expect(processManager, hasNoRemainingExpectations);
      },
      overrides: <Type, Generator>{
        Java: () => FakeJava(),
        GradleUtils: () => FakeGradleUtils(),
        Platform: () => fakePlatform('android'),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );
  });

  group('higher minSdkVersion', () {
    const String stdoutLine =
        'uses-sdk:minSdkVersion 16 cannot be smaller than version 21 declared in library [:webview_flutter] /tmp/cirrus-ci-build/all_plugins/build/webview_flutter/intermediates/library_manifest/release/AndroidManifest.xml as the library might be using APIs not available in 21';

    testWithoutContext('pattern', () {
      expect(minSdkVersionHandler.test(stdoutLine), isTrue);
    });

    testUsingContext(
      'suggestion',
      () async {
        await minSdkVersionHandler.handler(
          line: stdoutLine,
          project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
          usesAndroidX: true,
        );

        expect(
          testLogger.statusText,
          contains(
            '\n'
            '┌─ Flutter Fix ─────────────────────────────────────────────────────────────────────────────────┐\n'
            '│ The plugin webview_flutter requires a higher Android SDK version.                             │\n'
            '│ Fix this issue by adding the following to the file /android/app/build.gradle:                 │\n'
            '│ android {                                                                                     │\n'
            '│   defaultConfig {                                                                             │\n'
            '│     minSdkVersion 21                                                                          │\n'
            '│   }                                                                                           │\n'
            '│ }                                                                                             │\n'
            '│                                                                                               │\n'
            '│ Following this change, your app will not be available to users running Android SDKs below 21. │\n'
            '│ Consider searching for a version of this plugin that supports these lower versions of the     │\n'
            '│ Android SDK instead.                                                                          │\n'
            '│ For more information, see: https://flutter.dev/to/review-gradle-config                        │\n'
            '└───────────────────────────────────────────────────────────────────────────────────────────────┘\n',
          ),
        );
      },
      overrides: <Type, Generator>{
        GradleUtils: () => FakeGradleUtils(),
        Platform: () => fakePlatform('android'),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );
  });

  // https://issuetracker.google.com/issues/141126614
  group('transform input issue', () {
    testWithoutContext('pattern', () {
      expect(
        transformInputIssueHandler.test('https://issuetracker.google.com/issues/158753935'),
        isTrue,
      );
    });

    testUsingContext(
      'suggestion',
      () async {
        await transformInputIssueHandler.handler(
          project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
          usesAndroidX: true,
          line: '',
        );

        expect(
          testLogger.statusText,
          contains(
            '\n'
            '┌─ Flutter Fix ─────────────────────────────────────────────────────────────────┐\n'
            '│ This issue appears to be https://github.com/flutter/flutter/issues/58247.     │\n'
            '│ Fix this issue by adding the following to the file /android/app/build.gradle: │\n'
            '│ android {                                                                     │\n'
            '│   lintOptions {                                                               │\n'
            '│     checkReleaseBuilds false                                                  │\n'
            '│   }                                                                           │\n'
            '│ }                                                                             │\n'
            '└───────────────────────────────────────────────────────────────────────────────┘\n',
          ),
        );
      },
      overrides: <Type, Generator>{
        GradleUtils: () => FakeGradleUtils(),
        Platform: () => fakePlatform('android'),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );
  });

  group('Dependency mismatch', () {
    testWithoutContext('pattern', () {
      expect(
        lockFileDepMissingHandler.test(
          '''
* What went wrong:
Execution failed for task ':app:generateDebugFeatureTransitiveDeps'.
> Could not resolve all artifacts for configuration ':app:debugRuntimeClasspath'.
   > Resolved 'androidx.lifecycle:lifecycle-common:2.2.0' which is not part of the dependency lock state
   > Resolved 'androidx.customview:customview:1.0.0' which is not part of the dependency lock state''',
        ),
        isTrue,
      );
    });

    testUsingContext(
      'suggestion',
      () async {
        await lockFileDepMissingHandler.handler(
          project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
          usesAndroidX: true,
          line: '',
        );

        expect(
          testLogger.statusText,
          contains(
            '\n'
            '┌─ Flutter Fix ────────────────────────────────────────────────────────────────────────────┐\n'
            '│ You need to update the lockfile, or disable Gradle dependency locking.                   │\n'
            '│ To regenerate the lockfiles run: `./gradlew :generateLockfiles` in /android/build.gradle │\n'
            '│ To remove dependency locking, remove the `dependencyLocking` from /android/build.gradle  │\n'
            '└──────────────────────────────────────────────────────────────────────────────────────────┘\n',
          ),
        );
      },
      overrides: <Type, Generator>{
        GradleUtils: () => FakeGradleUtils(),
        Platform: () => fakePlatform('android'),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );
  });

  testUsingContext(
    'generates correct gradle command for Unix-like environment',
    () async {
      await lockFileDepMissingHandler.handler(
        project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        usesAndroidX: true,
        line: '',
      );

      expect(
        testLogger.statusText,
        contains(
          '\n'
          '┌─ Flutter Fix ────────────────────────────────────────────────────────────────────────────┐\n'
          '│ You need to update the lockfile, or disable Gradle dependency locking.                   │\n'
          '│ To regenerate the lockfiles run: `./gradlew :generateLockfiles` in /android/build.gradle │\n'
          '│ To remove dependency locking, remove the `dependencyLocking` from /android/build.gradle  │\n'
          '└──────────────────────────────────────────────────────────────────────────────────────────┘\n'
          '',
        ),
      );
    },
    overrides: <Type, Generator>{
      GradleUtils: () => FakeGradleUtils(),
      Platform: () => fakePlatform('linux'),
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'generates correct gradle command for windows environment',
    () async {
      await lockFileDepMissingHandler.handler(
        project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        usesAndroidX: true,
        line: '',
      );
      expect(
        testLogger.statusText,
        contains(
          '\n'
          '┌─ Flutter Fix ────────────────────────────────────────────────────────────────────────────────┐\n'
          '│ You need to update the lockfile, or disable Gradle dependency locking.                       │\n'
          '│ To regenerate the lockfiles run: `.\\gradlew.bat :generateLockfiles` in /android/build.gradle │\n'
          '│ To remove dependency locking, remove the `dependencyLocking` from /android/build.gradle      │\n'
          '└──────────────────────────────────────────────────────────────────────────────────────────────┘\n'
          '',
        ),
      );
    },
    overrides: <Type, Generator>{
      GradleUtils: () => FakeGradleUtils(),
      Platform: () => fakePlatform('windows'),
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  group('Incompatible Kotlin version', () {
    testWithoutContext('pattern', () {
      expect(
        incompatibleKotlinVersionHandler.test(
          'Module was compiled with an incompatible version of Kotlin. The binary version of its metadata is 1.5.1, expected version is 1.1.15.',
        ),
        isTrue,
      );
      expect(
        incompatibleKotlinVersionHandler.test(
          "class 'kotlin.Unit' was compiled with an incompatible version of Kotlin.",
        ),
        isTrue,
      );
    });

    testUsingContext(
      'suggestion',
      () async {
        await incompatibleKotlinVersionHandler.handler(
          project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
          usesAndroidX: true,
          line: '',
        );

        expect(
          testLogger.statusText,
          contains(
            '\n'
            '┌─ Flutter Fix ────────────────────────────────────────────────────────────────────────────────┐\n'
            '│ [!] Your project requires a newer version of the Kotlin Gradle plugin.                       │\n'
            '│ Find the latest version on https://kotlinlang.org/docs/releases.html#release-details, then   │\n'
            '│ update the                                                                                   │\n'
            '│ version number of the plugin with id "org.jetbrains.kotlin.android" in the plugins block of  │\n'
            '│ /android/settings.gradle.                                                                    │\n'
            '│                                                                                              │\n'
            '│ Alternatively (if your project was created before Flutter 3.19), update                      │\n'
            '│ /android/build.gradle                                                                        │\n'
            "│ ext.kotlin_version = '<latest-version>'                                                      │\n"
            '└──────────────────────────────────────────────────────────────────────────────────────────────┘\n',
          ),
        );
      },
      overrides: <Type, Generator>{
        GradleUtils: () => FakeGradleUtils(),
        Platform: () => fakePlatform('android'),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );
  });

  group('Bump Gradle', () {
    const String errorMessage = '''
A problem occurred evaluating project ':app'.
> Failed to apply plugin [id 'kotlin-android']
   > The current Gradle version 4.10.2 is not compatible with the Kotlin Gradle plugin. Please use Gradle 6.1.1 or newer, or the previous version of the Kotlin plugin.
''';

    testWithoutContext('pattern', () {
      expect(outdatedGradleHandler.test(errorMessage), isTrue);
    });

    testUsingContext(
      'suggestion',
      () async {
        await outdatedGradleHandler.handler(
          line: errorMessage,
          project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
          usesAndroidX: true,
        );

        expect(
          testLogger.statusText,
          contains(
            '\n'
            '┌─ Flutter Fix ────────────────────────────────────────────────────────────────────┐\n'
            '│ [!] Your project needs to upgrade Gradle and the Android Gradle plugin.          │\n'
            '│                                                                                  │\n'
            '│ To fix this issue, replace the following content:                                │\n'
            '│ /android/build.gradle:                                                           │\n'
            "│     - classpath 'com.android.tools.build:gradle:<current-version>'               │\n"
            "│     + classpath 'com.android.tools.build:gradle:$templateAndroidGradlePluginVersion'                           │\n"
            '│ /android/gradle/wrapper/gradle-wrapper.properties:                               │\n'
            '│     - https://services.gradle.org/distributions/gradle-<current-version>-all.zip │\n'
            '│     + https://services.gradle.org/distributions/gradle-$templateDefaultGradleVersion-all.zip            │\n'
            '└──────────────────────────────────────────────────────────────────────────────────┘\n',
          ),
        );
      },
      overrides: <Type, Generator>{
        GradleUtils: () => FakeGradleUtils(),
        Platform: () => fakePlatform('android'),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );
  });

  group('Required compileSdkVersion', () {
    const String errorMessage = '''
Execution failed for task ':app:checkDebugAarMetadata'.
> A failure occurred while executing com.android.build.gradle.internal.tasks.CheckAarMetadataWorkAction
   > One or more issues found when checking AAR metadata values:

     The minCompileSdk (31) specified in a
     dependency's AAR metadata (META-INF/com/android/build/gradle/aar-metadata.properties)
     is greater than this module's compileSdkVersion (android-30).
     Dependency: androidx.window:window-java:1.0.0-beta04.
     AAR metadata file: ~/.gradle/caches/transforms-3/2adc32c5b3f24bed763d33fbfb203338/transformed/jetified-window-java-1.0.0-beta04/META-INF/com/android/build/gradle/aar-metadata.properties.

     The minCompileSdk (31) specified in a
     dependency's AAR metadata (META-INF/com/android/build/gradle/aar-metadata.properties)
     is greater than this module's compileSdkVersion (android-30).
     Dependency: androidx.window:window:1.0.0-beta04.
     AAR metadata file: ~/.gradle/caches/transforms-3/88f7e476ef68cecca729426edff955b5/transformed/jetified-window-1.0.0-beta04/META-INF/com/android/build/gradle/aar-metadata.properties.
''';

    testWithoutContext('pattern', () {
      expect(minCompileSdkVersionHandler.test(errorMessage), isTrue);
    });

    testUsingContext(
      'suggestion',
      () async {
        await minCompileSdkVersionHandler.handler(
          line: errorMessage,
          project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
          usesAndroidX: true,
        );

        expect(
          testLogger.statusText,
          contains(
            '\n'
            '┌─ Flutter Fix ──────────────────────────────────────────────────────────────────┐\n'
            '│ [!] Your project requires a higher compileSdk version.                         │\n'
            '│ Fix this issue by bumping the compileSdk version in /android/app/build.gradle: │\n'
            '│ android {                                                                      │\n'
            '│   compileSdk 31                                                                │\n'
            '│ }                                                                              │\n'
            '└────────────────────────────────────────────────────────────────────────────────┘\n',
          ),
        );
      },
      overrides: <Type, Generator>{
        GradleUtils: () => FakeGradleUtils(),
        Platform: () => fakePlatform('android'),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );
  });

  group('incompatible java and android gradle plugin versions error', () {
    const String errorMessage = '''
* What went wrong:
An exception occurred applying plugin request [id: 'com.android.application']
> Failed to apply plugin 'com.android.internal.application'.
   > Android Gradle plugin requires Java 17 to run. You are currently using Java 11.
      You can try some of the following options:
       - changing the IDE settings.
       - changing the JAVA_HOME environment variable.
       - changing `org.gradle.java.home` in `gradle.properties`.
''';

    testWithoutContext('pattern', () {
      expect(incompatibleJavaAndAgpVersionsHandler.test(errorMessage), isTrue);
    });

    testUsingContext(
      'suggestion',
      () async {
        await incompatibleJavaAndAgpVersionsHandler.handler(
          line: errorMessage,
          project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
          usesAndroidX: true,
        );

        // Ensure the error notes the required Java version, the Java version currently used,
        // the android studio and android sdk installation link, the flutter command to set
        // the Java version Flutter uses, and the flutter doctor command.
        expect(
          testLogger.statusText,
          contains(
            'Android Gradle plugin requires Java 17 to run. You are currently using Java 11.',
          ),
        );
        expect(testLogger.statusText, contains('https://developer.android.com/studio/install'));
        expect(testLogger.statusText, contains('`flutter config --jdk-dir=“</path/to/jdk>“`'));
        expect(testLogger.statusText, contains('`flutter doctor --verbose`'));
      },
      overrides: <Type, Generator>{
        GradleUtils: () => FakeGradleUtils(),
        Platform: () => fakePlatform('android'),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );
  });

  group('SSLException', () {
    testWithoutContext('pattern', () {
      expect(
        sslExceptionHandler.test(r'''
Exception in thread "main" javax.net.ssl.SSLException: Tag mismatch!
at java.base/sun.security.ssl.Alert.createSSLException(Alert.java:129)
at java.base/sun.security.ssl.TransportContext.fatal(TransportContext.java:321)
at java.base/sun.security.ssl.TransportContext.fatal(TransportContext.java:264)
at java.base/sun.security.ssl.TransportContext.fatal(TransportContext.java:259)
at java.base/sun.security.ssl.SSLTransport.decode(SSLTransport.java:129)
at java.base/sun.security.ssl.SSLSocketImpl.decode(SSLSocketImpl.java:1155)
at java.base/sun.security.ssl.SSLSocketImpl.readApplicationRecord(SSLSocketImpl.java:1125)
at java.base/sun.security.ssl.SSLSocketImpl$AppInputStream.read(SSLSocketImpl.java:823)
at java.base/java.io.BufferedInputStream.read1(BufferedInputStream.java:290)
at java.base/java.io.BufferedInputStream.read(BufferedInputStream.java:351)
at java.base/sun.net.www.MeteredStream.read(MeteredStream.java:134)
at java.base/java.io.FilterInputStream.read(FilterInputStream.java:133)
at java.base/sun.net.www.protocol.http.HttpURLConnection$HttpInputStream.read(HttpURLConnection.java:3444)
at java.base/sun.net.www.protocol.http.HttpURLConnection$HttpInputStream.read(HttpURLConnection.java:3437)
at org.gradle.wrapper.Download.downloadInternal(Download.java:62)
at org.gradle.wrapper.Download.download(Download.java:44)
at org.gradle.wrapper.Install$1.call(Install.java:61)
at org.gradle.wrapper.Install$1.call(Install.java:48)
at org.gradle.wrapper.ExclusiveFileAccessManager.access(ExclusiveFileAccessManager.java:65)
at org.gradle.wrapper.Install.createDist(Install.java:48)
at org.gradle.wrapper.WrapperExecutor.execute(WrapperExecutor.java:128)
at org.gradle.wrapper.GradleWrapperMain.main(GradleWrapperMain.java:61)'''),
        isTrue,
      );

      expect(
        sslExceptionHandler.test(r'''
Caused by: javax.crypto.AEADBadTagException: Tag mismatch!
at java.base/com.sun.crypto.provider.GaloisCounterMode.decryptFinal(GaloisCounterMode.java:580)
at java.base/com.sun.crypto.provider.CipherCore.finalNoPadding(CipherCore.java:1049)
at java.base/com.sun.crypto.provider.CipherCore.doFinal(CipherCore.java:985)
at java.base/com.sun.crypto.provider.AESCipher.engineDoFinal(AESCipher.java:491)
at java.base/javax.crypto.CipherSpi.bufferCrypt(CipherSpi.java:779)
at java.base/javax.crypto.CipherSpi.engineDoFinal(CipherSpi.java:730)
at java.base/javax.crypto.Cipher.doFinal(Cipher.java:2497)
at java.base/sun.security.ssl.SSLCipher$T12GcmReadCipherGenerator$GcmReadCipher.decrypt(SSLCipher.java:1613)
at java.base/sun.security.ssl.SSLSocketInputRecord.decodeInputRecord(SSLSocketInputRecord.java:262)
at java.base/sun.security.ssl.SSLSocketInputRecord.decode(SSLSocketInputRecord.java:190)
at java.base/sun.security.ssl.SSLTransport.decode(SSLTransport.java:108)'''),
        isTrue,
      );
    });

    testUsingContext(
      'suggestion',
      () async {
        final GradleBuildStatus status = await sslExceptionHandler.handler(
          project: FakeFlutterProject(),
          usesAndroidX: true,
          line: '',
        );

        expect(status, GradleBuildStatus.retry);
        expect(
          testLogger.errorText,
          contains('Gradle threw an error while downloading artifacts from the network.'),
        );
      },
      overrides: <Type, Generator>{
        GradleUtils: () => FakeGradleUtils(),
        Platform: () => fakePlatform('android'),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );
  });

  group('Zip exception', () {
    testWithoutContext('pattern', () {
      expect(
        zipExceptionHandler.test(r'''
Exception in thread "main" java.util.zip.ZipException: error in opening zip file
at java.util.zip.ZipFile.open(Native Method)
at java.util.zip.ZipFile.(ZipFile.java:225)
at java.util.zip.ZipFile.(ZipFile.java:155)
at java.util.zip.ZipFile.(ZipFile.java:169)
at org.gradle.wrapper.Install.unzip(Install.java:214)
at org.gradle.wrapper.Install.access$600(Install.java:27)
at org.gradle.wrapper.Install$1.call(Install.java:74)
at org.gradle.wrapper.Install$1.call(Install.java:48)
at org.gradle.wrapper.ExclusiveFileAccessManager.access(ExclusiveFileAccessManager.java:65)
at org.gradle.wrapper.Install.createDist(Install.java:48)
at org.gradle.wrapper.WrapperExecutor.execute(WrapperExecutor.java:128)
at org.gradle.wrapper.GradleWrapperMain.main(GradleWrapperMain.java:61)'''),
        isTrue,
      );
    });

    testUsingContext(
      'suggestion',
      () async {
        fileSystem.file('foo/.gradle/fizz.zip').createSync(recursive: true);

        final GradleBuildStatus result = await zipExceptionHandler.handler(
          project: FakeFlutterProject(),
          usesAndroidX: true,
          line: '',
        );

        expect(result, equals(GradleBuildStatus.retry));
        expect(fileSystem.file('foo/.gradle/fizz.zip'), exists);
        expect(
          testLogger.errorText,
          contains('[!] Your .gradle directory under the home directory might be corrupted.\n'),
        );
        expect(testLogger.statusText, '');
      },
      overrides: <Type, Generator>{
        Platform: () => FakePlatform(environment: <String, String>{'HOME': 'foo/'}),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        BotDetector: () => const FakeBotDetector(false),
      },
    );

    testUsingContext(
      'suggestion if running as bot',
      () async {
        fileSystem.file('foo/.gradle/fizz.zip').createSync(recursive: true);

        final GradleBuildStatus result = await zipExceptionHandler.handler(
          project: FakeFlutterProject(),
          usesAndroidX: true,
          line: '',
        );

        expect(result, equals(GradleBuildStatus.retry));
        expect(fileSystem.file('foo/.gradle/fizz.zip'), isNot(exists));

        expect(
          testLogger.errorText,
          contains('[!] Your .gradle directory under the home directory might be corrupted.\n'),
        );
        expect(testLogger.statusText, contains('Deleting foo/.gradle\n'));
      },
      overrides: <Type, Generator>{
        Platform: () => FakePlatform(environment: <String, String>{'HOME': 'foo/'}),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        BotDetector: () => const FakeBotDetector(true),
      },
    );

    testUsingContext(
      'suggestion if stdin has terminal and user entered y',
      () async {
        fileSystem.file('foo/.gradle/fizz.zip').createSync(recursive: true);

        final GradleBuildStatus result = await zipExceptionHandler.handler(
          line: '',
          usesAndroidX: true,
          project: FakeFlutterProject(),
        );

        expect(result, equals(GradleBuildStatus.retry));
        expect(fileSystem.file('foo/.gradle/fizz.zip'), isNot(exists));
        expect(
          testLogger.errorText,
          contains('[!] Your .gradle directory under the home directory might be corrupted.\n'),
        );
        expect(testLogger.statusText, contains('Deleting foo/.gradle\n'));
      },
      overrides: <Type, Generator>{
        Platform: () => FakePlatform(environment: <String, String>{'HOME': 'foo/'}),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        AnsiTerminal: () => _TestPromptTerminal('y'),
        BotDetector: () => const FakeBotDetector(false),
      },
    );

    testUsingContext(
      'suggestion if stdin has terminal and user entered n',
      () async {
        fileSystem.file('foo/.gradle/fizz.zip').createSync(recursive: true);

        final GradleBuildStatus result = await zipExceptionHandler.handler(
          line: '',
          usesAndroidX: true,
          project: FakeFlutterProject(),
        );

        expect(result, equals(GradleBuildStatus.retry));
        expect(fileSystem.file('foo/.gradle/fizz.zip'), exists);
        expect(
          testLogger.errorText,
          contains('[!] Your .gradle directory under the home directory might be corrupted.\n'),
        );
        expect(testLogger.statusText, '');
      },
      overrides: <Type, Generator>{
        Platform: () => FakePlatform(environment: <String, String>{'HOME': 'foo/'}),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        AnsiTerminal: () => _TestPromptTerminal('n'),
        BotDetector: () => const FakeBotDetector(false),
      },
    );
  });

  group('incompatible java and gradle versions error', () {
    const String errorMessage = '''
Could not compile build file '…/example/android/build.gradle'.
> startup failed:
  General error during conversion: Unsupported class file major version 61
  java.lang.IllegalArgumentException: Unsupported class file major version 61
''';

    testWithoutContext('pattern', () {
      expect(incompatibleJavaAndGradleVersionsHandler.test(errorMessage), isTrue);
    });

    testUsingContext(
      'suggestion',
      () async {
        await incompatibleJavaAndGradleVersionsHandler.handler(
          line: errorMessage,
          project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
          usesAndroidX: true,
        );

        // Ensure the error notes the incompatible Gradle/AGP/Java versions, links to related resources,
        // and a portion of the path to where to change their gradle version.
        expect(
          testLogger.statusText,
          contains('Gradle version is incompatible with the Java version'),
        );
        expect(testLogger.statusText, contains('flutter.dev/to/java-gradle-incompatibility'));
        expect(testLogger.statusText, contains('gradle-wrapper.properties'));
        expect(
          testLogger.statusText,
          contains('https://docs.gradle.org/current/userguide/compatibility.html#java'),
        );
      },
      overrides: <Type, Generator>{
        GradleUtils: () => FakeGradleUtils(),
        Platform: () => fakePlatform('android'),
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
      },
    );
  });

  testUsingContext(
    'couldNotOpenCacheDirectoryHandler',
    () async {
      final GradleBuildStatus status = await couldNotOpenCacheDirectoryHandler.handler(
        line: '''
FAILURE: Build failed with an exception.

* Where:
Script '/Volumes/Work/s/w/ir/x/w/flutter/packages/flutter_tools/gradle/src/main/groovy/flutter.groovy' line: 276

* What went wrong:
A problem occurred evaluating script.
> Failed to apply plugin class 'FlutterPlugin'.
   > Could not open cache directory 41rl0ui7kgmsyfwn97o2jypl6 (/Volumes/Work/s/w/ir/cache/gradle/caches/6.7/gradle-kotlin-dsl/41rl0ui7kgmsyfwn97o2jypl6).
      > Failed to create Jar file /Volumes/Work/s/w/ir/cache/gradle/caches/6.7/generated-gradle-jars/gradle-api-6.7.jar.''',
        project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        usesAndroidX: true,
      );
      expect(testLogger.errorText, contains('Gradle threw an error while resolving dependencies'));
      expect(status, GradleBuildStatus.retry);
    },
    overrides: <Type, Generator>{
      GradleUtils: () => FakeGradleUtils(),
      Platform: () => fakePlatform('android'),
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'compileSdk 35 and AGP < 8.1',
    () async {
      const String errorExample = r'''
Execution failed for task ':app:bundleReleaseResources'.
> A failure occurred while executing com.android.build.gradle.internal.res.Aapt2ProcessResourcesRunnable
   > Android resource linking failed
     aapt2 E 08-19 15:06:26 76078 5921862 LoadedArsc.cpp:94] RES_TABLE_TYPE_TYPE entry offsets overlap actual entry data.
     aapt2 E 08-19 15:06:26 76078 5921862 ApkAssets.cpp:152] Failed to load resources table in APK '/Users/mackall/Library/Android/sdk/platforms/android-35/android.jar'.
     error: failed to load include path /Users/mackall/Library/Android/sdk/platforms/android-35/android.jar.
    ''';

      await incompatibleCompileSdk35AndAgpVersionHandler.handler(
        line: errorExample,
        project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        usesAndroidX: true,
      );

      expect(
        testLogger.statusText,
        contains(
          '\n'
          '┌─ Flutter Fix ────────────────────────────────────────────────────────────────────────────────────┐\n'
          '│ [!] Using compileSdk 35 requires Android Gradle Plugin (AGP) 8.1.0 or higher.                    │\n'
          '│  Please upgrade to a newer AGP version. The version of AGP that your project uses is likely      │\n'
          '│  defined in:                                                                                     │\n'
          '│ /android/settings.gradle,                                                                        │\n'
          '│ in the \'plugins\' closure (by the number following "com.android.application").                    │\n'
          '│  Alternatively, if your project was created with an older version of the templates, it is likely │\n'
          '│ in the buildscript.dependencies closure of the top-level build.gradle:                           │\n'
          '│ /android/build.gradle,                                                                           │\n'
          '│ as the number following "com.android.tools.build:gradle:".                                       │\n'
          '│                                                                                                  │\n'
          '│  Finally, if you have a strong reason to avoid upgrading AGP, you can temporarily lower the      │\n'
          '│  compileSdk version in the following file:                                                       │\n'
          '│ /android/app/build.gradle                                                                        │\n'
          '└──────────────────────────────────────────────────────────────────────────────────────────────────┘\n'
          '',
        ),
      );
    },
    overrides: <Type, Generator>{
      GradleUtils: () => FakeGradleUtils(),
      Platform: () => fakePlatform('android'),
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'AGP 7.3.0 R8 bug',
    () async {
      const String errorExample = r'''
ERROR:/Users/mackall/.gradle/caches/transforms-3/bd2c84591857c6d4c308221ffece862e/transformed/jetified-media3-exoplayer-dash-1.4.0-runtime.jar: R8: com.android.tools.r8.internal.Y10: Unused argument with users in androidx
    ''';

      await r8DexingBugInAgp73Handler.handler(
        line: errorExample,
        project: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        usesAndroidX: true,
      );

      expect(
        testLogger.statusText,
        contains(
          '\n'
          '┌─ Flutter Fix ────────────────────────────────────────────────────────────────────────────────────┐\n'
          '│ [!] Version 7.3 of the Android Gradle Plugin (AGP) uses a version of R8 that contains a bug      │\n'
          '│ which causes this error (see more info at https://issuetracker.google.com/issues/242308990).     │\n'
          '│ To fix this error, update to a newer version of AGP (at least 7.4.0).                            │\n'
          '│                                                                                                  │\n'
          '│  The version of AGP that your project uses is likely defined in:                                 │\n'
          '│ /android/settings.gradle,                                                                        │\n'
          '│ in the \'plugins\' closure (by the number following "com.android.application").                    │\n'
          '│  Alternatively, if your project was created with an older version of the templates, it is likely │\n'
          '│ in the buildscript.dependencies closure of the top-level build.gradle:                           │\n'
          '│ /android/build.gradle,                                                                           │\n'
          '│ as the number following "com.android.tools.build:gradle:".                                       │\n'
          '└──────────────────────────────────────────────────────────────────────────────────────────────────┘\n'
          '',
        ),
      );
    },
    overrides: <Type, Generator>{
      GradleUtils: () => FakeGradleUtils(),
      Platform: () => fakePlatform('android'),
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'Java 21 and jlink bug',
    () async {
      const String errorExample = r'''
* What went wrong:
Execution failed for task ':shared_preferences_android:compileReleaseJavaWithJavac'.
> Could not resolve all files for configuration ':shared_preferences_android:androidJdkImage'.
   > Failed to transform core-for-system-modules.jar to match attributes {artifactType=_internal_android_jdk_image, org.gradle.libraryelements=jar, org.gradle.usage=java-runtime}.
      > Execution failed for JdkImageTransform: /Users/mackall/Library/Android/sdk/platforms/android-34/core-for-system-modules.jar.
         > Error while executing process /Users/mackall/Desktop/JDKs/21/jdk-21.0.2.jdk/Contents/Home/bin/jlink with arguments {--module-path /Users/mackall/.gradle/caches/8.9/transforms/2890fec03da42154757073d3208548e5-79660961-f91d-4df2-90bc-b9a3f2a270bd/transformed/output/temp/jmod --add-modules java.base --output /Users/mackall/.gradle/caches/8.9/transforms/2890fec03da42154757073d3208548e5-79660961-f91d-4df2-90bc-b9a3f2a270bd/transformed/output/jdkImage --disable-plugin system-modules}
    ''';

      final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);
      await jlinkErrorWithJava21AndSourceCompatibility.handler(
        line: errorExample,
        project: project,
        usesAndroidX: true,
      );

      // Main fix text.
      expect(
        testLogger.statusText,
        contains('To fix this error, please upgrade your AGP version to at least 8.2.1.'),
      );
      // Paths to AGP location.
      expect(testLogger.statusText, contains('/android/settings.gradle'));
      expect(testLogger.statusText, contains('/android/build.gradle'));
      // Links to info.
      expect(testLogger.statusText, contains('https://issuetracker.google.com/issues/294137077'));
      expect(testLogger.statusText, contains('https://github.com/flutter/flutter/issues/156304'));
    },
    overrides: <Type, Generator>{
      GradleUtils: () => FakeGradleUtils(),
      Platform: () => fakePlatform('android'),
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );
}

bool formatTestErrorMessage(String errorMessage, GradleHandledError error) {
  return errorMessage.split('\n').any((String line) => error.test(line));
}

Platform fakePlatform(String name) {
  return FakePlatform(environment: <String, String>{'HOME': '/'}, operatingSystem: name);
}

class FakeGradleUtils extends Fake implements GradleUtils {
  @override
  String getExecutable(FlutterProject project) {
    return 'gradlew';
  }
}

/// Simple terminal that returns the specified string when
/// promptForCharInput is called.
class _TestPromptTerminal extends Fake implements AnsiTerminal {
  _TestPromptTerminal(this.promptResult);

  final String promptResult;

  @override
  bool get stdinHasTerminal => true;

  @override
  Future<String> promptForCharInput(
    List<String> acceptedCharacters, {
    required Logger logger,
    String? prompt,
    int? defaultChoiceIndex,
    bool displayAcceptedCharacters = true,
  }) {
    return Future<String>.value(promptResult);
  }
}

class FakeFlutterProject extends Fake implements FlutterProject {}
