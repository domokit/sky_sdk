// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';
import 'dart:math';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';

/// An example that sets up local http server for serving single
/// image, creates single flutter widget with five copies of requested
/// image and prints how long the loading took.
///
/// This is used in [$FH/flutter/devicelab/bin/tasks/image_list_reported_duration.dart] test.
///
///
/// To generate new certificate:
///
/// $ openssl req -new -out image_list.csr
///   Generating a 2048 bit RSA private key
///   Enter PEM pass phrase: <random string>
///   ...
///   Common Name (eg, fully qualified host name) []:localhost
///
/// Copy content of the privateKey below into image_list.key file, then
/// $ openssl x509 -req -sha256 -days 3650 -in image_list.csr -signkey image_list.key -out image_list.crt
///
/// Copy content of the image_list.crt into certificate string below.
String certificate = '''
-----BEGIN CERTIFICATE-----
MIICpDCCAYwCCQD1kfAz8IhbazANBgkqhkiG9w0BAQsFADAUMRIwEAYDVQQDDAls
b2NhbGhvc3QwHhcNMjAwODI0MjE1MTUwWhcNMzAwODIyMjE1MTUwWjAUMRIwEAYD
VQQDDAlsb2NhbGhvc3QwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQCi
/fmozdYuCIZbJS7y4zYPp2NRboLXrpUcUzzvzz+24k/TYUPNeRvf6wiNXHvr1ijM
g1j3wQP72RphxI7cY7XCwzRiM5QeQy3EtRz4ETYBzOev3mHLLEgZ9RnSq/u42siS
S9CNjoz97liPyQUq8h37/09qhYG0hR/2pRN+YB9g7sNYoGe2B7zkh3azRS0/Ltgl
tXwHUId7QzJc15W9Q7adsNVTpOCo7dOj2KWz6sEtFGkYfwLV5uiTslRdWCCOUD9i
ZjCtlPqALkGnWyhNiFJESLbVNC6MURyMngcALW0JTMwc2oDjMxtdNkMl0cdzPlhX
MDKIKpY9bWbRKUUdsfOnAgMBAAEwDQYJKoZIhvcNAQELBQADggEBAHZo/Io7hE9P
jDhSSP+4iSwx6xjnkRjBHReoea+XwSCv1s9Alwe1Nub6u5jUBhCpGCyU4diKFV1W
zhunXKY+zRGGtr09nYoN9UVizS5fAFb+h2x3Tw8lsxs4JpPQeWTbGK9Ci+jyfuZu
xPvdU8I8oxiTRPoWa1KpPm6UVvcrjyftvbqJ4l7cZ8KZN4JNSZlphX8lIM14xR4H
12sFFTcYWPNDTqO1A9MSflG4OkG59LDHV36JAEqB61pP8hipowVp48+rzD2DVpqb
r/Mw+0x0HENUTMVExSA5rj/3fxNMggUSl2YsujVJjkb1LiQNPORX7rBndcjknAMt
TvaTkrwwZA4=
-----END CERTIFICATE-----
''';

String privateKey = '''
-----BEGIN PRIVATE KEY-----
MIIEvAIBADANBgkqhkiG9w0BAQEFAASCBKYwggSiAgEAAoIBAQCi/fmozdYuCIZb
JS7y4zYPp2NRboLXrpUcUzzvzz+24k/TYUPNeRvf6wiNXHvr1ijMg1j3wQP72Rph
xI7cY7XCwzRiM5QeQy3EtRz4ETYBzOev3mHLLEgZ9RnSq/u42siSS9CNjoz97liP
yQUq8h37/09qhYG0hR/2pRN+YB9g7sNYoGe2B7zkh3azRS0/LtgltXwHUId7QzJc
15W9Q7adsNVTpOCo7dOj2KWz6sEtFGkYfwLV5uiTslRdWCCOUD9iZjCtlPqALkGn
WyhNiFJESLbVNC6MURyMngcALW0JTMwc2oDjMxtdNkMl0cdzPlhXMDKIKpY9bWbR
KUUdsfOnAgMBAAECggEADUStiS9Qayjofwz02HLkmLugmyVq41Hj841XHZJ6dlHP
+74kPdrJCR5h8NgBgn5JjfR3TpvYziyrOCA/HPPE/RjU79WRDjGbzTKNLCiCg/0B
M1DgFyEAsZRBSOQVNsQgpcAkNxHOqnE3pmTP1eIlzLjI5zv9Bgv8QSDJCHWcuFA2
NrvGudq3dlFnZwjipx0k0E1hCsaClqLsi5jEXIBA6TX7RTeeXjC+j2/DlmTpBxo0
c34o/sSoCl+mfJDQ3QApXLFuycBl7nauO0M+VsUWrKYqHHr4NIcgoIpIX28QjWc/
Y2+iooMSBO1ToK2nPD8hZQwNDF8xz6Xf7QNkTOpnqQKBgQDNgfNYkEvgWxhSMmW7
cK06supZC/isj2JfQmlJc80JcAvf8rpynZLi4XWZWRo0PykM4szI91h3YHTG27VX
YVHSsFs+6FwP7fLAHR7FYn65p/bwTuqpWGjcmQhc0HWx7y45HRIoSK/m+Z7lNA/J
QxnCp1khTmhPqCNglo+38vZSzQKBgQDLCeBhji0K/BOmCrhNxRxrCvHYEd6gqRtC
+rKhco6mMrxma/UmggRKoGXg0yMcya3199y3pDkumHSlcMdUb+I9b+3j8R6ivoqN
TI1ned5K1uq3FyZpD0dZQWuunVeqYXuUQw5y5GxvK7haohbVpUG6lC3qYq2ubiYC
D0ENUfNoQwKBgEGucOozZCzWsJVEykL4JkWGfWPscZQlV5l+jkwNmNCVYRY4a+LJ
/fJJgN58HeXo8ePOcQkiFMJCr9AG1JSS5CXke6VFencU4+sG45jOfBY2WrQ/ZLyv
JwSqXIPdlGBEQ4+5fN4nLSEzUteKpij7KzaNae09NBWRdY0fUdvG6XdZAoGAf02S
/TfKsB97JlmEU2aqOcdj+WjC4JMG/8j2JVoRbM1U6Rb5X4qXrD7DgeKAGnWteBJP
tmjmXXvDb1O19xArlv/N9WRiJAI6FvwPkPiNUvlLsz51m9uzjZgCLzqCE9cJR91/
erQT9ORBs7n7fTsfah+sZlA2u65ecF4mGHbwmccCgYAqHNRHnx1iHrYfr97cmXc9
fNjJ7e1NHhVdgpGjaOiBSKj2rHNRy6iwCNbs5wjmRWlgqnFEM5r0VfFn9L0PvcQK
7iExMTm/PkSqHUntpy82Q8zRWmhw0G5p9DYyIPtaeW1NIKpIlCw6dTlf750BiGkr
mhBKvYQc85gja0s1c+1VXA==
-----END PRIVATE KEY-----
''';

class MyHttpOverrides extends HttpOverrides {
  @override
  HttpClient createHttpClient(SecurityContext? context) {
    return super.createHttpClient(
      (context ?? SecurityContext())..setTrustedCertificatesBytes(certificate.codeUnits),
    );
  }
}

Future<void> main() async {
  HttpOverrides.global = MyHttpOverrides();

  final SecurityContext serverContext =
      SecurityContext()
        ..useCertificateChainBytes(certificate.codeUnits)
        ..usePrivateKeyBytes(privateKey.codeUnits);

  final HttpServer httpServer = await HttpServer.bindSecure('localhost', 0, serverContext);
  final int port = httpServer.port;
  debugPrint('Listening on port $port.');

  // Initializes bindings before using any platform channels.
  WidgetsFlutterBinding.ensureInitialized();
  final ByteData byteData = await rootBundle.load('images/coast.jpg');
  httpServer.listen((HttpRequest request) async {
    const int chunk_size = 2048;
    int offset = byteData.offsetInBytes;
    while (offset < byteData.lengthInBytes) {
      final int length = min(byteData.lengthInBytes - offset, chunk_size);
      final Uint8List bytes = byteData.buffer.asUint8List(offset, length);
      offset += length;
      request.response.add(bytes);
      // Let other isolates and microtasks to run.
      await Future<void>.delayed(Duration.zero);
    }
    request.response.close();
  });

  runApp(MyApp(port));
}

const int images = 50;

@immutable
class MyApp extends StatelessWidget {
  const MyApp(this.port, {super.key});

  final int port;

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(primarySwatch: Colors.blue),
      home: MyHomePage(title: 'Flutter Demo Home Page', port: port),
    );
  }
}

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key, required this.title, required this.port});
  final String title;
  final int port;

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> with TickerProviderStateMixin {
  int _counter = 0;

  void _incrementCounter() {
    setState(() {
      _counter++;
    });
  }

  Widget createImage(final int index, final Completer<bool> completer) {
    return Image.network(
      'https://localhost:${widget.port}/${_counter * images + index}',
      frameBuilder: (BuildContext context, Widget child, int? frame, bool wasSynchronouslyLoaded) {
        if (frame == 0 && !completer.isCompleted) {
          completer.complete(true);
        }
        return child;
      },
    );
  }

  @override
  Widget build(BuildContext context) {
    final List<AnimationController> controllers = <AnimationController>[
      for (int i = 0; i < images; i++)
        AnimationController(duration: const Duration(milliseconds: 3600), vsync: this)..repeat(),
    ];
    final List<Completer<bool>> completers = <Completer<bool>>[
      for (int i = 0; i < images; i++) Completer<bool>(),
    ];
    final List<Future<bool>> futures =
        completers.map((Completer<bool> completer) => completer.future).toList();
    final DateTime started = DateTime.now();
    Future.wait(futures).then((_) {
      debugPrint(
        '===image_list=== all loaded in ${DateTime.now().difference(started).inMilliseconds}ms.',
      );
    });
    return Scaffold(
      appBar: AppBar(title: Text(widget.title)),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Row(children: createImageList(images, completers, controllers)),
            const Text('You have pushed the button this many times:'),
            Text('$_counter', style: Theme.of(context).textTheme.headlineMedium),
          ],
        ),
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: _incrementCounter,
        tooltip: 'Increment',
        child: const Icon(Icons.add),
      ),
    );
  }

  List<Widget> createImageList(
    int count,
    List<Completer<bool>> completers,
    List<AnimationController> controllers,
  ) {
    final List<Widget> list = <Widget>[];
    for (int i = 0; i < count; i++) {
      list.add(
        Flexible(
          fit: FlexFit.tight,
          flex: i + 1,
          child: RotationTransition(
            turns: controllers[i],
            child: createImage(i + 1, completers[i]),
          ),
        ),
      );
    }
    return list;
  }
}
