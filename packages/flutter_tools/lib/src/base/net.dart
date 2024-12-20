// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:meta/meta.dart';

import '../convert.dart';
import 'common.dart';
import 'file_system.dart';
import 'io.dart';
import 'logger.dart';
import 'platform.dart';

const int kNetworkProblemExitCode = 50;
const String kFlutterStorageBaseUrl = 'FLUTTER_STORAGE_BASE_URL';

typedef HttpClientFactory = HttpClient Function();

typedef UrlTunneller = Future<String> Function(String url);

/// If [httpClientFactory] is null, a default [HttpClient] is used.
class Net {
  Net({HttpClientFactory? httpClientFactory, required Logger logger, required Platform platform})
    : _httpClientFactory = httpClientFactory ?? (() => HttpClient()),
      _logger = logger,
      _platform = platform;

  final HttpClientFactory _httpClientFactory;

  final Logger _logger;

  final Platform _platform;

  /// Download a file from the given URL.
  ///
  /// If a destination file is not provided, returns the bytes.
  ///
  /// If a destination file is provided, streams the bytes to that file and
  /// returns an empty list.
  ///
  /// If [maxAttempts] is exceeded, returns null.
  Future<List<int>?> fetchUrl(
    Uri url, {
    int? maxAttempts,
    File? destFile,
    @visibleForTesting Duration? durationOverride,
  }) async {
    int attempts = 0;
    int durationSeconds = 1;
    while (true) {
      attempts += 1;
      _MemoryIOSink? memorySink;
      IOSink sink;
      if (destFile == null) {
        memorySink = _MemoryIOSink();
        sink = memorySink;
      } else {
        sink = destFile.openWrite();
      }

      final bool result = await _attempt(url, destSink: sink);
      if (result) {
        return memorySink?.writes.takeBytes() ?? <int>[];
      }

      if (maxAttempts != null && attempts >= maxAttempts) {
        _logger.printStatus('Download failed -- retry $attempts');
        return null;
      }
      _logger.printStatus(
        'Download failed -- attempting retry $attempts in '
        '$durationSeconds second${durationSeconds == 1 ? "" : "s"}...',
      );
      await Future<void>.delayed(durationOverride ?? Duration(seconds: durationSeconds));
      if (durationSeconds < 64) {
        durationSeconds *= 2;
      }
    }
  }

  /// Check if the given URL points to a valid endpoint.
  Future<bool> doesRemoteFileExist(Uri url) => _attempt(url, onlyHeaders: true);

  // Returns true on success and false on failure.
  Future<bool> _attempt(Uri url, {IOSink? destSink, bool onlyHeaders = false}) async {
    assert(onlyHeaders || destSink != null);
    _logger.printTrace('Downloading: $url');
    final HttpClient httpClient = _httpClientFactory();
    HttpClientRequest request;
    HttpClientResponse? response;
    try {
      if (onlyHeaders) {
        request = await httpClient.headUrl(url);
      } else {
        request = await httpClient.getUrl(url);
      }
      response = await request.close();
    } on ArgumentError catch (error) {
      final String? overrideUrl = _platform.environment[kFlutterStorageBaseUrl];
      if (overrideUrl != null && url.toString().contains(overrideUrl)) {
        _logger.printError(error.toString());
        throwToolExit(
          'The value of $kFlutterStorageBaseUrl ($overrideUrl) could not be '
          'parsed as a valid url. Please see https://flutter.dev/to/use-mirror-site '
          'for an example of how to use it.\n'
          'Full URL: $url',
          exitCode: kNetworkProblemExitCode,
        );
      }
      _logger.printError(error.toString());
      rethrow;
    } on HandshakeException catch (error) {
      _logger.printTrace(error.toString());
      throwToolExit(
        'Could not authenticate download server. You may be experiencing a man-in-the-middle attack,\n'
        'your network may be compromised, or you may have malware installed on your computer.\n'
        'URL: $url',
        exitCode: kNetworkProblemExitCode,
      );
    } on SocketException catch (error) {
      _logger.printTrace('Download error: $error');
      return false;
    } on HttpException catch (error) {
      _logger.printTrace('Download error: $error');
      return false;
    }

    // If we're making a HEAD request, we're only checking to see if the URL is
    // valid.
    if (onlyHeaders) {
      return response.statusCode == HttpStatus.ok;
    }
    if (response.statusCode != HttpStatus.ok) {
      if (response.statusCode > 0 && response.statusCode < 500) {
        throwToolExit(
          'Download failed.\n'
          'URL: $url\n'
          'Error: ${response.statusCode} ${response.reasonPhrase}',
          exitCode: kNetworkProblemExitCode,
        );
      }
      // 5xx errors are server errors and we can try again
      _logger.printTrace('Download error: ${response.statusCode} ${response.reasonPhrase}');
      return false;
    }
    _logger.printTrace('Received response from server, collecting bytes...');
    try {
      assert(destSink != null);
      await response.forEach(destSink!.add);
      return true;
    } on IOException catch (error) {
      _logger.printTrace('Download error: $error');
      return false;
    } finally {
      await destSink?.flush();
      await destSink?.close();
    }
  }
}

/// An IOSink that collects whatever is written to it.
class _MemoryIOSink implements IOSink {
  @override
  Encoding encoding = utf8;

  final BytesBuilder writes = BytesBuilder(copy: false);

  @override
  void add(List<int> data) {
    writes.add(data);
  }

  @override
  Future<void> addStream(Stream<List<int>> stream) {
    final Completer<void> completer = Completer<void>();
    stream.listen(add).onDone(completer.complete);
    return completer.future;
  }

  @override
  void writeCharCode(int charCode) {
    add(<int>[charCode]);
  }

  @override
  void write(Object? obj) {
    add(encoding.encode('$obj'));
  }

  @override
  void writeln([Object? obj = '']) {
    add(encoding.encode('$obj\n'));
  }

  @override
  void writeAll(Iterable<dynamic> objects, [String separator = '']) {
    bool addSeparator = false;
    for (final dynamic object in objects) {
      if (addSeparator) {
        write(separator);
      }
      write(object);
      addSeparator = true;
    }
  }

  @override
  void addError(dynamic error, [StackTrace? stackTrace]) {
    throw UnimplementedError();
  }

  @override
  Future<void> get done => close();

  @override
  Future<void> close() async {}

  @override
  Future<void> flush() async {}
}

/// Returns [true] if [address] is an IPv6 address.
bool isIPv6Address(String address) {
  try {
    Uri.parseIPv6Address(address);
    return true;
  } on FormatException {
    return false;
  }
}
