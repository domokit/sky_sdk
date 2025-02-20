// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:collection';
import 'dart:convert';
import 'dart:io' as io;

import 'package:archive/archive.dart' as a;
import 'package:path/path.dart' as path;

import 'cache.dart';
import 'limits.dart';
import 'patterns.dart';

enum FileType {
  binary, // won't have its own license block
  text, // might have its own UTF-8 license block
  latin1Text, // might have its own Windows-1252 license block
  zip, // should be parsed as an archive and drilled into
  tar, // should be parsed as an archive and drilled into
  gz, // should be parsed as a single compressed file and exposed
  bzip2, // should be parsed as a single compressed file and exposed
  notPartOfBuild, // can be skipped entirely (e.g. Mac OS X ._foo files, bash scripts)
}

typedef Reader = List<int> Function();

class BytesOf extends Key {
  BytesOf(super.value);
}

class UTF8Of extends Key {
  UTF8Of(super.value);
}

class Latin1Of extends Key {
  Latin1Of(super.value);
}

bool matchesSignature(List<int> bytes, List<int> signature) {
  if (bytes.length < signature.length) {
    return false;
  }
  for (int index = 0; index < signature.length; index += 1) {
    if (signature[index] != -1 && bytes[index] != signature[index]) {
      return false;
    }
  }
  return true;
}

bool hasSubsequence(List<int> bytes, List<int> signature, int limit) {
  if (bytes.length < limit) {
    limit = bytes.length;
  }
  for (int index = 0; index < limit; index += 1) {
    if (bytes.length - index < signature.length) {
      return false;
    }
    for (int offset = 0; offset < signature.length; offset += 1) {
      if (signature[offset] != -1 && bytes[index + offset] != signature[offset]) {
        break;
      }
      if (offset + 1 == signature.length) {
        return true;
      }
    }
  }
  return false;
}

const String kMultiLicenseFileHeader = 'Notices for files contained in';

bool isMultiLicenseNotice(Reader reader) {
  final List<int> bytes = reader();
  return ascii.decode(bytes.take(kMultiLicenseFileHeader.length).toList(), allowInvalid: true) ==
      kMultiLicenseFileHeader;
}

FileType identifyFile(String name, Reader reader) {
  List<int>? bytes;
  if ((path.split(name).reversed.take(6).toList().reversed.join('/') ==
          'third_party/icu/source/extra/uconv/README') || // This specific ICU README isn't in UTF-8.
      (path.split(name).reversed.take(6).toList().reversed.join('/') ==
          'third_party/icu/source/samples/uresb/sr.txt') || // This specific sample contains non-UTF-8 data (unlike other sr.txt files).
      (path.split(name).reversed.take(2).toList().reversed.join('/') ==
          'builds/detect.mk') || // This specific freetype sample contains non-UTF-8 data (unlike other .mk files).
      (path.split(name).reversed.take(3).toList().reversed.join('/') ==
          'third_party/cares/cares.rc')) {
    return FileType.latin1Text;
  }
  if (path.split(name).reversed.take(6).toList().reversed.join('/') ==
      'dart/runtime/tests/vm/dart/bad_snapshot') {
    // Not any particular format
    return FileType.binary;
  }
  if (path.split(name).reversed.take(9).toList().reversed.join('/') ==
          'fuchsia/sdk/linux/dart/zircon/lib/src/fakes/handle_disposition.dart' || // has bogus but benign "authors" reference, reported to jamesr@
      path.split(name).reversed.take(6).toList().reversed.join('/') ==
          'third_party/angle/src/common/fuchsia_egl/fuchsia_egl.c' || // has bogus but benign "authors" reference, reported to author and legal team
      path.split(name).reversed.take(6).toList().reversed.join('/') ==
          'third_party/angle/src/common/fuchsia_egl/fuchsia_egl.h' || // has bogus but benign "authors" reference, reported to author and legal team
      path.split(name).reversed.take(6).toList().reversed.join('/') ==
          'third_party/angle/src/common/fuchsia_egl/fuchsia_egl_backend.h') {
    // has bogus but benign "authors" reference, reported to author and legal team
    return FileType.binary;
  }
  if (path.split(name).reversed.take(6).toList().reversed.join('/') ==
      'flutter/third_party/brotli/c/common/dictionary.bin.br') {
    // Brotli-compressed Brotli dictionary
    return FileType.binary;
  }
  final String base = path.basename(name);
  if (base.startsWith('._')) {
    bytes ??= reader();
    if (matchesSignature(bytes, <int>[
      0x00,
      0x05,
      0x16,
      0x07,
      0x00,
      0x02,
      0x00,
      0x00,
      0x4d,
      0x61,
      0x63,
      0x20,
      0x4f,
      0x53,
      0x20,
      0x58,
    ])) {
      return FileType.notPartOfBuild;
    } // The ._* files in Mac OS X archives that gives icons and stuff
  }
  if (path.split(name).contains('cairo')) {
    bytes ??= reader();
    // "Copyright <latin1 copyright symbol> "
    if (hasSubsequence(bytes, <int>[
      0x43,
      0x6f,
      0x70,
      0x79,
      0x72,
      0x69,
      0x67,
      0x68,
      0x74,
      0x20,
      0xA9,
      0x20,
    ], kMaxSize)) {
      return FileType.latin1Text;
    }
  }
  switch (base) {
    // Build files
    case 'DEPS':
      return FileType.text;
    case 'MANIFEST':
      return FileType.text;
    // Licenses
    case 'COPYING':
      return FileType.text;
    case 'LICENSE':
      return FileType.text;
    case 'NOTICE.txt':
      return isMultiLicenseNotice(reader) ? FileType.binary : FileType.text;
    case 'NOTICE':
      return FileType.text;
    // Documentation
    case 'Changes':
      return FileType.text;
    case 'change.log':
      return FileType.text;
    case 'ChangeLog':
      return FileType.text;
    case 'CHANGES.0':
      return FileType.latin1Text;
    case 'README':
      return FileType.text;
    case 'TODO':
      return FileType.text;
    case 'NEWS':
      return FileType.text;
    case 'README.chromium':
      return FileType.text;
    case 'README.flutter':
      return FileType.text;
    case 'README.tests':
      return FileType.text;
    case 'OWNERS':
      return FileType.text;
    case 'AUTHORS':
      return FileType.text;
    // Signatures (found in .jar files typically)
    case 'CERT.RSA':
      return FileType.binary;
    case 'ECLIPSE_.RSA':
      return FileType.binary;
    // Binary data files
    case 'tzdata':
      return FileType.binary;
    case 'compressed_atrace_data.txt':
      return FileType.binary;
    // Source files that don't use UTF-8
    case 'Messages_de_DE.properties': // has a few non-ASCII characters they forgot to escape (from gnu-libstdc++)
    case 'mmx_blendtmp.h': // author name in comment contains latin1 (mesa)
    case 'calling_convention.txt': // contains a soft hyphen instead of a real hyphen for some reason (mesa)
    // Character encoding data files
    case 'danish-ISO-8859-1.txt':
    case 'eucJP.txt':
    case 'hangul-eucKR.txt':
    case 'hania-eucKR.txt':
    case 'ibm-37-test.txt':
    case 'iso8859-1.txt':
    case 'ISO-8859-2.txt':
    case 'ISO-8859-3.txt':
    case 'koi8r.txt':
      return FileType.latin1Text;
    // Giant data files
    case 'icudtl_dat.S':
    case 'icudtl.dat':
    case 'icudtl.dat.hash':
      return FileType.binary;
  }
  switch (path.extension(name)) {
    // C/C++ code
    case '.h':
      return FileType.text;
    case '.c':
      return FileType.text;
    case '.cc':
      return FileType.text;
    case '.cpp':
      return FileType.text;
    case '.inc':
      return FileType.text;
    // Go code
    case '.go':
      return FileType.text;
    // ObjectiveC code
    case '.m':
      return FileType.text;
    // Assembler
    case '.asm':
      return FileType.text;
    // Shell
    case '.sh':
      return FileType.notPartOfBuild;
    case '.bat':
      return FileType.notPartOfBuild;
    // Build files
    case '.ac':
      return FileType.notPartOfBuild;
    case '.am':
      return FileType.notPartOfBuild;
    case '.gn':
      return FileType.notPartOfBuild;
    case '.gni':
      return FileType.notPartOfBuild;
    case '.gyp':
      return FileType.notPartOfBuild;
    case '.gypi':
      return FileType.notPartOfBuild;
    // Java code
    case '.java':
      return FileType.text;
    case '.jar':
      return FileType.zip; // Java package
    case '.class':
      return FileType.binary; // compiled Java bytecode (usually found inside .jar archives)
    case '.dex':
      return FileType.binary; // Dalvik Executable (usually found inside .jar archives)
    // Dart code
    case '.dart':
      return FileType.text;
    case '.dill':
      return FileType.binary; // Compiled Dart code
    // LLVM bitcode
    case '.bc':
      return FileType.binary;
    // Python code
    case '.py':
      bytes ??= reader();
      // # -*- coding: Latin-1 -*-
      if (matchesSignature(bytes, <int>[
        0x23,
        0x20,
        0x2d,
        0x2a,
        0x2d,
        0x20,
        0x63,
        0x6f,
        0x64,
        0x69,
        0x6e,
        0x67,
        0x3a,
        0x20,
        0x4c,
        0x61,
        0x74,
        0x69,
        0x6e,
        0x2d,
        0x31,
        0x20,
        0x2d,
        0x2a,
        0x2d,
      ])) {
        return FileType.latin1Text;
      }
      return FileType.text;
    case '.pyc':
      return FileType.binary; // compiled Python bytecode
    // Machine code
    case '.so':
      return FileType.binary; // ELF shared object
    case '.xpt':
      return FileType.binary; // XPCOM Type Library
    // Graphics code
    case '.glsl':
      return FileType.text;
    case '.spvasm':
      return FileType.text;
    // Documentation
    case '.md':
      return FileType.text;
    case '.txt':
      return FileType.text;
    case '.html':
      return FileType.text;
    // Fonts
    case '.ttf':
      return FileType.binary; // TrueType Font
    case '.ttcf': // (mac)
    case '.ttc':
      return FileType.binary; // TrueType Collection (windows)
    case '.woff':
      return FileType.binary; // Web Open Font Format
    case '.otf':
      return FileType.binary; // OpenType Font
    // Graphics formats
    case '.gif':
      return FileType.binary; // GIF
    case '.png':
      return FileType.binary; // PNG
    case '.tga':
      return FileType.binary; // Truevision TGA (TARGA)
    case '.dng':
      return FileType.binary; // Digial Negative (Adobe RAW format)
    case '.jpg':
    case '.jpeg':
      return FileType.binary; // JPEG
    case '.ico':
      return FileType.binary; // Windows icon format
    case '.icns':
      return FileType.binary; // macOS icon format
    case '.bmp':
      return FileType.binary; // Windows bitmap format
    case '.wbmp':
      return FileType.binary; // Wireless bitmap format
    case '.webp':
      return FileType.binary; // WEBP
    case '.pdf':
      return FileType.binary; // PDF
    case '.emf':
      return FileType.binary; // Windows enhanced metafile format
    case '.skp':
      return FileType.binary; // Skia picture format
    case '.mskp':
      return FileType.binary; // Skia picture format
    case '.spv':
      return FileType.binary; // SPIR-V
    // Videos
    case '.ogg':
      return FileType.binary; // Ogg media
    case '.mp4':
      return FileType.binary; // MPEG media
    case '.ts':
      return FileType.binary; // MPEG2 transport stream
    // Other binary files
    case '.raw':
      return FileType.binary; // raw audio or graphical data
    case '.bin':
      return FileType.binary; // some sort of binary data
    case '.rsc':
      return FileType.binary; // some sort of resource data
    case '.arsc':
      return FileType.binary; // Android compiled resources
    case '.apk':
      return FileType.zip; // Android Package
    case '.crx':
      return FileType.binary; // Chrome extension
    case '.keystore':
      return FileType.binary;
    case '.icc':
      return FileType.binary; // Color profile
    case '.swp':
      return FileType.binary; // Vim swap file
    case '.bfbs':
      return FileType.binary; // Flatbuffers Binary Schema
    // Archives
    case '.zip':
      return FileType.zip; // ZIP
    case '.tar':
      return FileType.tar; // Tar
    case '.gz':
      return FileType.gz; // GZip
    case '.bzip2':
      return FileType.bzip2; // BZip2
    // Image file types from the Fuchsia SDK.
    case '.blk':
    case '.vboot':
    case '.snapshot':
    case '.zbi':
      return FileType.binary;
    // Special cases
    case '.patch':
    case '.diff':
      // Don't try to read the copyright out of patch files, since there'll be fragments.
      return FileType.binary;
    case '.plist':
      // These commonly include the word "copyright" but in a way that isn't necessarily a copyright statement that applies to the file.
      // Since there's so few of them, and none have their own copyright statement, we just treat them as binary files.
      return FileType.binary;
  }
  bytes ??= reader();
  assert(bytes.isNotEmpty);
  if (matchesSignature(bytes, <int>[0x1F, 0x8B])) {
    return FileType.gz;
  } // GZip archive
  if (matchesSignature(bytes, <int>[0x42, 0x5A])) {
    return FileType.bzip2;
  } // BZip2 archive
  if (matchesSignature(bytes, <int>[0x42, 0x43])) {
    return FileType.binary;
  } // LLVM Bitcode
  if (matchesSignature(bytes, <int>[0xAC, 0xED])) {
    return FileType.binary;
  } // Java serialized object
  if (matchesSignature(bytes, <int>[0x4D, 0x5A])) {
    return FileType.binary;
  } // MZ executable (DOS, Windows PEs, etc)
  if (matchesSignature(bytes, <int>[0xFF, 0xD8, 0xFF])) {
    return FileType.binary;
  } // JPEG
  if (matchesSignature(bytes, <int>[-1, -1, 0xda, 0x27])) {
    return FileType.binary;
  } // ICU data files (.brk, .dict, etc)
  if (matchesSignature(bytes, <int>[0x03, 0x00, 0x08, 0x00])) {
    return FileType.binary;
  } // Android Binary XML
  if (matchesSignature(bytes, <int>[0x25, 0x50, 0x44, 0x46])) {
    return FileType.binary;
  } // PDF
  if (matchesSignature(bytes, <int>[0x43, 0x72, 0x32, 0x34])) {
    return FileType.binary;
  } // Chrome extension
  if (matchesSignature(bytes, <int>[0x4F, 0x67, 0x67, 0x53])) {
    return FileType.binary;
  } // Ogg media
  if (matchesSignature(bytes, <int>[0x50, 0x4B, 0x03, 0x04])) {
    return FileType.zip;
  } // ZIP archive
  if (matchesSignature(bytes, <int>[0x7F, 0x45, 0x4C, 0x46])) {
    return FileType.binary;
  } // ELF
  if (matchesSignature(bytes, <int>[0xCA, 0xFE, 0xBA, 0xBE])) {
    return FileType.binary;
  } // compiled Java bytecode (usually found inside .jar archives)
  if (matchesSignature(bytes, <int>[0xCE, 0xFA, 0xED, 0xFE])) {
    return FileType.binary;
  } // Mach-O binary, 32 bit, reverse byte ordering scheme
  if (matchesSignature(bytes, <int>[0xCF, 0xFA, 0xED, 0xFE])) {
    return FileType.binary;
  } // Mach-O binary, 64 bit, reverse byte ordering scheme
  if (matchesSignature(bytes, <int>[0xFE, 0xED, 0xFA, 0xCE])) {
    return FileType.binary;
  } // Mach-O binary, 32 bit
  if (matchesSignature(bytes, <int>[0xFE, 0xED, 0xFA, 0xCF])) {
    return FileType.binary;
  } // Mach-O binary, 64 bit
  if (matchesSignature(bytes, <int>[0x75, 0x73, 0x74, 0x61, 0x72])) {
    return FileType.bzip2;
  } // Tar
  if (matchesSignature(bytes, <int>[0x47, 0x49, 0x46, 0x38, 0x37, 0x61])) {
    return FileType.binary;
  } // GIF87a
  if (matchesSignature(bytes, <int>[0x47, 0x49, 0x46, 0x38, 0x39, 0x61])) {
    return FileType.binary;
  } // GIF89a
  if (matchesSignature(bytes, <int>[0x64, 0x65, 0x78, 0x0A, 0x30, 0x33, 0x35, 0x00])) {
    return FileType.binary;
  } // Dalvik Executable
  if (matchesSignature(bytes, <int>[0x21, 0x3C, 0x61, 0x72, 0x63, 0x68, 0x3E, 0x0A])) {
    // TODO(ianh): implement .ar parser, https://github.com/flutter/flutter/issues/25633
    return FileType.binary; // Unix archiver (ar)
  }
  if (matchesSignature(bytes, <int>[0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0a])) {
    return FileType.binary;
  } // PNG
  if (matchesSignature(bytes, <int>[
    0x58,
    0x50,
    0x43,
    0x4f,
    0x4d,
    0x0a,
    0x54,
    0x79,
    0x70,
    0x65,
    0x4c,
    0x69,
    0x62,
    0x0d,
    0x0a,
    0x1a,
  ])) {
    return FileType.binary;
  } // XPCOM Type Library
  if (matchesSignature(bytes, <int>[0x23, 0x21])) {
    // #! indicates a shell script, those are not part of the build
    return FileType.notPartOfBuild;
  }
  return FileType.text;
}

String _normalize(String fileContents) {
  fileContents = fileContents.replaceAll(newlinePattern, '\n');
  fileContents = fileContents.replaceAll('\t', ' ' * 4);
  return fileContents;
}

// INTERFACE

// base class
abstract class IoNode {
  // Subclasses of IoNode are not mutually exclusive.
  // For example, a ZIP file is represented as a File that also implements Directory.
  String get name;
  String get fullName;

  @override
  String toString() => fullName;
}

// interface
abstract class File extends IoNode {
  List<int>? readBytes();
}

// interface
abstract class TextFile extends File {
  String readString();
}

mixin UTF8TextFile implements TextFile {
  @override
  String readString() {
    try {
      return cache(UTF8Of(this), () => _normalize(utf8.decode(readBytes()!)));
    } on FormatException {
      print(fullName);
      rethrow;
    }
  }
}

mixin Latin1TextFile implements TextFile {
  @override
  String readString() {
    return cache(Latin1Of(this), () {
      final List<int> bytes = readBytes()!;
      if (bytes.any((int byte) => byte == 0x00)) {
        throw '$fullName contains a U+0000 NULL and is probably not actually encoded as Win1252';
      }
      bool isUTF8 = false;
      try {
        cache(UTF8Of(this), () => utf8.decode(readBytes()!));
        isUTF8 = true;
      } on FormatException {
        // Exceptions are fine/expected for non-UTF8 text, which we test for
        // immediately below.
      }
      if (isUTF8) {
        throw '$fullName contains valid UTF-8 and is probably not actually encoded as Win1252';
      }
      return _normalize(latin1.decode(bytes));
    });
  }
}

// interface
abstract class Directory extends IoNode {
  // lists children (shallow walk, not deep walk)
  Iterable<IoNode> get walk;
}

// interface
abstract class Link extends IoNode {}

mixin ZipFile on File implements Directory {
  ArchiveDirectory? _root;

  @override
  Iterable<IoNode> get walk {
    try {
      _root ??= ArchiveDirectory.parseArchive(a.ZipDecoder().decodeBytes(readBytes()!), fullName);
      return _root!.walk;
    } catch (exception) {
      print('failed to parse archive:\n$fullName');
      rethrow;
    }
  }
}

mixin TarFile on File implements Directory {
  ArchiveDirectory? _root;

  @override
  Iterable<IoNode> get walk {
    try {
      _root ??= ArchiveDirectory.parseArchive(a.TarDecoder().decodeBytes(readBytes()!), fullName);
      return _root!.walk;
    } catch (exception) {
      print('failed to parse archive:\n$fullName');
      rethrow;
    }
  }
}

mixin GZipFile on File implements Directory {
  InMemoryFile? _data;

  @override
  Iterable<IoNode> get walk sync* {
    try {
      final String innerName = path.basenameWithoutExtension(fullName);
      _data ??=
          InMemoryFile.parse('$fullName!$innerName', a.GZipDecoder().decodeBytes(readBytes()!))!;
      if (_data != null) {
        yield _data!;
      }
    } catch (exception) {
      print('failed to parse archive:\n$fullName');
      rethrow;
    }
  }
}

mixin BZip2File on File implements Directory {
  InMemoryFile? _data;

  @override
  Iterable<IoNode> get walk sync* {
    try {
      final String innerName = path.basenameWithoutExtension(fullName);
      _data ??=
          InMemoryFile.parse('$fullName!$innerName', a.BZip2Decoder().decodeBytes(readBytes()!))!;
      if (_data != null) {
        yield _data!;
      }
    } catch (exception) {
      print('failed to parse archive:\n$fullName');
      rethrow;
    }
  }
}

// FILESYSTEM IMPLEMENTATIoN

class FileSystemDirectory extends IoNode implements Directory {
  FileSystemDirectory(this._directory);

  factory FileSystemDirectory.fromPath(String name) {
    return FileSystemDirectory(io.Directory(name));
  }

  final io.Directory _directory;

  @override
  String get name => path.basename(_directory.path);

  @override
  String get fullName => _directory.path;

  List<int> _readBytes(io.File file) {
    return cache(BytesOf(file), () => file.readAsBytesSync());
  }

  @override
  Iterable<IoNode> get walk sync* {
    final List<io.FileSystemEntity> list = _directory.listSync().toList();
    list.sort((io.FileSystemEntity a, io.FileSystemEntity b) => a.path.compareTo(b.path));
    for (final io.FileSystemEntity entity in list) {
      if (entity is io.Directory) {
        yield FileSystemDirectory(entity);
      } else if (entity is io.Link) {
        yield FileSystemLink(entity);
      } else {
        assert(entity is io.File);
        final io.File fileEntity = entity as io.File;
        if (fileEntity.lengthSync() > 0) {
          switch (identifyFile(fileEntity.path, () => _readBytes(fileEntity))) {
            case FileType.binary:
              yield FileSystemFile(fileEntity);
            case FileType.zip:
              yield FileSystemZipFile(fileEntity);
            case FileType.tar:
              yield FileSystemTarFile(fileEntity);
            case FileType.gz:
              yield FileSystemGZipFile(fileEntity);
            case FileType.bzip2:
              yield FileSystemBZip2File(fileEntity);
            case FileType.text:
              yield FileSystemUTF8TextFile(fileEntity);
            case FileType.latin1Text:
              yield FileSystemLatin1TextFile(fileEntity);
            case FileType.notPartOfBuild:
              break; // ignore this file
          }
        }
      }
    }
  }
}

class FileSystemLink extends IoNode implements Link {
  FileSystemLink(this._link);

  final io.Link _link;

  @override
  String get name => path.basename(_link.path);

  @override
  String get fullName => _link.path;
}

class FileSystemFile extends IoNode implements File {
  FileSystemFile(this._file);

  final io.File _file;

  @override
  String get name => path.basename(_file.path);

  @override
  String get fullName => _file.path;

  @override
  List<int> readBytes() {
    return cache(BytesOf(_file), () => _file.readAsBytesSync());
  }
}

class FileSystemUTF8TextFile extends FileSystemFile with UTF8TextFile {
  FileSystemUTF8TextFile(super.file);
}

class FileSystemLatin1TextFile extends FileSystemFile with Latin1TextFile {
  FileSystemLatin1TextFile(super.file);
}

class FileSystemZipFile extends FileSystemFile with ZipFile {
  FileSystemZipFile(super.file);
}

class FileSystemTarFile extends FileSystemFile with TarFile {
  FileSystemTarFile(super.file);
}

class FileSystemGZipFile extends FileSystemFile with GZipFile {
  FileSystemGZipFile(super.file);
}

class FileSystemBZip2File extends FileSystemFile with BZip2File {
  FileSystemBZip2File(super.file);
}

// ARCHIVES

class ArchiveDirectory extends IoNode implements Directory {
  ArchiveDirectory(this.fullName, this.name);

  @override
  final String fullName;

  @override
  final String name;

  final Map<String, ArchiveDirectory> _subdirectories = SplayTreeMap<String, ArchiveDirectory>();
  final List<ArchiveFile> _files = <ArchiveFile>[];

  void _add(a.ArchiveFile entry, List<String> remainingPath) {
    if (remainingPath.length > 1) {
      final String subdirectoryName = remainingPath.removeAt(0);
      _subdirectories
          .putIfAbsent(
            subdirectoryName,
            () => ArchiveDirectory('$fullName/$subdirectoryName', subdirectoryName),
          )
          ._add(entry, remainingPath);
    } else {
      if (entry.size > 0) {
        final String entryFullName = '$fullName/${path.basename(entry.name)}';
        switch (identifyFile(entry.name, () => entry.content as List<int>)) {
          case FileType.binary:
            _files.add(ArchiveFile(entryFullName, entry));
          case FileType.zip:
            _files.add(ArchiveZipFile(entryFullName, entry));
          case FileType.tar:
            _files.add(ArchiveTarFile(entryFullName, entry));
          case FileType.gz:
            _files.add(ArchiveGZipFile(entryFullName, entry));
          case FileType.bzip2:
            _files.add(ArchiveBZip2File(entryFullName, entry));
          case FileType.text:
            _files.add(ArchiveUTF8TextFile(entryFullName, entry));
          case FileType.latin1Text:
            _files.add(ArchiveLatin1TextFile(entryFullName, entry));
          case FileType.notPartOfBuild:
            break; // ignore this file
        }
      }
    }
  }

  static ArchiveDirectory parseArchive(a.Archive archive, String ownerPath) {
    final ArchiveDirectory root = ArchiveDirectory('$ownerPath!', '');
    for (final a.ArchiveFile file in archive.files) {
      if (file.size > 0) {
        root._add(file, file.name.split('/'));
      }
    }
    return root;
  }

  @override
  Iterable<IoNode> get walk sync* {
    yield* _subdirectories.values;
    yield* _files;
  }
}

class ArchiveFile extends IoNode implements File {
  ArchiveFile(this.fullName, this._file);

  final a.ArchiveFile _file;

  @override
  String get name => path.basename(_file.name);

  @override
  final String fullName;

  @override
  List<int>? readBytes() {
    return _file.content as List<int>?;
  }
}

class ArchiveUTF8TextFile extends ArchiveFile with UTF8TextFile {
  ArchiveUTF8TextFile(super.fullName, super.file);
}

class ArchiveLatin1TextFile extends ArchiveFile with Latin1TextFile {
  ArchiveLatin1TextFile(super.fullName, super.file);
}

class ArchiveZipFile extends ArchiveFile with ZipFile {
  ArchiveZipFile(super.fullName, super.file);
}

class ArchiveTarFile extends ArchiveFile with TarFile {
  ArchiveTarFile(super.fullName, super.file);
}

class ArchiveGZipFile extends ArchiveFile with GZipFile {
  ArchiveGZipFile(super.fullName, super.file);
}

class ArchiveBZip2File extends ArchiveFile with BZip2File {
  ArchiveBZip2File(super.fullName, super.file);
}

// IN-MEMORY FILES (e.g. contents of GZipped files)

class InMemoryFile extends IoNode implements File {
  InMemoryFile(this.fullName, this._bytes);

  static InMemoryFile? parse(String fullName, List<int> bytes) {
    if (bytes.isEmpty) {
      return null;
    }
    switch (identifyFile(fullName, () => bytes)) {
      case FileType.binary:
        return InMemoryFile(fullName, bytes);
      case FileType.zip:
        return InMemoryZipFile(fullName, bytes);
      case FileType.tar:
        return InMemoryTarFile(fullName, bytes);
      case FileType.gz:
        return InMemoryGZipFile(fullName, bytes);
      case FileType.bzip2:
        return InMemoryBZip2File(fullName, bytes);
      case FileType.text:
        return InMemoryUTF8TextFile(fullName, bytes);
      case FileType.latin1Text:
        return InMemoryLatin1TextFile(fullName, bytes);
      case FileType.notPartOfBuild:
        break; // ignore this file
    }
    assert(false);
    return null;
  }

  final List<int> _bytes;

  @override
  String get name => '<data>';

  @override
  final String fullName;

  @override
  List<int> readBytes() => _bytes;
}

class InMemoryUTF8TextFile extends InMemoryFile with UTF8TextFile {
  InMemoryUTF8TextFile(super.fullName, super.file);
}

class InMemoryLatin1TextFile extends InMemoryFile with Latin1TextFile {
  InMemoryLatin1TextFile(super.fullName, super.file);
}

class InMemoryZipFile extends InMemoryFile with ZipFile {
  InMemoryZipFile(super.fullName, super.file);
}

class InMemoryTarFile extends InMemoryFile with TarFile {
  InMemoryTarFile(super.fullName, super.file);
}

class InMemoryGZipFile extends InMemoryFile with GZipFile {
  InMemoryGZipFile(super.fullName, super.file);
}

class InMemoryBZip2File extends InMemoryFile with BZip2File {
  InMemoryBZip2File(super.fullName, super.file);
}
