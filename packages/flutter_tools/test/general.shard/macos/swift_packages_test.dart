// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/isolated/mustache_template.dart';
import 'package:flutter_tools/src/macos/swift_packages.dart';

import '../../src/common.dart';

const String _doubleIndent = '        ';

void main() {
  group('SwiftPackage', () {
    testWithoutContext('createSwiftPackage also creates source file for each default target', () {
      final MemoryFileSystem fs = MemoryFileSystem();
      final File swiftPackageFile = fs.systemTempDirectory.childFile(
        'Packages/FlutterGeneratedPluginSwiftPackage/Package.swift',
      );
      const String target1Name = 'Target1';
      const String target2Name = 'Target2';
      final File target1SourceFile = fs.systemTempDirectory.childFile(
        'Packages/FlutterGeneratedPluginSwiftPackage/Sources/$target1Name/$target1Name.swift',
      );
      final File target2SourceFile = fs.systemTempDirectory.childFile(
        'Packages/FlutterGeneratedPluginSwiftPackage/Sources/$target2Name/$target2Name.swift',
      );
      final SwiftPackage swiftPackage = SwiftPackage(
        manifest: swiftPackageFile,
        name: 'FlutterGeneratedPluginSwiftPackage',
        platforms: <SwiftPackageSupportedPlatform>[],
        products: <SwiftPackageProduct>[],
        dependencies: <SwiftPackagePackageDependency>[],
        targets: <SwiftPackageTarget>[
          SwiftPackageTarget.defaultTarget(name: target1Name),
          SwiftPackageTarget.defaultTarget(name: 'Target2'),
        ],
        templateRenderer: const MustacheTemplateRenderer(),
      );
      swiftPackage.createSwiftPackage();
      expect(swiftPackageFile.existsSync(), isTrue);
      expect(target1SourceFile.existsSync(), isTrue);
      expect(target2SourceFile.existsSync(), isTrue);
    });

    testWithoutContext('createSwiftPackage also creates source file for binary target', () {
      final MemoryFileSystem fs = MemoryFileSystem();
      final File swiftPackageFile = fs.systemTempDirectory.childFile(
        'Packages/FlutterGeneratedPluginSwiftPackage/Package.swift',
      );
      final SwiftPackage swiftPackage = SwiftPackage(
        manifest: swiftPackageFile,
        name: 'FlutterGeneratedPluginSwiftPackage',
        platforms: <SwiftPackageSupportedPlatform>[],
        products: <SwiftPackageProduct>[],
        dependencies: <SwiftPackagePackageDependency>[],
        targets: <SwiftPackageTarget>[
          SwiftPackageTarget.binaryTarget(name: 'BinaryTarget', relativePath: ''),
        ],
        templateRenderer: const MustacheTemplateRenderer(),
      );
      swiftPackage.createSwiftPackage();
      expect(swiftPackageFile.existsSync(), isTrue);
      expect(
        fs.systemTempDirectory
            .childFile(
              'Packages/FlutterGeneratedPluginSwiftPackage/Sources/BinaryTarget/BinaryTarget.swift',
            )
            .existsSync(),
        isFalse,
      );
    });

    testWithoutContext('createSwiftPackage does not creates source file if already exists', () {
      final MemoryFileSystem fs = MemoryFileSystem();
      final File swiftPackageFile = fs.systemTempDirectory.childFile(
        'Packages/FlutterGeneratedPluginSwiftPackage/Package.swift',
      );
      const String target1Name = 'Target1';
      const String target2Name = 'Target2';
      final File target1SourceFile = fs.systemTempDirectory.childFile(
        'Packages/FlutterGeneratedPluginSwiftPackage/Sources/$target1Name/$target1Name.swift',
      );
      final File target2SourceFile = fs.systemTempDirectory.childFile(
        'Packages/FlutterGeneratedPluginSwiftPackage/Sources/$target2Name/$target2Name.swift',
      );

      fs.systemTempDirectory
          .childFile(
            'Packages/FlutterGeneratedPluginSwiftPackage/Sources/$target1Name/SomeSourceFile.swift',
          )
          .createSync(recursive: true);
      fs.systemTempDirectory
          .childFile(
            'Packages/FlutterGeneratedPluginSwiftPackage/Sources/$target2Name/SomeSourceFile.swift',
          )
          .createSync(recursive: true);

      final SwiftPackage swiftPackage = SwiftPackage(
        manifest: swiftPackageFile,
        name: 'FlutterGeneratedPluginSwiftPackage',
        platforms: <SwiftPackageSupportedPlatform>[],
        products: <SwiftPackageProduct>[],
        dependencies: <SwiftPackagePackageDependency>[],
        targets: <SwiftPackageTarget>[
          SwiftPackageTarget.defaultTarget(name: target1Name),
          SwiftPackageTarget.defaultTarget(name: 'Target2'),
        ],
        templateRenderer: const MustacheTemplateRenderer(),
      );
      swiftPackage.createSwiftPackage();
      expect(swiftPackageFile.existsSync(), isTrue);
      expect(target1SourceFile.existsSync(), isFalse);
      expect(target2SourceFile.existsSync(), isFalse);
    });

    group('create Package.swift from template', () {
      testWithoutContext('with none in each field', () {
        final MemoryFileSystem fs = MemoryFileSystem();
        final File swiftPackageFile = fs.systemTempDirectory.childFile(
          'Packages/FlutterGeneratedPluginSwiftPackage/Package.swift',
        );
        final SwiftPackage swiftPackage = SwiftPackage(
          manifest: swiftPackageFile,
          name: 'FlutterGeneratedPluginSwiftPackage',
          platforms: <SwiftPackageSupportedPlatform>[],
          products: <SwiftPackageProduct>[],
          dependencies: <SwiftPackagePackageDependency>[],
          targets: <SwiftPackageTarget>[],
          templateRenderer: const MustacheTemplateRenderer(),
        );
        swiftPackage.createSwiftPackage();
        expect(swiftPackageFile.readAsStringSync(), '''
// swift-tools-version: 5.9
// The swift-tools-version declares the minimum version of Swift required to build this package.
//
//  Generated file. Do not edit.
//

import PackageDescription

let package = Package(
    name: "FlutterGeneratedPluginSwiftPackage",
    products: [
$_doubleIndent
    ],
    dependencies: [
$_doubleIndent
    ],
    targets: [
$_doubleIndent
    ]
)
''');
      });

      testWithoutContext('with single in each field', () {
        final MemoryFileSystem fs = MemoryFileSystem();
        final File swiftPackageFile = fs.systemTempDirectory.childFile(
          'Packages/FlutterGeneratedPluginSwiftPackage/Package.swift',
        );
        final SwiftPackage swiftPackage = SwiftPackage(
          manifest: swiftPackageFile,
          name: 'FlutterGeneratedPluginSwiftPackage',
          platforms: <SwiftPackageSupportedPlatform>[
            SwiftPackageSupportedPlatform(
              platform: SwiftPackagePlatform.ios,
              version: Version(12, 0, null),
            ),
          ],
          products: <SwiftPackageProduct>[
            SwiftPackageProduct(name: 'Product1', targets: <String>['Target1']),
          ],
          dependencies: <SwiftPackagePackageDependency>[
            SwiftPackagePackageDependency(name: 'Dependency1', path: '/path/to/dependency1'),
          ],
          targets: <SwiftPackageTarget>[
            SwiftPackageTarget.defaultTarget(
              name: 'Target1',
              dependencies: <SwiftPackageTargetDependency>[
                SwiftPackageTargetDependency.product(
                  name: 'TargetDependency1',
                  packageName: 'TargetDependency1Package',
                ),
              ],
            ),
          ],
          templateRenderer: const MustacheTemplateRenderer(),
        );
        swiftPackage.createSwiftPackage();
        expect(swiftPackageFile.readAsStringSync(), '''
// swift-tools-version: 5.9
// The swift-tools-version declares the minimum version of Swift required to build this package.
//
//  Generated file. Do not edit.
//

import PackageDescription

let package = Package(
    name: "FlutterGeneratedPluginSwiftPackage",
    platforms: [
        .iOS("12.0")
    ],
    products: [
        .library(name: "Product1", targets: ["Target1"])
    ],
    dependencies: [
        .package(name: "Dependency1", path: "/path/to/dependency1")
    ],
    targets: [
        .target(
            name: "Target1",
            dependencies: [
                .product(name: "TargetDependency1", package: "TargetDependency1Package")
            ]
        )
    ]
)
''');
      });

      testWithoutContext('with multiple in each field', () {
        final MemoryFileSystem fs = MemoryFileSystem();
        final File swiftPackageFile = fs.systemTempDirectory.childFile(
          'Packages/FlutterGeneratedPluginSwiftPackage/Package.swift',
        );
        final SwiftPackage swiftPackage = SwiftPackage(
          manifest: swiftPackageFile,
          name: 'FlutterGeneratedPluginSwiftPackage',
          platforms: <SwiftPackageSupportedPlatform>[
            SwiftPackageSupportedPlatform(
              platform: SwiftPackagePlatform.ios,
              version: Version(12, 0, null),
            ),
            SwiftPackageSupportedPlatform(
              platform: SwiftPackagePlatform.macos,
              version: Version(10, 14, null),
            ),
          ],
          products: <SwiftPackageProduct>[
            SwiftPackageProduct(name: 'Product1', targets: <String>['Target1']),
            SwiftPackageProduct(name: 'Product2', targets: <String>['Target2']),
          ],
          dependencies: <SwiftPackagePackageDependency>[
            SwiftPackagePackageDependency(name: 'Dependency1', path: '/path/to/dependency1'),
            SwiftPackagePackageDependency(name: 'Dependency2', path: '/path/to/dependency2'),
          ],
          targets: <SwiftPackageTarget>[
            SwiftPackageTarget.binaryTarget(name: 'Target1', relativePath: '/path/to/target1'),
            SwiftPackageTarget.defaultTarget(
              name: 'Target2',
              dependencies: <SwiftPackageTargetDependency>[
                SwiftPackageTargetDependency.target(name: 'TargetDependency1'),
                SwiftPackageTargetDependency.product(
                  name: 'TargetDependency2',
                  packageName: 'TargetDependency2Package',
                ),
              ],
            ),
          ],
          templateRenderer: const MustacheTemplateRenderer(),
        );
        swiftPackage.createSwiftPackage();
        expect(swiftPackageFile.readAsStringSync(), '''
// swift-tools-version: 5.9
// The swift-tools-version declares the minimum version of Swift required to build this package.
//
//  Generated file. Do not edit.
//

import PackageDescription

let package = Package(
    name: "FlutterGeneratedPluginSwiftPackage",
    platforms: [
        .iOS("12.0"),
        .macOS("10.14")
    ],
    products: [
        .library(name: "Product1", targets: ["Target1"]),
        .library(name: "Product2", targets: ["Target2"])
    ],
    dependencies: [
        .package(name: "Dependency1", path: "/path/to/dependency1"),
        .package(name: "Dependency2", path: "/path/to/dependency2")
    ],
    targets: [
        .binaryTarget(
            name: "Target1",
            path: "/path/to/target1"
        ),
        .target(
            name: "Target2",
            dependencies: [
                .target(name: "TargetDependency1"),
                .product(name: "TargetDependency2", package: "TargetDependency2Package")
            ]
        )
    ]
)
''');
      });
    });
  });

  testWithoutContext('Format SwiftPackageSupportedPlatform', () {
    final SwiftPackageSupportedPlatform supportedPlatform = SwiftPackageSupportedPlatform(
      platform: SwiftPackagePlatform.ios,
      version: Version(17, 0, null),
    );
    expect(supportedPlatform.format(), '.iOS("17.0")');
  });

  group('Format SwiftPackageProduct', () {
    testWithoutContext('without targets and libraryType', () {
      final SwiftPackageProduct product = SwiftPackageProduct(
        name: 'ProductName',
        targets: <String>[],
      );
      expect(product.format(), '.library(name: "ProductName")');
    });

    testWithoutContext('with targets', () {
      final SwiftPackageProduct singleProduct = SwiftPackageProduct(
        name: 'ProductName',
        targets: <String>['Target1'],
      );
      expect(singleProduct.format(), '.library(name: "ProductName", targets: ["Target1"])');

      final SwiftPackageProduct multipleProducts = SwiftPackageProduct(
        name: 'ProductName',
        targets: <String>['Target1', 'Target2'],
      );
      expect(
        multipleProducts.format(),
        '.library(name: "ProductName", targets: ["Target1", "Target2"])',
      );
    });

    testWithoutContext('with libraryType', () {
      final SwiftPackageProduct product = SwiftPackageProduct(
        name: 'ProductName',
        targets: <String>[],
        libraryType: SwiftPackageLibraryType.dynamic,
      );
      expect(product.format(), '.library(name: "ProductName", type: .dynamic)');
    });

    testWithoutContext('with targets and libraryType', () {
      final SwiftPackageProduct product = SwiftPackageProduct(
        name: 'ProductName',
        targets: <String>['Target1', 'Target2'],
        libraryType: SwiftPackageLibraryType.dynamic,
      );
      expect(
        product.format(),
        '.library(name: "ProductName", type: .dynamic, targets: ["Target1", "Target2"])',
      );
    });
  });

  testWithoutContext('Format SwiftPackagePackageDependency', () {
    final SwiftPackagePackageDependency supportedPlatform = SwiftPackagePackageDependency(
      name: 'DependencyName',
      path: '/path/to/dependency',
    );
    expect(
      supportedPlatform.format(),
      '.package(name: "DependencyName", path: "/path/to/dependency")',
    );
  });

  group('Format SwiftPackageTarget', () {
    testWithoutContext('as default target with multiple SwiftPackageTargetDependency', () {
      final SwiftPackageTarget product = SwiftPackageTarget.defaultTarget(
        name: 'ProductName',
        dependencies: <SwiftPackageTargetDependency>[
          SwiftPackageTargetDependency.target(name: 'Dependency1'),
          SwiftPackageTargetDependency.product(
            name: 'Dependency2',
            packageName: 'Dependency2Package',
          ),
        ],
      );
      expect(product.format(), '''
.target(
            name: "ProductName",
            dependencies: [
                .target(name: "Dependency1"),
                .product(name: "Dependency2", package: "Dependency2Package")
            ]
        )''');
    });

    testWithoutContext('as default target with no SwiftPackageTargetDependency', () {
      final SwiftPackageTarget product = SwiftPackageTarget.defaultTarget(name: 'ProductName');
      expect(product.format(), '''
.target(
            name: "ProductName"
        )''');
    });

    testWithoutContext('as binaryTarget', () {
      final SwiftPackageTarget product = SwiftPackageTarget.binaryTarget(
        name: 'ProductName',
        relativePath: '/path/to/target',
      );
      expect(product.format(), '''
.binaryTarget(
            name: "ProductName",
            path: "/path/to/target"
        )''');
    });
  });

  group('Format SwiftPackageTargetDependency', () {
    testWithoutContext('with only name', () {
      final SwiftPackageTargetDependency targetDependency = SwiftPackageTargetDependency.target(
        name: 'DependencyName',
      );
      expect(targetDependency.format(), '                .target(name: "DependencyName")');
    });

    testWithoutContext('with name and package', () {
      final SwiftPackageTargetDependency targetDependency = SwiftPackageTargetDependency.product(
        name: 'DependencyName',
        packageName: 'PackageName',
      );
      expect(
        targetDependency.format(),
        '                .product(name: "DependencyName", package: "PackageName")',
      );
    });
  });
}
