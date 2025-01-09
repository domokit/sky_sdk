import Flutter
import UIKit
import XCTest
{{#withPlatformChannelPluginHook}}

{{#withSwiftPackageManager}}
// If your plugin has been explicitly set to "type: .dynamic" in the Package.swift,
// you will need to add your plugin as a dependency of RunnerTests within Xcode.
{{/withSwiftPackageManager}}

@testable import {{pluginProjectName}}

// This demonstrates a simple unit test of the Swift portion of this plugin's implementation.
//
// See https://developer.apple.com/documentation/xctest for more information about using XCTest.
{{/withPlatformChannelPluginHook}}

class RunnerTests: XCTestCase {

{{#withPlatformChannelPluginHook}}
  func testGetPlatformVersion() {
    let plugin = {{pluginClass}}()

    let call = FlutterMethodCall(methodName: "getPlatformVersion", arguments: [])

    let resultExpectation = expectation(description: "result block must be called.")
    plugin.handle(call) { result in
      XCTAssertEqual(result as! String, "iOS " + UIDevice.current.systemVersion)
      resultExpectation.fulfill()
    }
    waitForExpectations(timeout: 1)
  }
{{/withPlatformChannelPluginHook}}
{{^withPlatformChannelPluginHook}}
  func testExample() {
    // If you add code to the Runner application, consider adding tests here.
    // See https://developer.apple.com/documentation/xctest for more information about using XCTest.
  }
{{/withPlatformChannelPluginHook}}

}
