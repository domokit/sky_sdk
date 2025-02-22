#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERRUNLOOP_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERRUNLOOP_H_

#import <Foundation/Foundation.h>

/**
 * Interface for scheduling tasks on the run loop.
 *
 * Main difference between using `FlutterRunLoop` to schedule tasks compared to
 * `dispatch_async` or `[NSRunLoop performBlock:]` is that `FlutterRunLoop`
 * schedules the task in both common run loop mode and a private run loop mode,
 * which allows it to run in mode where it only processes Flutter messages
 * (`[FlutterRunLoop pollFlutterMessagesOnce]`).
 */
@interface FlutterRunLoop : NSObject

/**
 * Ensures that the `FlutterRunLoop` for main thread is initialized. Only
 * needs to be called once and must be called on the main thread.
 */
+ (void)ensureMainLoopInitialized;

/**
 * Returns the `FlutterRunLoop` for the main thread.
 */
+ (FlutterRunLoop*)mainRunLoop;

/**
 * Schedules a block to be executed on the main thread.
 */
- (void)performBlock:(void (^)(void))block;

/**
 * Schedules a block to be executed on the main thread after a delay.
 */
- (void)performBlock:(void (^)(void))block afterDelay:(NSTimeInterval)delay;

/**
 * Executes single iteration of the run loop in the mode where only Flutter
 * messages are processed.
 */
- (void)pollFlutterMessagesOnce;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERRUNLOOP_H_
