#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERDISPLAYLINK_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERDISPLAYLINK_H_

#import <AppKit/AppKit.h>

@protocol FlutterDisplayLinkDelegate <NSObject>
- (void)onDisplayLink:(CFTimeInterval)timestamp targetTimestamp:(CFTimeInterval)targetTimestamp;
@end

/// Provides notifications of display refresh.
///
/// Internally FlutterDisplayLink will use at most one CVDisplayLink per
/// screen shared for all views belonging to that screen. This is necessary
/// because each CVDisplayLink comes with its own thread.
@interface FlutterDisplayLink : NSObject

/// Creates new instance tied to provided NSView. FlutterDisplayLink
/// will track view display changes transparently to synchronize
/// update with display refresh.
/// This function must be called on the main thread.
+ (instancetype)displayLinkWithView:(NSView*)view;

/// Delegate must be set on main thread. Delegate method will be called on
/// on display link thread.
@property(nonatomic, weak) id<FlutterDisplayLinkDelegate> delegate;

/// Pauses and resumes the display link. May be called from any thread.
@property(readwrite) BOOL paused;

/// Returns the nominal refresh period of the display to which the view
/// currently belongs (in seconds). If view does not belong to any display,
/// returns 0. Can be called from any thread.
@property(readonly) CFTimeInterval nominalOutputRefreshPeriod;

/// Invalidates the display link. Must be called on the main thread.
- (void)invalidate;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERDISPLAYLINK_H_
