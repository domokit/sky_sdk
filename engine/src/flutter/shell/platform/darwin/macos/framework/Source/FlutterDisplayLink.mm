#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterDisplayLink.h"

#include "flutter/fml/logging.h"

#include <algorithm>
#include <optional>
#include <thread>
#include <vector>

// Note on thread safety and locking:
//
// There are three mutexes used within the scope of this file:
// - CVDisplayLink internal mutex. This is locked during every CVDisplayLink method
//   and is also held while display link calls the output handler.
// - DisplayLinkManager mutex.
// - _FlutterDisplayLink mutex (through @synchronized blocks).
//
// Special care must be taken to avoid deadlocks. Because CVDisplayLink holds the
// mutex for the entire duration of the output handler, it is necessary for
// DisplayLinkManager to not call any CVDisplayLink methods while holding its
// mutex. Instead it must retain the display link instance and then call the
// appropriate method with the mutex unlocked.
//
// Similarly _FlutterDisplayLink must not call any DisplayLinkManager methods
// within the @synchronized block.

@class _FlutterDisplayLinkView;

@interface _FlutterDisplayLink : FlutterDisplayLink {
  _FlutterDisplayLinkView* _view;
  std::optional<CGDirectDisplayID> _display_id;
  BOOL _paused;
}

- (void)didFireWithTimestamp:(CFTimeInterval)timestamp
             targetTimestamp:(CFTimeInterval)targetTimestamp;

@end

namespace {
class DisplayLinkManager {
 public:
  static DisplayLinkManager& Instance() {
    static DisplayLinkManager instance;
    return instance;
  }

  void UnregisterDisplayLink(_FlutterDisplayLink* display_link);
  void RegisterDisplayLink(_FlutterDisplayLink* display_link, CGDirectDisplayID display_id);
  void PausedDidChange(_FlutterDisplayLink* display_link);
  CFTimeInterval GetNominalOutputPeriod(CGDirectDisplayID display_id);

 private:
  void OnDisplayLink(CVDisplayLinkRef display_link,
                     const CVTimeStamp* in_now,
                     const CVTimeStamp* in_output_time,
                     CVOptionFlags flags_in,
                     CVOptionFlags* flags_out);

  struct ScreenEntry {
    CGDirectDisplayID display_id;
    std::vector<_FlutterDisplayLink*> clients;

    /// Display link for this screen. It is not safe to call display link methods
    /// on this object while holding the mutex. Instead the instance should be
    /// retained, mutex unlocked and then released.
    CVDisplayLinkRef display_link_locked;

    bool ShouldBeRunning() {
      return std::any_of(clients.begin(), clients.end(),
                         [](FlutterDisplayLink* link) { return !link.paused; });
    }
  };
  std::vector<ScreenEntry> entries_;
  std::mutex mutex_;
};

void RunOrStopDisplayLink(CVDisplayLinkRef display_link, bool should_be_running) {
  bool is_running = CVDisplayLinkIsRunning(display_link);
  if (should_be_running && !is_running) {
    if (CVDisplayLinkStart(display_link) == kCVReturnError) {
      // CVDisplayLinkStart will fail if it was called from the display link thread.
      // The problem is that it CVDisplayLinkStop doesn't clean the pthread_t value in the display
      // link itself. If the display link is started and stopped before before the UI thread is
      // started (*), pthread_self() of the UI thread may have same value as the one stored in
      // CVDisplayLink. Because this can happen at most once starting the display link from a
      // temporary thread is a reasonable workaround.
      //
      // (*) Display link is started before UI thread because FlutterVSyncWaiter will run display
      // link for one tick at the beginning to determine vsync phase.
      //
      // http://www.openradar.me/radar?id=5520107644125184
      CVDisplayLinkRef retained = CVDisplayLinkRetain(display_link);
      [NSThread detachNewThreadWithBlock:^{
        CVDisplayLinkStart(retained);
        CVDisplayLinkRelease(retained);
      }];
    }
  } else if (!should_be_running && is_running) {
    CVDisplayLinkStop(display_link);
  }
}

void DisplayLinkManager::UnregisterDisplayLink(_FlutterDisplayLink* display_link) {
  std::unique_lock<std::mutex> lock(mutex_);
  for (auto entry = entries_.begin(); entry != entries_.end(); ++entry) {
    auto it = std::find(entry->clients.begin(), entry->clients.end(), display_link);
    if (it != entry->clients.end()) {
      entry->clients.erase(it);
      if (entry->clients.empty()) {
        // Erasing the entry - take the display link instance and stop / release it
        // outside of the mutex.
        CVDisplayLinkRef display_link = entry->display_link_locked;
        entries_.erase(entry);
        lock.unlock();
        CVDisplayLinkStop(display_link);
        CVDisplayLinkRelease(display_link);
      } else {
        // Update the display link state outside of the mutex.
        bool should_be_running = entry->ShouldBeRunning();
        CVDisplayLinkRef display_link = CVDisplayLinkRetain(entry->display_link_locked);
        lock.unlock();
        RunOrStopDisplayLink(display_link, should_be_running);
        CVDisplayLinkRelease(display_link);
      }
      return;
    }
  }
}

void DisplayLinkManager::RegisterDisplayLink(_FlutterDisplayLink* display_link,
                                             CGDirectDisplayID display_id) {
  std::unique_lock<std::mutex> lock(mutex_);
  for (ScreenEntry& entry : entries_) {
    if (entry.display_id == display_id) {
      entry.clients.push_back(display_link);
      bool should_be_running = entry.ShouldBeRunning();
      CVDisplayLinkRef display_link = CVDisplayLinkRetain(entry.display_link_locked);
      lock.unlock();
      RunOrStopDisplayLink(display_link, should_be_running);
      CVDisplayLinkRelease(display_link);
      return;
    }
  }

  ScreenEntry entry;
  entry.display_id = display_id;
  entry.clients.push_back(display_link);
  CVDisplayLinkCreateWithCGDisplay(display_id, &entry.display_link_locked);

  CVDisplayLinkSetOutputHandler(
      entry.display_link_locked,
      ^(CVDisplayLinkRef display_link, const CVTimeStamp* in_now, const CVTimeStamp* in_output_time,
        CVOptionFlags flags_in, CVOptionFlags* flags_out) {
        OnDisplayLink(display_link, in_now, in_output_time, flags_in, flags_out);
        return 0;
      });

  // This is a new display link so it is safe to start it with mutex held.
  bool should_be_running = entry.ShouldBeRunning();
  RunOrStopDisplayLink(entry.display_link_locked, should_be_running);
  entries_.push_back(entry);
}

void DisplayLinkManager::PausedDidChange(_FlutterDisplayLink* display_link) {
  std::unique_lock<std::mutex> lock(mutex_);
  for (ScreenEntry& entry : entries_) {
    auto it = std::find(entry.clients.begin(), entry.clients.end(), display_link);
    if (it != entry.clients.end()) {
      bool running = entry.ShouldBeRunning();
      CVDisplayLinkRef display_link = CVDisplayLinkRetain(entry.display_link_locked);
      lock.unlock();
      RunOrStopDisplayLink(display_link, running);
      CVDisplayLinkRelease(display_link);
      return;
    }
  }
}

CFTimeInterval DisplayLinkManager::GetNominalOutputPeriod(CGDirectDisplayID display_id) {
  std::unique_lock<std::mutex> lock(mutex_);
  for (ScreenEntry& entry : entries_) {
    if (entry.display_id == display_id) {
      CVDisplayLinkRef display_link = CVDisplayLinkRetain(entry.display_link_locked);
      lock.unlock();
      CVTime latency = CVDisplayLinkGetNominalOutputVideoRefreshPeriod(display_link);
      CVDisplayLinkRelease(display_link);
      return (CFTimeInterval)latency.timeValue / (CFTimeInterval)latency.timeScale;
    }
  }
  return 0;
}

void DisplayLinkManager::OnDisplayLink(CVDisplayLinkRef display_link,
                                       const CVTimeStamp* in_now,
                                       const CVTimeStamp* in_output_time,
                                       CVOptionFlags flags_in,
                                       CVOptionFlags* flags_out) {
  // Hold the mutex only while copying clients.
  std::vector<_FlutterDisplayLink*> clients;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (ScreenEntry& entry : entries_) {
      if (entry.display_link_locked == display_link) {
        clients = entry.clients;
        break;
      }
    }
  }

  CFTimeInterval timestamp = (CFTimeInterval)in_now->hostTime / CVGetHostClockFrequency();
  CFTimeInterval target_timestamp =
      (CFTimeInterval)in_output_time->hostTime / CVGetHostClockFrequency();

  for (_FlutterDisplayLink* client : clients) {
    [client didFireWithTimestamp:timestamp targetTimestamp:target_timestamp];
  }
}
}  // namespace

@interface _FlutterDisplayLinkView : NSView {
}

@end

static NSString* const kFlutterDisplayLinkViewDidMoveToWindow =
    @"FlutterDisplayLinkViewDidMoveToWindow";

@implementation _FlutterDisplayLinkView

- (void)viewDidMoveToWindow {
  [super viewDidMoveToWindow];
  [[NSNotificationCenter defaultCenter] postNotificationName:kFlutterDisplayLinkViewDidMoveToWindow
                                                      object:self];
}

@end

@implementation _FlutterDisplayLink

@synthesize delegate = _delegate;

- (instancetype)initWithView:(NSView*)view {
  FML_DCHECK([NSThread isMainThread]);
  if (self = [super init]) {
    self->_view = [[_FlutterDisplayLinkView alloc] initWithFrame:CGRectZero];
    [view addSubview:self->_view];
    _paused = YES;
    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(viewDidChangeWindow:)
                                                 name:kFlutterDisplayLinkViewDidMoveToWindow
                                               object:self->_view];
    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(windowDidChangeScreen:)
                                                 name:NSWindowDidChangeScreenNotification
                                               object:nil];
    [self updateScreen];
  }
  return self;
}

- (void)invalidate {
  @synchronized(self) {
    FML_DCHECK([NSThread isMainThread]);
    // Unregister observer before removing the view to ensure
    // that the viewDidChangeWindow notification is not received
    // while in @synchronized block.
    [[NSNotificationCenter defaultCenter] removeObserver:self];
    [_view removeFromSuperview];
    _view = nil;
    _delegate = nil;
  }
  DisplayLinkManager::Instance().UnregisterDisplayLink(self);
}

- (void)updateScreen {
  DisplayLinkManager::Instance().UnregisterDisplayLink(self);
  std::optional<CGDirectDisplayID> displayId;
  @synchronized(self) {
    NSScreen* screen = _view.window.screen;
    if (screen != nil) {
      // https://developer.apple.com/documentation/appkit/nsscreen/1388360-devicedescription?language=objc
      _display_id = (CGDirectDisplayID)[
          [[screen deviceDescription] objectForKey:@"NSScreenNumber"] unsignedIntValue];
    } else {
      _display_id = std::nullopt;
    }
    displayId = _display_id;
  }
  if (displayId.has_value()) {
    DisplayLinkManager::Instance().RegisterDisplayLink(self, *displayId);
  }
}

- (void)viewDidChangeWindow:(NSNotification*)notification {
  NSView* view = notification.object;
  if (_view == view) {
    [self updateScreen];
  }
}

- (void)windowDidChangeScreen:(NSNotification*)notification {
  NSWindow* window = notification.object;
  if (_view.window == window) {
    [self updateScreen];
  }
}

- (void)didFireWithTimestamp:(CFTimeInterval)timestamp
             targetTimestamp:(CFTimeInterval)targetTimestamp {
  @synchronized(self) {
    if (!_paused) {
      id<FlutterDisplayLinkDelegate> delegate = _delegate;
      [delegate onDisplayLink:timestamp targetTimestamp:targetTimestamp];
    }
  }
}

- (BOOL)paused {
  @synchronized(self) {
    return _paused;
  }
}

- (void)setPaused:(BOOL)paused {
  @synchronized(self) {
    if (_paused == paused) {
      return;
    }
    _paused = paused;
  }
  DisplayLinkManager::Instance().PausedDidChange(self);
}

- (CFTimeInterval)nominalOutputRefreshPeriod {
  CGDirectDisplayID display_id;
  @synchronized(self) {
    if (_display_id.has_value()) {
      display_id = *_display_id;
    } else {
      return 0;
    }
  }
  return DisplayLinkManager::Instance().GetNominalOutputPeriod(display_id);
}

@end

@implementation FlutterDisplayLink
+ (instancetype)displayLinkWithView:(NSView*)view {
  return [[_FlutterDisplayLink alloc] initWithView:view];
}

- (void)invalidate {
  [self doesNotRecognizeSelector:_cmd];
}

@end
