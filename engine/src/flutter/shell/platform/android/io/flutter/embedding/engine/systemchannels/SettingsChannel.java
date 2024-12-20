package io.flutter.embedding.engine.systemchannels;

import static io.flutter.Build.API_LEVELS;

import android.annotation.SuppressLint;
import android.os.Build;
import android.util.DisplayMetrics;
import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.UiThread;
import androidx.annotation.VisibleForTesting;
import io.flutter.Log;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.plugin.common.BasicMessageChannel;
import io.flutter.plugin.common.JSONMessageCodec;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;

public class SettingsChannel {
  private static final String TAG = "SettingsChannel";

  public static final String CHANNEL_NAME = "flutter/settings";
  private static final String TEXT_SCALE_FACTOR = "textScaleFactor";
  private static final String NATIVE_SPELL_CHECK_SERVICE_DEFINED = "nativeSpellCheckServiceDefined";
  private static final String BRIEFLY_SHOW_PASSWORD = "brieflyShowPassword";
  private static final String ALWAYS_USE_24_HOUR_FORMAT = "alwaysUse24HourFormat";
  private static final String PLATFORM_BRIGHTNESS = "platformBrightness";
  private static final String CONFIGURATION_ID = "configurationId";

  // When hasNonlinearTextScalingSupport() returns false, this will not be initialized.
  private static final ConfigurationQueue CONFIGURATION_QUEUE = new ConfigurationQueue();

  @NonNull public final BasicMessageChannel<Object> channel;

  public SettingsChannel(@NonNull DartExecutor dartExecutor) {
    this.channel = new BasicMessageChannel<>(dartExecutor, CHANNEL_NAME, JSONMessageCodec.INSTANCE);
  }

  @SuppressLint("AnnotateVersionCheck")
  public static boolean hasNonlinearTextScalingSupport() {
    return Build.VERSION.SDK_INT >= API_LEVELS.API_34;
  }

  // This method will only be called on Flutter's UI thread.
  public static DisplayMetrics getPastDisplayMetrics(int configId) {
    assert hasNonlinearTextScalingSupport();
    final ConfigurationQueue.SentConfiguration configuration =
        CONFIGURATION_QUEUE.getConfiguration(configId);
    return configuration == null ? null : configuration.displayMetrics;
  }

  @NonNull
  public MessageBuilder startMessage() {
    return new MessageBuilder(channel);
  }

  public static class MessageBuilder {
    @NonNull private final BasicMessageChannel<Object> channel;
    @NonNull private Map<String, Object> message = new HashMap<>();
    @Nullable private DisplayMetrics displayMetrics;

    MessageBuilder(@NonNull BasicMessageChannel<Object> channel) {
      this.channel = channel;
    }

    @NonNull
    public MessageBuilder setDisplayMetrics(@NonNull DisplayMetrics displayMetrics) {
      this.displayMetrics = displayMetrics;
      return this;
    }

    @NonNull
    public MessageBuilder setTextScaleFactor(float textScaleFactor) {
      message.put(TEXT_SCALE_FACTOR, textScaleFactor);
      return this;
    }

    @NonNull
    public MessageBuilder setNativeSpellCheckServiceDefined(
        boolean nativeSpellCheckServiceDefined) {
      message.put(NATIVE_SPELL_CHECK_SERVICE_DEFINED, nativeSpellCheckServiceDefined);
      return this;
    }

    @NonNull
    public MessageBuilder setBrieflyShowPassword(@NonNull boolean brieflyShowPassword) {
      message.put(BRIEFLY_SHOW_PASSWORD, brieflyShowPassword);
      return this;
    }

    @NonNull
    public MessageBuilder setUse24HourFormat(boolean use24HourFormat) {
      message.put(ALWAYS_USE_24_HOUR_FORMAT, use24HourFormat);
      return this;
    }

    @NonNull
    public MessageBuilder setPlatformBrightness(@NonNull PlatformBrightness brightness) {
      message.put(PLATFORM_BRIGHTNESS, brightness.name);
      return this;
    }

    public void send() {
      Log.v(
          TAG,
          "Sending message: \n"
              + "textScaleFactor: "
              + message.get(TEXT_SCALE_FACTOR)
              + "\n"
              + "alwaysUse24HourFormat: "
              + message.get(ALWAYS_USE_24_HOUR_FORMAT)
              + "\n"
              + "platformBrightness: "
              + message.get(PLATFORM_BRIGHTNESS));
      final DisplayMetrics metrics = this.displayMetrics;
      if (!hasNonlinearTextScalingSupport() || metrics == null) {
        channel.send(message);
        return;
      }
      final ConfigurationQueue.SentConfiguration sentConfiguration =
          new ConfigurationQueue.SentConfiguration(metrics);
      final BasicMessageChannel.Reply deleteCallback =
          CONFIGURATION_QUEUE.enqueueConfiguration(sentConfiguration);
      message.put(CONFIGURATION_ID, sentConfiguration.generationNumber);
      channel.send(message, deleteCallback);
    }
  }

  /**
   * The brightness mode of the host platform.
   *
   * <p>The {@code name} property is the serialized representation of each brightness mode when
   * communicated via message channel.
   */
  public enum PlatformBrightness {
    light("light"),
    dark("dark");

    @NonNull public String name;

    PlatformBrightness(@NonNull String name) {
      this.name = name;
    }
  }

  /**
   * A FIFO queue that maintains generations of configurations that are potentially used by the
   * Flutter application.
   *
   * <p>Platform configurations needed by the Flutter app (for example, text scale factor) are
   * retrived on the platform thread, serialized and sent to the Flutter application running on the
   * Flutter UI thread. However, configurations exposed as functions that take parameters are
   * typically not serializable. To allow the Flutter app to access these configurations, one
   * possible solution is to create dart bindings that allow the Flutter framework to invoke these
   * functions via JNI synchronously. To ensure the serialized configuration and these functions
   * represent the same set of configurations at any given time, a "generation" id is used in these
   * synchronous calls, to keep them consistent with the serialized configuration that the Flutter
   * app most recently received and is currently using.
   *
   * <p>A unique generation identifier is generated by the {@link SettingsChannel} and associated
   * with a configuration when it sends a serialized configuration to the Flutter framework. This
   * queue keeps different generations of configurations that could be used by the Flutter
   * framework, and cleans up old configurations that the Flutter framework no longer uses. When the
   * Flutter framework invokes a function to access the configuration with a generation identifier,
   * this queue finds the configuration with that identifier and also cleans up configurations that
   * are no longer needed.
   *
   * <p>This mechanism is only needed because {@code TypedValue#applyDimension} does not take the
   * current text scale factor as an input. Once the AndroidX API that allows us to query the scaled
   * font size with a pure function is available, we can scrap this class and make the
   * implementation much simpler.
   */
  @VisibleForTesting
  public static class ConfigurationQueue {
    private final ConcurrentLinkedQueue<SentConfiguration> sentQueue =
        new ConcurrentLinkedQueue<>();

    // The current SentConfiguration the Flutter application is using, according
    // to the most recent getConfiguration call.
    //
    // This instance variable will only be accessed by getConfiguration, on
    // Flutter's UI thread.
    private SentConfiguration currentConfiguration;

    /**
     * Returns the {@link SentConfiguration} associated with the given {@code configGeneration}, and
     * removes configurations older than the returned configurations from the queue as they are no
     * longer needed.
     */
    public SentConfiguration getConfiguration(int configGeneration) {
      if (currentConfiguration == null) {
        currentConfiguration = sentQueue.poll();
      }

      // Remove the older entries, up to the entry associated with
      // configGeneration. Here we assume the generationNumber never overflows.
      while (currentConfiguration != null
          && currentConfiguration.generationNumber < configGeneration) {
        currentConfiguration = sentQueue.poll();
      }

      if (currentConfiguration == null) {
        Log.e(
            TAG,
            "Cannot find config with generation: "
                + String.valueOf(configGeneration)
                + ", after exhausting the queue.");
        return null;
      } else if (currentConfiguration.generationNumber != configGeneration) {
        Log.e(
            TAG,
            "Cannot find config with generation: "
                + String.valueOf(configGeneration)
                + ", the oldest config is now: "
                + String.valueOf(currentConfiguration.generationNumber));
        return null;
      }
      return currentConfiguration;
    }

    private SentConfiguration previousEnqueuedConfiguration;

    /**
     * Adds the most recently sent {@link SentConfiguration} to the queue.
     *
     * @return a {@link BasicMessageChannel.Reply} whose {@code reply} method must be called when
     *     the embedder receives the reply for the sent configuration, to properly clean up older
     *     configurations in the queue.
     */
    @UiThread
    @Nullable
    public BasicMessageChannel.Reply enqueueConfiguration(SentConfiguration config) {
      sentQueue.add(config);
      final SentConfiguration configurationToRemove = previousEnqueuedConfiguration;
      previousEnqueuedConfiguration = config;
      return configurationToRemove == null
          ? null
          : new BasicMessageChannel.Reply() {
            @UiThread
            @Override
            public void reply(Object reply) {
              // Removes the SentConfiguration sent right before `config`. Since
              // platform messages are also FIFO older messages will be removed
              // before newer ones.
              sentQueue.remove(configurationToRemove);
              if (!sentQueue.isEmpty()) {
                Log.e(
                    TAG,
                    "The queue becomes empty after removing config generation "
                        + String.valueOf(configurationToRemove.generationNumber));
              }
            }
          };
    }

    public static class SentConfiguration {
      private static int nextConfigGeneration = Integer.MIN_VALUE;

      @NonNull public final int generationNumber;
      @NonNull private final DisplayMetrics displayMetrics;

      public SentConfiguration(@NonNull DisplayMetrics displayMetrics) {
        this.generationNumber = nextConfigGeneration++;
        this.displayMetrics = displayMetrics;
      }
    }
  }
}
