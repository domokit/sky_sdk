package io.flutter.plugin.text;

import static io.flutter.Build.API_LEVELS;
import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.any;
import static org.mockito.Mockito.anyInt;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.annotation.TargetApi;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageItemInfo;
import android.content.pm.PackageManager;
import android.content.pm.ResolveInfo;
import androidx.annotation.RequiresApi;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.plugins.activity.ActivityPluginBinding;
import io.flutter.embedding.engine.systemchannels.ProcessTextChannel;
import io.flutter.plugin.common.BinaryMessenger;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugin.common.StandardMethodCodec;
import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.ArgumentCaptor;

@RunWith(AndroidJUnit4.class)
@TargetApi(API_LEVELS.API_24)
@RequiresApi(API_LEVELS.API_24)
public class ProcessTextPluginTest {

  private static void sendToBinaryMessageHandler(
      BinaryMessenger.BinaryMessageHandler binaryMessageHandler, String method, Object args) {
    MethodCall methodCall = new MethodCall(method, args);
    ByteBuffer encodedMethodCall = StandardMethodCodec.INSTANCE.encodeMethodCall(methodCall);
    binaryMessageHandler.onMessage(
        (ByteBuffer) encodedMethodCall.flip(), mock(BinaryMessenger.BinaryReply.class));
  }

  @SuppressWarnings("deprecation")
  // setMessageHandler is deprecated.
  @Test
  public void respondsToProcessTextChannelMessage() {
    ArgumentCaptor<BinaryMessenger.BinaryMessageHandler> binaryMessageHandlerCaptor =
        ArgumentCaptor.forClass(BinaryMessenger.BinaryMessageHandler.class);
    DartExecutor mockBinaryMessenger = mock(DartExecutor.class);
    ProcessTextChannel.ProcessTextMethodHandler mockHandler =
        mock(ProcessTextChannel.ProcessTextMethodHandler.class);
    PackageManager mockPackageManager = mock(PackageManager.class);
    ProcessTextChannel processTextChannel =
        new ProcessTextChannel(mockBinaryMessenger, mockPackageManager);

    processTextChannel.setMethodHandler(mockHandler);

    verify(mockBinaryMessenger, times(1))
        .setMessageHandler(any(String.class), binaryMessageHandlerCaptor.capture());

    BinaryMessenger.BinaryMessageHandler binaryMessageHandler =
        binaryMessageHandlerCaptor.getValue();

    sendToBinaryMessageHandler(binaryMessageHandler, "ProcessText.queryTextActions", null);

    verify(mockHandler).queryTextActions();
  }

  @SuppressWarnings("deprecation")
  // setMessageHandler is deprecated.
  @Test
  public void performQueryTextActions() {
    DartExecutor mockBinaryMessenger = mock(DartExecutor.class);
    PackageManager mockPackageManager = mock(PackageManager.class);
    ProcessTextChannel processTextChannel =
        new ProcessTextChannel(mockBinaryMessenger, mockPackageManager);

    // Set up mocked result for PackageManager.queryIntentActivities.
    ResolveInfo action1 = createFakeResolveInfo("Action1", mockPackageManager);
    ResolveInfo action2 = createFakeResolveInfo("Action2", mockPackageManager);
    List<ResolveInfo> infos = new ArrayList<ResolveInfo>(Arrays.asList(action1, action2));
    Intent intent = new Intent().setAction(Intent.ACTION_PROCESS_TEXT).setType("text/plain");
    when(mockPackageManager.queryIntentActivities(
            any(Intent.class), any(PackageManager.ResolveInfoFlags.class)))
        .thenReturn(infos);

    // ProcessTextPlugin should retrieve the mocked text actions.
    ProcessTextPlugin processTextPlugin = new ProcessTextPlugin(processTextChannel);
    Map<String, String> textActions = processTextPlugin.queryTextActions();
    final String action1Id = "mockActivityName.Action1";
    final String action2Id = "mockActivityName.Action2";
    assertEquals(textActions, Map.of(action1Id, "Action1", action2Id, "Action2"));
  }

  @SuppressWarnings("deprecation")
  // setMessageHandler is deprecated.
  @Test
  public void performProcessTextActionWithNoReturnedValue() {
    DartExecutor mockBinaryMessenger = mock(DartExecutor.class);
    PackageManager mockPackageManager = mock(PackageManager.class);
    ProcessTextChannel processTextChannel =
        new ProcessTextChannel(mockBinaryMessenger, mockPackageManager);

    // Set up mocked result for PackageManager.queryIntentActivities.
    ResolveInfo action1 = createFakeResolveInfo("Action1", mockPackageManager);
    ResolveInfo action2 = createFakeResolveInfo("Action2", mockPackageManager);
    List<ResolveInfo> infos = new ArrayList<ResolveInfo>(Arrays.asList(action1, action2));
    when(mockPackageManager.queryIntentActivities(
            any(Intent.class), any(PackageManager.ResolveInfoFlags.class)))
        .thenReturn(infos);

    // ProcessTextPlugin should retrieve the mocked text actions.
    ProcessTextPlugin processTextPlugin = new ProcessTextPlugin(processTextChannel);
    Map<String, String> textActions = processTextPlugin.queryTextActions();
    final String action1Id = "mockActivityName.Action1";
    final String action2Id = "mockActivityName.Action2";
    assertEquals(textActions, Map.of(action1Id, "Action1", action2Id, "Action2"));

    // Set up the activity binding.
    ActivityPluginBinding mockActivityPluginBinding = mock(ActivityPluginBinding.class);
    Activity mockActivity = mock(Activity.class);
    when(mockActivityPluginBinding.getActivity()).thenReturn(mockActivity);
    processTextPlugin.onAttachedToActivity(mockActivityPluginBinding);

    // Execute th first action.
    String textToBeProcessed = "Flutter!";
    MethodChannel.Result result = mock(MethodChannel.Result.class);
    processTextPlugin.processTextAction(action1Id, textToBeProcessed, false, result);

    // Activity.startActivityForResult should have been called.
    ArgumentCaptor<Intent> intentCaptor = ArgumentCaptor.forClass(Intent.class);
    verify(mockActivity, times(1)).startActivityForResult(intentCaptor.capture(), anyInt());
    Intent intent = intentCaptor.getValue();
    assertEquals(intent.getStringExtra(Intent.EXTRA_PROCESS_TEXT), textToBeProcessed);

    // Simulate an Android activity answer which does not return a value.
    Intent resultIntent = new Intent();
    processTextPlugin.onActivityResult(result.hashCode(), Activity.RESULT_OK, resultIntent);

    // Success with no returned value is expected.
    verify(result).success(null);
  }

  @SuppressWarnings("deprecation")
  // setMessageHandler is deprecated.
  @Test
  public void performProcessTextActionWithReturnedValue() {
    DartExecutor mockBinaryMessenger = mock(DartExecutor.class);
    PackageManager mockPackageManager = mock(PackageManager.class);
    ProcessTextChannel processTextChannel =
        new ProcessTextChannel(mockBinaryMessenger, mockPackageManager);

    // Set up mocked result for PackageManager.queryIntentActivities.
    ResolveInfo action1 = createFakeResolveInfo("Action1", mockPackageManager);
    ResolveInfo action2 = createFakeResolveInfo("Action2", mockPackageManager);
    List<ResolveInfo> infos = new ArrayList<ResolveInfo>(Arrays.asList(action1, action2));
    when(mockPackageManager.queryIntentActivities(
            any(Intent.class), any(PackageManager.ResolveInfoFlags.class)))
        .thenReturn(infos);

    // ProcessTextPlugin should retrieve the mocked text actions.
    ProcessTextPlugin processTextPlugin = new ProcessTextPlugin(processTextChannel);
    Map<String, String> textActions = processTextPlugin.queryTextActions();
    final String action1Id = "mockActivityName.Action1";
    final String action2Id = "mockActivityName.Action2";
    assertEquals(textActions, Map.of(action1Id, "Action1", action2Id, "Action2"));

    // Set up the activity binding.
    ActivityPluginBinding mockActivityPluginBinding = mock(ActivityPluginBinding.class);
    Activity mockActivity = mock(Activity.class);
    when(mockActivityPluginBinding.getActivity()).thenReturn(mockActivity);
    processTextPlugin.onAttachedToActivity(mockActivityPluginBinding);

    // Execute the first action.
    String textToBeProcessed = "Flutter!";
    MethodChannel.Result result = mock(MethodChannel.Result.class);
    processTextPlugin.processTextAction(action1Id, textToBeProcessed, false, result);

    // Activity.startActivityForResult should have been called.
    ArgumentCaptor<Intent> intentCaptor = ArgumentCaptor.forClass(Intent.class);
    verify(mockActivity, times(1)).startActivityForResult(intentCaptor.capture(), anyInt());
    Intent intent = intentCaptor.getValue();
    assertEquals(intent.getStringExtra(Intent.EXTRA_PROCESS_TEXT), textToBeProcessed);

    // Simulate an Android activity answer which returns a transformed text.
    String processedText = "Flutter!!!";
    Intent resultIntent = new Intent();
    resultIntent.putExtra(Intent.EXTRA_PROCESS_TEXT, processedText);
    processTextPlugin.onActivityResult(result.hashCode(), Activity.RESULT_OK, resultIntent);

    // Success with the transformed text is expected.
    verify(result).success(processedText);
  }

  @SuppressWarnings("deprecation")
  // setMessageHandler is deprecated.
  @Test
  public void doNotCrashOnNonRelatedActivityResult() {
    DartExecutor mockBinaryMessenger = mock(DartExecutor.class);
    PackageManager mockPackageManager = mock(PackageManager.class);
    ProcessTextChannel processTextChannel =
        new ProcessTextChannel(mockBinaryMessenger, mockPackageManager);

    // Set up mocked result for PackageManager.queryIntentActivities.
    ResolveInfo action1 = createFakeResolveInfo("Action1", mockPackageManager);
    ResolveInfo action2 = createFakeResolveInfo("Action2", mockPackageManager);
    List<ResolveInfo> infos = new ArrayList<ResolveInfo>(Arrays.asList(action1, action2));
    when(mockPackageManager.queryIntentActivities(
            any(Intent.class), any(PackageManager.ResolveInfoFlags.class)))
        .thenReturn(infos);

    // ProcessTextPlugin should retrieve the mocked text actions.
    ProcessTextPlugin processTextPlugin = new ProcessTextPlugin(processTextChannel);
    Map<String, String> textActions = processTextPlugin.queryTextActions();
    final String action1Id = "mockActivityName.Action1";
    final String action2Id = "mockActivityName.Action2";
    assertEquals(textActions, Map.of(action1Id, "Action1", action2Id, "Action2"));

    // Set up the activity binding.
    ActivityPluginBinding mockActivityPluginBinding = mock(ActivityPluginBinding.class);
    Activity mockActivity = mock(Activity.class);
    when(mockActivityPluginBinding.getActivity()).thenReturn(mockActivity);
    processTextPlugin.onAttachedToActivity(mockActivityPluginBinding);

    // Execute the first action.
    String textToBeProcessed = "Flutter!";
    MethodChannel.Result result = mock(MethodChannel.Result.class);
    processTextPlugin.processTextAction(action1Id, textToBeProcessed, false, result);

    // Activity.startActivityForResult should have been called.
    ArgumentCaptor<Intent> intentCaptor = ArgumentCaptor.forClass(Intent.class);
    verify(mockActivity, times(1)).startActivityForResult(intentCaptor.capture(), anyInt());
    Intent intent = intentCaptor.getValue();
    assertEquals(intent.getStringExtra(Intent.EXTRA_PROCESS_TEXT), textToBeProcessed);

    // Result to a request not sent by this plugin should be ignored.
    final int externalRequestCode = 42;
    processTextPlugin.onActivityResult(externalRequestCode, Activity.RESULT_OK, new Intent());

    // Simulate an Android activity answer which returns a transformed text.
    String processedText = "Flutter!!!";
    Intent resultIntent = new Intent();
    resultIntent.putExtra(Intent.EXTRA_PROCESS_TEXT, processedText);
    processTextPlugin.onActivityResult(result.hashCode(), Activity.RESULT_OK, resultIntent);

    // Success with the transformed text is expected.
    verify(result).success(processedText);
  }

  private ResolveInfo createFakeResolveInfo(String label, PackageManager mockPackageManager) {
    ResolveInfo resolveInfo = mock(ResolveInfo.class);
    ActivityInfo activityInfo = new ActivityInfo();
    when(resolveInfo.loadLabel(mockPackageManager)).thenReturn(label);

    // Use Java reflection to set required member variables.
    try {
      Field activityField = ResolveInfo.class.getDeclaredField("activityInfo");
      activityField.setAccessible(true);
      activityField.set(resolveInfo, activityInfo);
      Field packageNameField = PackageItemInfo.class.getDeclaredField("packageName");
      packageNameField.setAccessible(true);
      packageNameField.set(activityInfo, "mockActivityPackageName");
      Field nameField = PackageItemInfo.class.getDeclaredField("name");
      nameField.setAccessible(true);
      nameField.set(activityInfo, "mockActivityName." + label);
    } catch (Exception ex) {
      // Test will failed if reflection APIs throw.
    }

    return resolveInfo;
  }
}
