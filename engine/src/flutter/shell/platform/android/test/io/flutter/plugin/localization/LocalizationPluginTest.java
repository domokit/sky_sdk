package io.flutter.plugin.localization;

import static io.flutter.Build.API_LEVELS;
import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.any;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import android.annotation.TargetApi;
import android.content.Context;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.os.LocaleList;
import androidx.test.core.app.ApplicationProvider;
import androidx.test.ext.junit.runners.AndroidJUnit4;
import io.flutter.embedding.engine.FlutterJNI;
import io.flutter.embedding.engine.dart.DartExecutor;
import io.flutter.embedding.engine.systemchannels.LocalizationChannel;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import java.util.Locale;
import org.json.JSONException;
import org.json.JSONObject;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.robolectric.annotation.Config;

@Config(manifest = Config.NONE)
@RunWith(AndroidJUnit4.class)
@TargetApi(API_LEVELS.API_24) // LocaleList and scriptCode are API 24+.
public class LocalizationPluginTest {
  private final Context ctx = ApplicationProvider.getApplicationContext();

  // This test should be synced with the version for API 24.
  @Test
  @Config(sdk = API_LEVELS.API_26)
  public void computePlatformResolvedLocaleAPI26() {
    // --- Test Setup ---
    FlutterJNI flutterJNI = new FlutterJNI();

    Context context = mock(Context.class);
    Resources resources = mock(Resources.class);
    Configuration config = mock(Configuration.class);
    DartExecutor dartExecutor = mock(DartExecutor.class);
    LocaleList localeList =
        new LocaleList(new Locale("es", "MX"), new Locale("zh", "CN"), new Locale("en", "US"));
    when(context.getResources()).thenReturn(resources);
    when(resources.getConfiguration()).thenReturn(config);
    when(config.getLocales()).thenReturn(localeList);

    flutterJNI.setLocalizationPlugin(
        new LocalizationPlugin(context, new LocalizationChannel(dartExecutor)));

    // Empty supportedLocales.
    String[] supportedLocales = new String[] {};
    String[] result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 0);

    // Empty preferredLocales.
    supportedLocales =
        new String[] {
          "fr", "FR", "",
          "zh", "", "",
          "en", "CA", ""
        };
    localeList = new LocaleList();
    when(config.getLocales()).thenReturn(localeList);
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    // The first locale is default.
    assertEquals(result.length, 3);
    assertEquals(result[0], "fr");
    assertEquals(result[1], "FR");
    assertEquals(result[2], "");

    // Example from https://developer.android.com/guide/topics/resources/multilingual-support#postN
    supportedLocales =
        new String[] {
          "en", "", "",
          "de", "DE", "",
          "es", "ES", "",
          "fr", "FR", "",
          "it", "IT", ""
        };
    localeList = new LocaleList(new Locale("fr", "CH"));
    when(config.getLocales()).thenReturn(localeList);
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    // The call will use the new (> API 24) algorithm.
    assertEquals(result.length, 3);
    assertEquals(result[0], "fr");
    assertEquals(result[1], "FR");
    assertEquals(result[2], "");

    supportedLocales =
        new String[] {
          "en", "", "",
          "de", "DE", "",
          "es", "ES", "",
          "fr", "FR", "",
          "fr", "", "",
          "it", "IT", ""
        };
    localeList = new LocaleList(new Locale("fr", "CH"));
    when(config.getLocales()).thenReturn(localeList);
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    // The call will use the new (> API 24) algorithm.
    assertEquals(result.length, 3);
    assertEquals(result[0], "fr");
    assertEquals(result[1], "");
    assertEquals(result[2], "");

    // Example from https://developer.android.com/guide/topics/resources/multilingual-support#postN
    supportedLocales =
        new String[] {
          "en", "", "",
          "de", "DE", "",
          "es", "ES", "",
          "it", "IT", ""
        };
    localeList = new LocaleList(new Locale("fr", "CH"), new Locale("it", "CH"));
    when(config.getLocales()).thenReturn(localeList);
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    // The call will use the new (> API 24) algorithm.
    assertEquals(result.length, 3);
    assertEquals(result[0], "it");
    assertEquals(result[1], "IT");
    assertEquals(result[2], "");

    supportedLocales =
        new String[] {
          "zh", "CN", "Hans",
          "zh", "HK", "Hant",
        };
    localeList = new LocaleList(new Locale("zh", "CN"));
    when(config.getLocales()).thenReturn(localeList);
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 3);
    assertEquals(result[0], "zh");
    assertEquals(result[1], "CN");
    assertEquals(result[2], "Hans");
  }

  // This test should be synced with the version for API 26.
  @Test
  @Config(minSdk = API_LEVELS.API_24)
  public void computePlatformResolvedLocale_fromAndroidN() {
    // --- Test Setup ---
    FlutterJNI flutterJNI = new FlutterJNI();

    Context context = mock(Context.class);
    Resources resources = mock(Resources.class);
    Configuration config = mock(Configuration.class);
    DartExecutor dartExecutor = mock(DartExecutor.class);
    LocaleList localeList =
        new LocaleList(new Locale("es", "MX"), new Locale("zh", "CN"), new Locale("en", "US"));
    when(context.getResources()).thenReturn(resources);
    when(resources.getConfiguration()).thenReturn(config);
    when(config.getLocales()).thenReturn(localeList);

    flutterJNI.setLocalizationPlugin(
        new LocalizationPlugin(context, new LocalizationChannel(dartExecutor)));

    // Empty supportedLocales.
    String[] supportedLocales = new String[] {};
    String[] result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 0);

    // Empty preferredLocales.
    supportedLocales =
        new String[] {
          "fr", "FR", "",
          "zh", "", "",
          "en", "CA", ""
        };
    localeList = new LocaleList();
    when(config.getLocales()).thenReturn(localeList);
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    // The first locale is default.
    assertEquals(result.length, 3);
    assertEquals(result[0], "fr");
    assertEquals(result[1], "FR");
    assertEquals(result[2], "");

    // Example from https://developer.android.com/guide/topics/resources/multilingual-support#postN
    supportedLocales =
        new String[] {
          "en", "", "",
          "de", "DE", "",
          "es", "ES", "",
          "fr", "FR", "",
          "it", "IT", ""
        };
    localeList = new LocaleList(new Locale("fr", "CH"));
    when(config.getLocales()).thenReturn(localeList);
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    // The call will use the new (> API 24) algorithm.
    assertEquals(result.length, 3);
    assertEquals(result[0], "fr");
    assertEquals(result[1], "FR");
    assertEquals(result[2], "");

    supportedLocales =
        new String[] {
          "en", "", "",
          "de", "DE", "",
          "es", "ES", "",
          "fr", "FR", "",
          "fr", "", "",
          "it", "IT", ""
        };
    localeList = new LocaleList(new Locale("fr", "CH"));
    when(config.getLocales()).thenReturn(localeList);
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    // The call will use the new (> API 24) algorithm.
    assertEquals(result.length, 3);
    assertEquals(result[0], "fr");
    assertEquals(result[1], "");
    assertEquals(result[2], "");

    // Example from https://developer.android.com/guide/topics/resources/multilingual-support#postN
    supportedLocales =
        new String[] {
          "en", "", "",
          "de", "DE", "",
          "es", "ES", "",
          "it", "IT", ""
        };
    localeList = new LocaleList(new Locale("fr", "CH"), new Locale("it", "CH"));
    when(config.getLocales()).thenReturn(localeList);
    result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    // The call will use the new (> API 24) algorithm.
    assertEquals(result.length, 3);
    assertEquals(result[0], "it");
    assertEquals(result[1], "IT");
    assertEquals(result[2], "");
  }

  // Tests the legacy pre API 24 algorithm.
  @Test
  @Config(minSdk = API_LEVELS.API_21, maxSdk = API_LEVELS.API_23, qualifiers = "es-rMX")
  public void computePlatformResolvedLocale_emptySupportedLocales_beforeAndroidN() {
    FlutterJNI flutterJNI = new FlutterJNI();
    DartExecutor dartExecutor = mock(DartExecutor.class);
    flutterJNI.setLocalizationPlugin(
        new LocalizationPlugin(ctx, new LocalizationChannel(dartExecutor)));
    String[] supportedLocales = new String[] {};
    String[] result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 0);
  }

  @Test
  @Config(minSdk = API_LEVELS.API_21, maxSdk = API_LEVELS.API_23, qualifiers = "")
  public void computePlatformResolvedLocale_selectFirstLocaleWhenNoUserSetting_beforeAndroidN() {
    FlutterJNI flutterJNI = new FlutterJNI();
    DartExecutor dartExecutor = mock(DartExecutor.class);
    flutterJNI.setLocalizationPlugin(
        new LocalizationPlugin(ctx, new LocalizationChannel(dartExecutor)));
    String[] supportedLocales =
        new String[] {
          "fr", "FR", "",
          "zh", "", "",
          "en", "CA", ""
        };
    String[] result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 3);
    assertEquals(result[0], "fr");
    assertEquals(result[1], "FR");
    assertEquals(result[2], "");
  }

  @Test
  @Config(minSdk = API_LEVELS.API_21, maxSdk = API_LEVELS.API_23, qualifiers = "fr-rCH")
  public void computePlatformResolvedLocale_selectFirstLocaleWhenNoExactMatch_beforeAndroidN() {
    FlutterJNI flutterJNI = new FlutterJNI();
    DartExecutor dartExecutor = mock(DartExecutor.class);
    flutterJNI.setLocalizationPlugin(
        new LocalizationPlugin(ctx, new LocalizationChannel(dartExecutor)));
    // Example from https://developer.android.com/guide/topics/resources/multilingual-support#postN
    String[] supportedLocales =
        new String[] {
          "en", "", "",
          "de", "DE", "",
          "es", "ES", "",
          "fr", "FR", "",
          "it", "IT", ""
        };
    String[] result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 3);
    assertEquals(result[0], "en");
    assertEquals(result[1], "");
    assertEquals(result[2], "");
  }

  @Test
  @Config(minSdk = API_LEVELS.API_21, maxSdk = API_LEVELS.API_23, qualifiers = "it-rIT")
  public void computePlatformResolvedLocale_selectExactMatchLocale_beforeAndroidN() {
    FlutterJNI flutterJNI = new FlutterJNI();
    DartExecutor dartExecutor = mock(DartExecutor.class);
    flutterJNI.setLocalizationPlugin(
        new LocalizationPlugin(ctx, new LocalizationChannel(dartExecutor)));

    String[] supportedLocales =
        new String[] {
          "en", "", "",
          "de", "DE", "",
          "es", "ES", "",
          "fr", "FR", "",
          "it", "IT", ""
        };
    String[] result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 3);
    assertEquals(result[0], "it");
    assertEquals(result[1], "IT");
    assertEquals(result[2], "");
  }

  @Test
  @Config(minSdk = API_LEVELS.API_21, maxSdk = API_LEVELS.API_23, qualifiers = "fr-rCH")
  public void computePlatformResolvedLocale_selectOnlyLanguageLocale_beforeAndroidN() {
    FlutterJNI flutterJNI = new FlutterJNI();
    DartExecutor dartExecutor = mock(DartExecutor.class);
    flutterJNI.setLocalizationPlugin(
        new LocalizationPlugin(ctx, new LocalizationChannel(dartExecutor)));

    String[] supportedLocales =
        new String[] {
          "en", "", "",
          "de", "DE", "",
          "es", "ES", "",
          "fr", "FR", "",
          "fr", "", "",
          "it", "IT", ""
        };
    String[] result = flutterJNI.computePlatformResolvedLocale(supportedLocales);
    assertEquals(result.length, 3);
    assertEquals(result[0], "fr");
    assertEquals(result[1], "");
    assertEquals(result[2], "");
  }

  @Test
  public void localeFromString_languageOnly() {
    Locale locale = LocalizationPlugin.localeFromString("en");
    assertEquals(locale, new Locale("en"));
  }

  @Test
  public void localeFromString_languageAndCountry() {
    Locale locale = LocalizationPlugin.localeFromString("en-US");
    assertEquals(locale, new Locale("en", "US"));
  }

  @Test
  public void localeFromString_languageCountryAndVariant() {
    Locale locale = LocalizationPlugin.localeFromString("zh-Hans-CN");
    assertEquals(locale, new Locale("zh", "CN", "Hans"));
  }

  @Test
  public void localeFromString_underscore() {
    Locale locale = LocalizationPlugin.localeFromString("zh_Hans_CN");
    assertEquals(locale, new Locale("zh", "CN", "Hans"));
  }

  @Test
  public void localeFromString_additionalVariantsAreIgnored() {
    Locale locale = LocalizationPlugin.localeFromString("de-DE-u-co-phonebk");
    assertEquals(locale, new Locale("de", "DE"));
  }

  @Test
  public void getStringResource_withoutLocale() throws JSONException {
    Context context = mock(Context.class);
    Resources resources = mock(Resources.class);
    DartExecutor dartExecutor = mock(DartExecutor.class);
    LocalizationChannel localizationChannel = new LocalizationChannel(dartExecutor);
    LocalizationPlugin plugin = new LocalizationPlugin(context, localizationChannel);

    MethodChannel.Result mockResult = mock(MethodChannel.Result.class);

    String fakePackageName = "package_name";
    String fakeKey = "test_key";
    int fakeId = 123;

    when(context.getPackageName()).thenReturn(fakePackageName);
    when(context.getResources()).thenReturn(resources);
    when(resources.getIdentifier(fakeKey, "string", fakePackageName)).thenReturn(fakeId);
    when(resources.getString(fakeId)).thenReturn("test_value");

    JSONObject param = new JSONObject();
    param.put("key", fakeKey);

    localizationChannel.handler.onMethodCall(
        new MethodCall("Localization.getStringResource", param), mockResult);

    verify(mockResult).success("test_value");
  }

  @Test
  public void getStringResource_withLocale() throws JSONException {
    Context context = mock(Context.class);
    Context localContext = mock(Context.class);
    Resources resources = mock(Resources.class);
    Resources localResources = mock(Resources.class);
    Configuration configuration = new Configuration();
    DartExecutor dartExecutor = mock(DartExecutor.class);
    LocalizationChannel localizationChannel = new LocalizationChannel(dartExecutor);
    LocalizationPlugin plugin = new LocalizationPlugin(context, localizationChannel);

    MethodChannel.Result mockResult = mock(MethodChannel.Result.class);

    String fakePackageName = "package_name";
    String fakeKey = "test_key";
    int fakeId = 123;

    when(context.getPackageName()).thenReturn(fakePackageName);
    when(context.createConfigurationContext(any())).thenReturn(localContext);
    when(context.getResources()).thenReturn(resources);
    when(localContext.getResources()).thenReturn(localResources);
    when(resources.getConfiguration()).thenReturn(configuration);
    when(localResources.getIdentifier(fakeKey, "string", fakePackageName)).thenReturn(fakeId);
    when(localResources.getString(fakeId)).thenReturn("test_value");

    JSONObject param = new JSONObject();
    param.put("key", fakeKey);
    param.put("locale", "en-US");

    localizationChannel.handler.onMethodCall(
        new MethodCall("Localization.getStringResource", param), mockResult);

    verify(mockResult).success("test_value");
  }

  @Test
  public void getStringResource_nonExistentKey() throws JSONException {
    Context context = mock(Context.class);
    Resources resources = mock(Resources.class);
    DartExecutor dartExecutor = mock(DartExecutor.class);
    LocalizationChannel localizationChannel = new LocalizationChannel(dartExecutor);
    LocalizationPlugin plugin = new LocalizationPlugin(context, localizationChannel);

    MethodChannel.Result mockResult = mock(MethodChannel.Result.class);

    String fakePackageName = "package_name";
    String fakeKey = "test_key";

    when(context.getPackageName()).thenReturn(fakePackageName);
    when(context.getResources()).thenReturn(resources);
    when(resources.getIdentifier(fakeKey, "string", fakePackageName))
        .thenReturn(0); // 0 means not exist

    JSONObject param = new JSONObject();
    param.put("key", fakeKey);

    localizationChannel.handler.onMethodCall(
        new MethodCall("Localization.getStringResource", param), mockResult);

    verify(mockResult).success(null);
  }
}
