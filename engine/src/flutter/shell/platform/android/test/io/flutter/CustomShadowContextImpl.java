package io.flutter;

import android.content.Context;
import org.robolectric.annotation.Implementation;
import org.robolectric.annotation.Implements;
import org.robolectric.shadows.ShadowContextImpl;

@Implements(className = ShadowContextImpl.CLASS_NAME)
public class CustomShadowContextImpl extends ShadowContextImpl {
  public static final String CLASS_NAME = "android.app.ContextImpl";

  @Implementation
  @Override
  public final Object getSystemService(String name) {
    if (name == Context.TEXT_SERVICES_MANAGER_SERVICE) {
      return null;
    }
    return super.getSystemService(name);
  }
}
