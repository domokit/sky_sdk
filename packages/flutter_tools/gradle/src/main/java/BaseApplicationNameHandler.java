import com.android.build.api.dsl.ApplicationExtension;
import org.gradle.api.Project;

// TODO(gmackall): maybe migrate this to a package-level function when FGP conversion is done.
public class BaseApplicationNameHandler {

    public static final String DEFAULT_BASE_APPLICATION_NAME = "android.app.Application";
    public static final String GRADLE_BASE_APPLICATION_NAME_PROPERTY = "base-application-name";

    public static void setBaseName(Project project) {
        // Only set the base application name for apps, skip otherwise (LibraryExtension, DynamicFeatureExtension).
        ApplicationExtension androidComponentsExtension =
                project.getExtensions().findByType(ApplicationExtension.class);

        if (androidComponentsExtension == null) {
            return;
        }

        // Setting to android.app.Application is the same as omitting the attribute.
        String baseApplicationName = DEFAULT_BASE_APPLICATION_NAME;

        // Respect this property if it is set by the Flutter tool.
        if (project.hasProperty(GRADLE_BASE_APPLICATION_NAME_PROPERTY)) {
            baseApplicationName = project.property(GRADLE_BASE_APPLICATION_NAME_PROPERTY).toString();
        }

        androidComponentsExtension.getDefaultConfig().getManifestPlaceholders()
                .put("applicationName", baseApplicationName);
    }
}
