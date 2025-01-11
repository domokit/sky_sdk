// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

apply<FlutterPluginKts>()

class FlutterPluginKts : Plugin<Project> {
    override fun apply(project: Project) {
        // Use withGroovyBuilder and getProperty() to access Groovy metaprogramming.
        project.withGroovyBuilder {
            getProperty("android").withGroovyBuilder {
                getProperty("defaultConfig").withGroovyBuilder {
                    var baseApplicationName: String = "android.app.Application"
                    if (project.hasProperty("base-application-name")) {
                        baseApplicationName = project.property("base-application-name").toString()
                    }
                    // Setting to android.app.Application is the same as omitting the attribute.
                    getProperty("manifestPlaceholders").withGroovyBuilder {
                        setProperty("applicationName", baseApplicationName)
                    }
                }
            }
        }
    }
}
