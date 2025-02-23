// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package com.flutter.gradle

import org.gradle.api.DefaultTask
import org.gradle.api.file.FileCollection
import org.gradle.api.logging.LogLevel
import org.gradle.api.tasks.Input
import org.gradle.api.tasks.Internal
import org.gradle.api.tasks.Optional
import org.gradle.api.tasks.OutputFiles
import java.io.File

abstract class BaseFlutterTask : DefaultTask() {
    @Internal
    lateinit var flutterRoot: File

    @Internal
    var flutterExecutable: File? = null

    @Input
    lateinit var buildMode: String

    @Input
    var minSdkVersion: Int? = null

    @Optional
    @Input
    var localEngine: String? = null

    @Optional
    @Input
    val localEngineHost: String? = null

    @Optional
    @Input
    var localEngineSrcPath: String? = null

    @Optional
    @Input
    var fastStart: Boolean = false

    @Input
    lateinit var targetPath: String

    @Optional
    @Input
    var verbose: Boolean = false

    @Optional
    @Input
    lateinit var fileSystemRoots: List<String>

    @Optional
    @Input
    lateinit var fileSystemScheme: String

    @Input
    var trackWidgetCreation: Boolean? = null

    @Optional
    @Input
    lateinit var targetPlatformValues: List<String>

    @Internal
    var sourceDir: File? = null

    @Internal
    lateinit var intermediateDir: File

    @Optional
    @Input
    var frontendServerStarterPath: String? = null

    @Optional
    @Input
    var extraFrontEndOptions: String? = null

    @Optional
    @Input
    var extraGenSnapshotOptions: String? = null

    @Optional
    @Input
    var splitDebugInfo: String? = null

    @Optional
    @Input
    var treeShakeIcons: Boolean? = null

    @Optional
    @Input
    var dartObfuscation: Boolean? = null

    @Optional
    @Input
    var dartDefines: String? = null

    @Optional
    @Input
    var bundleSkSLPath: String? = null

    @Optional
    @Input
    var codeSizeDirectory: String? = null

    @Optional
    @Input
    var performanceMeasurementFile: String? = null

    @Optional
    @Input
    var deferredComponents: Boolean = false

    @Optional
    @Input
    var validateDeferredComponents: Boolean? = null

    @Optional
    @Input
    var skipDependencyChecks: Boolean? = null

    @Optional
    @Input
    var flavor: String? = null

    @OutputFiles
    fun getDependenciesFiles(): FileCollection {
        var depfiles: FileCollection = project.files()

        // Includes all sources used in the flutter compilation.
        depfiles += project.files("$intermediateDir/flutter_build.d")
        return depfiles
    }

    // base flutter task will have a reference to a the util class (w/ business logic)
    // in build bundle, there will be something to check preconditions
    // maybe there's a function called generate args that returns a string
    //
    //
    fun buildBundle() {
        // might be passing in the Task instead of the parameter
        // below sourceDir is the parameter
        val helper: BaseFlutterTaskHelper = BaseFlutterTaskHelper(baseFlutterTask = this)
        helper.checkPreConditions()

        intermediateDir.mkdirs()

        // maybe i can pass in logging...?
        logging.captureStandardError(LogLevel.ERROR)
        project.exec(helper.createExecSpecActionFromTask())
    }
}
