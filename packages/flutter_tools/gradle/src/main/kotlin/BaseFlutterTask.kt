// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

package com.flutter.gradle

import java.nio.file.Paths
import org.gradle.api.DefaultTask
import org.gradle.api.GradleException
import org.gradle.api.file.FileCollection
import org.gradle.api.logging.LogLevel
import org.gradle.api.tasks.Input
import org.gradle.api.tasks.Internal
import org.gradle.api.tasks.OutputFiles
import org.gradle.api.tasks.Optional
import java.io.File

abstract class BaseFlutterTask : DefaultTask() {

    @Internal
    lateinit var flutterRoot: File

    @Internal
    lateinit var flutterExecutable: File

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
    lateinit var localEngineSrcPath:String

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
    lateinit var sourceDir: File

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
        depfiles += project.files("${intermediateDir}/flutter_build.d")
        return depfiles
    }

    fun buildBundle() {
        if (!sourceDir.isDirectory) {
            throw GradleException("Invalid Flutter source directory: $sourceDir")
        }

        intermediateDir.mkdirs()

        // Compute the rule name for flutter assemble. To speed up builds that contain
        // multiple ABIs, the target name is used to communicate which ones are required
        // rather than the TargetPlatform. This allows multiple builds to share the same
        // cache.
        val ruleNames: List<String> =
            when {
                buildMode == "debug" -> listOf("debug_android_application")
                deferredComponents -> targetPlatformValues.map {"android_aot_deferred_components_bundle_${buildMode}_$it"}
                else -> targetPlatformValues.map { "android_aot_bundle_${buildMode}_$it"}
            }
        project.exec {
            logging.captureStandardError(LogLevel.ERROR)
            executable(flutterExecutable.absolutePath)
            workingDir(sourceDir)
            localEngine?.let {
                args("--local-engine", localEngine)
                args ("--local-engine-src-path", localEngineSrcPath)
            }
            localEngineHost?.let {
                args ("--local-engine-host", localEngineHost)
            }
            if (verbose) {
                args("--verbose")
            } else {
                args("--quiet")
            }
            args("assemble")
            args("--no-version-check")
            args("--depfile", "${intermediateDir}/flutter_build.d")
            args("--output", "$intermediateDir")
            performanceMeasurementFile?.let {
                args("--performance-measurement-file=${performanceMeasurementFile}")
            }
            if (!fastStart || buildMode != "debug") {
                args("-dTargetFile=${targetPath}")
            } else {
                args("-dTargetFile=${Paths.get(flutterRoot.absolutePath, "examples", "splash", "lib", "main.dart")}")
            }
            args("-dTargetPlatform=android")
            args("-dBuildMode=${buildMode}")
            trackWidgetCreation?.let {
                args("-dTrackWidgetCreation=${trackWidgetCreation}")
            }
            splitDebugInfo?.let {
                args("-dSplitDebugInfo=${splitDebugInfo}")
            }
            if (treeShakeIcons == true) {
                args("-dTreeShakeIcons=true")
            }
            if (dartObfuscation == true) {
                args("-dDartObfuscation=true")
            }
            dartDefines?.let {
                args("--DartDefines=${dartDefines}")
            }
            bundleSkSLPath?.let {
                args("-dBundleSkSLPath=${bundleSkSLPath}")
            }
            codeSizeDirectory?.let {
                args("-dCodeSizeDirectory=${codeSizeDirectory}")
            }
            flavor?.let {
                args("-dFlavor=${flavor}")
            }
            extraGenSnapshotOptions?.let {
                args("--ExtraGenSnapshotOptions=${extraGenSnapshotOptions}")
            }
            frontendServerStarterPath?.let {
                args("-dFrontendServerStarterPath=${frontendServerStarterPath}")
            }
            extraFrontEndOptions?.let {
                args("--ExtraFrontEndOptions=${extraFrontEndOptions}")
            }

            args("-dAndroidArchs=${targetPlatformValues.joinToString(" ")}")
            args("-dMinSdkVersion=${minSdkVersion}")
            args(ruleNames)
        }
    }

}