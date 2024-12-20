// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

type JSCompileTarget = "dart2js" | "dartdevc";
type WasmCompileTarget = "dart2wasm";

export type CompileTarget = JSCompileTarget | WasmCompileTarget;

export type WebRenderer =
  "auto" |
  "html" |
  "canvaskit" |
  "skwasm";

interface ApplicationBuildBase {
  renderer: WebRenderer;
}

export interface JSApplicationBuild extends ApplicationBuildBase {
  compileTarget: JSCompileTarget;
  mainJsPath: string;
}

export interface WasmApplicationBuild extends ApplicationBuildBase {
  compileTarget: WasmCompileTarget;
  mainWasmPath: string;
  jsSupportRuntimePath: string;
}

export type ApplicationBuild = JSApplicationBuild | WasmApplicationBuild;

export interface BuildConfig {
  serviceWorkerVersion: string;
  engineRevision: string;
  useLocalCanvasKit: bool?;
  builds: ApplicationBuild[];
}

export interface BrowserEnvironment {
  hasImageCodecs: boolean;
  hasChromiumBreakIterators: boolean;
  supportsWasmGC: boolean;
  crossOriginIsolated: boolean;
}

type CanvasKitVariant =
  "auto" |
  "full" |
  "chromium";

export interface FlutterConfiguration {
  assetBase: string?;
  canvasKitBaseUrl: string?;
  canvasKitVariant: CanvasKitVariant?;
  renderer: WebRenderer?;
  hostElement: HtmlElement?;
  fontFallbackBaseUrl: string?;
  entryPointBaseUrl: string?;
  forceSingleThreadedSkwasm: boolean?;
}

export interface ServiceWorkerSettings {
  serviceWorkerVersion: string;
  serviceWorkerUrl: string?;
  timeoutMillis: number?;
}

export interface AppRunner {
  runApp: () => void;
}

export interface EngineInitializer {
  initializeEngine: () => Promise<AppRunner>;
}

export type OnEntrypointLoadedCallback =
  (initializer: EngineInitializer) => void;
