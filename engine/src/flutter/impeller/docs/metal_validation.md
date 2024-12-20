# Enable Metal Validation without Xcode.

Metal validation can be enabled for command-line application using environment
variables.

Apple [documents these environment variables on their developer site](https://developer.apple.com/documentation/xcode/validating-your-apps-metal-api-usage#Enable-API-Validation-with-environment-variables).
More documentation about these environment variables is also available via a man
page entry: `man MetalValidation`

To enable all relevant Metal API and shader validation without using Xcode, add
the following to your `.rc` file.

``` sh
# Metal Validation Defaults
export MTL_DEBUG_LAYER=1
export MTL_DEBUG_LAYER_ERROR_MODE=assert
# Set this to assert for stricter runtime checks. Set to "ignore" if too chatty.
export MTL_DEBUG_LAYER_WARNING_MODE=nslog
export MTL_SHADER_VALIDATION=1
```

These environment variable are good defaults but there are more validation
related knobs and dials to turn. See `man MetalValidation`.

# Enable Metal Profiling HUD without Xcode

Applications can optionally display a HUD that displays real-time information
about Metal related performance.

![Profiling HUD](https://raw.githubusercontent.com/flutter/assets-for-api-docs//5da33067f5cfc7f177d9c460d618397aad9082ca/assets/engine/impeller/metal_validation/performance_hud.avif)

More documentation about the specific elements of the HUD is present on the
[Apple developer site](https://developer.apple.com/documentation/xcode/monitoring-your-metal-apps-graphics-performance).

The Profiling HUD is separate from Metal Validation and can be enabled for apps
that are launched from the command like (like Impeller Playgrounds) using
environment variables. Add the following to your `.rc` file.

```sh
export MTL_HUD_ENABLED=1
```
