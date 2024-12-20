# Color blending

Impeller currently supports the same set of blending operations that
[Skia](https://api.skia.org/SkBlendMode_8h.html#ad96d76accb8ff5f3eafa29b91f7a25f0)
supports. Internally, Impeller distinguishes between two different kinds of
blend modes: Those which can be performed using the raster pipeline blend
configuration (called "Pipeline Blends"), and those which cannot (called
"Advanced Blends").

All blend modes conform to the
[W3C Compositing and Blending recommendation](https://www.w3.org/TR/compositing-1/).

Blend operations are driven by the `BlendMode` enum. In the Aiks layer,
all drawing operations conform to the given `Paint::blend_mode`. In the Entities
layer, all Entities have an associated blend mode, which can be set via
`Entity::SetBlendMode(BlendMode)`.

## Glossary of blending terms
| Term | Definition |
| --- | --- |
| Source color | Any color that is output by a fragment shader. |
| Destination color | The backdrop color in a blend operation. |
| Premultiplied color | A color that has its alpha multiplied into it. Used for additive blending operations as well as colors presented to a surface. |
| Porter-Duff alpha composite | One of several operations that add together a source color and destination color, with both the source and destination colors being multiplied by respective alpha factors. |
| Pipeline blend | A blend mode that Impeller can always implement by using the raster pipeline blend configuration provided by the underlying graphics backend. Most of these are simple _Porter-Duff alpha composites_. |
| Advanced blend | A blend mode that Impeller computes using a fragment program. |

## Premultiplied colors

In Impeller, all blending _source colors_ are assumed to be _premultiplied_ for
the purpose of blending. This means that all Entity shaders must output colors
with premultiplied alpha. In general, these shaders also assume that sampled
textures and uniform color inputs are premultiplied.

The reason for this is that it enables us to implement all of the _Porter-Duff
alpha composites_ using the built-in raster pipeline blend configuration offered
by all major graphics backends.

## Pipeline blends

Most of the pipeline blends are actually _Porter-Duff alpha composites_, which
add together the source color and destination color -- both the source and
destination colors are multiplied by an alpha factor which determines the
behavior of the blend.

Pipeline blends are always cheap and don't require additional draw calls to
render.

| Pipeline blend |
| --- |
| Clear |
| Source |
| Destination |
| SourceOver |
| DestinationOver |
| SourceIn |
| DestinationIn |
| SourceOut |
| DestinationOut |
| SourceATop |
| DestinationATop |
| Xor |
| Plus |
| Modulate |

## Advanced blends

Advanced blends are blends that Impeller can't always implement using the
built-in raster pipeline blend configuration offered by graphics backends.
Instead, they're implemented using special blend shaders that bind the backdrop
texture in a separate render pass.

Note that all of the advanced blends are _color blends_ rather than _alpha
composites_, and they can technically be combined with any _pipeline blend_ with
predictable compositing behavior. However, in order to keep in line with
Flutter's (and Skia's) current behavior, Impeller uses _Source Over_ compositing
when rendering all advanced blends.

Advanced blends are expensive when compared to pipeline blends (which are
essentially free) for the following reasons:
* For each advanced blend, the current render pass ends because the backdrop
  texture needs to be sampled.
* A potentially large texture (the render pass backdrop) is sampled. Although in
  practice, just the coverage rectangle of the source being blended is actually
  used.
* An intermediary texture is allocated for the blend output before being blitted
  back to the render pass texture.

| Advanced blend |
| --- |
| kScreen |
| Overlay |
| Darken |
| Lighten |
| ColorDodge |
| ColorBurn |
| HardLight |
| SoftLight |
| Difference |
| Exclusion |
| Multiply |
| Hue |
| Saturation |
| Color |
| Luminosity |
