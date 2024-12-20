# Web Locale Keymap Generator

This script generates mapping data for `web_locale_keymap`.

## Usage

1. `cd` to this folder, and run `dart pub get`.
2. [Create a Github access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token), then store it to environment variable `$GITHUB_TOKEN`. This token is only for quota controlling and does not need any scopes.
```
# ~/.zshrc
export GITHUB_TOKEN=<YOUR_TOKEN>
```
3. Run
```
dart --enable-asserts bin/gen_web_locale_keymap.dart
```

### Help
For help on CLI,
```
dart --enable-asserts bin/gen_web_locale_keymap.dart -h
```

## Explanation

To derive a key map that allows international layout to properly trigger
shortcuts, we can't [simply map logical keys from the current
event](https://github.com/flutter/flutter/issues/100456). Instead, we need to
analyze the entire current layout and plan ahead. This algorithm,
which we call the benchmark planner, goes as follows:

> Analyze every key of the current layout,
> 1. If a key can produce an alnum under some modifier, then this key is mapped to this alnum.
> 2. After the previous step, if some alnum is not mapped, they're mapped to their corresponding key on the US keyboard.
> 3. The remaining keys are mapped to the unicode plane according to their produced character.

However, we can't simply apply this algorithm to Web: unlike other desktop
platforms, Web DOM API does not tell which keyboard layout the user is on, or
how the current layout maps keys (there is a KeyboardLayout API that is
supported only by Chrome, and explicitly refused by all other browsers). So we
have to invent a "blind" algorithm that applies to any layout, while keeping the
same result.

Luckily, we're able to fetch a list of "all keyboard layouts" from
`Microsoft/VSCode` repo, and we analyzed all layouts beforehand, and managed to
combine the result into a huge `code -> key -> result` map. You would imagine it
being impossible, since different layouts might use the same `(code, key)` pair
for different characters, but in fact such conflicts are surprisingly few, and
all the conflicts are mapped to letters. For example, `es-linux` maps
`('KeyY', '←')` to `y`, while `de-linux` maps `('KeyY', '←')` to `z`.

We can't distinguished these conflicts only by the `(code, key)` pair, but we
can use other information: `keyCode`. Now, keyCode is a deprecated property, but
we really don't see it being removed any time foreseeable. Also, although
keyCode is infamous for being platform-dependent, for letter keys it is always
equal to the letter character. Therefore such conflicting cases are all mapped
to a special value, `kUseKeyCode`, indicating "use keyCode".

Moreover, to reduce the size of the map, we noticed there are certain patterns
that can be easily represented by some if statements. These patterns are
extracted as the so-called "heuristic mapper". This reduces the map from over
1600 entries to ~450 entries.

To further reduce the package size overhead, the map is encoded into a string
that is decoded at run time. This reduces the package size over by 27% at the
cost of code complexity.
