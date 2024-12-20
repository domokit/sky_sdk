# VSCode Workspace

This is the tools and the template used for updating //engine.code-workspace.

VSCode uses a custom version of JSONC for their config files, those config files
don't provide any mechanism for reducing redundancy. Since the engine has a lot
of test targets, without that mechanism it can get very unwieldy. YAML does
however support ways to reduce redundancy, namely anchors.

## Updating //engine.code-workspace

```sh
./refresh.sh
```

## Backporting //engine.code-workspace

If something is accidentally introduced into //engine.code-workspace without editing
the YAML file here there are tools that can be used to more easily fix that.

```sh
./merge.sh
```

Since JSON doesn't support anchors some work may be needed to resolve any
conflicts that happen when merging. These aren't necessary to use the VSCode
workspace, just to edit them.

## Requirements

The `refresh.sh` and `merge.sh` tools require certain tools to be present on
your PATH. They can be installed on macos with homebrew.

- `json5` - A variant of JSON that is a superset of the JSON variant that VSCode
  uses. It's used to strip away comments and trailing commas.
- `yq` - This is a tool for manipulating yaml files. It can convert back and
  forth from YAML to YAML and merge YAML files.
