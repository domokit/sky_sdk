# Engine Binary Hashing

Today; the framework finds the engine binaries to download from google storage via a file checked into the tree:

```shell
cat bin/internal/engine.version
76b7abb5c853860cb5b488ab5b8e1ad8c41b603e
```

This hash represents the Git commit hash of the engine version used to produce the production binaries. However, this approach becomes problematic when repositories are merged:

1. Requiring engineers to manually update this file would lead to frequent merge conflicts for any engine changes.
1. Predicting the hash value beforehand is impossible, as the HEAD commit is constantly changing.
1. Git merge queues will produce binaries for engine changes before they are merged to the main branch.

Therefore, we need a mechanism to hash the specific content used to generate the engine binaries, enabling reproducible builds and easier A/B testing.

## Content-based hashing

One approach is to calculate a checksum (e.g., SHA1) of all relevant files locally, similar to using `git ls-files`. However, `ls-files` operates on the working tree, which introduces challenges for A/B testing. Local modifications should be testable with `et run` using only the modified content, independent of the committed state.

Git provides a solution by allowing us to operate on the index with `git ls-tree -r HEAD`. This command lists the tree objects within the index, providing a consistent snapshot of the content. Here's an example showing how `ls-tree` works for hashing:

```bash
# Regenerate a "blob" hash
file_name="engine/src/flutter/vulkan/vulkan_window.h";  (printf "blob $(wc -c < "$file_name" | awk '{print $1}')\0"; cat "$file_name") | sha1sum
11a5a03d15ae21bde366e41291a7899eec44e5ae  -

git ls-tree -r HEAD  engine/src/flutter/vulkan/vulkan_window.h
100644 blob 11a5a03d15ae21bde366e41291a7899eec44e5ae	engine/src/flutter/vulkan/vulkan_window.h
```

## Scoping the Hash to the Engine

To accurately track engine binaries, we only want to include files that directly contribute to the engine build. This includes the `engine/` directory and the root `DEPS` file, which tracks third-party dependencies managed by `gclient sync`. Using `git ls-tree -r HEAD engine DEPS` effectively captures all necessary files while excluding irrelevant content from the `third_party` directory.

```shell
100644 blob 5143313ce5826665309e8a086a281ad3ab1a9ce7    DEPS
100644 blob 205edfe43306c4dbf9a4a6f15e83cf5d49b9fc7d    engine/src/flutter/.ci.yaml
100644 blob 3c73f32a334086d9a0f4fd468dcdf9505d74e9c5    engine/src/flutter/.clang-format
100644 blob b74be267bc42f08ebf9afe8eec5cbbfe75c5a1c9    engine/src/flutter/.clang-tidy
100644 blob dd395bfd2104526d4f865313eab578f15ee5775b    engine/src/flutter/.engine-release.version
100644 blob e69de29bb2d1d6434b8b29ae775ad8c2e48c5391    engine/src/flutter/.git-blame-ignore-revs
100644 blob 915d1ed51d121f1986c9dfe71cf1745c1a11286d    engine/src/flutter/.gitattributes
100644 blob c1c1d3d05f37b0e09155b32aceb6d2ec62ee464b    engine/src/flutter/.github/PULL_REQUEST_TEMPLATE.md
100644 blob 9688ddae25af122d7c17d9c27d887b84888f3619    engine/src/flutter/.github/dependabot.yml
100644 blob ed7171a9638274d8f411b6bededec61feab15a7b    engine/src/flutter/.github/labeler.yml
100644 blob be245c915e7eb5377317cc6eb038442628071790    engine/src/flutter/.github/release.yml
# ... all files
```

To generate a consistent hash across different platforms (including Windows CI environments), we can use `git hash-object`:

```bash
git ls-tree -r HEAD engine DEPS | git hash-object --stdin
3b9abe00dec28902a589c982b5b460b0f9f38e93
```

## Supporting A/B Testing

When developing a pull request (PR), your branch might contain multiple commits. To enable A/B testing against the engine version at the time of branching, we can modify the hash calculation to use the merge-base. This ensures that the generated hash reflects the engine state at the branch point, facilitating accurate comparisons.

```bash
git ls-tree -r $(git merge-base HEAD master) engine DEPS | git hash-object --stdin
```

## Recommended Formula and Implementation

For now, the recommended formula for calculating the engine hash is:

```bash
git ls-tree -r $(git merge-base HEAD master) engine DEPS | git hash-object --stdin
```

To ensure backwards compatibility and allow for future updates, this formula should be implemented in both `.sh` and `.bat` scripts checked into the repository. This approach enables controlled updates to the hash calculation logic without disrupting existing workflows.

## Considerations and Future Refinements

Using the recomended formula incorporates the blob hash, permissions, and paths into the hash calculation. Consequently, moving, renaming, or changing permissions of a file will change the hash output and trigger rebuilding the engine. While acceptable initially, this behavior could be fine tuned in the future.

If we want to focus solely on file contents, we could use `git ls-tree -r --object-only engine DEPS | sort | git hash-object --stdin`. The output of `ls-tree` will only contain the githash of the blobs; sorting that output should make it resiliant to renames. However, this relies on consistent sorting across operating systems, which might introduce complexities.

An example showing renaming doesn't affect `ls-tree` blob hash:
```shell
#
# Not using --object-only for demonstration. We would use --blob-only to get just the hash
#
$ git ls-tree -r HEAD README.md
100644 blob 38daa079e3693e4940f0e9bc0201b7f5fda627e2	README.md

$ git mv README.md DONTREADME.md
$ git commit -a -m "test"

$ git ls-tree -r HEAD README.md
#nothing to see here, its not in the tree

$ git ls-tree -r HEAD DONTREADME.md
100644 blob 38daa079e3693e4940f0e9bc0201b7f5fda627e2	DONTREADME.md
```
