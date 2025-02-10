# Primary issue triage process

The process of triaging new incoming bugs consists of processing the list of [issues without team-* labels, with no assignees, and not labeled `will need additional triage`](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+no%3Aassignee+-label%3A%22will+need+additional+triage%22+-label%3Ateam-release%2Cteam-codelabs%2Cteam-ecosystem%2Cteam-infra%2Cteam-engine%2Cteam-framework%2Cteam-news%2Cteam-ios%2Cteam-tool%2Cteam-web%2Cteam-linux%2Cteam-macos%2Cteam-windows%2Cteam-design%2Cteam-android%2Cteam-go_router%2Cteam-games%2Cteam-text-input+) as described in this section, so as to make that list empty.

_See also: [Issue triage reports](https://github.com/flutter/flutter/wiki/Issue-triage-reports)_

### General

To triage an issue, first look at the bug report, and try to understand what the described problem is. Edit the original comment to remove boilerplate that the bug reporter didn't remove. Edit the original comment to add backticks (\`\`\`) around blocks of stack traces, code, the output of shell scripts like `flutter doctor`, etc. Ensure that the title is a meaningful summary of the issue. These changes make the bug much easier to manage.

If their report is **unclear**, doesn't give sufficient steps to reproduce, or is otherwise lacking in sufficient detail for us to act on it, add a polite comment asking for additional information, add the `waiting for customer response` label, then skip the remaining steps.

If the bug is **still unclear** -- we have previously asked for more detail, and the bug reporter has had a chance to provide additional feedback, but has not been able to do so in a way that makes the bug actionable -- either apologize for us not being able to fix it and then close the bug, or add the `waiting for customer response` label, depending on your confidence that the reporter will be able to eventually provide sufficient detail. Then, skip the remaining steps. It is fine to be aggressive in closing bugs where the issue is not clear, because we have plenty of other issues where the bug _is_ clear and there's really no value to us in having low-quality bugs open in our bug database.

If the issue describes something that you know for a fact has been **fixed** since the bug report was filed, add a cheerful comment saying so, close the issue, and skip the remaining steps.

If the bug is **clear enough** for us to act on it, continue with the following steps. To reach this step, the bug should be actionable, with clear steps to reproduce the problem. We have enough bugs filed that we will not run out of work any time soon; therefore, it is reasonable to be quite aggressive in establishing if a bug is sufficiently clear.

#### Artifacts

Ideally every issue would have a sample app that demonstrated the problem.

Performance bugs should have timeline traces.

Crashes should have crash logs with a Flutter version so that the [flutter-symbolizer-bot](https://github.com/flutter-symbolizer-bot) can do its work (see also [Crashes](../engine/Crashes.md)).

#### What makes an issue actionable

An _actionable_ issue is one for which it is easy to determine if a particular PR means the issue can be closed or not. Issues whose descriptions are vague, or that express a general malaise or general desire, issues that specify a failure mode but no steps to reproduce the problem, and other issues where the nature of the problem is not clear and where it would be difficult to determine if any particular change could actually fix the problem, should be closed.

One example of an unactionable issue is one with such vaguely described symptoms that lots of people claim to have the same problem even when their described situations differ in mutually exclusive ways.

As a project with literally thousands of open issues, we are not lacking in feedback. Time that would be spent trying to understand an unclear issue could be more effectively spent on a bug with a clear description. Unactionable bugs are simply not valuable enough to keep around when we have many actionable bugs already. Indeed, given how such issues are likely to affect search results, confuse new users filing issues, or attract hostile comments due to remaining open for a long time, they may literally have a negative value to the project.

#### Unactionable bugs with unusual symptoms

As discussed above, if a filed issue is unactionable due to vagueness or a lack of steps to reproduce, it should be closed, because we're never going to get to it if we don't know what the problem is given that we have many, _many_ other bugs that we _can_ make progress on today.

In the specific case of a bug with unclear steps to reproduce but very specific symptoms, we like to leave the issue open so that other people having the same problem can congregate together and maybe together we can figure out the underlying cause. This only applies to issues that have very specific symptoms like a specific and unusual crash signature, a specific and unusual error message, or other unusual and recognizable symptoms, and where some effort was made on the part of the bug reporter to determine the cause (even if that effort was ultimately futile).

### Duplicates

If you recognize that this bug is a duplicate of an existing bug, add a reference to that bug in a comment, then close the bug. Skip the remaining steps. As you triage more and more bugs you will become more and more familiar with the existing bugs and so you will get better and better at marking duplicates in this way.

When closing the duplicate bug, the GitHub issue tracker does not copy the list of people being notified on the closed bug into the original bug. This can matter, especially when asking on the original bug for things like reproduction steps. Consider cc'ing the author of the duplicate issue into the original issue, especially if we're still trying to determine reproduction steps for the issue.

### Requests for help (documentation issues)

If the bug report is a question, then it probably belongs in Stack Overflow or on our #help channel or some other forum for getting help. However, if it appears that the reporter did try to read our documentation to find the answer, and failed, or, if you look in our documentation and find it is inadequate here, then please consider it a documentation bug (and update the summary accordingly).

If you are confident our official documentation (on flutter.dev or api.flutter.dev) fully answers their question, then provide a link to the relevant page and close the issue, being very polite and asking them to reopen if the documentation is not sufficiently clear for them.

### Labels

**General rule: The more labels an issue has, the better!** _See also: [List of labels](https://github.com/flutter/flutter/labels)_

Some labels are used to track the flow of issues from the time they're filed until they're assigned to a specific team for execution. You should use these to track the state of an issue through your first-level triage process. These are:

  * `in triage`: You are presently looking at an issue and trying to determine what other labels you should give it.
  * `assigned for triage`: The issue is assigned to a domain expert for further triage.
  * `has reproducible steps`: The issue has a reproducible case or test, Flutter doctor output, and usable stack traces if appropriate. It is actionable in the sense that it can be routed to a domain team for action.
  * `needs repro info`: We need more reproduction steps in order to be able to act on this issue.
  * `workaround available`: A workaround is available to overcome the issue until it is properly addressed. Read more about [providing workarounds](../contributing/issue_hygiene/README.md#comments-providing-workarounds).
  * `will need additional triage`: Assign this if you don't know how to route it to a team.

**To complete the triage of an issue, add one (and only one) `team-*` label**. Team labels differ from the similar category names (such as `engine` or `framework`) in that the category labels indicate what part(s) of the codebase an issue affects, while `team-*` labels indicate the team that owns that work. Most issues will have both, and they won't always match.

In general the flow chart for team assignment is as follows, stopping as soon as the first `team-` label is assigned:

- If it's about the flutter/news_toolkit repository, add `team-news`.
- If it's about a codelab, add `team-codelab`.
- If it's about the release process or tooling (e.g., `conductor`), add `team-release`.
- If it's about the Flutter team's CI or infrastructure, add `team-infra`.
- If it's about Impeller, add `team-engine`.
- If it's about accessibility (e.g. `Semantics`, `talkBack`, `voiceOver`), add `team-accessibility`.
  - If it's specific to a single platform, also add that platform's fyi label.
- If it's about Cupertino or Material Design, add `team-design`.
- If it's about text fields or other user-facing text input issues, add `team-text-input`.
  - If it's specific to a single platform, also add that platform's fyi label.
- If it's specific to a single platform, add that platform's team (`team-android`, `team-ios`, `team-linux`, `team-macos`, `team-web`, or `team-windows`).
  - If the issue is about a first-party package, also add `fyi-ecosystem`.
- If it's about one of our games templates, add `team-games`.
- If it's about the Flutter engine, add `team-engine`.
- If it's about the Flutter framework, add `team-framework`.
- If it's about the Flutter tool, add `team-tool`.
- If it's about a first-party package:
  - If it's about `go_router` or `go_router_builder`, add `team-go_router`.
  - If it's about `two_dimensional_scrollables`, add `team-framework`.
  - If it's about `flutter_svg` or `vector_graphics`, add `team-engine`.
  - Otherwise, add `team-ecosystem`.
- If none of the above apply, add `will need additional triage`.

_It is expected that some bugs will end up being re-assigned to a different team during secondary triage. If there are specific categories of issues where this always happens, the flow chart above should be updated accordingly, but having it happen occasionally is just the process working as expected; in some cases only the engineers working on an issue will know how the work is divided among teams._

Bugs relating to the developer tools should be moved to the `flutter/devtools` repo, unless it looks like the first step is a change to the core parts of Flutter (in which case it should receive the `d: devtools` label as well as the pertinent labels for where the work should occur). Issues tagged with `d: devtools` or moved to the `flutter/devtools` repo will be triaged as described by [flutter/devtools/wiki/Triage](https://github.com/flutter/devtools/wiki/Triage).

Bugs relating to the IntelliJ IDEs should be moved to the `flutter/flutter-intellij` repo, unless it looks like the first step is a change to the core parts of Flutter (in which case it should receive the `d: intellij` label as well as the pertinent labels for where the work should occur).
Issues tagged with `d: intellij` will be reviewed by the Flutter IntelliJ team as described by [flutter/flutter-intellij/wiki/Triaging](https://github.com/flutter/flutter-intellij/wiki/Triaging).

Bugs relating to the website should be moved to the `flutter/website` repo.

#### Additional labels

Once the main labels above are added, consider what additional labels could be added, in particular:

Add any of the applicable "c: *" labels; typically only one will apply but sometimes `c: regression` will apply in conjunction with one of the others.

Add any of the applicable "a: *" labels. There are many, it's worth browsing the list to get an idea of which ones might apply.

### Additional comments

If you have something to say regarding the bug, for example if you happen to notice what the problem is, or if you have some insight based on having seen many other bugs over time, feel free to add a comment to that effect. Your experience is valuable and may help both the reporter and the rest of the Flutter team.

# Triage process for teams

We intend for each area of the product to go through the following triage regularly:

* Look at open bugs and determine what needs to be worked on.
* Look at open PRs and review them.

It is recommended to do these in separate dedicated meetings. For teams with multiple areas of focus (e.g. design languages), it's recommended that each area of focus have its own meeting.

## Team issue triage process

Each team has a label, for example `team-engine` is the engine team's label. Each issue gets assigned to a team during primary triage. In addition, each team has a "triaged" label (e.g. `triaged-engine`) and an "FYI" label (e.g. `fyi-engine`).

Each team has an **incoming issue list**, the issues assigned to that team (team-foo), or marked for the attention of that team (fyi-foo), that the team has not yet triaged (triaged-foo). See below for links to those issue lists for each team.

Each issue in this list should be examined, cleaned up (see next section), and either:
- closed, with a comment saying why (e.g. is a duplicate, is not actionable, is invalid). The [`r:`](https://github.com/flutter/flutter/labels?q=r%3A) labels may be of use when closing an issue.
- given a [priority](../contributing/issue_hygiene/README.md#priorities), and tagged with the team's corresponding `triaged-*` label. This marks the issue as triaged. If the priority is P3 and the reporter has expressed that the issue is important to them, it will help the reporter feel welcome if a comment is added expressing empathy for their plight and explaining why it is not something we consider important.
- sent to another team, by removing the current `team-*` label and adding another one. A comment should be added explaining the action.
- sent back to primary triage, by removing the `team-*` label but not adding another one. A comment should be added explaining the action.
- escalated to critical triage, by adding the `will need additional triage` label. A comment should be added explaining the action.

In addition, each team should look at their **P0 list**, and ensure that progress is being made on each one. Every P0 issue should have an update posted at least once a week.

Teams may also look at other lists (e.g. checking over previously-triaged regression issues, checking all P1 issues, checking assigned issues, etc), as appropriate.

To flag an issue for attention by another team, the `fyi-*` label for that team can be set on the label. This does not assign the issue to that team, it just adds it to their triage queue.

### Checklist for cleaning up issues

When looking at an issue, perform the following cleanup steps:

* Correct any typos and inaccuracies in the summary.
* Correct the set of labels applied.
* Hide low quality comments.

Restrict the time you spend diagnosing each issue during triage. You don’t have to fix the issue! 30 seconds for P0 and 10~15 seconds for the others is reasonable! (Plus whatever time it takes to update the issue.)

<details>
<summary>JS script to open all PRs or issues on a GitHub page</summary>

```js
// This script is intended to be run in chrome devtools console
// during triage to open PRs and Issues faster.

// Before the script can be run you need to enable popups in Chrome
// 1. On your computer, open Chrome Chrome.
// 2. At the top right, click More More and then Settings.
// 3. Click Privacy and security and then Site Settings.
// 4. Click Pop-ups and redirects.
// 5. Choose the option you want as your default setting.
//
// https://support.google.com/chrome/answer/95472?hl=en&co=GENIE.Platform%3DDesktop

const plural = window.location.toString().split('?')[0];
const singular = plural.substring(0, plural.length-1);
const suffix = singular.includes("issue") ? "s" : "";
const re = new RegExp("^" + singular + suffix + "/\\d+$");

var urls = document.getElementsByTagName('a');

var targets = []
for (url in urls) {
    var link = urls[url].href;
    if(link == undefined) continue;
    if(link.match(re) == null) continue;
    if(targets.includes(link)) continue;
    targets.push(link);
}

targets.forEach((target) => window.open(target));
```

</details>

## PR triage process

Teams should also go through all PRs in their area (ideally in a separate meeting). PRs can apply to multiple areas, and different teams have different methods of organizing code, so there is no uniform guidance for this process. However, in general:

1. Check that PRs have an assigned reviewer.
2. Check that the assigned reviewers have left comments; if not, contact them to remind them.
3. Check that any questions on the PR from the contributor have been answered.

For more guidance on reviewing PRs, see [Tree Hygiene](../contributing/Tree-hygiene.md#how).

## Links for teams

### Accessibility team (`team-accessibility`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-accessibility%2Cfyi-accessibility+-label%3Atriaged-accessibility+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc+)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-accessibility+label%3AP0+sort%3Aupdated-asc+)
- [Package PRs](https://github.com/flutter/flutter/pulls?q=is%3Aopen+is%3Apr+label%3A%22a%3A+accessibility%22+sort%3Aupdated-asc+-is%3Adraft+)

### Android platform team (`team-android`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-android%2Cfyi-android+-label%3Atriaged-android+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-android+label%3AP0+sort%3Aupdated-asc)
- [P1, No Assignee list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-android+label%3Ap1+no%3Aassignee+sort%3Aupdated-asc)
- PRs: [Framework/Tool](https://github.com/flutter/flutter/pulls?q=is%3Aopen+draft%3Afalse+is%3Apr+label%3Aplatform-android+-label%3A%22work+in+progress%3B+do+not+review%22+sort%3Aupdated-asc+), [Plugins \(non-dependabot\)](https://github.com/flutter/packages/pulls?q=is%3Aopen+draft%3Afalse+is%3Apr+label%3Atriage-android+sort%3Aupdated-asc+-author%3Aapp%2Fdependabot+), [Plugins \(dependabot\)](https://github.com/flutter/packages/pulls?q=is%3Aopen+draft%3Afalse+is%3Apr+label%3Aplatform-android+sort%3Aupdated-asc+author%3Aapp%2Fdependabot+)

### Codelabs team (`team-codelabs`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-codelabs%2Cfyi-codelabs+-label%3Atriaged-codelabs+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-codelabs+label%3AP0+sort%3Aupdated-asc)

### Design Languages team (`team-design`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-design%2Cfyi-design+-label%3Atriaged-design+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc+-label%3A%22waiting+for+customer+response%22+)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-design+label%3AP0+sort%3Aupdated-asc)
- [Design Languages PRs](https://github.com/flutter/flutter/pulls?q=is%3Aopen+is%3Apr+label%3A%22f%3A+material+design%22%2C%22f%3A+cupertino%22+sort%3Aupdated-asc+draft%3Afalse)

### Ecosystem team (`team-ecosystem`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-ecosystem%2Cfyi-ecosystem+-label%3Atriaged-ecosystem+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-ecosystem+label%3AP0+sort%3Aupdated-asc)
- [PR list](https://github.com/flutter/packages/pulls?q=is%3Apr+is%3Aopen+sort%3Aupdated-asc+-label%3A%22p%3A+go_router%22+-label%3A%22p%3A+go_router_builder%22)

In addition, consider these issues that fall under another team's triage, but are things the ecosystem team might want to be aware of:
 * [`a: plugins` issues](https://github.com/flutter/flutter/issues?q=is%3Aopen+label%3A%22a%3A+plugins%22+-label%3Ateam-ecosystem+-label%3Atriaged-ecosystem)
 * [`package` issues](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Apackage+-label%3Ateam-ecosystem+-label%3Atriaged-ecosystem+-label%3Ateam-go_router+sort%3Acreated-desc+)
 * [Package regressions](https://github.com/flutter/flutter/issues?q=is%3Aopen+is%3Aissue+-label%3Ateam-go_router+label%3Apackage+label%3A%22c%3A+regression%22+sort%3Acreated-desc)


### Engine team (`team-engine`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-engine%2Cfyi-engine+-label%3Atriaged-engine+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-engine+label%3AP0+sort%3Aupdated-asc)
- [Buildroot PRs](https://github.com/flutter/buildroot/pulls)
- [Approved PRs that have not yet landed](https://github.com/flutter/engine/pulls?q=is%3Aopen+is%3Apr+draft%3Afalse+-label%3A%22Work+in+progress+%28WIP%29%22+review%3Aapproved+NOT+%22Roll+Skia%22+-label%3Aplatform-web+sort%3Acreated-asc)
- [PRs awaiting review](https://github.com/flutter/engine/pulls?q=is%3Aopen+is%3Apr+draft%3Afalse+-label%3A%22Work+in+progress+%28WIP%29%22+-label%3A%22waiting+for+tree+to+go+green%22+-label%3A%22platform-web%22+-review%3Aapproved+-label%3A%22waiting+for+customer+response%22+NOT+%22Roll+Skia%22+NOT+%22Roll+Dart%22+NOT+%22Roll+Fuchsia%22+sort%3Aupdated-asc++-label%3A%22platform-web%22+)
- [Draft PRs](https://github.com/flutter/engine/pulls?q=is%3Aopen+is%3Apr+label%3A%22Work+in+progress+%28WIP%29%22+-label%3A%22waiting+for+tree+to+go+green%22+-label%3A%22platform-web%22+-review%3Aapproved+-label%3A%22waiting+for+customer+response%22+NOT+%22Roll+Skia%22+sort%3Aupdated-asc++-label%3A%22platform-web%22+)

### Framework team (`team-framework`)

- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-framework+label%3AP0+sort%3Aupdated-asc)
- [PR list](https://github.com/flutter/flutter/pulls?q=is%3Aopen+is%3Apr+label%3Aframework+-label%3A%22f%3A+material+design%22+-label%3A%22f%3A+cupertino%22+sort%3Acreated-desc+draft%3Afalse)
- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-framework%2Cfyi-framework+-label%3Atriaged-framework+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc+-label%3A%22waiting+for+customer+response%22+)

### Games team (`team-games`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-games%2Cfyi-games+-label%3Atriaged-games+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-games+label%3AP0+sort%3Aupdated-asc)
- [PRs awaiting review](https://github.com/flutter/games/pulls?q=is%3Apr+is%3Aopen+sort%3Aupdated-asc)

### Go Router team (`team-go_router`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-go_router%2Cfyi-go_router+-label%3Atriaged-go_router+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-go_router+label%3AP0+sort%3Aupdated-asc)
- [Package PRs](https://github.com/flutter/packages/pulls?q=is%3Apr+is%3Aopen+label%3A%22p%3A+go_router%22%2C%22p%3A+go_router_builder%22)

### Infrastructure team (`team-infra`)

- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-infra+label%3AP0+sort%3Aupdated-asc)
- [Cocoon PRs](https://github.com/flutter/cocoon/pulls)
- [GoB CLs](https://flutter-review.googlesource.com/q/status:open+-is:wip)
- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-infra%2Cfyi-infra+-label%3Atriaged-infra+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc)
- [Latest updated issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-infra%2Cfyi-infra+sort%3Aupdated-desc)

### iOS and macOS platform team (`team-ios` and `team-macos`)

- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-ios%2Cteam-macos+label%3AP0+sort%3Aupdated-asc+)
- [iOS incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-ios%2Cfyi-ios+-label%3Atriaged-ios+-label%3A%22will+need+additional+triage%22+-label%3A%22waiting+for+customer+response%22+sort%3Aupdated-asc+)
- [macOS incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-macos%2Cfyi-macos+-label%3Atriaged-macos+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc+)
- [Apple news](https://developer.apple.com/news) - check for updates that might affect us.

PRs are reviewed weekly across the framework, packages, and engine repositories:

- [iOS PRs on the engine](https://github.com/flutter/engine/pulls?q=is%3Aopen+is%3Apr+label%3Aplatform-ios+sort%3Acreated-asc+-is%3Adraft)
- [macOS PRs on the engine](https://github.com/flutter/engine/pulls?q=is%3Aopen+is%3Apr+label%3A%22affects%3A+desktop%22+label%3Aplatform-macos+sort%3Aupdated-asc)
- [iOS PRs on the framework](https://github.com/flutter/flutter/pulls?q=is%3Aopen+is%3Apr+label%3Aplatform-ios+sort%3Acreated-asc+-is%3Adraft)
- [macOS PRs on the framework](https://github.com/flutter/flutter/pulls?q=is%3Aopen+is%3Apr+label%3A%22a%3A+desktop%22+platform-macos++sort%3Aupdated-asc)
- [iOS and macOS PRs on packages](https://github.com/flutter/packages/pulls?q=is%3Aopen+is%3Apr+label%3Atriage-macos%2Ctriage-ios+sort%3Aupdated-asc+)

### Linux platforms team (`team-linux`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-linux%2Cfyi-linux+-label%3Atriaged-linux+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-linux+label%3AP0+sort%3Aupdated-asc)
- [Linux PRs on the engine](https://github.com/flutter/engine/pulls?q=is%3Aopen+is%3Apr+label%3A%22affects%3A+desktop%22+label%3Aplatform-linux+sort%3Aupdated-asc)
- [Linux PRs on the framework](https://github.com/flutter/flutter/pulls?q=is%3Aopen+is%3Apr+label%3A%22a%3A+desktop%22+label%3Aplatform-linux+sort%3Aupdated-asc)
- [Linux PRs on packages](https://github.com/flutter/packages/pulls?q=is%3Aopen+is%3Apr+label%3Atriage-linux+sort%3Aupdated-asc)

### News Toolkit team (`team-news`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-news%2Cfyi-news+-label%3Atriaged-news+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-news+label%3AP0+sort%3Aupdated-asc)

### Release team (`team-release`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-release%2Cfyi-release+-label%3Atriaged-release+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-release+label%3AP0+sort%3Aupdated-asc)

### Text Input team (`team-text-input`)

- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3A%22a%3A+text+input%22%2Cteam-text-input%2Cfyi-text-input+sort%3Aupdated-asc+label%3AP0+)
- [PR list](https://github.com/flutter/flutter/pulls?q=is%3Aopen+is%3Apr+sort%3Acreated-desc+draft%3Afalse+label%3A%22a%3A+text+input%22%2Cteam-text-input%2Cfyi-text-input+)
- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aopen+is%3Aissue+label%3A%22a%3A+text+input%22%2Cteam-text-input%2Cfyi-text-input+no%3Aassignee+-label%3A%22triaged-design%22+-label%3A%22triaged-framework%22+-label%3A%22triaged-linux%22+-label%3A%22triaged-macos%22+-label%3A%22triaged-windows%22+-label%3A%22triaged-android%22+-label%3A%22triaged-ios%22+-label%3A%22triaged-web%22+-label%3A%22triaged-ecosystem%22+-label%3A%22triaged-engine%22+-label%3A%22triaged-tool%22+-label%3A%22triaged-text-input%22+-project%3Aflutter%2F111+)

### Flutter Tool team (`team-tool`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-tool%2Cfyi-tool+-label%3Atriaged-tool+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-tool+label%3AP0+sort%3Aupdated-asc)
- [PR list](https://github.com/flutter/flutter/pulls?q=is%3Aopen+is%3Apr+label%3Atool+sort%3Aupdated-asc+-is%3Adraft)

### Web platform team (`team-web`)

- See the [Flutter Web Triage](Flutter-Web-Triage.md) page.

### Windows platforms team (`team-windows`)

- [Incoming issue list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-windows%2Cfyi-windows+-label%3Atriaged-windows+no%3Aassignee+-label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc)
- [P0 list](https://github.com/flutter/flutter/issues?q=is%3Aissue+is%3Aopen+label%3Ateam-windows+label%3AP0+sort%3Aupdated-asc)
- [Windows PRs on the framework](https://github.com/flutter/flutter/pulls?q=is%3Aopen+is%3Apr+label%3A%22a%3A+desktop%22+label%3Aplatform-windows+sort%3Aupdated-asc)
- [Windows PRs on packages](https://github.com/flutter/packages/pulls?q=is%3Aopen+is%3Apr+label%3Atriage-windows+sort%3Aupdated-asc)

## Adding a new team

To add a team:

* Create the `team-*`, `triaged-*`, and `fyi-*` labels with the same colors as other teams' labels.
* Add the team to the list of teams in the triage bot ([the `GitHubSettings.teams` set](https://github.com/flutter/cocoon/blob/main/triage_bot/lib/engine.dart#L50)) and tell Hixie so he can restart the bot.
* Add the team to the list of excluded labels in the link at the top of this page.
* Add a section above with the incoming issue list and P0 issue list.

# Critical triage

Each week we have a "critical triage" meeting where we check how things are going, to make sure nothing falls through the cracks. (It's not really "critical", the name is historical.)

During these meetings, we go through the following lists:

* [P0](https://github.com/flutter/flutter/issues?q=is%3Aopen+label%3AP0+sort%3Aupdated-asc): all bugs should be assigned, and progress should be happening actively. There should be an update within the last week. If no progress is happening and owner cannot work on it immediately (e.g. they're on vacation, they're busy with their day job, family reasons, etc), find a new owner.
* [Bugs flagged for additional triage](https://github.com/flutter/flutter/issues?q=is%3Aopen+label%3A%22will+need+additional+triage%22+sort%3Aupdated-asc+no%3Aassignee): figure out what should be done with the bug, then remove the `will need additional triage` label.
* [flutter-pub-roller-bot](https://github.com/flutter/flutter/pulls/flutter-pub-roller-bot): check that the pub auto roller is chugging along. If it has gotten trivially stuck, such as having a merge conflict, close the PR so that it can open a new one. If it is non-trivially stuck, file an issue for the infra ticket queue: https://github.com/flutter/flutter/blob/main/docs/infra/Infra-Ticket-Queue.md.
* [The stale PRs](https://github.com/pulls?q=is%3Aopen+is%3Apr+archived%3Afalse+user%3Aflutter+-repo%3Aflutter%2Fwebsite-cms+sort%3Aupdated-asc+): examine the 25 least-recently updated PRs, if the least recently updated one was updated more than 2 months ago.

## Self test issue

The automation that supports our triage processes will periodically file an issue with every team's label on it. After a couple of weeks, it then adds the `will need additional triage` label to send it to the attention of the "critical triage" meeting, at which point we discover how well the triage processes are going, and which teams are not following the process completely, or are understaffed for the volume of issues they are dealing with.
