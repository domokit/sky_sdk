If you want to write a design doc for people to review, we recommend using Google Docs.
We have a template you can use, at [flutter.dev/go/template](https://flutter.dev/go/template). It describes the process for minting a `flutter.dev/go/foo` shortlink for your design doc.
We recommend you use that template so that people can immediately recognize that this is a Flutter design document and that it is shared publicly.

After you have created your design doc, the next step is to create a tracking GitHub issue for it. File a new issue to track the design doc using [the design doc issue template](https://github.com/flutter/flutter/issues/new?template=7_design_doc.yml). Assign it to yourself. Add the label "design doc" to the issue.

Don't forget to configure your document's Sharing settings so that everyone has comment access. The idea of sharing the document in this way is not necessarily to proactively obtain feedback from the entire community; it is to make it _possible_ for people to share the document with anyone in the community, whether they work for your employer or not, and whether you have personally shared the document with them yet or not.

The template discusses how to create a shortlink for your design doc (flutter.dev/go/...). When creating the shortlink, remember to test the URL you are publishing in an incognito window!

Googlers: Design docs must be created by non-corp accounts! See [Contributor Access](Contributor-access.md#fcontriborg-accounts) for details on getting `fcontrib.org` accounts if you don't want to use your personal GMail account.

When you implement a design, document it in the source code in detail. The API documentation is the usual place where we document our designs. It's perfectly reasonable for API docs to be multiple pages long with subheadings (e.g. see the docs for [RenderBox](https://master-api.flutter.dev/flutter/rendering/RenderBox-class.html)!). Do not assume that anyone will ever read your design doc after the discussion has finished. Similarly, do not assume that anyone will look at closed GitHub issues or PR discussions.

See also:

1. [`design doc`][] GitHub issue label: list of all design documents.
2. [Archive of design documents][] from before the [`design doc`][]
   GitHub issue label was introduced.

[`design doc`]: https://github.com/flutter/flutter/issues?q=is%3Aopen+is%3Aissue+label%3A%22design+doc%22
[Archive of design documents]: https://github.com/flutter/flutter/issues/151486

## Purpose of design docs

The Flutter project uses design docs as a tool for guiding discussions.

Decisions are made in PRs, not in design docs.

Approvals are given in PRs, not in design docs.

## Soliciting feedback

If you wish to get feedback on your design doc, you have many options for doing so, depending on how much feedback you want:

* If there is an issue already filed on the topic, definitely put a link to the design doc there. People who have found the issue and want to get updates on the topic will have subscribed to the issue, so this is the most effective way to communicate with them.

* Post the link on Discord. You can post it to #hidden-chat to just get feedback from team members. You can post it to one or more of the #hackers-* channels if you want feedback from people who are interested in the general area. You can post it to the global #hackers channel if you want feedback from anyone interested in working on Flutter. If you really want feedback, you can post a request to #announcements and publish it to any server that is following ours.

* If you want feedback from the broad community, tweet out the link and let other team members know so that we can retweet it. Similarly, you can post the request to one of the Flutter reddit channels, such as r/FlutterDev.

* You can ask our developer relations (devrel) team to broadcast a request for comments. (Start by asking in #hackers-devrel; if nobody responds, ping Hixie on that channel.)

* You can ask our user experience researcher (UXR) team to study the proposal and potentially test it with real users, or collect relevant data from the next quarterly survey. (Start by asking in #hackers-devexp; if nobody responds, ping Hixie on that channel.)

* If you have commit access, you can ask to talk about the design doc at the next Dash Forum meeting (normally held on Tuesdays at 11am US west coast time). Ping Hixie on #hidden-chat to get on the schedule, or use the form to request to be added, the link for which is pinned in the #hidden-chat channel.

### How to get good feedback

Often, you will solicit feedback, and get none. There are many causes of this.

Maybe your proposal is unclear, and so people don't really know what to suggest. People are often reluctant to provide broad criticisms. Consider if you can improve the clarity of your design doc. Do you have a clear problem statement separate from your solution? Do you show example code of the problem? Do you have screenshots or diagrams of the problem? For your solution, do you start from first principles and explain it? Often it's easy to forget that your readers don't have the same context you do, so without a gentle introduction they'll get lost very quickly. Do you have sample code of your proposed solution(s)? Do you need more diagrams or screenshots? Ask someone you trust if they think your document is sufficiently clear.

Maybe your proposal is too big for anyone to get their head around. Can it be split into smaller components, so that each one can be understood separately, before bringing all the pieces together into your grand design? (You can do this all in the same doc.)

Maybe people don't know what to provide feedback about. If you have an area you are particularly interested in getting feedback about, it can be very helpful to explicitly invite such feedback.

Maybe you are asking the wrong people. Consider the suggestions in the earlier section, and reach out explicitly to people who are affected by your proposal. Consider escalating, asking more and more people until you get the volume of feedback you desire.

Maybe everyone agrees. Consider leaving some intentionally sketchy details in your proposal to encourage people to engage! (This is a risky strategy, sometimes people end up _liking_ your "bad" ideas...)

Maybe your proposal is too obvious or uninteresting. Sometimes, a change is so uncontroversial and simple that frankly it would be better just to write the PR and submit it.

## Content in design docs

### Screen captures

The easiest way to capture videos for design docs is using macOS. Press Command+Shift+5 for a whole bunch of options.

### Diagrams

As we use Google Docs for the text portion of design docs, the easiest way to draw diagrams is using Google Diagrams. Select `Insert` > `Drawing` > `New` to create a new diagram.
