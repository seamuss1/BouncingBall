# Agent instructions

## GitHub issue work queues

Apply the [issue classification convention](.github/ISSUE_CONVENTION.md) to every
new issue, including CLI/API-created issues. Use exactly one of `owner-requested`
(explicit owner request, even when written by an agent) or `auto-generated`
(unsolicited agent/audit/review follow-up). Classify by the request, not the author.
Preserve existing topic, priority, review-severity labels, and Project workflows.
An owner-requested review does not make each finding owner-requested. Record the
source; leave historical issues unclassified when evidence is unclear. Prefer
owner-requested backlog work over routine automated follow-ups without ignoring
urgent security/safety findings or P1 blockers. If the owner explicitly adopts an
automated issue, replace its queue label, retain severity labels, and record that
adoption. Issue classification never authorizes implementation or deployment.
