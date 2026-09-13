# Repository working instructions

Optimize correctness per unit of model usage, not agent count. This file sets
the default workflow; follow the user's task-specific instructions and actual
tool capabilities. See [docs/CODEX_WORKFLOW.md](docs/CODEX_WORKFLOW.md) for the
reusable task prompt and verification commands.

## Lead workflow

For substantial tasks, inspect the relevant code, git status, README, applicable
handoff and build/test configuration before editing. State the requested outcome,
a short plan, task size (small/medium/large), and whether delegation pays off.
Avoid reading every handoff or generated tree when a targeted search suffices.

- Small: lead only; trivial edits rarely repay delegation overhead.
- Medium: 1–2 focused workers only when worthwhile.
- Large: approximately 2–4 workers only for independent workstreams; respect
  available slots (the lead also consumes a slot). Never spawn to meet a quota.

Before each spawn, evaluate independent scope, cheaper-model reliability,
useful parallelism, avoided expensive lead reasoning, and whether coordination
and duplicated context cost less than doing the work directly. Mostly no means
work locally; mostly yes means delegate. Prefer 1–2 useful workers; exceed two
only for several independent workstreams. An explicit delegation demonstration
may justify workers even when the underlying investigation is small.

Keep architecture, integration, difficult debugging, disagreements, security,
concurrency and release decisions with the lead. Critically review worker
findings and diffs, reject unsupported suggestions, integrate, inspect the final
diff and personally run relevant final verification. Fix failures caused by the
change where reasonably possible. Continue through implementation and checks;
worker completion is not task completion.

## Workers and context

Prefer available cheaper/faster models such as Luna for searches, call-path
tracing, logs, documentation/API checks, configuration checks, isolated modules,
tests, repetitive changes and independent review. Use the lowest sufficient
reasoning effort; escalate only when evidence warrants it. Reserve strong-model
reasoning for work that benefits from it. Do not invent model IDs, prices or
usage savings. If selection is unavailable, use useful built-in delegation and
report the limitation without claiming a model was selected.

Give workers only their objective, constraints, relevant paths, ownership and
expected output. Prefer no conversation fork; have workers inspect files rather
than copying large files or histories into prompts. Request a concise report:
findings, file/line evidence, recommended action, uncertainty, and changes/tests
if any. Summarize results once instead of reproducing transcripts.

Assign disjoint files/directories to implementation workers. Do not have workers
edit the same files concurrently, rebuild the same workspace concurrently, or
compete for hardware. Architecture and tightly coupled edits stay with the lead.
Use supported isolated workspaces when useful; avoid manual worktree machinery
without a clear benefit. Prefer read-only workers for investigation/review.

## Repository context and safety

- Main package: `ros2_ws/src/px4_vio_bridge/` (C++ cores/nodes, Python nodes,
  launch/config, and tests). `scripts/` holds diagnostics and replay tools.
- Start with `README.md` and the handoff matching the task. For ROS migration,
  the latest resume sections of `HANDOFF_ROS_MIGRATION.md` supersede older
  experiments. Historical test counts and passing replays are not fresh results.
- `build/`, `install/`, and `log/` trees are generated. Edit source, not installed
  copies. Preserve existing modifications, untracked reports and incomplete bags.
- Inspect git status before editing. Never reset/discard user work or perform
  destructive git operations without an explicit request. Keep edits in scope.
- Hardware/flight experiments require the relevant user authorization and handoff
  procedure. A docs task or read-only demonstration does not authorize ROS
  launches, camera use, arming, service changes or PX4 publication.
- Keep raw continuous odometry distinct from corrected mapping poses; replay
  success alone is not permission to connect an experimental path to PX4.

## Verification and report

Discover commands from `CMakeLists.txt`, README and the current handoff. Use
focused checks first, then package build/tests for runtime changes. The package
uses ament/colcon, C++ gtest and Python pytest; avoid repository-root pytest
because generated trees can cause duplicate imports. Lint dependencies alone
do not establish a configured lint target. Do not invent lint/typecheck gates.
Docs-only changes need diff, link/path, command and consistency checks, not
hardware or a full rebuild. State unavailable prerequisites and checks not run.

End substantial tasks with a concise engineering report: approach/root cause,
files changed, workers used and why, exact verification commands and results,
and remaining uncertainty. Never claim unrun tests passed or measured savings
without usage data.
