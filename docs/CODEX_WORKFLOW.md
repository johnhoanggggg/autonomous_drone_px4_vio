# Cost-aware Codex workflow

[AGENTS.md](../AGENTS.md) is the persistent policy. The lead decides whether
delegation reduces costly reasoning or adds useful parallelism, owns integration,
reviews worker output, and verifies the final result. Small edits stay local;
medium tasks may benefit from 1–2 workers; large tasks may justify 2–4, subject to
tool limits. These are guidelines, never quotas. Model selection is a preference
when exposed by the environment, not a guaranteed capability or pricing claim.

## Reusable task prompt

```text
Implement: <TASK>

Act as the lead engineer and optimize for correctness per unit of model usage.
Inspect the relevant repository state first.
Classify this task as small, medium, or large.

Before spawning subagents, decide whether delegation is actually likely to save
expensive-model usage, improve useful parallelism, or provide valuable independent
review. Do not spawn subagents for trivial tasks.

If small: handle it directly.
If medium: use at most 1–2 focused workers when useful.
If large: delegate independent bounded work to approximately 2–4 workers where
justified and permitted by available slots.

When model selection is available, prefer cheaper/faster models such as Luna for
clearly scoped worker tasks. Keep strong-model usage focused on planning,
architecture, integration, difficult reasoning, and final verification.

Give workers minimal necessary context. Do not allow overlapping edits.

After delegated work finishes:
1. Review worker findings.
2. Reject incorrect or unnecessary suggestions.
3. Integrate the correct solution.
4. Inspect the final diff.
5. Run relevant build/tests/lint/static checks.
6. Fix failures caused by the implementation.
7. Verify the final result yourself.

Do not stop after planning or after workers return. Continue through
implementation and verification unless genuinely blocked.

At the end report: approach/root cause, files changed, workers used and why,
verification performed, results, and remaining uncertainties.
```

Worker brief: objective; read-only or owned paths; constraints; acceptance check;
report findings with file/line evidence, recommendation, uncertainty and changes.
Do not hand every worker the full conversation. Prefer disjoint investigations
or modules; the lead makes architectural decisions and performs final checks.

## Repository-specific verification

Main implementation is in `ros2_ws/src/px4_vio_bridge`: C++ geometry/planning,
ROS components, Python flight/telemetry nodes, launch/config and `test/`.
Root `scripts/` contains bag/replay/measurement tools; handoffs record experiment
state. Inspect the relevant handoff before copying any runtime launch command.

From the repository root, standard build/test commands are:

```bash
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash  # this Pi's px4_msgs underlay
# Current migration branch on this Pi also needs the DepthAI v3 overlays:
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
cd ros2_ws
colcon build --packages-select px4_vio_bridge
source install/setup.bash
colcon test --packages-select px4_vio_bridge --event-handlers console_direct+
colcon test-result --test-result-base build/px4_vio_bridge --verbose
```

The migration branch also requires its DepthAI v3 message/driver overlays. On
this Pi, `source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash` supplies the
documented underlays; use it before the build when working on migration. These
absolute paths are machine-specific; resolve equivalents on another machine.
Use the isolated RTAB-Map overlay only for experiments that require it.

For focused Python checks, from `ros2_ws`:

```bash
python3 -m pytest src/px4_vio_bridge/test/ -q
# Or select the affected test file(s) from that directory.
```

CTest registers gtest and pytest through the package `CMakeLists.txt`. Python
tests alone do not verify the C++ targets. Avoid broad repository-root pytest
and generated install/build trees. Inspect requirements before runtime tests;
do not launch a flight graph as a generic smoke test. No repository-wide lint
or typecheck command is established here merely from manifest dependencies.

For documentation-only edits, the lead checks content, local link targets,
command locations, discovery precedence, and `git diff --check`. New untracked
files are absent from ordinary `git diff`; inspect them explicitly, for example:

```bash
git diff --no-index -- /dev/null AGENTS.md
git diff --no-index -- /dev/null docs/CODEX_WORKFLOW.md
```

Exit 1 from these two commands means differences were shown, not a test failure.

## Discovery and limitations

Start future Codex sessions in this repository. Codex discovers a nonempty root
`AGENTS.md` along its project instruction path; a same-directory
`AGENTS.override.md` takes precedence. Nested guidance may refine it. Instructions
are loaded at session start, so restart an existing session to pick up new policy.
This setup is repository guidance, not a forced model/router configuration.
It does not enable unavailable delegation tools or override platform controls.
See the [official instruction-discovery documentation](https://learn.chatgpt.com/docs/agent-configuration/agents-md).
