# Snappy: design notes

Snappy is a natural-language agent for the PX4 drone, built in Python on ROS 2 Jazzy, simulation only for now. This file records what we learned and decided. It has no implementation steps. It is updated as decisions change.

## 0. Map (read this first)

One tool call travels through the agent like this. Each stage has one job.

```
user message
    |
 session.py      conductor: runs the turn, calls the others, stops the turn
    |  asks the model for the next step (ModelPort)
    |
 tools.py        does the tool exist? are the arguments well formed?
    |  bad call -> guard.record_invalid -> DENIED (invalid)
    |
 guard.py        the rules: hard deny, budget, repeat, observe-only, disarmed, supervised
    |  not allowed -> DENIED (rule)
    |
 ratelimit.py    minimum time between calls to the same tool
    |
 robot.py        RobotPort: where data comes from (FakeRobot now, ROS later)
    |
 result, wrapped as "data, not instructions", goes back to the model
```

| File | One-line purpose |
|---|---|
| `profile.py` | Loads and validates the settings (`Policy`); unknown keys fail |
| `guard.py` | Decides allow, deny or needs approval; counts calls; holds the arm switch |
| `tools.py` | Tool definitions, argument validation, the registry |
| `plans.py` | Dry-run plans: `Step`, `Plan`, the mission skill catalog, `compile_plan`, canonical JSON and hash, `render_card`, and the `submit_plan` tool (section 4b) |
| `ratelimit.py` | Per-tool minimum interval, with an injectable clock |
| `robot.py` | `RobotPort` interface plus `FakeRobot` for tests and offline chat |
| `session.py` | The turn loop; the only place that calls `begin_turn()`; turns model failures into `Model error: ...` |
| `ollama_model.py` | `OllamaModel`: our messages to Ollama's `/api/chat` and back (a `ModelPort`) |
| `px4_mapping.py` | `to_telemetry`: PX4 messages to our `Telemetry` (pure function, no ROS; NED to ENU, battery fraction to percent, enums to names) |
| `ros_robot.py` | `RosRobot`: the real `RobotPort`. Subscribes only (no publishers), keeps the newest message per topic, raises `RobotUnavailable` when nothing arrived or the last message is older than 5 s |
| `chat.py` | Terminal chat (`--robot fake\|ros`): reads a line, calls `Session`, prints the answer and a tool log, then the plan card when a new plan is pending; handles `/approve <hash>` and `/reject` itself (`python3 -m drone_agent_core.chat`) |

Five ideas to keep in mind:
1. **The model is untrusted.** It only proposes. Plain code decides.
2. **Every call takes exactly one path:** invalid, or judged by the guard. Both count against the budget.
3. **Fakes sit at the seams** (model and robot), so tests can script behavior without Ollama or ROS.
4. **Guard `armed` is the operator's permission**, not PX4's armed state.
5. **Tool results are data.** Nothing in them can unlock anything; the guard checks rules, not text.

Tests mirror the files (`tests/test_<file>.py`). `test_session.py` reads like short plays: a scripted model says X, the guard says Y.

## 1. Core idea

The language model is an untrusted advisor. It can propose, but only plain code decides and acts.

```
you -> session -> model -> run_mission call -> guard -> compiler -> plan card + hash
                                                                        |
                                                                 you /yes N
                                                                        v
        PX4 <- mux <- skills <- executor (re-checks hash and limits, ticks the tree)
```

- Prompt wording is a request. Code is an enforcement. Only code counts as a defense (disarmed refusal, approval, executor limits).
- Approval and arming come only from the command channel (what you type), never from model output or tool results.
- The executor is a separate ROS node, so safety survives a crash or hang of the agent.

## 2. Tools

- A tool is a name, a description, a JSON Schema for arguments, and a Python function.
- The model only asks ("please call X with these arguments"). Our code decides, runs it, and returns the result.
- Schemas also validate calls. Invalid calls are bounced with a readable error and count against the budget (with a separate, moderate cap).
- Tool results are data, not instructions. Text from sensors or detections is wrapped and labeled as data.
- Observe tools in Milestone 1: telemetry, YOLO detections, ROS graph inspection. Camera description is deferred to v2/v3.
- Rate limits per tool: a minimum interval or a replenishing budget. Prefer returning the cached value with its age over refusing. Conditions (such as low battery) may relax limits for observe tools only. Motion limits never relax.

## 3. Guard

- Risk levels: Observe, Annotate, Motion.
- Software arm switch: starts disarmed; `/arm` and `/disarm`. It is the operator's "the agent may request motion" permission, separate from PX4's own armed state, and is not copied from it (otherwise arming for manual flight would also grant the agent permission). It persists across turns. A later tightening rule may auto-disarm it when PX4 disarms.
- Modes: observe (no motion at all) and supervised (motion needs approval, 60 s timeout counts as no). No autonomous mode is planned.
- Check order: hard deny, budget, repeat-loop breaker, observe-only mode, disarmed, supervised approval, allow. Hard deny is first so a forbidden call always reports `hard_deny`, even past the budget.
- Budget: `max_calls_per_turn` (default 10). Every call counts, including denied ones. A turn is one user message; only the session loop calls `Guard.begin_turn()`, never the model or a tool result.
- Repeat breaker: `repeat_limit` (default 3) is how many identical calls in a row are allowed; the next one is denied with rule `repeat`. Identical means same tool name and same arguments (`None` equals `{}`). Any different call restarts the run. Slower patterns such as A, B, A, B are not caught on purpose (legitimate set-then-fetch looks the same); the budget is the backstop. The guard stores a copy of the arguments.
- `begin_turn()` resets the call counter and the repeat memory. It does not change the armed state.
- Invalid calls: the session (or tool layer) validates arguments against the tool schema before calling `decide`; the guard has no schema knowledge. A bad call goes to `Guard.record_invalid()`, which counts it against both the budget and a separate cap, `max_invalid_per_turn` (default 3, all invalid calls in the turn, not only consecutive ones), and returns a readable error. Past the cap the turn should end. `begin_turn()` resets it. Order: malformed (session), hard deny, budget, repeat, observe-only, disarmed, supervised, allow.
- A resource lock allows one motion mission at a time. It is taken when execution starts, not while approval is pending. Deferred to Milestone 3: nothing in Milestone 1 can start a mission.
- Hard-deny lists name topics and message types the agent can never touch (`/fmu/in/*`, velocity topics). Rules can only tighten.

## 4. Plans, behavior trees, executor

- The model submits a whole plan once: ordered steps of skill, arguments, retries, timeout. Skills must exist in the executor's catalog.
- The compiler validates arguments, limits, size and order. It rejects plans that violate preconditions (for example GoTo while landed). The model must add the Takeoff itself. Nothing is auto-inserted.
- The plan becomes a py_trees behavior tree: Sequence with Retry and Timeout decorators, leaf skills with initialise / update / terminate, ticked about 10 Hz.
- The tree is stored and hashed as canonical JSON (sorted keys, fixed formatting), SHA-256. The plan card shows one plain-language line per step, generated from the same tree.
- The hash proves the executed tree equals the approved tree. It does not prove the plan is sensible; you still read the card.
- The executor allows only known node types, enforces size and time limits, runs one mission at a time, clamps altitude, speed and geofence, reports step feedback, and holds position on cancel.
- Running missions are immutable. A new request becomes a separate plan with its own approval. Replans are limited to 2, each re-approved.
- Altitude check lesson: compiler rejection is a convenience; the executor clamp is the essential safety. Approval expresses intent; executor limits provide safety.

## 4b. Dry-run plans (Milestone 2, done 2026-10-23)

What exists. Nothing in it moves the drone; approval is a dry run.

- **Catalog.** `MISSION_SKILLS` (takeoff, goto, hold, land) are `ToolDef`s with `Risk.MOTION`, reusing `validate()`. They live in their own registry (`SKILLS`) and are not in the live tool registry, so the model cannot call them directly. Its only way in is the `submit_plan` tool.
- **Shape check.** `parse_plan` accepts a list of `{skill, args}` (`args` optional, unknown keys rejected) and copies the args dicts. It checks shape only; meaning is the compiler's job.
- **Compiler.** `compile_plan(plan, limits, airborne=False)` returns a list of error strings, all of them, not just the first. Checks: not empty, at most `max_steps`, each step through `check_step`, then `check_limits`, then order (no goto/hold before takeoff, no second takeoff, no land on the ground, must end with land). State (`airborne`) is tracked from the step's intent even when its arguments are bad, so one mistake does not cascade into more errors.
- **Limits.** `PlanLimits`: 12 steps, 30 m altitude, 50 m geofence radius around the start point, 120 s hold. They live in `plans.py`, not in `Policy`; moving them into the YAML profile is open.
- **Hash.** `canonical_json` (sorted keys, no whitespace, `allow_nan=False`, float parameters normalised to float so `5` and `5.0` hash alike) then SHA-256. A pinned test string is the first portable test vector for the Rust port.
- **Card.** `render_card` is built by code from the compiled plan and refuses a plan that does not compile. It shows the first 12 hash characters, one line per step, and a summary line (highest altitude, farthest point, hold time, each next to its limit). The model's own description is never what the operator approves.
- **`submit_plan`.** `Risk.ANNOTATE` (allowed when supervised, denied when observe-only). `review_plan` returns the plan or `None` plus the text, so the session can keep the plan; `submit_plan` returns just the text (the card, or `PLAN REJECTED` with the errors). The result goes through `wrap_result` like any tool result.
- **Pending plan.** `Session.pending` is a `PendingPlan` (plan, hash, submitted time). A new accepted plan replaces it; a rejected submission leaves it alone.
- **Approve and reject.** `Session.approve(hash_prefix)` and `Session.reject()` are methods, not tools: there is no approve tool in the registry, so the model cannot approve its own plan (a test pins this). Approval needs at least 8 hash characters that match the pending plan, and expires after `approval_ttl_s` from the profile (checked before the hash; an expired plan is cleared). A wrong hash leaves the plan pending. Approval returns `APPROVED <hash>. Dry run: nothing was executed.`
- **Chat.** `/approve` and `/reject` never reach the model. After each turn, if a new plan is pending, the chat prints the card and the exact `/approve` line from `session.pending`, not from the model's answer. `tool_status` labels rejected plans `plan rejected` (the wrapped result no longer starts with the problem word, which hid rejections as `ok`).

Pitfalls found while building it:
- `validate` rejected `5` for a float parameter; models write `5` for `5.0`. Ints are now accepted for float parameters (bool still rejected, check order matters).
- A comparison like `distance > radius` lets `NaN` through (`json.loads` accepts `NaN`). Write checks as "must be inside the range": `not distance <= radius`.
- `tool_schema` had no entry for `list`; a list parameter would have crashed at startup. It now emits `array` with object items.
- Live finding: the 3B model produced plans the compiler refused, misread the errors ("land only when on the ground") and stopped without resubmitting. The compiler held; a clean request ("take off to 5 m, hold 3 seconds, then land") works end to end.

Limits of this milestone: the hash covers the steps, not the limits in force; the executor's own clamps (Milestone 3) are the real safety. The compiler cannot know if the drone is armed or whether a takeoff will succeed.

## 5. Priorities and emergencies

- Tier 0 (reflex): stop and deterministic triggers. Tier 1 (express): land/RTL proposals, jump the queue, need confirmation. Tier 2 (normal): everything else, paused while a Tier 1 is pending.
- The model never declares urgency. Triggers are plain code in the executor, each with a fixed action in the profile:
  - battery critical: 10 s cancelable countdown, then land.
  - geofence or altitude breach: 10 s cancelable countdown, then the configured action.
  - agent heartbeat lost: short grace (about 3 s, tune in sim), then land, no countdown (nobody is left to veto).
  - `/emergency` typed by you: configured action (land) immediately, reported afterwards.
- `stop` means stop and hold position, never land. Landing is a separate deliberate command.
- Joystick override: the mux ignores the agent while teleop is active and publishes a status topic. The executor aborts the mission (hold, report) and never silently resumes.
- Mux priority: AUTO > NAV2 > TELEOP > AGENT > IDLE.

## 6. Session

- A command queue in (say, arm, disarm, stop, approve/deny N, emergency) and an event stream out (thinking, tool called, approval needed, step progress, refused, finished).
- The command handler never blocks. Each model turn runs as a separate task (turn worker), so commands are handled while the model thinks.
- `stop` is not queued. It calls StopAll on the executor, cancels the running turn worker, and clears pending plans and approvals.
- Plain-word stop ("stop", "halt", "freeze") only as an exact match at the start of a message.
- The turn worker runs the conversation. The executor runs the plan. They are independent.
- Python: asyncio queues; rclpy spins in a background thread. A terminal and a later dashboard use the same commands and events. This is the Milestone 3 design. In Milestone 1 `Session` is synchronous: `handle_user_message` blocks until the turn is over, and the terminal chat (`chat.py`) calls it in a plain loop. The async version arrives when `stop` and approvals must work while the model is thinking.
- Model failures: `Session` catches `ModelError` (defined next to `ModelPort`; `OllamaError` is a subclass, so `Session` never imports Ollama code). An empty reply (no text, no tool calls) is raised as a `ModelError` too. Either way the whole turn is rolled back with `del self.messages[turn_start:]`, so the history looks as if the turn never happened, and the user sees `Model error: ...`. `pop()` would not do, because the error can come after earlier passes of the same turn. Revisit at Milestone 3: once motion tools exist, rolling back would erase the record of what the drone already did.

## 7. Language model

- System prompt: rules, plan-writing rules, a skills index, robot identity. It shapes behavior and guarantees nothing.
- Ollama serves an OpenAI-compatible API at `http://localhost:11434/v1`, so the same client works for cloud providers later. Milestone 1 uses Ollama's native `/api/chat` instead (it exposes `keep_alive` and `num_ctx`, and tool results use `tool_name`); a cloud client would be a second `ModelPort` implementation.
- Server: Ollama runs in the `ollama` service of `docker-compose.yaml` (host network, GPU), with everything stored in `/mnt/ubuntu-data/ollama` because the root disk is nearly full. Model for tool calling: `qwen2.5:3b` (Q4_K_M, lists the `tools` capability). `qwen2.5vl:3b` has no tools and is kept for the map-target experiments in `px4_drone_intelligence`. The first call after loading takes about 15 s; `keep_alive` keeps the model loaded.

### Message roles

A conversation is a list of messages, each with a `role`:

| Role | Who writes it | Trust | Our internal shape | Ollama shape |
|---|---|---|---|---|
| `system` | Our code (the system prompt), added by the client on every request, not stored in the session | Trusted, but only a request to the model | n/a | `{"role":"system","content":...}` |
| `user` | The operator, typed text | Input to the model; arming and approval never come from here, only from commands | `{"role":"user","content":...}` | same |
| `assistant` | The model: text, tool calls, or both | Untrusted advice | `{"role":"assistant","content":...,"tool_calls":[{"name","args"}]}` | `tool_calls` become `[{"function":{"name","arguments"}}]` |
| `tool` | Our session: the result (or denial) of one tool call | Untrusted data, wrapped and labeled | `{"role":"tool","name":...,"content":...}` | `{"role":"tool","tool_name":...,"content":...}` |

A normal tool exchange is: `user` asks, `assistant` requests a tool, `tool` returns the result, `assistant` answers in text. The turn ends when the assistant replies without tool calls. Some APIs have more roles (OpenAI has a `developer` role, and links tool results by call id); we only use these four.
- Live check, 2026-10-22 (`qwen2.5:3b`, `FakeRobot`): tool calls come back as structured `tool_calls`, and the battery value is read from the tool result. What went wrong: (1) the model guessed `kind='topic'` because the tool menu did not list the legal values, so `list_ros_graph` now names them (`GRAPH_KINDS` in `tools.py`) and `validate` rejects anything else as `invalid`; (2) it ignored the system prompt line saying it can only observe, mixed up PX4's armed state with the operator's arm switch, and once answered about telemetry without calling the tool; (3) it sometimes returns an empty reply (not reproducible on demand). Lesson: the prompt is a request. Precise refusals will come from the guard once motion tools exist.
- Small models mis-pick tools, invent arguments, break JSON and forget preconditions. Mitigations: few tools, strict schema validation, short descriptions, a small evaluation suite per candidate model.
- Hardware: RTX 4060 Laptop 8 GB VRAM shared with the simulator, 31 GB RAM. Start with a 3-4B model at 4-bit and a 4-8k context. Model names and sizes are estimates to verify by measurement.

## 8. Profile (YAML)

One file describes the drone and every limit. Unknown keys fail at startup. Defaults are safe (disarmed, supervised). Sections: robot, ros, policy (budgets, hard-deny lists), limits (altitude, speed, geofence), tools, triggers, places, models. The agent reads policy, tools, models and places. The executor loads and validates limits and triggers itself and is the source of truth; the agent asks it for the catalog and limits. The loader prints the active limits at startup.

## 9. Testing, cheapest first

1. Unit tests: guard, profile, compiler, rate limiter.
2. Scenario tests: scripted model plus fake robot (disarmed refusal, user says no, injected text in a tool result, precondition violation, heartbeat lost, battery critical during approval).
3. Model evaluations: fixed prompts, several runs each, on the real model.
4. Simulation: failure paths first (kill the agent, cut the heartbeat, geofence). Verify with `ros2 topic info` that the agent has no publishers on `/fmu/in/*` or velocity topics.
5. Hardware: out of scope.

## 10. Milestones

1. Observe-only: profile loader, guard with rate limits, robot port, three observe tools, Ollama client, terminal chat, unit and scenario tests.
2. Dry-run plans: catalog, compiler, validator, plan card with hash. Nothing executes. **Done 2026-10-23** (section 4b).
3. Guarded execution: executor with skills through the mux, busy lock, arm and approval, heartbeat deadman, triggers, StopAll, clamps.

Later: camera tool, dashboard, cloud models, Rust port.

## 10b. ROS-backed robot (done 2026-10-22, observe-only)

- Topics read: `/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_status_v1`, `/fmu/out/battery_status_v1` (the `_v1` suffix is real for the installed px4_msgs), and `/yolo/detections` (JSON in a `std_msgs/String`; empty list when no detector runs).
- PX4 publishes BEST_EFFORT with TRANSIENT_LOCAL. A RELIABLE subscriber never connects and silently gets nothing, and `ros2 topic hz` (default RELIABLE, no QoS flags in this ROS version) shows nothing for the same reason. Use `ros2 topic echo` with `--qos-reliability best_effort --qos-durability transient_local`, or a small rclpy script.
- Measured rates: position about 50 Hz, status about 2 Hz, battery about 1 Hz, largest gap 1 s. So the 5 s staleness limit is about five times the slowest topic.
- Frames and units: PX4 local position is NED (x north, y east, z down). `Telemetry` reports ENU like the rest of ROS: `x` = east (PX4 y), `y` = north (PX4 x), `altitude_m` = -z. Battery `remaining` is a 0..1 fraction (-1 invalid, reported as `None`); `arming_state` 2 is armed; `nav_state` 4 is HOLD.
- Staleness uses `time.monotonic()`, not `time.time()`: the wall clock can jump (NTP, suspend), a monotonic clock cannot, and only durations are needed.
- No data is an error, never a zero: `RobotUnavailable` becomes an `UNAVAILABLE: ...` tool message (not a guard decision, and not a model error). A zero battery would look like a real reading.
- `rclpy` installs its own Ctrl-C handler that already shuts the context down, so `close()` calls `rclpy.shutdown()` only if `rclpy.ok()`.
- Verified live: answers match the sim (the user flew the drone: OFFBOARD, armed, altitude changing), `ros2 node info /snappy_observer` shows four subscribers and no publishers except `/parameter_events` and `/rosout`, and with the sim stopped the answer carries no numbers.
- Tool log in `chat.py`: after each answer it prints which tools ran this turn (`ok`, or the start of a `DENIED` / `UNAVAILABLE` / `RATE LIMITED` message). It comes from `Session.messages`, so it shows what our code did, not what the model claims. `[no tools used]` on a telemetry question means the model answered from memory.
- The 3B model still adds guesses and advice ("make sure the drone is charged", "arm the drone and command a take-off" although no such tool exists, "operating autonomously" for OFFBOARD). Expected; the guard is the defense.

## 11. Open items

- Which small model: `qwen2.5:3b` works for tool calls; compare others only if the answers prove too unreliable.
- Auto-disarm the guard when PX4 reports disarmed (the ROS wiring now exists; `RosRobot` could expose PX4's armed state to the guard).
- Detector status: an empty detections list cannot tell "nothing seen" from "detector not running".
- Housekeeping: pin the `ollama/ollama` image version, `.gitignore` and root-owned `__pycache__` / `.pytest_cache`, delete the old 14 GB `/usr/share/ollama/.ollama/models` copy (root disk 93% full).
- Interplay of our triggers with PX4's own failsafes; PX4 RTL bypasses the LIDAR filter.
- Move `PlanLimits` into the YAML profile; pass `airborne` to `compile_plan` from live telemetry (Milestone 3).
- Rust port: Python and Rust must print floats the same way in canonical JSON (for example `1e-07` versus `1e-7`); the shared test vectors will show it.
- The model does not self-correct after `PLAN REJECTED`; consider returning a shorter, clearer error text or a retry hint.
- The tests run on the host (`pytest pyyaml requests`, no ROS). The `sim` container only sees `snappy/` in dev mode (`docker-compose.dev.yaml`); `RosRobot` is the only part that needs the container.

## 12. Glossary

- **RTL**: Return To Launch. Climb, fly home, land. A PX4 mode.
- **Mux**: `px4_offboard_mux.py`; picks the velocity source and clamps.
- **OFFBOARD**: PX4 mode where ROS supplies setpoints.
- **Setpoint**: a target position or velocity sent to PX4.
- **NED**: north-east-down frame; altitude is negative z.
- **Heartbeat**: periodic "I'm alive" message; silence triggers the deadman.
- **Action server**: ROS interface for long-running goals with feedback and cancel.
- **Behavior tree**: tree of nodes returning Success, Failure or Running, ticked repeatedly.

## Credits

The overall design (guarded tools, approved behavior-tree missions, separate executor) is inspired by the NervROS project by Adyansh Gupta, https://github.com/Adyansh04/nervros (BSD-3-Clause). This project is an independent implementation.
