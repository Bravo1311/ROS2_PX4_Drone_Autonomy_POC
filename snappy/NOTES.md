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
| `ratelimit.py` | Per-tool minimum interval, with an injectable clock |
| `robot.py` | `RobotPort` interface plus `FakeRobot` for tests and offline chat |
| `session.py` | The turn loop; the only place that calls `begin_turn()`; turns model failures into `Model error: ...` |
| `ollama_model.py` | `OllamaModel`: our messages to Ollama's `/api/chat` and back (a `ModelPort`) |
| `chat.py` | Terminal chat: reads a line, calls `Session`, prints the answer (`python3 -m drone_agent_core.chat`) |

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
2. Dry-run plans: catalog, compiler, validator, plan card with hash. Nothing executes.
3. Guarded execution: executor with skills through the mux, busy lock, arm and approval, heartbeat deadman, triggers, StopAll, clamps.

Later: camera tool, dashboard, cloud models, Rust port.

## 11. Open items

- Which small model (check `ollama list` once the server is running).
- Auto-disarm the guard when PX4 reports disarmed (needs the ROS wiring).
- `/fmu/out/vehicle_status` versus `_v1` for the installed px4_msgs.
- Interplay of our triggers with PX4's own failsafes; PX4 RTL bypasses the LIDAR filter.

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
