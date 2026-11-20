import React, { useState, useEffect, useRef } from "react";
import mqtt from "mqtt";
import ClogCheckCard from "./ClogCheckCard";
import Backdrop from "./Backdrop";

const BROKER_URL = `ws://${window.location.hostname}:9001`;
const STATUS_TOPIC = "ros2/routine_executor/status";
const CANCEL_TOPIC = "cmd/routine_executor/cancel";
const PAUSE_TOPIC = "cmd/routine_executor/pause";
const RESUME_TOPIC = "cmd/routine_executor/resume";
const RETRY_TOPIC = "cmd/routine_executor/retry";
// Give up on "Retrying…" if the executor never reports running (it publishes status at 1 Hz).
const RETRY_PENDING_MS = 5000;

// Same default robot as routine_executor/node.py. behavior.launch.xml pushes the
// /<ROBOT_NAME>/behavior namespace, so executive topics live under it.
const ROBOT = "robot_1";

// Keys are step names; values are absolute ROS2 topic paths.
// MQTT topic = "ros2" + ros2_path  (e.g. /robot_1/behavior/manipulation_phase → ros2/robot_1/behavior/manipulation_phase)
// manipulation_phase is published every executive tick; /planning_state only on planner transitions.
const MANIP_TOPICS = [`/${ROBOT}/behavior/manipulation_phase`, "/planning_state"];
const STEP_DISPLAY_TOPICS = {
  pick: MANIP_TOPICS,
  place: MANIP_TOPICS,
  pick_base: MANIP_TOPICS,
  place_base: MANIP_TOPICS,
};

const ALL_DISPLAY_MQTT_TOPICS = [
  ...new Set(
    Object.values(STEP_DISPLAY_TOPICS)
      .flat()
      .map((t) => `ros2${t}`)
  ),
];

const STATE_LABEL = {
  idle: "Idle",
  running: "Running",
  paused: "Paused",
  success: "Done",
  failed: "Failed",
};

// Colors mirror the underglow LEDs (6_behavior/led_status README "State → LED"), in the deck palette.
const LED_STYLE = {
  I: { name: "Idle",    color: "var(--teal)" },
  M: { name: "Moving",  color: "var(--cyan)" },
  A: { name: "Arm",     color: "var(--violet)" },
  W: { name: "Waiting", color: "var(--amber)" },
  E: { name: "Fault",   color: "var(--red)" },
  R: { name: "Ready",   color: "var(--green)" },
};

// Step type → LED group (led_status/node.py GROUPS). Steps with no watched BT action (wait) leave the LED idle.
const STEP_LED = {
  go_to: "M",
  home: "M",
  go_to_pose: "M",
  pick: "A",
  place: "A",
  pick_base: "A",
  place_base: "A",
  ot2: "W",
  shaker: "W",
};

function ledFor(state, stepName) {
  if (state === "success") return "R";
  if (state === "failed") return "E";
  if (state === "running" || state === "paused") return STEP_LED[stepName] ?? "I";
  return "I";
}

function formatExecPayload(payload) {
  if (payload === null || payload === undefined) return "—";
  if (typeof payload === "string") return payload;
  // String ROS2 message: { data: "..." }
  if (payload.data !== undefined) return String(payload.data);
  return JSON.stringify(payload);
}

// Short "where / how" under a step name in the timeline: ot2 · 150 pwm · 20 s · 4 commands.
function stepDetail(step) {
  if (!step || typeof step !== "object") return "";
  if (step.target_machine) return step.target_machine;
  if (typeof step.time_s === "number") return `${step.time_s} s`;
  if (step.pwm !== undefined) return Number(step.pwm) === 0 ? "stop" : `pwm ${step.pwm}`;
  let params = step.parameters_json;
  if (typeof params === "string") {
    try {
      params = JSON.parse(params);
    } catch {
      params = null;
    }
  }
  if (Array.isArray(params?.steps)) return `${params.steps.length} command${params.steps.length === 1 ? "" : "s"}`;
  return "";
}

const pad2 = (n) => String(n).padStart(2, "0");

export default function ExperimentSequence({ onNewExperiment, onHome }) {
  const [status, setStatus] = useState(null);
  const [retryPending, setRetryPending] = useState(false);
  const [confirmForce, setConfirmForce] = useState(false);
  const [connected, setConnected] = useState(false);
  const [executorStatus, setExecutorStatus] = useState({});
  const clientRef = useRef(null);

  function handleCancel() {
    clientRef.current?.publish(CANCEL_TOPIC, '');
  }

  function handlePause() {
    clientRef.current?.publish(PAUSE_TOPIC, '');
  }

  function handleResume() {
    clientRef.current?.publish(RESUME_TOPIC, '');
  }

  function handleRetry() {
    // A liquid-handling step is never re-sent without the operator confirming the machine is safe.
    if (status?.retry_needs_force && !confirmForce) {
      setConfirmForce(true);
      return;
    }
    clientRef.current?.publish(RETRY_TOPIC, status?.retry_needs_force ? JSON.stringify({ force: true }) : '');
    setConfirmForce(false);
    setRetryPending(true);
  }

  useEffect(() => {
    if (!retryPending) return undefined;
    const timer = setTimeout(() => setRetryPending(false), RETRY_PENDING_MS);
    return () => clearTimeout(timer);
  }, [retryPending]);

  useEffect(() => {
    const client = mqtt.connect(BROKER_URL);
    clientRef.current = client;

    client.on("connect", () => {
      setConnected(true);
      client.subscribe(STATUS_TOPIC);
      ALL_DISPLAY_MQTT_TOPICS.forEach((t) => client.subscribe(t));
    });

    client.on("message", (topic, payload) => {
      const raw = payload.toString();
      if (topic === STATUS_TOPIC) {
        try {
          const outer = JSON.parse(raw);
          // Real system wraps the status JSON in a { data: "<json>" } envelope
          const parsed = typeof outer?.data === "string" ? JSON.parse(outer.data) : outer;
          setStatus(parsed);
        } catch {
          // ignore malformed payloads
        }
      } else if (ALL_DISPLAY_MQTT_TOPICS.includes(topic)) {
        try {
          setExecutorStatus((prev) => ({ ...prev, [topic]: JSON.parse(raw) }));
        } catch {
          setExecutorStatus((prev) => ({ ...prev, [topic]: raw }));
        }
      }
    });

    client.on("close", () => setConnected(false));

    return () => client.end();
  }, []);

  const steps = status?.steps ?? [];
  const totalSteps = steps.length;
  const currentStepIdx = status?.current_step ?? 0;
  const state = status?.state ?? "idle";
  const stepName = (step) => (typeof step === "string" ? step : step?.name ?? "");
  const failedStepName = stepName(steps[currentStepIdx]);
  const canRetry = state === "failed" && currentStepIdx < totalSteps;

  // The executor left "failed": the retry went through.
  useEffect(() => {
    if (state !== "failed") {
      setRetryPending(false);
      setConfirmForce(false);
    }
  }, [state]);

  const completedCount = state === "success" ? totalSteps : currentStepIdx;

  const currentStepName = status?.current_step_name ?? null;
  const stateLabel = STATE_LABEL[state] ?? STATE_LABEL.idle;
  const led = LED_STYLE[ledFor(state, currentStepName)];
  const displayTopics = (currentStepName && STEP_DISPLAY_TOPICS[currentStepName]) ?? [];
  const active = state === "running" || state === "paused";
  const pct = totalSteps > 0 ? Math.round((completedCount / totalSteps) * 100) : 0;

  // Big line at the top of the run: the step being worked on, like the deck's mission-control stage.
  let stageNum = null;
  let stageText = "Waiting for executor";
  if (status) {
    if (active && currentStepName) {
      stageNum = pad2(currentStepIdx + 1);
      stageText = currentStepName;
    } else if (state === "failed") {
      stageNum = totalSteps > 0 ? pad2(Math.min(currentStepIdx + 1, totalSteps)) : null;
      stageText = failedStepName ? `${failedStepName} stopped` : "Stopped";
    } else if (state === "success") {
      stageNum = pad2(totalSteps);
      stageText = "All steps done";
    } else {
      stageText = "Standing by";
    }
  }

  // Ways out when the executor has nothing to run.
  const idleActions = (
    <div className="mt-5 flex flex-wrap gap-2">
      {onNewExperiment && (
        <button onClick={onNewExperiment} className="ax-btn solid">
          New experiment
        </button>
      )}
      {onHome && (
        <button onClick={onHome} className="ax-btn ghost">
          Back to home
        </button>
      )}
    </div>
  );

  return (
    <div className="ax-page" style={{ "--accent": led.color }}>
      <Backdrop />
      <div className="ax-wrap">
        {/* Top bar */}
        <header className="ax-top">
          <div className="ax-brand">
            <span className="ax-mark" />
            <span>APEX Labs</span>
            <span className="section">Experiment run</span>
          </div>
          <div className="ax-top-right">
            {state === "running" && (
              <button onClick={handlePause} className="ax-btn amber">
                Pause
              </button>
            )}
            {state === "paused" && (
              <button onClick={handleResume} className="ax-btn green">
                Resume
              </button>
            )}
            <button onClick={handleCancel} className="ax-btn stop" disabled={!active}>
              Cancel
            </button>
            <span className={`ax-badge ${connected ? "ok live" : ""}`}>
              <i />
              {connected ? "Connected" : "Disconnected"}
            </span>
          </div>
        </header>

        {/* Run header */}
        <section className="ax-runhead mb-8">
          <div className="min-w-0">
            <h2 className="ax-eyebrow ax-rise" style={{ "--i": 0 }}>Experiment Sequence</h2>
            <div
              key={`${state}-${currentStepIdx}`}
              className="ax-stage flip"
              style={{ "--k": led.color }}
            >
              {stageNum && <b>{stageNum}</b>}
              <span>{stageText}</span>
              {stageNum && totalSteps > 0 && <span className="of">/ {pad2(totalSteps)}</span>}
            </div>
            <p className="ax-hint mt-3">Live routine executor status</p>
          </div>
          <div className="flex items-center gap-3">
            {active && (
              <span className="ax-badge k" style={{ "--k": led.color }}>
                <i />
                {led.name}
              </span>
            )}
            <span
              className={`ax-state ${state === "idle" ? "" : "fill"}`}
              style={{ "--k": led.color }}
            >
              {stateLabel}
            </span>
          </div>
        </section>

        {/* Waiting placeholder */}
        {!status && (
          <div className="ax-panel text-center">
            <div className="ax-spinner mx-auto" />
            <p className="ax-hint mt-4">Waiting for routine executor…</p>
          </div>
        )}

        {/* Progress tracker: the deck's step timeline */}
        {status && totalSteps > 0 && (
          <div className="ax-panel ax-rise" style={{ "--i": 1 }}>
            <div className="ax-panel-head !mb-0">
              <span className="ax-h3">Routine</span>
              <span className="ax-mono text-sm" style={{ color: "var(--dim)" }}>
                {completedCount}/{totalSteps} Steps Completed
              </span>
            </div>
            <div className="ax-progress">
              <i style={{ width: `${pct}%` }} />
            </div>

            <ol className="ax-timeline">
              {steps.map((step, i) => {
                let cls = "";
                if (state === "success" || i < currentStepIdx) cls = "done";
                else if (i === currentStepIdx && state === "failed") cls = "fail";
                else if (i === currentStepIdx && state !== "idle") cls = state === "paused" ? "now paused" : "now";
                const detail = stepDetail(step);
                return (
                  <li
                    key={i}
                    title={stepName(step)}
                    className={`ax-tl ${cls}`}
                    style={{ "--cs": cls.startsWith("now") ? led.color : "var(--teal)" }}
                  >
                    <b>{pad2(i + 1)}</b>
                    <span>{stepName(step)}</span>
                    <small>{detail || "\u00a0"}</small>
                  </li>
                );
              })}
            </ol>

            <p className="text-sm mt-5" style={{ color: "var(--dim)" }}>
              {active && status.current_step_name && (
                <>
                  {state === "paused" ? "Paused on:" : "Currently executing:"}{" "}
                  <span className="font-medium" style={{ color: led.color }}>
                    {status.current_step_name}
                  </span>
                </>
              )}
              {state === "success" && (
                <span className="font-medium" style={{ color: "var(--green)" }}>Routine complete</span>
              )}
              {state === "failed" && (
                <span className="font-medium" style={{ color: "var(--red)" }}>
                  {status.error ?? "Failed"}
                </span>
              )}
              {state === "idle" && "No routine running"}
            </p>
            {state === "idle" && idleActions}
          </div>
        )}

        <div className="grid gap-[18px] mt-[18px] lg:grid-cols-[minmax(0,1.15fr)_minmax(0,1fr)] items-start">
          <div className="grid gap-[18px] min-w-0">
            {/* Active step card */}
            {status && active && status.current_step_name && (
              <div className="ax-panel ax-accent ax-rise" style={{ "--k": led.color, "--i": 2 }}>
                <div className="ax-panel-head">
                  <h3 className="ax-h3">
                    Step {currentStepIdx + 1} of {totalSteps}
                  </h3>
                  <span className="ax-badge k" style={{ "--k": led.color }}>
                    <i />
                    {state === "paused" ? "Paused" : "Running"} · {led.name}
                  </span>
                </div>

                <div className="ax-mono text-xs" style={{ color: "var(--faint)" }}>Step name</div>
                <div
                  className="mt-1"
                  style={{ font: "700 30px/1.1 var(--display)", letterSpacing: ".01em" }}
                >
                  {status.current_step_name}
                </div>
                {stepDetail(steps[currentStepIdx]) && (
                  <div className="ax-hint mt-1">{stepDetail(steps[currentStepIdx])}</div>
                )}

                {status.retry_count > 0 && (
                  <p className="ax-badge warn mt-4">
                    Retry {status.retry_count} of {status.max_retries}
                  </p>
                )}

                {/* Executor status for this step */}
                {displayTopics.length > 0 && (
                  <div className="mt-5 pt-4" style={{ borderTop: "1px solid var(--line)" }}>
                    <p className="ax-h3 mb-3">Executor Status</p>
                    <dl className="ax-kv">
                      {displayTopics.map((ros2Topic) => {
                        const val = executorStatus[`ros2${ros2Topic}`];
                        return (
                          <React.Fragment key={ros2Topic}>
                            <dt title={ros2Topic}>{ros2Topic}</dt>
                            <dd>{formatExecPayload(val)}</dd>
                          </React.Fragment>
                        );
                      })}
                    </dl>
                  </div>
                )}
              </div>
            )}

            {/* Error card */}
            {status && state === "failed" && (
              <div className="ax-panel ax-accent k-bad ax-rise" style={{ "--i": 2 }}>
                <small className="ax-mono text-xs font-bold tracking-[.18em]" style={{ color: "var(--red)" }}>
                  FAULT
                </small>
                <h3 className="mt-2" style={{ font: "700 28px/1.1 var(--display)" }}>Routine Failed</h3>
                <p className="ax-mono text-sm mt-2" style={{ color: "var(--dim)" }}>
                  {status.error ?? "Unknown error"}
                </p>
                {canRetry && confirmForce && (
                  <p className="text-sm mt-4 font-medium" style={{ color: "var(--amber)" }}>
                    This step handles liquid and was not retried automatically: the tip may still hold liquid.
                    Check the OT-2 before running the step again.
                  </p>
                )}
                <div className="mt-5 flex flex-wrap gap-2">
                  {canRetry && (
                    <button
                      onClick={handleRetry}
                      disabled={!connected || retryPending}
                      className={`ax-btn solid ${confirmForce ? "amber" : "stop"}`}
                    >
                      {retryPending
                        ? "Retrying…"
                        : confirmForce
                        ? "I've inspected the OT-2 — retry"
                        : `Retry from step ${currentStepIdx + 1}${failedStepName ? ` (${failedStepName})` : ""}`}
                    </button>
                  )}
                  {onNewExperiment && (
                    <button onClick={onNewExperiment} className="ax-btn ghost">
                      New experiment
                    </button>
                  )}
                </div>
              </div>
            )}

            {/* Idle card: executor is idle with no routine loaded */}
            {status && state === "idle" && totalSteps === 0 && (
              <div className="ax-panel ax-accent k-none ax-rise" style={{ "--i": 1 }}>
                <h3 style={{ font: "700 26px/1.1 var(--display)" }}>No routine running</h3>
                <p className="text-sm mt-2" style={{ color: "var(--dim)" }}>
                  The routine executor is idle. Start a new experiment or head back home.
                </p>
                {idleActions}
              </div>
            )}

            {/* Success card */}
            {status && state === "success" && (
              <div className="ax-panel ax-accent k-ok ax-rise" style={{ "--i": 2 }}>
                <small className="ax-mono text-xs font-bold tracking-[.18em]" style={{ color: "var(--green)" }}>
                  DONE
                </small>
                <h3 className="mt-2" style={{ font: "700 28px/1.1 var(--display)" }}>Routine Complete</h3>
                <p className="text-sm mt-2" style={{ color: "var(--dim)" }}>
                  All {totalSteps} steps executed successfully.
                </p>
                {onNewExperiment && (
                  <button onClick={onNewExperiment} className="ax-btn solid green mt-5">
                    New experiment
                  </button>
                )}
              </div>
            )}
          </div>

          {/* Clog-detection verdicts for this run's aspirates (stay after the run ends) */}
          {status?.recordings?.length > 0 && (
            <div className="min-w-0">
              <ClogCheckCard recordings={status.recordings} />
            </div>
          )}
        </div>
      </div>
    </div>
  );
}
