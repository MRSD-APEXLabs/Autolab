import React, { useState, useEffect, useRef } from "react";
import mqtt from "mqtt";
import ClogCheckCard from "./ClogCheckCard";
import CameraStage from "./CameraStage";
import Ot2Progress from "./Ot2Progress";
import { Brand, TitleBar } from "./Chrome";
import { GROUPS, OT2_ACTIONS, ot2Commands, stepDetail, stepGroup, stepName, stepStation, stepTitle } from "./shared/steps";
import { OT2_PHASE_LABEL, ot2ProgressFor, tipCheckStageText } from "./shared/ot2Progress";

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
const TOPIC_LABEL = { [MANIP_TOPICS[0]]: "Manipulation phase", "/planning_state": "Planner" };
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

const STATE_TONE = {
  idle: "var(--slate)",
  running: "var(--orange)",
  paused: "var(--amber)",
  success: "var(--green)",
  failed: "var(--red)",
};

function formatExecPayload(payload) {
  if (payload === null || payload === undefined) return "—";
  if (typeof payload === "string") return payload;
  // String ROS2 message: { data: "..." }
  if (payload.data !== undefined) return String(payload.data);
  return JSON.stringify(payload);
}

const pad2 = (n) => String(n).padStart(2, "0");

// One line for the timeline under the running OT-2 step: "Aspirate · clog check pass 2 of 5".
function ot2LiveLine(step, progress) {
  if (!progress) return null;
  if (progress.phase !== "command" && progress.phase !== "tip_check") return OT2_PHASE_LABEL[progress.phase] ?? null;
  const cmd = ot2Commands(step)?.[progress.command];
  const label = OT2_ACTIONS[cmd?.action]?.label ?? cmd?.action ?? `Command ${progress.command + 1}`;
  return progress.phase === "tip_check" ? `Clog check · ${tipCheckStageText(progress.tip_check)}` : label;
}

// Right-hand run timeline: one node per step, the rail filled up to the current step.
function Timeline({ steps, state, currentStepIdx, completedCount, liveLine }) {
  const total = steps.length;
  const pct = total > 0 ? Math.round((completedCount / total) * 100) : 0;
  return (
    <aside className="xt" aria-label="Run progress">
      <div className="xt-head">
        <span className="ax-h3">Progress</span>
        <div className="xt-pct"><b>{pct}</b>%</div>
        <div className="xt-count">{completedCount}/{total} Steps Completed</div>
      </div>
      <ol className="xt-list">
        {steps.map((step, i) => {
          let cls = "todo";
          if (state === "success" || i < currentStepIdx) cls = "done";
          else if (i === currentStepIdx && state === "failed") cls = "fail";
          else if (i === currentStepIdx && state !== "idle") cls = state === "paused" ? "now paused" : "now";
          const detail = (i === currentStepIdx && liveLine) || stepDetail(step);
          return (
            <li key={i} className={`xt-step ${cls}`} style={{ "--g": GROUPS[stepGroup(step)].color }}
              aria-current={cls.startsWith("now") ? "step" : undefined}>
              <i className="xt-node">{cls === "done" ? "✓" : cls === "fail" ? "✕" : i + 1}</i>
              <div className="min-w-0">
                <span className="xt-name" title={stepName(step)}>{stepTitle(step)}</span>
                {detail && <small>{detail}</small>}
              </div>
            </li>
          );
        })}
      </ol>
    </aside>
  );
}

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
  const active = state === "running" || state === "paused";
  const tone = STATE_TONE[state] ?? STATE_TONE.idle;
  const currentStep = steps[currentStepIdx];
  const displayTopics = (active && STEP_DISPLAY_TOPICS[stepName(currentStep)]) || [];
  const station = active || state === "failed" ? stepStation(currentStep) : null;
  const nextStep = active ? steps[currentStepIdx + 1] : null;
  const isOt2 = stepName(currentStep) === "ot2";
  const ot2Progress = isOt2 ? ot2ProgressFor(status) : null;
  const recordings = status?.recordings ?? [];
  // The tip check running right now, until its result is attached to the run.
  const pendingCheck = ot2Progress?.phase === "tip_check"
    && !recordings.some((r) => r?.id === `${ot2Progress.run_id}-${ot2Progress.command}`)
    ? { progress: ot2Progress.tip_check, aspirate: ot2Commands(currentStep)?.[ot2Progress.command], step: currentStepIdx }
    : null;

  // Ways out when the executor has nothing to run.
  const idleActions = (
    <div className="mt-4 flex flex-wrap gap-2">
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
    <div className="ax-page">
      <div className="ax-wrap wide">
        <header className="ax-top">
          <Brand />
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

        <TitleBar
          title="Experiment Sequence"
          right={<span className="ax-state fill" style={{ "--k": tone }}>{STATE_LABEL[state] ?? STATE_LABEL.idle}</span>}
        />

        <div className="xp">
          <div className="xp-cams"><CameraStage station={station} /></div>

          <div className="xp-detail">
            {!status && (
              <div className="ax-panel flex items-center gap-4">
                <div className="ax-spinner" />
                <p className="ax-hint">Waiting for routine executor…</p>
              </div>
            )}

            {/* Current step */}
            {status && active && currentStep && (
              <div className="ax-panel ax-accent" style={{ "--k": tone }}>
                <div className="ax-panel-head">
                  <span className="ax-h3">
                    {state === "paused" ? "Paused on" : "Now running"} · Step {currentStepIdx + 1} of {totalSteps}
                  </span>
                  {status.retry_count > 0 && (
                    <span className="ax-badge warn">Retry {status.retry_count} of {status.max_retries}</span>
                  )}
                </div>
                <div key={currentStepIdx} className="xp-now flip">
                  <b>{pad2(currentStepIdx + 1)}</b>
                  <span>{stepTitle(currentStep)}</span>
                </div>
                {stepDetail(currentStep) && !isOt2 && <p className="xp-sub">{stepDetail(currentStep)}</p>}
                {isOt2 && <Ot2Progress step={currentStep} progress={ot2Progress} />}

                {displayTopics.length > 0 && (
                  <dl className="ax-kv mt-4 pt-3" style={{ borderTop: "1px solid var(--line)" }}>
                    {displayTopics.map((ros2Topic) => (
                      <React.Fragment key={ros2Topic}>
                        <dt title={ros2Topic}>{TOPIC_LABEL[ros2Topic] ?? ros2Topic}</dt>
                        <dd>{formatExecPayload(executorStatus[`ros2${ros2Topic}`])}</dd>
                      </React.Fragment>
                    ))}
                  </dl>
                )}
                {nextStep && <p className="ax-hint mt-4">Next: {stepTitle(nextStep)}</p>}
              </div>
            )}

            {/* Failed */}
            {status && state === "failed" && (
              <div className="ax-panel ax-accent k-bad">
                <span className="ax-h3" style={{ color: "var(--red)" }}>Fault</span>
                <h3 className="xp-big">Routine Failed</h3>
                <p className="ax-mono text-sm mt-1" style={{ color: "var(--dim)" }}>
                  {status.error ?? "Unknown error"}
                </p>
                {isOt2 && ot2Progress && <Ot2Progress step={currentStep} progress={ot2Progress} />}
                {canRetry && confirmForce && (
                  <p className="text-sm mt-3 font-semibold" style={{ color: "var(--amber)" }}>
                    This step handles liquid and was not retried automatically: the tip may still hold liquid.
                    Check the OT-2 before running the step again.
                  </p>
                )}
                <div className="mt-4 flex flex-wrap gap-2">
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

            {/* Done */}
            {status && state === "success" && (
              <div className="ax-panel ax-accent k-ok">
                <span className="ax-h3" style={{ color: "var(--green)" }}>Done</span>
                <h3 className="xp-big">Routine Complete</h3>
                <p className="text-sm mt-1" style={{ color: "var(--dim)" }}>
                  All {totalSteps} steps executed successfully.
                </p>
                {onNewExperiment && (
                  <button onClick={onNewExperiment} className="ax-btn solid green mt-4">
                    New experiment
                  </button>
                )}
              </div>
            )}

            {/* Idle */}
            {status && state === "idle" && (
              <div className="ax-panel ax-accent k-none">
                <h3 className="xp-big">No routine running</h3>
                {totalSteps === 0 && (
                  <p className="text-sm mt-1" style={{ color: "var(--dim)" }}>
                    The routine executor is idle. Start a new experiment or head back home.
                  </p>
                )}
                {idleActions}
              </div>
            )}

            {/* Clog-detection verdicts for this run's aspirates (stay after the run ends) */}
            {(recordings.length > 0 || pendingCheck) && <ClogCheckCard recordings={recordings} pending={pendingCheck} />}
          </div>

          <div className="xp-time">
            {status && totalSteps > 0 ? (
              <Timeline steps={steps} state={state} currentStepIdx={currentStepIdx} completedCount={completedCount}
                liveLine={active && isOt2 ? ot2LiveLine(currentStep, ot2Progress) : null} />
            ) : (
              <aside className="xt"><span className="ax-h3">Progress</span><p className="ax-hint mt-3">No steps loaded</p></aside>
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
