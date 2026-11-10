import React, { useState, useEffect, useRef } from "react";
import mqtt from "mqtt";
import ClogCheckCard from "./ClogCheckCard";

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

// Colors mirror the underglow LEDs (6_behavior/led_status README "State → LED").
const LED_STYLE = {
  I: { name: "Idle",    badge: "bg-teal-100 text-teal-700",     text: "text-teal-600",   dot: "bg-teal-500" },
  M: { name: "Moving",  badge: "bg-cyan-100 text-cyan-700",     text: "text-cyan-600",   dot: "bg-cyan-500" },
  A: { name: "Arm",     badge: "bg-purple-100 text-purple-700", text: "text-purple-600", dot: "bg-purple-500" },
  W: { name: "Waiting", badge: "bg-amber-100 text-amber-700",   text: "text-amber-600",  dot: "bg-amber-500" },
  E: { name: "Fault",   badge: "bg-red-100 text-red-600",       text: "text-red-600",    dot: "bg-red-500" },
  R: { name: "Ready",   badge: "bg-green-100 text-green-700",   text: "text-green-600",  dot: "bg-green-500" },
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

export default function ExperimentSequence({ onNewExperiment }) {
  const [status, setStatus] = useState(null);
  const [retryPending, setRetryPending] = useState(false);
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
    clientRef.current?.publish(RETRY_TOPIC, '');
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
    if (state !== "failed") setRetryPending(false);
  }, [state]);

  const completedCount = state === "success" ? totalSteps : currentStepIdx;

  const currentStepName = status?.current_step_name ?? null;
  const stateLabel = STATE_LABEL[state] ?? STATE_LABEL.idle;
  const led = LED_STYLE[ledFor(state, currentStepName)];
  const { badge: stateBadge, text: stateText } = led;
  const displayTopics = (currentStepName && STEP_DISPLAY_TOPICS[currentStepName]) ?? [];

  return (
    <div className="min-h-screen bg-gray-50 p-8">
      {/* Header */}
      <div className="flex items-center gap-2 mb-6">
        <div className="w-6 h-6 rounded-full bg-gray-800" />
        <h1 className="text-2xl font-semibold text-gray-800">APEX Labs</h1>
        <div className="ml-auto flex items-center gap-2">
          <button
            onClick={handleCancel}
            className="text-xs px-3 py-1 rounded-full bg-red-100 text-red-700 hover:bg-red-200 disabled:opacity-40"
            disabled={state !== "running" && state !== "paused"}
          >
            Cancel
          </button>
          {state === "running" && (
            <button
              onClick={handlePause}
              className="text-xs px-3 py-1 rounded-full bg-yellow-100 text-yellow-700 hover:bg-yellow-200"
            >
              Pause
            </button>
          )}
          {state === "paused" && (
            <button
              onClick={handleResume}
              className="text-xs px-3 py-1 rounded-full bg-green-100 text-green-700 hover:bg-green-200"
            >
              Resume
            </button>
          )}
          <span
            className={`text-xs px-2 py-1 rounded-full ${
              connected ? "bg-green-100 text-green-700" : "bg-gray-200 text-gray-500"
            }`}
          >
            {connected ? "Connected" : "Disconnected"}
          </span>
        </div>
      </div>

      {/* Title */}
      <div className="mb-4 w-full flex items-center justify-between">
        <div>
          <h2 className="text-xl font-semibold">Experiment Sequence</h2>
          <p className="text-gray-500 text-sm">Live routine executor status</p>
        </div>
        <span className={`text-sm font-medium px-3 py-1 rounded-full ${stateBadge}`}>
          {stateLabel}
        </span>
      </div>

      {/* Waiting placeholder */}
      {!status && (
        <div className="bg-white p-8 rounded-2xl shadow text-center text-gray-400 text-sm">
          Waiting for routine executor…
        </div>
      )}

      {/* Progress Tracker */}
      {status && totalSteps > 0 && (
        <div className="bg-white p-4 rounded-2xl shadow mb-8">
          <div className="flex items-center justify-between mb-2">
            <span className={`font-medium ${stateText}`}>{stateLabel}</span>
            <span className="text-sm text-gray-500">
              {completedCount}/{totalSteps} Steps Completed
            </span>
          </div>

          <div className="flex gap-2 flex-wrap">
            {steps.map((step, i) => {
              let bg;
              if (state === "success") {
                bg = "bg-green-500";
              } else if (i < currentStepIdx) {
                bg = "bg-green-500";
              } else if (i === currentStepIdx && state !== "idle") {
                bg = led.dot;
              } else {
                bg = "bg-gray-300";
              }

              return (
                <div
                  key={i}
                  title={stepName(step)}
                  className={`w-6 h-6 rounded-full ${bg} text-white flex items-center justify-center text-xs font-medium`}
                >
                  {i + 1}
                </div>
              );
            })}
          </div>

          <p className="text-sm mt-2 text-gray-500">
            {(state === "running" || state === "paused") && status.current_step_name && (
              <>
                {state === "paused" ? "Paused on:" : "Currently executing:"}{" "}
                <span className={`font-medium ${stateText}`}>
                  {status.current_step_name}
                </span>
              </>
            )}
            {state === "success" && (
              <span className="text-green-600 font-medium">Routine complete</span>
            )}
            {state === "failed" && (
              <span className="text-red-600 font-medium">
                {status.error ?? "Failed"}
              </span>
            )}
            {state === "idle" && "No routine running"}
          </p>
        </div>
      )}

      {/* Clog-detection verdicts for this run's aspirates (stay after the run ends) */}
      {status?.recordings?.length > 0 && <ClogCheckCard clipIds={status.recordings} />}

      {/* Active Step Card */}
      {status && (state === "running" || state === "paused") && status.current_step_name && (
        <div className="bg-white rounded-2xl shadow p-6">
          <div className="flex justify-between items-center mb-4">
            <h3 className="text-lg font-semibold">
              Step {currentStepIdx + 1} of {totalSteps}
            </h3>
            <span className={`flex items-center gap-2 text-sm font-medium ${stateText}`}>
              <span className={`w-2.5 h-2.5 rounded-full ${led.dot}`} />
              {state === "paused" ? "Paused" : "Running"} · {led.name}
            </span>
          </div>

          <div className="mb-4">
            <label className="text-gray-700 text-sm font-medium">Step name</label>
            <input
              className="w-full border rounded-lg p-2 mt-1 text-sm"
              value={status.current_step_name}
              readOnly
            />
          </div>

          {status.retry_count > 0 && (
            <p className="text-sm text-amber-600">
              Retry {status.retry_count} of {status.max_retries}
            </p>
          )}

          {/* Executor status for this step */}
          {displayTopics.length > 0 && (
            <div className="mt-4 border-t pt-4">
              <p className="text-xs font-semibold text-gray-500 uppercase tracking-wide mb-2">
                Executor Status
              </p>
              <div className="space-y-2">
                {displayTopics.map((ros2Topic) => {
                  const mqttTopic = `ros2${ros2Topic}`;
                  const val = executorStatus[mqttTopic];
                  return (
                    <div key={ros2Topic} className="flex items-center justify-between gap-4">
                      <span className="text-xs text-gray-400 font-mono truncate">
                        {ros2Topic}
                      </span>
                      <span className="text-xs text-gray-700 font-medium shrink-0">
                        {formatExecPayload(val)}
                      </span>
                    </div>
                  );
                })}
              </div>
            </div>
          )}
        </div>
      )}

      {/* Error Card */}
      {status && state === "failed" && (
        <div className="bg-red-50 border border-red-200 rounded-2xl p-6 text-red-700">
          <h3 className="font-semibold mb-1">Routine Failed</h3>
          <p className="text-sm">{status.error ?? "Unknown error"}</p>
          <div className="mt-4 flex flex-wrap gap-2">
            {canRetry && (
              <button
                onClick={handleRetry}
                disabled={!connected || retryPending}
                className="text-sm px-4 py-2 rounded-lg bg-red-600 text-white hover:bg-red-700 disabled:opacity-50"
              >
                {retryPending
                  ? "Retrying…"
                  : `Retry from step ${currentStepIdx + 1}${failedStepName ? ` (${failedStepName})` : ""}`}
              </button>
            )}
            {onNewExperiment && (
              <button
                onClick={onNewExperiment}
                className="text-sm px-4 py-2 rounded-lg border border-red-300 text-red-700 hover:bg-red-100"
              >
                New experiment
              </button>
            )}
          </div>
        </div>
      )}

      {/* Success Card */}
      {status && state === "success" && (
        <div className="bg-green-50 border border-green-200 rounded-2xl p-6 text-green-700">
          <h3 className="font-semibold">Routine Complete</h3>
          <p className="text-sm mt-1">
            All {totalSteps} steps executed successfully.
          </p>
          {onNewExperiment && (
            <button
              onClick={onNewExperiment}
              className="mt-4 text-sm px-4 py-2 rounded-lg bg-green-600 text-white hover:bg-green-700"
            >
              New experiment
            </button>
          )}
        </div>
      )}
    </div>
  );
}
