import React, { useState, useEffect } from "react";
import mqtt from "mqtt";

const BROKER_URL = `ws://${window.location.hostname}:9001`;
const STATUS_TOPIC = "ros2/routine_executor/status";

const STATE_STYLE = {
  idle:    { label: "Idle",    badge: "bg-gray-200 text-gray-600",     text: "text-gray-500" },
  running: { label: "Running", badge: "bg-orange-100 text-orange-600", text: "text-orange-600" },
  success: { label: "Done",    badge: "bg-green-100 text-green-700",   text: "text-green-600" },
  failed:  { label: "Failed",  badge: "bg-red-100 text-red-600",       text: "text-red-600" },
};

export default function ExperimentSequence() {
  const [status, setStatus] = useState(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const client = mqtt.connect(BROKER_URL);

    client.on("connect", () => {
      setConnected(true);
      client.subscribe(STATUS_TOPIC);
    });

    client.on("message", (_topic, payload) => {
      try {
        setStatus(JSON.parse(payload.toString()));
      } catch {
        // ignore malformed payloads
      }
    });

    client.on("close", () => setConnected(false));

    return () => client.end();
  }, []);

  const steps = status?.steps ?? [];
  const totalSteps = steps.length;
  const currentStepIdx = status?.current_step ?? 0;
  const state = status?.state ?? "idle";
  const { label: stateLabel, badge: stateBadge, text: stateText } =
    STATE_STYLE[state] ?? STATE_STYLE.idle;

  const completedCount = state === "success" ? totalSteps : currentStepIdx;

  return (
    <div className="min-h-screen bg-gray-50 p-8">
      {/* Header */}
      <div className="flex items-center gap-2 mb-6">
        <div className="w-6 h-6 rounded-full bg-gray-800" />
        <h1 className="text-2xl font-semibold text-gray-800">APEX Labs</h1>
        <span
          className={`ml-auto text-xs px-2 py-1 rounded-full ${
            connected ? "bg-green-100 text-green-700" : "bg-gray-200 text-gray-500"
          }`}
        >
          {connected ? "Connected" : "Disconnected"}
        </span>
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
            {steps.map((name, i) => {
              let bg;
              if (state === "success") {
                bg = "bg-green-500";
              } else if (i < currentStepIdx) {
                bg = "bg-green-500";
              } else if (i === currentStepIdx && state === "running") {
                bg = "bg-orange-500";
              } else if (i === currentStepIdx && state === "failed") {
                bg = "bg-red-500";
              } else {
                bg = "bg-gray-300";
              }

              return (
                <div
                  key={i}
                  title={name}
                  className={`w-6 h-6 rounded-full ${bg} text-white flex items-center justify-center text-xs font-medium`}
                >
                  {i + 1}
                </div>
              );
            })}
          </div>

          <p className="text-sm mt-2 text-gray-500">
            {state === "running" && status.current_step_name && (
              <>
                Currently executing:{" "}
                <span className="text-orange-600 font-medium">
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

      {/* Active Step Card */}
      {status && state === "running" && status.current_step_name && (
        <div className="bg-white rounded-2xl shadow p-6">
          <div className="flex justify-between items-center mb-4">
            <h3 className="text-lg font-semibold">
              Step {currentStepIdx + 1} of {totalSteps}
            </h3>
            <span className="text-sm text-orange-500 font-medium">Running</span>
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
        </div>
      )}

      {/* Error Card */}
      {status && state === "failed" && (
        <div className="bg-red-50 border border-red-200 rounded-2xl p-6 text-red-700">
          <h3 className="font-semibold mb-1">Routine Failed</h3>
          <p className="text-sm">{status.error ?? "Unknown error"}</p>
        </div>
      )}

      {/* Success Card */}
      {status && state === "success" && (
        <div className="bg-green-50 border border-green-200 rounded-2xl p-6 text-green-700">
          <h3 className="font-semibold">Routine Complete</h3>
          <p className="text-sm mt-1">
            All {totalSteps} steps executed successfully.
          </p>
        </div>
      )}
    </div>
  );
}
