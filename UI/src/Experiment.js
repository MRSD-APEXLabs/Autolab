import React, { useState } from "react";

export default function ExperimentSequence() {
  const [currentStep, setCurrentStep] = useState(8);
  const totalSteps = 18;

  return (
    <div className="min-h-screen bg-gray-50 p-8">
      {/* Header */}
      <div className="flex items-center gap-2 mb-6">
        <div className="w-6 h-6 rounded-full bg-gray-800" />
        <h1 className="text-2xl font-semibold text-gray-800">APEX Labs</h1>
      </div>

    {/* Title */}
    <div className="mb-4 w-full flex items-center justify-between">
        <div>
            <h2 className="text-xl font-semibold">Experiment Sequence</h2>
            <p className="text-gray-500 text-sm">Review and execute your workflow</p>
        </div>
        <button
            className="ml-4 bg-orange-500 hover:bg-orange-600 text-white text-sm font-medium px-4 py-2 rounded-lg"
            onClick={() => setCurrentStep((s) => Math.min(s + 1, totalSteps))}
        >
            Continue
        </button>
    </div>

    {/* Progress Tracker */}
      <div className="bg-white p-4 rounded-2xl shadow mb-8">
        <div className="flex items-center justify-between mb-2">
          <span className="text-orange-600 font-medium">Executing...</span>
          <span className="text-sm text-gray-500">
            {currentStep - 1}/{totalSteps} Steps Completed
          </span>
        </div>

        <div className="flex gap-2">
          {Array.from({ length: totalSteps }, (_, i) => {
            const step = i + 1;
            const state =
              step < currentStep
                ? "bg-green-500"
                : step === currentStep
                ? "bg-orange-500"
                : "bg-gray-300";
            return (
              <div
                key={step}
                className={`w-6 h-6 rounded-full ${state} text-white flex items-center justify-center text-xs font-medium`}
              >
                {step}
              </div>
            );
          })}
        </div>

        <p className="text-sm mt-2 text-gray-500">
          Currently executing:{" "}
          <span className="text-orange-600 font-medium">Step {currentStep}</span>
        </p>
      </div>

      {/* Step Card */}
      <div className="bg-white rounded-2xl shadow p-6">
        <div className="flex justify-between items-center mb-4">
          <h3 className="text-lg font-semibold">Step {currentStep}</h3>
          <span className="text-sm text-orange-500 font-medium">Running</span>
        </div>

        <div className="mb-4">
          <label className="text-gray-700 text-sm font-medium">Step</label>
          <input
            className="w-full border rounded-lg p-2 mt-1 text-sm"
            value="Step 8: Pick up three new pipette tips from pipette_tip_box"
            readOnly
          />
        </div>

        <div className="mb-4">
          <label className="text-gray-700 text-sm font-medium">
            Description
          </label>
          <textarea
            rows={4}
            className="w-full border rounded-lg p-2 mt-1 text-sm"
            value={`1. xarm7 moves the pipette to 30cm above the center of the pipette_tip_box.\n2. ATTACH_PIPETTE_TIP, count=3\n3. xarm7 raises the pipette to 30cm above the pipette_tip_box.`}
            readOnly
          />
        </div>

        <div>
          <label className="text-gray-700 text-sm font-medium">
            Robotic Code
          </label>
          <pre className="bg-gray-100 border rounded-lg p-3 mt-1 text-sm overflow-auto">
{`{
  "move_above_tip_box": [
    { "xarm": "-1" },
    { "-1": "" }
  ]
}`}
          </pre>
        </div>
      </div>
    </div>
  );
}
