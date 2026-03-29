import React, { useEffect, useState } from "react";
import ExperimentSequence from "./Experiment";


export default function LoaderPage() {
  const [done, setDone] = useState(false);

  useEffect(() => {
    const timer = setTimeout(() => setDone(true), 5000); // 5 seconds
    return () => clearTimeout(timer);
  }, []);
if (done) {
    return <ExperimentSequence />;
}

  return (
    <div className="flex flex-col items-center justify-center h-screen bg-white">
      <div className="relative w-12 h-12">
        <div className="absolute inset-0 border-4 border-gray-200 rounded-full"></div>
        <div
          className="absolute inset-0 border-4 border-orange-500 rounded-full border-t-transparent animate-spin"
          style={{ animationDuration: "1s" }}
        ></div>
      </div>
      <h2 className="mt-6 text-lg font-semibold text-gray-800">
        Preparing Your Experiment
      </h2>
      <p className="text-gray-500 text-sm mt-1">
        Setting up your workflow sequence...
      </p>
    </div>
  );
}
