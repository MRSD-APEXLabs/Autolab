import React, { useEffect, useState } from "react";
import ExperimentSequence from "./Experiment";

export default function LoaderPage({ onNewExperiment, onHome }) {
  const [done, setDone] = useState(false);

  useEffect(() => {
    const timer = setTimeout(() => setDone(true), 5000); // 5 seconds
    return () => clearTimeout(timer);
  }, []);

  if (done) {
    return <ExperimentSequence onNewExperiment={onNewExperiment} onHome={onHome} />;
  }

  return (
    <div className="ax-page flex flex-col items-center justify-center">
      <span className="ax-mark lg" />
      <h2 className="ax-display mt-8">Preparing your experiment</h2>
      <div className="ax-loadbar mt-6"><i /></div>
      <p className="ax-hint mt-4">Sending the routine to the executor…</p>
    </div>
  );
}
