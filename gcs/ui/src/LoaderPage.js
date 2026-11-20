import React, { useEffect, useState } from "react";
import ExperimentSequence from "./Experiment";
import Backdrop from "./Backdrop";


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
      <Backdrop />
      <span className="ax-mark lg ax-rise" style={{ "--i": 0 }} />
      <div className="ax-spinner mt-10 ax-rise" style={{ "--i": 1 }} />
      <h2 className="ax-eyebrow mt-8 ax-rise" style={{ "--i": 2 }}>
        Preparing Your Experiment
      </h2>
      <p className="ax-hint mt-3 ax-rise" style={{ "--i": 3 }}>
        Setting up your workflow sequence...
      </p>
    </div>
  );
}
