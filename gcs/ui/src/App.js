// import logo from './logo.svg';
import './App.css';
import './theme.css';
import React, { useEffect, useState } from 'react';
import { motion } from 'framer-motion';
import mqtt from 'mqtt';
import LoaderPage from "./LoaderPage";
import ExperimentSequence from './Experiment';
import CameraPanel from './CameraPanel';
import Backdrop from './Backdrop';

const BROKER_URL = `ws://${window.location.hostname}:9001`;
const START_CMD_TOPIC = 'cmd/routine_executor/start_routine_cmd';
const STATUS_TOPIC = 'ros2/routine_executor/status';

// Which page this tab is on, kept across reloads (Safari reloads background tabs it evicted).
// 'loader' is stored as 'experiment' so a reload skips the 5 s loader.
const VIEW_KEY = 'apex.view';
// Last routine sent, so "New experiment" (and reloads) start from it instead of the default.
const ROUTINE_KEY = 'apex.routine';

function loadRoutine() {
  try {
    return window.sessionStorage.getItem(ROUTINE_KEY) ?? DEFAULT_ROUTINE;
  } catch {
    return DEFAULT_ROUTINE;
  }
}

function saveRoutine(json) {
  try {
    window.sessionStorage.setItem(ROUTINE_KEY, json);
  } catch {
    // storage unavailable: the editor keeps it in memory only
  }
}

function loadView() {
  try {
    return window.sessionStorage.getItem(VIEW_KEY) === 'experiment' ? 'experiment' : false;
  } catch {
    return false;
  }
}

function saveView(started) {
  try {
    if (started === 'loader' || started === 'experiment') {
      window.sessionStorage.setItem(VIEW_KEY, 'experiment');
    } else {
      window.sessionStorage.removeItem(VIEW_KEY);
    }
  } catch {
    // storage unavailable (private mode): reloads fall back to the executor status check
  }
}

function routineState(payload) {
  const outer = JSON.parse(payload.toString());
  const status = typeof outer?.data === 'string' ? JSON.parse(outer.data) : outer;
  return status?.state;
}

const DEFAULT_ROUTINE = JSON.stringify(
  {
    steps: [
      { name: 'go_to', target_machine: 'ot2' },
      { name: 'pick_base' },
      { name: 'place', target_machine: 'ot2' },
      { name: 'ot2', parameters_json: {steps: [
                  {action: 'pick_up_tips', resource_name: 'tip_rack', well_indices: [0]},
                  {action: 'aspirate', resource_name: 'tube_rack', well_indices: [0], volumes: [200]},
                  {action: 'dispense', resource_name: 'empty_plate', well_indices: [0, 5, 10, 15, 20], volumes: [40, 40, 40, 40, 40]},
                  {action: 'return_tips'}
                  ]}
      },
      { name: 'pick', target_machine: 'ot2' },
      { name: 'go_to', target_machine: 'shaker' },
      { name: 'place', target_machine: 'shaker' },
      { name: 'shaker', pwm: 150 },
      { name: 'wait', 'time_s': 20 },
      { name: 'shaker', pwm: 0 },
      { name: 'pick', target_machine: 'shaker' },
      { name: 'place_base' },
      { name: 'go_to', target_machine: 'home' }
    ],
    max_retries: 3,
    robot: 'robot_1',
  },
  null,
  2
);

export default function App() {
  // false = welcome, true = routine editor, 'loader' = starting, 'experiment' = live status
  const [started, setStarted] = useState(loadView);
  const [routineJson, setRoutineJson] = useState(loadRoutine);

  useEffect(() => saveView(started), [started]);
  useEffect(() => saveRoutine(routineJson), [routineJson]);

  const newExperiment = () => setStarted(true);
  const goHome = () => setStarted(false);

  // On load, if a routine is already running (new tab, or a reload that lost storage), go straight to
  // its status. Only the first status message counts, so the user is never pulled off a page later.
  useEffect(() => {
    if (loadView()) return undefined;
    const client = mqtt.connect(BROKER_URL);
    client.on('connect', () => client.subscribe(STATUS_TOPIC));
    client.on('message', (topic, payload) => {
      if (topic !== STATUS_TOPIC) return;
      let state;
      try {
        state = routineState(payload);
      } catch {
        return;
      }
      if (state === 'running' || state === 'paused') {
        setStarted((current) => (current === false ? 'experiment' : current));
      }
      client.end();
    });
    return () => client.end();
  }, []);

  function handleContinue() {
    const client = mqtt.connect(BROKER_URL);
    client.on('connect', () => {
      client.publish(START_CMD_TOPIC, routineJson, {}, () => {
        client.end();
        setStarted('loader');
      });
    });
    // If broker is unreachable, still navigate so the UI doesn't freeze
    client.on('error', () => {
      client.end();
      setStarted('loader');
    });
  }

  let routinePreview = null;
  try {
    const parsed = JSON.parse(routineJson);
    routinePreview = Array.isArray(parsed?.steps) ? parsed.steps : null;
  } catch {
    routinePreview = null;
  }

  return (
    <>
      {!started && (
        <div className="ax-page">
          <Backdrop />
          <div className="ax-wrap">
            <header className="ax-top">
              <div className="ax-brand">
                <span className="ax-mark" />
                <span>APEX Labs</span>
                <span className="section">Workcell</span>
              </div>
              <span className="ax-badge ok live"><i />System ready</span>
            </header>

            <section className="mt-6 md:mt-16">
              <div className="ax-eyebrow ax-rise" style={{ '--i': 0 }}>
                <b>01</b> AI-powered laboratory automation
              </div>
              <h1 className="ax-headline ax-rise flex flex-wrap items-baseline gap-x-5" style={{ '--i': 1 }}>
                <span>Welcome to <em>Apex</em></span>
                <span className="ax-badge warn" style={{ fontSize: 14 }}>Alpha</span>
              </h1>
              <p className="ax-lede ax-rise" style={{ '--i': 2 }}>
                Apex combines advanced AI planning, computer vision, and precision robotics to automate laboratory
                workflows — from pipetting to automated plate handling.
              </p>
            </section>

            <div className="ax-features mt-12">
              {[
                {
                  title: 'Vision Understanding',
                  desc: 'Precise object detection and spatial analysis with high-fidelity environment mapping.',
                  k: 'var(--teal)',
                },
                {
                  title: 'Intelligent Planning',
                  desc: 'Advanced LLM planners generate optimized laboratory protocols with contextual awareness.',
                  k: 'var(--blue)',
                },
                {
                  title: 'Robotic Execution',
                  desc: 'Precision control systems with real-time feedback for reliable laboratory workflow automation.',
                  k: 'var(--violet)',
                },
              ].map((item, i) => (
                <motion.div
                  key={i}
                  initial={{ opacity: 0, y: 24 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ delay: 0.3 + i * 0.09, duration: 0.8, ease: [0.16, 1, 0.3, 1] }}
                  className="ax-feature"
                  style={{ '--k': item.k }}
                >
                  <b>{String(i + 1).padStart(2, '0')}</b>
                  <h3>{item.title}</h3>
                  <p>{item.desc}</p>
                </motion.div>
              ))}
            </div>

            <div className="ax-panel ax-rise mt-10 flex flex-wrap items-center justify-between gap-4" style={{ '--i': 5 }}>
              <div>
                <div className="ax-h3">Getting started</div>
                <p className="mt-2 text-[15px]" style={{ color: 'var(--dim)' }}>
                  Begin by mapping your workspace objects.
                </p>
              </div>
              <button onClick={() => setStarted(true)} className="ax-btn solid lg">
                Begin →
              </button>
            </div>

            <p className="ax-hint mt-6" style={{ color: 'var(--green)' }}>● AI system initialized</p>
          </div>
        </div>
      )}

      {started === 'loader' && <LoaderPage onNewExperiment={newExperiment} onHome={goHome} />}

      {started === 'experiment' && <ExperimentSequence onNewExperiment={newExperiment} onHome={goHome} />}

      {started === true && (
        <div className="ax-page">
          <Backdrop />
          <div className="ax-wrap narrow">
            <header className="ax-top">
              <div className="ax-brand">
                <span className="ax-mark" />
                <span>APEX Labs</span>
                <span className="section">New experiment</span>
              </div>
              <span className="ax-badge ok live"><i />AI Planning Ready</span>
            </header>

            <div className="ax-eyebrow ax-rise" style={{ '--i': 0 }}>
              <b>02</b> Routine
            </div>
            <h1 className="ax-headline sm ax-rise" style={{ '--i': 1 }}>
              Describe the <em>run</em>
            </h1>

            <div className="ax-panel ax-rise mt-8" style={{ '--i': 2 }}>
              <div className="ax-panel ax-accent k-busy !shadow-none !p-4 mb-6" style={{ backdropFilter: 'none' }}>
                <span className="text-sm" style={{ color: 'var(--dim)' }}>
                  <strong style={{ color: 'var(--text)' }}>Objects detected:</strong> 3 items with spatial coordinates loaded.
                </span>
              </div>

              <div className="ax-panel-head">
                <label htmlFor="routine-json" className="ax-h3">Routine Command (JSON)</label>
                {routinePreview ? (
                  <span className="ax-badge ok">{routinePreview.length} step{routinePreview.length === 1 ? '' : 's'}</span>
                ) : (
                  <span className="ax-badge warn">Invalid JSON</span>
                )}
              </div>
              <textarea
                id="routine-json"
                value={routineJson}
                onChange={(e) => setRoutineJson(e.target.value)}
                rows="14"
                spellCheck={false}
                className="ax-code"
              />
              <p className="ax-hint mt-2">
                Edit the JSON directly to customise steps and parameters before sending.
              </p>

              {routinePreview && routinePreview.length > 0 && (
                <div className="ax-chips mt-5">
                  {routinePreview.map((step, i) => (
                    <span key={i}>
                      <b style={{ color: 'var(--faint)', marginRight: 8 }}>{String(i + 1).padStart(2, '0')}</b>
                      {typeof step === 'string' ? step : step?.name ?? '?'}
                      {step?.target_machine ? ` · ${step.target_machine}` : ''}
                    </span>
                  ))}
                </div>
              )}

              <div className="flex justify-between items-center mt-8">
                <button onClick={() => setStarted(false)} className="ax-btn ghost">
                  ← Back
                </button>
                <button onClick={handleContinue} className="ax-btn solid lg">
                  Continue →
                </button>
              </div>
            </div>

            <p className="ax-hint mt-6">No coding required</p>
          </div>
        </div>
      )}

      <CameraPanel />
    </>
  );
}
