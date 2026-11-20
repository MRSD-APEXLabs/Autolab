// import logo from './logo.svg';
import './App.css';
import './theme.css';
import React, { useEffect, useState } from 'react';
import mqtt from 'mqtt';
import LoaderPage from "./LoaderPage";
import ExperimentSequence from './Experiment';
import RoutineBuilder, { parseRoutine, routineProblems } from './RoutineBuilder';
import { Brand, TitleBar } from './Chrome';

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
      { name: 'shaker', pwm: 150, wait_time_s: 20 },
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

  const routine = parseRoutine(routineJson);
  const problems = routineProblems(routine);

  return (
    <>
      {!started && (
        <div className="ax-page ax-home-page">
          <div className="ax-wrap">
            <header className="ax-top">
              <Brand />
              <span className="ax-badge ok live"><i />System ready</span>
            </header>

            <section className="ax-home">
              <p className="ax-h3">Team H · Workcell control</p>
              <h1 className="ax-home-title">Apex Labs</h1>
              <i className="ax-rule" />
              <p className="ax-home-tag">Automated Precision EXperimentation Laboratories</p>
              <div className="flex flex-wrap gap-3 mt-8">
                <button onClick={() => setStarted(true)} className="ax-btn solid lg">
                  Begin →
                </button>
                <a href="/debug" className="ax-btn ghost lg">Debug console</a>
              </div>
            </section>

            <ol className="ax-home-steps">
              {[
                ['Build', 'Put the routine together from blocks: moves, plate handling, OT-2 and shaker.'],
                ['Send', 'The routine executor runs it step by step on the robot.'],
                ['Watch', 'Follow the run on the robot and station cameras, with progress alongside.'],
              ].map(([title, text], i) => (
                <li key={title}>
                  <b>{String(i + 1).padStart(2, '0')}</b>
                  <h3>{title}</h3>
                  <p>{text}</p>
                </li>
              ))}
            </ol>
          </div>
        </div>
      )}

      {started === 'loader' && <LoaderPage onNewExperiment={newExperiment} onHome={goHome} />}

      {started === 'experiment' && <ExperimentSequence onNewExperiment={newExperiment} onHome={goHome} />}

      {started === true && (
        <div className="ax-page">
          <div className="ax-wrap">
            <header className="ax-top">
              <Brand />
              <button onClick={() => setStarted(false)} className="ax-btn ghost">
                ← Back
              </button>
            </header>

            <TitleBar
              title="Build the routine"
              right={routine
                ? <span className="ax-badge">{routine.steps.length} step{routine.steps.length === 1 ? '' : 's'}</span>
                : <span className="ax-badge warn">Invalid JSON</span>}
            />

            <div className="ax-panel mt-6">
              <RoutineBuilder value={routineJson} onChange={setRoutineJson} />
            </div>

            <div className="rb-send">
              <div className="min-w-0">
                {problems.length > 0 ? (
                  <ul className="rb-problems">
                    {problems.slice(0, 3).map((p) => <li key={p}>{p}</li>)}
                    {problems.length > 3 && <li>and {problems.length - 3} more</li>}
                  </ul>
                ) : (
                  <p className="ax-hint">Ready to send {routine.steps.length} steps to the routine executor.</p>
                )}
              </div>
              <button onClick={handleContinue} className="ax-btn solid lg" disabled={problems.length > 0}>
                Continue →
              </button>
            </div>
          </div>
        </div>
      )}

    </>
  );
}
