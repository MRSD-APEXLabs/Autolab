import React, { useCallback, useEffect, useRef, useState } from 'react';
import { useMqttContext } from '../../shared/useMqtt';
import { CHECK_PASSES, runTipCheckToCompletion } from '../../shared/tipCheckApi';
import ConfirmButton from '../ConfirmButton';
import { COMMAND_TOPIC, STATUS_TOPIC } from './Ot2PartsControls';
import { TipCheckResult } from './TipCheckControls';

const ACK_TIMEOUT_MS = 10000;            // same as the Part 1 / Part 2 buttons
const PART_TIMEOUT_MS = 10 * 60 * 1000;  // homing + tip handling + liquid moves
const STRIKES = 3;                       // non-running statuses in a row before a part counts as failed
const STEPS = [
  ['part1', 'OT-2 Part 1: aspirate & hold fully raised above tube A1'],
  ['check', 'Clog check: L515 snapshot of the held tip -> d3:4b'],
  ['part2', 'OT-2 Part 2: dispense, discard tip'],
];
const FULL_STEPS = [
  ['firstPart1', 'Tip 0 (A1): aspirate 200 µL from source tube A1 and raise fully'],
  ['firstPart2', 'Dispense into plate A1, discard tip'],
  ['part1', 'Tip 1 (B1): aspirate 200 µL from source tube A2 and raise fully'],
  ['check', 'Run five clog checks, average predictions, then continue'],
  ['part2', 'Dispense into plate A1, discard tip'],
];
const FULL_IDLE_STEPS = Object.fromEntries(FULL_STEPS.map(([key]) => [key, 'pending']));
const IDLE_STEPS = { part1: 'pending', check: 'pending', part2: 'pending' };
const MARK = { pending: '○', running: '…', done: '✓', failed: '✗', skipped: '–' };

// Runs OT-2 Part 1, checks the tip with an L515 snapshot while Part 1 holds it fully
// raised above tube A1, then runs OT-2 Part 2. Uses the same MQTT commands as the
// Part 1 / Part 2 buttons. If anything fails after Part 1 the tip stays held above
// tube A1, and Part 2 can still be run with its own button.
export default function Ot2ClogSequence({ onRunning = () => {}, locked = false, fullWorkflow = false }) {
  const idleSteps = fullWorkflow ? FULL_IDLE_STEPS : IDLE_STEPS;
  const stepList = fullWorkflow ? FULL_STEPS : STEPS;
  const { publish, subscribe, connected } = useMqttContext();
  const [status, setStatus] = useState(null);
  const [steps, setSteps] = useState(idleSteps);
  const [running, setRunning] = useState(false);
  const [job, setJob] = useState(null);
  const [checkProgress, setCheckProgress] = useState(0);
  const [error, setError] = useState(null);
  const [note, setNote] = useState(null);
  const [tip, setTip] = useState('');
  const [tipEdited, setTipEdited] = useState(false);
  const [volume, setVolume] = useState('200');
  const [stopOnClog, setStopOnClog] = useState(false);
  const waiters = useRef(new Set());
  const mounted = useRef(true);

  useEffect(() => subscribe(STATUS_TOPIC, (message) => {
    let s;
    try { s = JSON.parse(message); } catch { return; }
    setStatus(s);
    waiters.current.forEach((w) => w(s));
  }), [subscribe]);
  useEffect(() => {
    if (connected) publish(COMMAND_TOPIC, JSON.stringify({ part: 'status' }));
  }, [connected, publish]);
  useEffect(() => {
    mounted.current = true;
    const activeWaiters = waiters.current;
    return () => {
      mounted.current = false;
      activeWaiters.forEach((waiter) => waiter.cancel?.());
    };
  }, []);
  useEffect(() => { onRunning(running); }, [running, onRunning]);

  // Suggest the tip after the one the last cycle used.
  useEffect(() => {
    if (!tipEdited && Number.isInteger(status?.tip_index)) setTip(String(Math.min(95, status.tip_index + 1)));
  }, [status, tipEdited]);

  // Publishes one part and follows its status messages with a single listener:
  // first the acknowledgement ("running", or an error), then the end ("running"
  // until ready_for_part2 / completed). ot2_parts.py sometimes reads its state file
  // while the part rewrites it and publishes a one-off needs_inspection ("Expecting
  // value ..."), so another status is requested and only a repeated one is a failure.
  const runPart = useCallback((part, body) => new Promise((resolve, reject) => {
    const goal = part === 1 ? 'ready_for_part2' : 'completed';
    let acked = false;
    let strikes = 0;
    let ackTimer = null;
    let endTimer = null;
    const finish = (err) => {
      clearTimeout(ackTimer);
      clearTimeout(endTimer);
      waiters.current.delete(waiter);
      if (err) reject(err); else resolve();
    };
    function waiter(s) {
      if (!acked) {
        if (s.status === 'running') {
          acked = true;
          clearTimeout(ackTimer);
        } else if (s.error) {
          finish(new Error(s.error));
        }
        return;
      }
      if (s.status === 'running') {
        strikes = 0;
        return;
      }
      if (s.status === goal) {
        finish();
        return;
      }
      strikes += 1;
      if (strikes >= STRIKES) finish(new Error(s.error || `Part ${part} ended with status ${s.status}`));
      else publish(COMMAND_TOPIC, JSON.stringify({ part: 'status' }));
    }
    ackTimer = setTimeout(() => finish(new Error(
      `No acknowledgement of Part ${part} within ${ACK_TIMEOUT_MS / 1000} s. Check the robot before retrying.`)),
    ACK_TIMEOUT_MS);
    endTimer = setTimeout(() => finish(new Error(
      `Part ${part} did not finish within ${PART_TIMEOUT_MS / 60000} min. Check the robot before retrying.`)),
    PART_TIMEOUT_MS);
    waiter.cancel = () => finish(new Error('Page closed; remaining workflow steps stopped.'));
    waiters.current.add(waiter);
    publish(COMMAND_TOPIC, JSON.stringify(body));
  }), [publish]);

  const start = async () => {
    setRunning(true);
    setError(null);
    setNote(null);
    setJob(null);
    setCheckProgress(0);
    setSteps({ ...idleSteps, [fullWorkflow ? 'firstPart1' : 'part1']: 'running' });
    const sourceTube = fullWorkflow ? 'A2' : 'A1';
    const selectedTip = fullWorkflow ? 1 : Number(tip);
    const selectedVolume = fullWorkflow ? 200 : Number(volume);
    try {
      if (fullWorkflow) {
        await runPart(1, { part: 1, tip_index: 0, volume: 200, source_tube: 'A1' });
        if (!mounted.current) return;
        setSteps((s) => ({ ...s, firstPart1: 'done', firstPart2: 'running' }));
        await runPart(2, { part: 2 });
        if (!mounted.current) return;
        setSteps((s) => ({ ...s, firstPart2: 'done', part1: 'running' }));
      }
      await runPart(1, fullWorkflow
        ? { part: 1, tip_index: selectedTip, volume: selectedVolume, source_tube: sourceTube }
        : { part: 1, tip_index: selectedTip, volume: selectedVolume });
      if (!mounted.current) return;
      setSteps((s) => ({ ...s, part1: 'done', check: 'running' }));

      let result;
      try {
        result = await runTipCheckToCompletion({ step: `OT-2 part 1 hold above tube ${sourceTube} (tip ${selectedTip}, ${selectedVolume} µL)` }, {
          onProgress: (count) => { if (mounted.current) setCheckProgress(count); },
        });
      } catch (e) {
        throw new Error(`Clog check failed: ${e.message}. The tip is held above tube ${sourceTube}; run Part 2 when ready.`);
      }
      if (!mounted.current) return;
      setJob(result);
      if (result?.status !== 'done') {
        throw new Error(`Clog check failed: ${result?.error || result?.status}. The tip is held above tube ${sourceTube}; run Part 2 when ready.`);
      }
      setSteps((s) => ({ ...s, check: 'done' }));
      if (result.anomaly_detected && stopOnClog && !fullWorkflow) {
        setSteps((s) => ({ ...s, part2: 'skipped' }));
        setNote(`Clog suspected, so the sequence stopped before Part 2. The tip is held above tube ${sourceTube}; run Part 2 when ready.`);
        return;
      }

      setSteps((s) => ({ ...s, part2: 'running' }));
      await runPart(2, { part: 2 });
      if (!mounted.current) return;
      setSteps((s) => ({ ...s, part2: 'done' }));
      setNote(result.anomaly_detected ? 'Sequence finished. Clog suspected at the check.' : 'Sequence finished. Tip held liquid at the check.');
    } catch (e) {
      if (!mounted.current) return;
      setSteps((s) => Object.fromEntries(Object.entries(s).map(([k, v]) => [k, v === 'running' ? 'failed' : v])));
      setError(e.message);
    } finally {
      if (mounted.current) setRunning(false);
    }
  };

  const stage = status?.status;
  const validTip = /^\d+$/.test(tip) && Number(tip) >= 0 && Number(tip) < 96;
  const validVolume = volume.trim() !== '' && Number(volume) > 0 && Number(volume) <= 200;
  const canStart = connected && !locked && !running && ['idle', 'completed'].includes(stage)
    && (fullWorkflow || (validTip && validVolume));

  return (
    <div className="border rounded p-3 mb-3">
      <h4 className="text-sm font-semibold mb-2">{fullWorkflow ? 'Full OT-2 workflow' : 'Part 1 → clog check → Part 2'}</h4>
      <p className="text-xs text-gray-600 mb-2">
        {fullWorkflow
          ? 'Transfer 200 µL from A1 to plate A1 with tip 0, then from A2 to plate A1 with tip 1. Check for clogs before the second dispense. Load fresh tips in rack A1 and B1. Keep this page open until finished.'
          : 'Runs OT-2 Part 1, checks the held tip with an L515 snapshot and d3:4b, then runs OT-2 Part 2.'}
      </p>
      {!fullWorkflow && <div className="flex flex-wrap gap-3 mb-2">
        <label className="text-xs">
          Sequence tip index (unused)
          <input type="number" min="0" max="95" step="1" value={tip} disabled={running}
                 onChange={(e) => { setTipEdited(true); setTip(e.target.value); }}
                 className="border rounded px-2 py-1 ml-2 w-20" />
        </label>
        <label className="text-xs">
          Sequence volume (µL)
          <input type="number" min="1" max="200" step="1" value={volume} disabled={running}
                 onChange={(e) => setVolume(e.target.value)}
                 className="border rounded px-2 py-1 ml-2 w-20" />
        </label>
        <label className="text-xs flex items-center gap-1">
          <input type="checkbox" checked={stopOnClog} disabled={running}
                 onChange={(e) => setStopOnClog(e.target.checked)} />
          Stop before Part 2 if a clog is suspected
        </label>
      </div>}
      {Number.isInteger(status?.tip_index) && (
        <p className="text-xs text-gray-500 mb-2">Last cycle used tip {status.tip_index}.</p>
      )}
      <ConfirmButton disabled={!canStart} active={running} onConfirm={start}>
        {fullWorkflow ? 'Run full OT-2 workflow' : 'Run Part 1 → Clog check → Part 2'}
      </ConfirmButton>
      <ol className="text-xs mt-2 space-y-0.5" aria-label="Sequence steps">
        {stepList.map(([key, label]) => (
          <li key={key} className={steps[key] === 'failed' ? 'text-red-600' : steps[key] === 'done' ? 'text-green-700' : ''}>
            <span className="inline-block w-4">{MARK[steps[key]]}</span>{label}
          </li>
        ))}
      </ol>
      {running && status?.message && <p className="text-xs text-gray-500 mt-1">OT-2: {status.message}</p>}
      {note && <p role="status" className="text-xs mt-2">{note}</p>}
      {error && <p role="alert" className="text-xs text-red-600 mt-2">{error}</p>}
      {steps.check === 'running' && <p className="text-xs mt-1">Clog predictions: {checkProgress}/{CHECK_PASSES}</p>}
      <TipCheckResult job={job} />
    </div>
  );
}
