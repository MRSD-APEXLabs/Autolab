import React, { useEffect, useRef, useState } from 'react';
import { planToRoutine, usesLiquid } from './shared/nlpRoutine';

// "Describe the experiment": a plain-English request goes to the language planner (gcs/nlp_proxy,
// POST /api/nlp) and its routine replaces the one in the builder. A refusal or an error leaves the
// routine as it is. Answers are LIVE (the model just produced it) or RECORDED (saved earlier for
// exactly this request, used when the model does not answer in time).

// The demo deck's saved prompts (FVD-Presentation/config.json "nlp.prompts"); all have saved answers.
export const DEMO_PROMPTS = [
  { label: 'Plain request', task: 'Please combine the brown liquid with the clear liquid, then shake the mixture.' },
  { label: 'Amounts and units', task: 'Mix 0.1 ml red with 0.05 ml clear and shake for 0.5 minutes.' },
  { label: 'Unknown liquid', task: 'Mix brown and blue liquids and shake them.' },
];

const REQUEST_TIMEOUT_MS = 30000;
const STATUS_EVERY_MS = 60000;

async function askPlanner(task, signal) {
  let res;
  try {
    res = await fetch('/api/nlp', {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ task }), signal,
    });
  } catch (err) {
    return { ok: false, error: err?.name === 'AbortError' ? 'no answer within 30 s' : 'nlp-proxy unreachable' };
  }
  try {
    return await res.json();
  } catch {
    return { ok: false, error: `nlp-proxy answered HTTP ${res.status}` };
  }
}

function resultFor(answer, routine) {
  if (!answer?.ok) return { tone: 'bad', text: `Planner unavailable: ${answer?.error || 'no answer'}` };
  if (answer.plan?.error) return { tone: 'warn', text: `The planner refused: ${answer.plan.error}` };
  let next;
  try {
    next = planToRoutine(answer.plan, routine);
  } catch (err) {
    return { tone: 'bad', text: `Planner unavailable: ${err.message}` };
  }
  const how = answer.source === 'live'
    ? `LIVE · ${Number(answer.seconds).toFixed(1)} s`
    : 'RECORDED (planner offline)';
  return {
    tone: 'ok',
    text: `Built ${next.steps.length} steps from your request · ${how}`,
    note: usesLiquid(next, 'clear')
      ? 'Clear liquid is hard for the tip camera to see, so its aspirate skips the clog check.'
      : '',
    routine: next,
  };
}

function useStatus() {
  const [status, setStatus] = useState(null);
  useEffect(() => {
    let alive = true;
    const check = async () => {
      let next;
      try {
        const res = await fetch('/api/nlp/status');
        next = await res.json();
      } catch {
        next = { state: 'down', detail: 'nlp-proxy unreachable' };
      }
      if (alive) setStatus(next);
    };
    check();
    const timer = setInterval(check, STATUS_EVERY_MS);
    return () => { alive = false; clearInterval(timer); };
  }, []);
  return status;
}

export default function NlpPanel({ routine, onRoutine }) {
  const [text, setText] = useState('');
  const [busy, setBusy] = useState(false);
  const [result, setResult] = useState(null);
  const pending = useRef(false);
  const alive = useRef(true);
  const status = useStatus();
  // Set on every mount: StrictMode (dev) mounts, unmounts and remounts, keeping refs.
  useEffect(() => {
    alive.current = true;
    return () => { alive.current = false; };
  }, []);

  const send = async (task) => {
    const t = task.trim();
    if (!t || pending.current) return;
    pending.current = true;
    setBusy(true);
    setResult(null);
    const ctrl = new AbortController();
    const timer = setTimeout(() => ctrl.abort(), REQUEST_TIMEOUT_MS);
    const answer = await askPlanner(t, ctrl.signal);
    clearTimeout(timer);
    pending.current = false;
    if (!alive.current) return;
    const r = resultFor(answer, routine);
    setBusy(false);
    setResult(r);
    if (r.routine) onRoutine(r.routine);
  };

  const ready = status?.state === 'ready';
  return (
    <section className="rb-nlp" aria-label="Describe the experiment">
      <div className="rb-nlp-head">
        <span className="rb-presets-label">Describe the experiment</span>
        {status && (
          <span className={`ax-badge ${ready ? 'ok' : 'bad'}`} title={ready ? status.model || '' : status.detail || ''}>
            <i />{ready ? 'Planner ready' : 'Planner offline'}
          </span>
        )}
      </div>
      <form className="rb-nlp-form" onSubmit={(e) => { e.preventDefault(); send(text); }}>
        <input
          className="rb-input rb-nlp-input" aria-label="Request" value={text} maxLength={500}
          placeholder="Mix 0.1 ml red with 0.05 ml clear and shake for 30 seconds"
          onChange={(e) => setText(e.target.value)}
        />
        <button type="submit" className="ax-btn solid" disabled={busy || !text.trim()}>Build</button>
      </form>
      <div className="rb-nlp-chips">
        <span className="rb-note">Try:</span>
        {DEMO_PROMPTS.map((p) => (
          <button key={p.label} type="button" disabled={busy} onClick={() => { setText(p.task); send(p.task); }}>
            {p.label}
          </button>
        ))}
      </div>
      {busy && <p className="rb-nlp-result" role="status">Planning…</p>}
      {result && (
        <div className={`rb-nlp-result k-${result.tone}`} role="status">
          <p>{result.text}</p>
          {result.note && <p className="rb-note">{result.note}</p>}
        </div>
      )}
    </section>
  );
}
