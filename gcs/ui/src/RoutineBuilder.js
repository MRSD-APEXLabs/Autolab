import React, { useEffect, useRef, useState } from 'react';
import {
  GROUPS, MACHINES, OT2_ACTIONS, OT2_RESOURCES, STEP_TYPES,
  missingParams, newOt2Command, ot2Commands, stepName, stepTitle,
} from './shared/steps';
import { PRESETS, matchingPreset } from './shared/presets';

const pad2 = (n) => String(n).padStart(2, '0');

export function parseRoutine(json) {
  try {
    const parsed = JSON.parse(json);
    if (parsed && typeof parsed === 'object' && Array.isArray(parsed.steps)) return parsed;
  } catch {
    // fall through
  }
  return null;
}

const serialize = (routine) => JSON.stringify(routine, null, 2);

// Problems that would make the executor reject the routine (shown before sending).
export function routineProblems(routine) {
  if (!routine) return ['The routine is not valid JSON.'];
  if (routine.steps.length === 0) return ['Add at least one step.'];
  const out = [];
  routine.steps.forEach((step, i) => {
    if (!STEP_TYPES[stepName(step)]) out.push(`Step ${i + 1}: unknown step "${stepName(step)}".`);
    const missing = missingParams(step);
    if (missing.length) out.push(`Step ${i + 1}: needs ${missing.join(', ')}.`);
    if (stepName(step) === 'ot2' && !ot2Commands(step)?.length) out.push(`Step ${i + 1}: OT-2 protocol has no commands.`);
  });
  return out;
}

// A text field whose text is kept while typing ("0, 5," is not yet a list) and committed when it
// parses. It follows outside changes whenever they differ from what the text means.
function ParsedInput({ value, parse, format, onCommit, ...props }) {
  const [text, setText] = useState(() => format(value));
  const last = useRef(value);
  useEffect(() => {
    if (JSON.stringify(value) !== JSON.stringify(last.current)) {
      last.current = value;
      setText(format(value));
    }
  }, [value, format]);
  return (
    <input
      {...props}
      value={text}
      onChange={(e) => {
        setText(e.target.value);
        const parsed = parse(e.target.value);
        if (parsed !== undefined) {
          last.current = parsed;
          onCommit(parsed);
        }
      }}
      onBlur={() => setText(format(last.current))}
    />
  );
}

const parseNumber = (t) => (t.trim() === '' || !Number.isFinite(Number(t)) ? undefined : Number(t));
const formatNumber = (v) => (v === undefined || v === null ? '' : String(v));
const parseList = (t) => {
  const parts = t.split(',').map((p) => p.trim()).filter(Boolean);
  if (!parts.length || parts.some((p) => !Number.isFinite(Number(p)))) return undefined;
  return parts.map(Number);
};
const formatList = (v) => (Array.isArray(v) ? v.join(', ') : '');

function NumberField({ label, value, onChange, unit, width = 'w-20', min, max, step }) {
  return (
    <label className="rb-field">
      <span>{label}</span>
      <span className="rb-input-wrap">
        <ParsedInput
          type="text" inputMode="decimal" className={`rb-input ${width}`} aria-label={label}
          value={value} parse={(t) => {
            const n = parseNumber(t);
            if (n === undefined) return undefined;
            if ((min !== undefined && n < min) || (max !== undefined && n > max)) return undefined;
            return n;
          }}
          format={formatNumber} onCommit={onChange} min={min} max={max} step={step}
        />
        {unit && <em>{unit}</em>}
      </span>
    </label>
  );
}

function MachinePicker({ value, onChange, options = MACHINES }) {
  return (
    <div className="rb-seg" role="radiogroup" aria-label="Station">
      {options.map((m) => (
        <button
          key={m.id} type="button" role="radio" aria-checked={value === m.id}
          className={value === m.id ? 'on' : ''} onClick={() => onChange(m.id)}
        >
          {m.label}
        </button>
      ))}
    </div>
  );
}

const STATIONS = MACHINES.filter((m) => m.id !== 'home');

function Ot2Editor({ step, onChange }) {
  const cmds = ot2Commands(step) ?? [];
  const setCmds = (next) => onChange({ ...step, parameters_json: { ...(typeof step.parameters_json === 'object' ? step.parameters_json : {}), steps: next } });
  const update = (i, patch) => setCmds(cmds.map((c, j) => (j === i ? { ...c, ...patch } : c)));
  const move = (i, d) => {
    const j = i + d;
    if (j < 0 || j >= cmds.length) return;
    const next = [...cmds];
    [next[i], next[j]] = [next[j], next[i]];
    setCmds(next);
  };

  return (
    <div className="rb-ot2">
      <ol>
        {cmds.map((cmd, i) => {
          const def = OT2_ACTIONS[cmd?.action];
          return (
            <li key={i}>
              <b>{i + 1}</b>
              <span className="rb-cmd">
              <select
                className="rb-input" aria-label={`Command ${i + 1}`} value={cmd?.action ?? ''}
                onChange={(e) => update(i, newOt2Command(e.target.value))}
              >
                {!def && <option value={cmd?.action ?? ''}>{cmd?.action ?? '?'}</option>}
                {Object.entries(OT2_ACTIONS).map(([id, a]) => <option key={id} value={id}>{a.label}</option>)}
              </select>
              {def?.fields.includes('resource_name') && (
                <select
                  className="rb-input" aria-label={`Command ${i + 1} labware`} value={cmd.resource_name ?? ''}
                  onChange={(e) => update(i, { resource_name: e.target.value })}
                >
                  {!OT2_RESOURCES.includes(cmd.resource_name) && <option value={cmd.resource_name ?? ''}>{cmd.resource_name ?? '—'}</option>}
                  {OT2_RESOURCES.map((r) => <option key={r} value={r}>{r.replace('_', ' ')}</option>)}
                </select>
              )}
              {def?.fields.includes('well_indices') && (
                <label className="rb-field inline">
                  <span>wells</span>
                  <ParsedInput
                    className="rb-input w-28" aria-label={`Command ${i + 1} wells`} value={cmd.well_indices}
                    parse={parseList} format={formatList} onCommit={(v) => update(i, { well_indices: v })}
                  />
                </label>
              )}
              {def?.fields.includes('volumes') && (
                <label className="rb-field inline">
                  <span>µL</span>
                  <ParsedInput
                    className="rb-input w-28" aria-label={`Command ${i + 1} volumes`} value={cmd.volumes}
                    parse={parseList} format={formatList} onCommit={(v) => update(i, { volumes: v })}
                  />
                </label>
              )}
              </span>
              <span className="rb-mini">
                <button type="button" onClick={() => move(i, -1)} disabled={i === 0} aria-label={`Move command ${i + 1} up`}>↑</button>
                <button type="button" onClick={() => move(i, 1)} disabled={i === cmds.length - 1} aria-label={`Move command ${i + 1} down`}>↓</button>
                <button type="button" onClick={() => setCmds(cmds.filter((_, j) => j !== i))} aria-label={`Remove command ${i + 1}`}>✕</button>
              </span>
            </li>
          );
        })}
      </ol>
      <div className="rb-ot2-add">
        {Object.entries(OT2_ACTIONS).map(([id, a]) => (
          <button key={id} type="button" onClick={() => setCmds([...cmds, newOt2Command(id)])}>+ {a.label}</button>
        ))}
      </div>
    </div>
  );
}

function BlockFields({ step, onChange }) {
  const name = stepName(step);
  const set = (patch) => onChange({ ...step, ...patch });
  switch (name) {
    case 'go_to':
      return <MachinePicker value={step.target_machine} onChange={(v) => set({ target_machine: v })} />;
    case 'pick':
    case 'place':
      return <MachinePicker value={step.target_machine} options={STATIONS} onChange={(v) => set({ target_machine: v })} />;
    case 'go_to_pose':
      return (
        <div className="flex flex-wrap gap-3">
          <NumberField label="x" unit="m" value={step.x} onChange={(v) => set({ x: v })} />
          <NumberField label="y" unit="m" value={step.y} onChange={(v) => set({ y: v })} />
          <NumberField label="yaw" unit="rad" value={step.yaw} onChange={(v) => set({ yaw: v })} />
        </div>
      );
    case 'shaker':
      return (
        <div className="flex flex-wrap items-center gap-3">
          <NumberField label="Speed" unit="PWM" value={step.pwm} min={0} max={255} onChange={(v) => set({ pwm: v })} />
          {Number(step.pwm) === 0 ? (
            <span className="rb-note">stops the shaker</span>
          ) : (
            <>
              <NumberField label="Run for" unit="s" value={step.wait_time_s ?? 0} min={0} onChange={(v) => set({ wait_time_s: v })} />
              <span className="rb-note">{Number(step.wait_time_s) > 0 ? 'then stops' : '0 s keeps it running'}</span>
            </>
          )}
        </div>
      );
    case 'wait':
      return <NumberField label="For" unit="s" value={step.time_s} min={0} onChange={(v) => set({ time_s: v })} />;
    case 'ot2':
      return <Ot2Editor step={step} onChange={onChange} />;
    default: {
      if (STEP_TYPES[name]) return null;
      const { name: _n, ...rest } = typeof step === 'object' && step ? step : {};
      return <code className="rb-note">{Object.keys(rest).length ? JSON.stringify(rest) : 'not a known step type'}</code>;
    }
  }
}

function Block({ step, index, count, onChange, onMove, onRemove, onDuplicate, dragProps, dropHint }) {
  const def = STEP_TYPES[stepName(step)];
  const group = GROUPS[def?.group] ?? GROUPS.wait;
  const missing = missingParams(step);
  const bad = !def || missing.length > 0;
  const title = ['go_to', 'pick', 'place'].includes(stepName(step)) ? def.title : stepTitle(step);
  return (
    <li
      className={`rb-block ${bad ? 'bad' : ''} ${dropHint ? `drop-${dropHint}` : ''}`}
      style={{ '--g': group.color }}
      data-testid="routine-block"
      {...dragProps}
    >
      <div className="rb-grip" title="Drag to reorder" aria-hidden="true">⋮⋮</div>
      <div className="rb-num">{pad2(index + 1)}</div>
      <div className="rb-body">
        <div className="rb-head">
          <span className="rb-group">{group.label}</span>
          <span className="rb-title">{title}</span>
          {!def && <span className="rb-flag">Unknown step</span>}
          {def && missing.length > 0 && <span className="rb-flag">Needs {missing.join(', ')}</span>}
        </div>
        <BlockFields step={step} onChange={onChange} />
      </div>
      <div className="rb-actions">
        <button type="button" onClick={() => onMove(-1)} disabled={index === 0} aria-label={`Move step ${index + 1} up`}>↑</button>
        <button type="button" onClick={() => onMove(1)} disabled={index === count - 1} aria-label={`Move step ${index + 1} down`}>↓</button>
        <button type="button" onClick={onDuplicate} aria-label={`Duplicate step ${index + 1}`}>⧉</button>
        <button type="button" onClick={onRemove} aria-label={`Remove step ${index + 1}`} className="del">✕</button>
      </div>
    </li>
  );
}

// The two ready-made experiments; picking one replaces the routine being built.
function PresetPicker({ routine, onPick }) {
  const current = matchingPreset(routine);
  return (
    <div className="rb-presets" role="radiogroup" aria-label="Ready-made experiments">
      <span className="rb-presets-label">Ready-made experiments</span>
      <div className="rb-presets-row">
        {PRESETS.map((p) => (
          <button
            key={p.id} type="button" role="radio" aria-checked={current?.id === p.id}
            className={`rb-preset k-${p.tone} ${current?.id === p.id ? 'on' : ''}`}
            onClick={() => onPick(p)}
          >
            <span className="rb-preset-title">{p.title}</span>
            <span className="rb-preset-blurb">{p.blurb}</span>
            <span className={`ax-badge ${p.tone}`}>Expect: {p.expect}</span>
          </button>
        ))}
      </div>
      {!current && <span className="rb-note">Custom routine. Pick one above to start from a ready-made experiment.</span>}
    </div>
  );
}

// Block editor for a routine_executor routine. `value` is the routine JSON string; edits come back
// through onChange as a pretty-printed JSON string, so the stored routine stays exactly what is sent.
export default function RoutineBuilder({ value, onChange }) {
  const routine = parseRoutine(value);
  const [mode, setMode] = useState(routine ? 'blocks' : 'json');
  const [drag, setDrag] = useState(null); // { from, over, where }

  const steps = routine?.steps ?? [];
  const commit = (nextSteps, extra = {}) => onChange(serialize({ ...routine, ...extra, steps: nextSteps }));
  const updateStep = (i, next) => commit(steps.map((s, j) => (j === i ? next : s)));
  const moveStep = (from, to) => {
    if (to < 0 || to >= steps.length || from === to) return;
    const next = [...steps];
    const [s] = next.splice(from, 1);
    next.splice(to, 0, s);
    commit(next);
  };

  const dragPropsFor = (i) => ({
    draggable: true,
    onDragStart: (e) => { e.dataTransfer.effectAllowed = 'move'; setDrag({ from: i, over: i, where: 'before' }); },
    onDragOver: (e) => {
      if (!drag) return;
      e.preventDefault();
      const r = e.currentTarget.getBoundingClientRect();
      const where = e.clientY < r.top + r.height / 2 ? 'before' : 'after';
      if (drag.over !== i || drag.where !== where) setDrag({ ...drag, over: i, where });
    },
    onDrop: (e) => {
      e.preventDefault();
      if (!drag) return;
      let to = drag.where === 'after' ? i + 1 : i;
      if (drag.from < to) to -= 1;
      moveStep(drag.from, to);
      setDrag(null);
    },
    onDragEnd: () => setDrag(null),
  });

  return (
    <div className="rb">
      <PresetPicker routine={routine} onPick={(p) => { onChange(serialize(p.routine)); setMode('blocks'); }} />
      <div className="rb-toolbar">
        <div className="rb-seg" role="tablist" aria-label="Editor">
          <button type="button" role="tab" aria-selected={mode === 'blocks'} className={mode === 'blocks' ? 'on' : ''}
            onClick={() => routine && setMode('blocks')} disabled={!routine}>
            Blocks
          </button>
          <button type="button" role="tab" aria-selected={mode === 'json'} className={mode === 'json' ? 'on' : ''}
            onClick={() => setMode('json')}>
            JSON
          </button>
        </div>
        {routine && (
          <NumberField
            label="Retries per step" value={routine.max_retries ?? 3} min={0} max={10} width="w-14"
            onChange={(v) => onChange(serialize({ ...routine, max_retries: v }))}
          />
        )}
      </div>

      {mode === 'json' || !routine ? (
        <>
          <textarea
            aria-label="Routine JSON" value={value} onChange={(e) => onChange(e.target.value)}
            rows="18" spellCheck={false} className="ax-code"
          />
          {!routine && <p className="ax-hint mt-2" style={{ color: 'var(--red)' }}>Fix the JSON to go back to blocks.</p>}
        </>
      ) : (
        <div className="rb-layout">
          <aside className="rb-palette" aria-label="Add a step">
            {Object.entries(GROUPS).map(([gid, g]) => (
              <div key={gid} className="rb-pgroup" style={{ '--g': g.color }}>
                <h4>{g.label}</h4>
                {Object.entries(STEP_TYPES).filter(([, t]) => t.group === gid).map(([id, t]) => (
                  <button key={id} type="button" onClick={() => commit([...steps, t.make()])} aria-label={`Add ${t.title}`}>
                    <span>+</span>{t.title}
                  </button>
                ))}
              </div>
            ))}
          </aside>
          <div className="min-w-0">
            {steps.length === 0 ? (
              <div className="rb-empty">No steps yet. Add one from the list on the left.</div>
            ) : (
              <ol className="rb-list">
                {steps.map((step, i) => (
                  <Block
                    key={i} step={step} index={i} count={steps.length}
                    onChange={(next) => updateStep(i, next)}
                    onMove={(d) => moveStep(i, i + d)}
                    onRemove={() => commit(steps.filter((_, j) => j !== i))}
                    onDuplicate={() => commit([...steps.slice(0, i + 1), JSON.parse(JSON.stringify(step)), ...steps.slice(i + 1)])}
                    dragProps={dragPropsFor(i)}
                    dropHint={drag && drag.over === i && drag.from !== i ? drag.where : null}
                  />
                ))}
              </ol>
            )}
          </div>
        </div>
      )}
    </div>
  );
}
