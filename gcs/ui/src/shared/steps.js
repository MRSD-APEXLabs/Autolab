// Routine step catalog for the main UI: what each routine_executor step is called, which group it
// belongs to, and which parameters it needs. Mirrors gcs/ros_ws/src/routine_executor/step_config.py.

export const MACHINES = [
  { id: 'ot2', label: 'OT-2' },
  { id: 'shaker', label: 'Shaker' },
  { id: 'home', label: 'Home' },
];
export const machineLabel = (id) => MACHINES.find((m) => m.id === id)?.label ?? id;

// Block groups, coloured like the deck's shapes.
export const GROUPS = {
  move: { label: 'Move', color: 'var(--sky-strong)' },
  plate: { label: 'Plate', color: 'var(--navy)' },
  machine: { label: 'Lab machine', color: 'var(--orange)' },
  wait: { label: 'Wait', color: 'var(--slate)' },
};

export const OT2_ACTIONS = {
  pick_up_tips: { label: 'Pick up tip', fields: ['resource_name', 'well_indices'], resource: 'tip_rack' },
  aspirate: { label: 'Aspirate', fields: ['resource_name', 'well_indices', 'volumes'], resource: 'tube_rack' },
  dispense: { label: 'Dispense', fields: ['resource_name', 'well_indices', 'volumes'], resource: 'empty_plate' },
  drop_tips: { label: 'Drop tip', fields: [] },
  return_tips: { label: 'Return tip', fields: [] },
};
export const OT2_RESOURCES = ['tip_rack', 'tube_rack', 'empty_plate'];

export function newOt2Command(action) {
  const def = OT2_ACTIONS[action];
  const cmd = { action };
  if (def.fields.includes('resource_name')) cmd.resource_name = def.resource;
  if (def.fields.includes('well_indices')) cmd.well_indices = [0];
  if (def.fields.includes('volumes')) cmd.volumes = [100];
  return cmd;
}

// name → { group, title, params, make() }. `params` are the required ones, in display order.
export const STEP_TYPES = {
  go_to: {
    group: 'move', title: 'Go to station', params: ['target_machine'],
    make: () => ({ name: 'go_to', target_machine: 'ot2' }),
  },
  home: { group: 'move', title: 'Return home', params: [], make: () => ({ name: 'home' }) },
  go_to_pose: {
    group: 'move', title: 'Go to pose', params: ['x', 'y', 'yaw'],
    make: () => ({ name: 'go_to_pose', x: 0, y: 0, yaw: 0 }),
  },
  pick_base: { group: 'plate', title: 'Pick plate from robot', params: [], make: () => ({ name: 'pick_base' }) },
  place: {
    group: 'plate', title: 'Place plate on', params: ['target_machine'],
    make: () => ({ name: 'place', target_machine: 'ot2' }),
  },
  pick: {
    group: 'plate', title: 'Pick plate from', params: ['target_machine'],
    make: () => ({ name: 'pick', target_machine: 'ot2' }),
  },
  place_base: { group: 'plate', title: 'Place plate on robot', params: [], make: () => ({ name: 'place_base' }) },
  ot2: {
    group: 'machine', title: 'OT-2 protocol', params: ['parameters_json'],
    make: () => ({
      name: 'ot2',
      parameters_json: { steps: ['pick_up_tips', 'aspirate', 'dispense', 'drop_tips'].map(newOt2Command) },
    }),
  },
  // wait_time_s (optional): run that long, then the executor stops it; 0 / absent = set and leave.
  shaker: {
    group: 'machine', title: 'Shaker', params: ['pwm'],
    make: () => ({ name: 'shaker', pwm: 150, wait_time_s: 20 }),
  },
  wait: { group: 'wait', title: 'Wait', params: ['time_s'], make: () => ({ name: 'wait', time_s: 10 }) },
};

export const stepName = (step) => (typeof step === 'string' ? step : step?.name ?? '');

// parameters_json may arrive as a JSON string (the executor accepts both).
export function ot2Commands(step) {
  let params = step?.parameters_json;
  if (typeof params === 'string') {
    try {
      params = JSON.parse(params);
    } catch {
      return null;
    }
  }
  return Array.isArray(params?.steps) ? params.steps : null;
}

// Human title of a step: "Go to OT-2", "Run shaker", "Wait".
export function stepTitle(step) {
  const name = stepName(step);
  const s = typeof step === 'object' && step ? step : {};
  switch (name) {
    case 'go_to': return `Go to ${machineLabel(s.target_machine ?? '?')}`;
    case 'home': return 'Return home';
    case 'go_to_pose': return 'Go to pose';
    case 'pick_base': return 'Pick plate from robot';
    case 'place_base': return 'Place plate on robot';
    case 'pick': return `Pick plate from ${machineLabel(s.target_machine ?? '?')}`;
    case 'place': return `Place plate on ${machineLabel(s.target_machine ?? '?')}`;
    case 'ot2': return 'OT-2 protocol';
    case 'shaker': return Number(s.pwm) === 0 ? 'Stop shaker' : 'Run shaker';
    case 'wait': return 'Wait';
    default: return name || 'Unknown step';
  }
}

// Short "how" line: 150 PWM · 20 s · 4 commands · (x, y, yaw).
const shakerDetail = (s) => {
  if (Number(s.pwm) === 0) return '';
  const t = Number(s.wait_time_s);
  return t > 0 ? `${s.pwm} PWM · ${t} s` : `${s.pwm} PWM · until stopped`;
};

export function stepDetail(step) {
  if (!step || typeof step !== 'object') return '';
  const name = stepName(step);
  if (name === 'shaker') return shakerDetail(step);
  if (name === 'wait' && typeof step.time_s === 'number') return `${step.time_s} s`;
  if (name === 'go_to_pose') return `x ${step.x} · y ${step.y} · yaw ${step.yaw}`;
  if (name === 'ot2') {
    const cmds = ot2Commands(step);
    if (cmds) return cmds.map((c) => OT2_ACTIONS[c?.action]?.label ?? c?.action).join(' → ');
  }
  if (step.target_machine && !['go_to', 'pick', 'place'].includes(name)) return machineLabel(step.target_machine);
  return '';
}

export const stepGroup = (step) => STEP_TYPES[stepName(step)]?.group ?? 'wait';

// Which station camera shows a step best (null: the robot's own camera).
export function stepStation(step) {
  const m = typeof step === 'object' && step ? step.target_machine : null;
  if (m === 'ot2' || m === 'shaker' || m === 'home') return m;
  const name = stepName(step);
  if (name === 'ot2' || name === 'shaker') return name;
  if (name === 'home') return 'home';
  return null;
}

// Missing required parameters of a step (what the executor would reject).
export function missingParams(step) {
  const def = STEP_TYPES[stepName(step)];
  if (!def || typeof step !== 'object') return [];
  return def.params.filter((p) => step[p] === undefined || step[p] === null || step[p] === ''
    || (typeof step[p] === 'number' && !Number.isFinite(step[p])));
}
