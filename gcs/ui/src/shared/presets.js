// Ready-made experiments for the routine builder. All run the full plate flow (OT-2 work, then the
// shaker). No anomaly / Anomaly differ only in the tube the OT-2 aspirates from, which decides what
// the clog check (anomaly detection) should find right after the aspirate. Gradient fills plate
// column 1 with red fading into yellow. Tubes and tips: shared/tips.js.
import { EMPTY_TUBE, TIP_FOR_EMPTY_TUBE, TIP_FOR_LIQUID, TUBE_FOR_LIQUID } from './tips';

const TUBE_WITH_LIQUID = TUBE_FOR_LIQUID.red; // A1, red: easy for the tip camera to see in the tip

function experiment(ot2Steps) {
  return {
    steps: [
      { name: 'go_to', target_machine: 'ot2' },
      { name: 'pick_base' },
      { name: 'place', target_machine: 'ot2' },
      {
        name: 'ot2',
        parameters_json: { steps: ot2Steps },
      },
      { name: 'pick', target_machine: 'ot2' },
      { name: 'go_to', target_machine: 'shaker' },
      { name: 'place', target_machine: 'shaker' },
      { name: 'shaker', pwm: 150, wait_time_s: 20 },
      { name: 'pick', target_machine: 'shaker' },
      { name: 'place_base' },
      { name: 'go_to', target_machine: 'home' },
    ],
    max_retries: 3,
    robot: 'robot_1',
  };
}

// One tip: aspirate 200 µL from the tube, 40 µL into each of five wells, return the tip.
function singleTube(tube, tip) {
  return experiment([
    { action: 'pick_up_tips', resource_name: 'tip_rack', well_indices: [tip] },
    { action: 'aspirate', resource_name: 'tube_rack', well_indices: [tube], volumes: [200] },
    { action: 'dispense', resource_name: 'empty_plate', well_indices: [0, 5, 10, 15, 20], volumes: [40, 40, 40, 40, 40] },
    { action: 'return_tips' },
  ]);
}

// Gradient down plate column 1 (wells 0-3 = A1-D1 on the 96-well plate): red goes 100% → 0% in equal
// steps and yellow makes up the rest. Each liquid is one full-tip aspirate (GRADIENT_ASPIRATE_UL) split
// over its wells with nothing left in the tip: red 100, 67, 33, 0 µL and yellow the mirror image, so
// every well holds 100 µL. Volumes are whole µL, rounded so each liquid's dispenses add up to exactly
// the aspirate; a well that gets 0 µL is left out of the dispense.
// Four wells because no single dispense may be under MIN_DISPENSE_UL: the smallest share of 200 µL
// over n wells is 400 / (n·(n−1)) µL, 33 µL for 4 wells but 20 µL for 5.
export const GRADIENT_WELLS = [0, 1, 2, 3];
export const GRADIENT_ASPIRATE_UL = 200;
export const MIN_DISPENSE_UL = 30;

// Red's share per well, falling linearly to 0, summing to exactly GRADIENT_ASPIRATE_UL.
function redShares() {
  const last = GRADIENT_WELLS.length - 1;
  const weight = (last * (last + 1)) / 2;
  const shares = GRADIENT_WELLS.map((_, i) => Math.round((GRADIENT_ASPIRATE_UL * (last - i)) / weight));
  shares[0] += GRADIENT_ASPIRATE_UL - shares.reduce((a, b) => a + b, 0); // rounding leftover, if any
  return shares;
}

function gradientShare(liquid, tip, volumes) {
  const wells = GRADIENT_WELLS.filter((_, i) => volumes[i] > 0);
  const amounts = volumes.filter((v) => v > 0);
  return [
    { action: 'pick_up_tips', resource_name: 'tip_rack', well_indices: [tip] },
    { action: 'aspirate', resource_name: 'tube_rack', well_indices: [TUBE_FOR_LIQUID[liquid]], volumes: [GRADIENT_ASPIRATE_UL] },
    { action: 'dispense', resource_name: 'empty_plate', well_indices: wells, volumes: amounts },
    { action: 'return_tips' },
  ];
}

function gradient() {
  const red = redShares();
  const yellow = [...red].reverse();
  return experiment([
    ...gradientShare('red', TIP_FOR_LIQUID.red, red),
    ...gradientShare('yellow', TIP_FOR_LIQUID.yellow, yellow),
  ]);
}

export const PRESETS = [
  {
    id: 'normal',
    title: 'No anomaly',
    blurb: 'Aspirates 200 µL from tube A1 (red liquid). The clog check should see liquid in the tip.',
    expect: 'Liquid in tip',
    tone: 'ok',
    routine: singleTube(TUBE_WITH_LIQUID, TIP_FOR_LIQUID.red),
  },
  {
    id: 'anomaly',
    title: 'Anomaly',
    blurb: 'Aspirates from slot A3 (listed as blue), which has no tube. The tip comes up dry and the clog check should flag it.',
    expect: 'Clog suspected',
    tone: 'bad',
    routine: singleTube(EMPTY_TUBE, TIP_FOR_EMPTY_TUBE),
  },
  {
    id: 'gradient',
    title: 'Gradient',
    blurb: 'Fills A1–D1 of plate column 1, 100 µL per well, from all red to all yellow in even steps. Each tip aspirates 200 µL and dispenses all of it. The clog check should see liquid in both tips.',
    expect: 'Red → yellow column',
    tone: 'ok',
    routine: gradient(),
  },
];

// The preset a routine is exactly (same JSON), or null once it was edited.
export function matchingPreset(routine) {
  if (!routine) return null;
  const json = JSON.stringify(routine);
  return PRESETS.find((p) => JSON.stringify(p.routine) === json) ?? null;
}
