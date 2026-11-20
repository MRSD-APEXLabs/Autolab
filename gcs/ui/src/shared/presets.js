// Ready-made experiments for the routine builder. Both run the full plate flow (OT-2 transfer, then
// the shaker); they differ only in the tube the OT-2 aspirates from, which decides what the clog
// check (anomaly detection) should find right after the aspirate.
//
// tube_rack child index per tube, as in FVD-Presentation gcs.py: A1 0, A2 3, A3 6, A4 9.
// config.json there puts clear liquid in A1, brown in A2, red in A3 and nothing in A4.
const TUBE_WITH_LIQUID = 3; // A2, brown: easy for the tip camera to see in the tip
const EMPTY_TUBE = 9; // A4: the tip comes up empty, so the check should flag it

function experiment(tube) {
  return {
    steps: [
      { name: 'go_to', target_machine: 'ot2' },
      { name: 'pick_base' },
      { name: 'place', target_machine: 'ot2' },
      {
        name: 'ot2',
        parameters_json: {
          steps: [
            { action: 'pick_up_tips', resource_name: 'tip_rack', well_indices: [0] },
            { action: 'aspirate', resource_name: 'tube_rack', well_indices: [tube], volumes: [200] },
            { action: 'dispense', resource_name: 'empty_plate', well_indices: [0, 5, 10, 15, 20], volumes: [40, 40, 40, 40, 40] },
            { action: 'return_tips' },
          ],
        },
      },
      { name: 'pick', target_machine: 'ot2' },
      { name: 'go_to', target_machine: 'shaker' },
      { name: 'place', target_machine: 'shaker' },
      { name: 'shaker', pwm: 150 },
      { name: 'wait', time_s: 20 },
      { name: 'shaker', pwm: 0 },
      { name: 'pick', target_machine: 'shaker' },
      { name: 'place_base' },
      { name: 'go_to', target_machine: 'home' },
    ],
    max_retries: 3,
    robot: 'robot_1',
  };
}

export const PRESETS = [
  {
    id: 'normal',
    title: 'No anomaly',
    blurb: 'Aspirates 200 µL from tube A2 (brown liquid). The clog check should see liquid in the tip.',
    expect: 'Liquid in tip',
    tone: 'ok',
    routine: experiment(TUBE_WITH_LIQUID),
  },
  {
    id: 'anomaly',
    title: 'Anomaly',
    blurb: 'Aspirates from tube A4, which is empty. The tip comes up dry and the clog check should flag it.',
    expect: 'Clog suspected',
    tone: 'bad',
    routine: experiment(EMPTY_TUBE),
  },
];

// The preset a routine is exactly (same JSON), or null once it was edited.
export function matchingPreset(routine) {
  if (!routine) return null;
  const json = JSON.stringify(routine);
  return PRESETS.find((p) => JSON.stringify(p.routine) === json) ?? null;
}
