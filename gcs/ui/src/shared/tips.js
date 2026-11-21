// Which tube holds which liquid, and which OT-2 tip each one uses. Every routine the UI offers takes
// its tip from here, so a tip only ever touches one liquid (it is returned to the rack after use).
//
// tube_rack child index per tube, as in FVD-Presentation gcs.py: A1 0, A2 3, A3 6, A4 9.
// config.json there puts clear liquid in A1, brown in A2, red in A3 and nothing in A4.
export const TUBE_FOR_LIQUID = { clear: 0, brown: 3, red: 6 };
export const EMPTY_TUBE = 9; // A4

// tip_rack index: 0 = A1, 1 = B1, … Tips 0–3 are reserved for these; reload them if anything else
// (the FVD deck drops tips from 0 upward) used them.
export const TIP_FOR_LIQUID = { clear: 0, brown: 1, red: 2 };
export const TIP_FOR_EMPTY_TUBE = 3;

// Liquids the tip camera cannot see in the tip: their aspirates carry "tip_check": false, so
// lab_machine_executive skips the clog check (it would read a full tip as empty).
export const NO_TIP_CHECK_LIQUIDS = ['clear'];
