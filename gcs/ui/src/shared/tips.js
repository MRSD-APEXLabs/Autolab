// Which tube holds which liquid, and which OT-2 tip each one uses. Every routine the UI offers takes
// its tip from here, so a tip only ever touches one liquid (it is returned to the rack after use).
//
// tube_rack child index per tube, as in FVD-Presentation gcs.py: A1 0, A2 3, A3 6, A4 9.
// Red liquid is in A1 and yellow in A2. A3 is listed as blue but there is NO tube there on the real
// deck: asking for blue aspirates nothing, which is how the anomaly experiment forces a clog detection.
export const TUBE_FOR_LIQUID = { red: 0, yellow: 3, blue: 6 };
export const EMPTY_LIQUID = 'blue';
export const EMPTY_TUBE = TUBE_FOR_LIQUID[EMPTY_LIQUID]; // A3, the "blue" tube that is not there

// tip_rack index: 0 = A1, 1 = B1, … Tips 0–3 are reserved for these; reload them if anything else
// (the FVD deck drops tips from 0 upward) used them.
export const TIP_FOR_LIQUID = { red: 0, yellow: 1, blue: 2 };
export const TIP_FOR_EMPTY_TUBE = 3;

// Liquids the tip camera cannot see in the tip: their aspirates carry "tip_check": false, so
// lab_machine_executive skips the clog check (it would read a full tip as empty). Red and yellow
// are both visible, so none today; the builder's per-aspirate toggle still turns it off by hand.
export const NO_TIP_CHECK_LIQUIDS = [];
