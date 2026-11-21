// The language planner's answer (gcs/nlp_proxy, FVD-Presentation/NLP/mix_colors.py) as a
// routine_executor routine. The planner writes the OT-2 work as abstract transfers
// ({action: 'transfer', source: 'red_tube', volume_ul: 100}); each becomes pick up the liquid's
// tip, aspirate from its tube (no clog check for clear liquid), dispense into plate well A1, return
// the tip. Other steps are
// already in executor form.
import { NO_TIP_CHECK_LIQUIDS, TIP_FOR_LIQUID, TUBE_FOR_LIQUID } from './tips';

function transferCommands(transfer) {
  const liquid = String(transfer?.source ?? '').replace(/_tube$/, '');
  if (!(liquid in TUBE_FOR_LIQUID)) throw new Error(`The planner named an unknown liquid "${liquid}".`);
  const volume = transfer.volume_ul;
  const aspirate = { action: 'aspirate', resource_name: 'tube_rack', well_indices: [TUBE_FOR_LIQUID[liquid]], volumes: [volume] };
  if (NO_TIP_CHECK_LIQUIDS.includes(liquid)) aspirate.tip_check = false;
  return [
    { action: 'pick_up_tips', resource_name: 'tip_rack', well_indices: [TIP_FOR_LIQUID[liquid]] },
    aspirate,
    { action: 'dispense', resource_name: 'empty_plate', well_indices: [0], volumes: [volume] },
    { action: 'return_tips' },
  ];
}

export function planToRoutine(plan, base) {
  const steps = plan.steps.map((step) => {
    if (step?.name !== 'ot2') return step;
    const transfers = step.parameters_json?.steps;
    if (!Array.isArray(transfers) || !transfers.length) throw new Error('The planner gave an OT-2 step without a transfer.');
    return { name: 'ot2', parameters_json: { steps: transfers.flatMap(transferCommands) } };
  });
  return { steps, max_retries: base?.max_retries ?? 3, robot: base?.robot ?? 'robot_1' };
}

// Whether any OT-2 aspirate in the routine reads that liquid's tube.
export function usesLiquid(routine, liquid) {
  const tube = TUBE_FOR_LIQUID[liquid];
  return (routine?.steps ?? []).some((s) => s?.name === 'ot2' && (s.parameters_json?.steps ?? [])
    .some((c) => c?.action === 'aspirate' && c.well_indices?.includes(tube)));
}
