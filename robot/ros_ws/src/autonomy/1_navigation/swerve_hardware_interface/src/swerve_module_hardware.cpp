#include "swerve_hardware_interface/swerve_module_hardware.hpp"

#include <cmath>

#include <ctre/phoenix6/configs/Configuration.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>

namespace swerve_hardware_interface
{

using namespace ctre::phoenix6;

SwerveModuleHardware::SwerveModuleHardware(
  const ModuleConfig & cfg,
  const SharedModuleParams & params,
  CANBus & bus)
: cfg_(cfg), params_(params),
  drive_(cfg.drive_can_id, bus),
  steer_(cfg.steer_can_id, bus),
  cancoder_(cfg.cancoder_id, bus)
{
}

bool SwerveModuleHardware::configure()
{
  configs::CANcoderConfiguration cc_cfg{};
  cc_cfg.MagnetSensor.MagnetOffset = units::angle::turn_t{cfg_.cancoder_offset_rot};
  bool ok = cancoder_.GetConfigurator().Apply(cc_cfg).IsOK();

  configs::TalonFXConfiguration steer_cfg{};
  steer_cfg.Slot0.kP = params_.steer_kp;
  steer_cfg.Slot0.kI = params_.steer_ki;
  steer_cfg.Slot0.kD = params_.steer_kd;
  steer_cfg.Slot0.kS = params_.steer_ks;
  steer_cfg.Slot0.kV = params_.steer_kv;
  steer_cfg.Feedback.FeedbackRemoteSensorID = cfg_.cancoder_id;
  // RemoteCANcoder, not FusedCANcoder — fusion is a Phoenix Pro-licensed
  // feature and these steer TalonFX are unlicensed. RemoteCANcoder still
  // drives steer position feedback directly off the CANcoder (same
  // RotorToSensorRatio mapping to mechanism units), just without Pro's
  // rotor-fusion smoothing/robustness.
  steer_cfg.Feedback.FeedbackSensorSource =
    signals::FeedbackSensorSourceValue::RemoteCANcoder;
  steer_cfg.Feedback.RotorToSensorRatio = params_.steer_gear_ratio;
  steer_cfg.CurrentLimits.StatorCurrentLimit =
    units::current::ampere_t{params_.steer_stator_current_limit_a};
  steer_cfg.CurrentLimits.StatorCurrentLimitEnable = true;
  ok = steer_.GetConfigurator().Apply(steer_cfg).IsOK() && ok;

  configs::TalonFXConfiguration drive_cfg{};
  drive_cfg.Slot0.kP = params_.drive_kp;
  drive_cfg.Slot0.kI = params_.drive_ki;
  drive_cfg.Slot0.kD = params_.drive_kd;
  drive_cfg.Slot0.kS = params_.drive_ks;
  drive_cfg.Slot0.kV = params_.drive_kv;
  drive_cfg.Feedback.SensorToMechanismRatio = params_.drive_gear_ratio;
  // NOTE: swerve coupling compensation (drive rotation induced by steering
  // motion, coupling_gear_ratio) is intentionally NOT implemented here.
  // TalonFXConfiguration has no field for it — CTRE's real coupling-ratio
  // compensation lives in their higher-level swerve API
  // (SwerveModuleConstants), not on the raw device config. This is a known,
  // accepted gap (see design spec) that will cause minor odometry
  // drift/wheel scrub when steering in place; out of scope for this task.
  ok = drive_.GetConfigurator().Apply(drive_cfg).IsOK() && ok;

  return ok;
}

void SwerveModuleHardware::set_drive_velocity_rotps(double wheel_rotps)
{
  // Firmware's SensorToMechanismRatio (drive_gear_ratio) already converts
  // control targets to/from wheel (mechanism) units, so no gear-ratio
  // conversion is applied here — doing so would double-apply the ratio.
  drive_.SetControl(controls::VelocityVoltage(units::angular_velocity::turns_per_second_t{wheel_rotps}));
}

void SwerveModuleHardware::set_steer_position_rad(double rad)
{
  // Under RemoteCANcoder feedback, the TalonFX's mechanism position already
  // tracks the CANcoder (module/azimuth) directly — one mechanism rotation
  // equals one module rotation, already gear-compensated by firmware. Only
  // convert rad -> mechanism turns, no steer_gear_ratio factor here.
  double mechanism_turns = rad / (2.0 * M_PI);
  steer_.SetControl(controls::PositionVoltage(units::angle::turn_t{mechanism_turns}));
}

double SwerveModuleHardware::get_drive_velocity_rotps()
{
  // See set_drive_velocity_rotps(): firmware already reports in wheel units.
  return drive_.GetVelocity().GetValueAsDouble();
}

double SwerveModuleHardware::get_steer_position_rad()
{
  // See set_steer_position_rad(): firmware already reports mechanism
  // (module/azimuth) turns via RemoteCANcoder, no gear ratio factor here.
  double mechanism_turns = steer_.GetPosition().GetValueAsDouble();
  return mechanism_turns * 2.0 * M_PI;
}

bool SwerveModuleHardware::is_healthy()
{
  return drive_.GetVelocity().GetStatus().IsOK() &&
         steer_.GetPosition().GetStatus().IsOK() &&
         cancoder_.GetPosition().GetStatus().IsOK();
}

}  // namespace swerve_hardware_interface
