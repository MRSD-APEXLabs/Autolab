# Robot Health Interface

## Overview

Monitors robot health by continuously validating that critical sensors and state topics are publishing valid data.

## Purpose

* Detect missing, stale, or invalid data
* Provide a single health signal for system gating

## Checked Signals

* Battery voltage
* IMU data
* Wheel odometry
* Stereo camera images

## Health Criteria

* Topics are publishing
* Data is non-empty and non-NaN
* Messages update within a fixed time window

## Output

* Overall health status
* Per-signal pass/fail indicators

## Intended Use

Runtime readiness and safety validation before and during robot operation.
