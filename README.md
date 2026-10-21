# MPC_MatlabProject-

This repository contains a speed controller for following a lead vehicle (Adaptive Cruise Control), built with **Model Predictive Control (MPC)** in MATLAB.

It was created as a **follow-up to our RCDC competition project**, where we implemented Adaptive Cruise Control and an automated lane change in Simulink **without using MPC** (the maneuver parameters were computed from vehicle speed and distance). Here I explore how the speed-following part can be solved with constrained, optimization-based control.


## Problem

The ego vehicle follows a lead vehicle. The controller should:

- track the driver's set speed `v_set`,
- keep a safe distance to the lead vehicle that grows with speed: `d >= d0 + t_gap * v`,
- respect limits on acceleration and jerk (comfort and actuator limits).

## Model

Linear state-space model with states `x = [d; v; a]`:

- `d` - distance to the lead vehicle [m]
- `v` - ego speed [m/s]
- `a` - ego acceleration [m/s^2] (first-order lag, time constant `tau`, models powertrain/brake delay)

Inputs: commanded acceleration `a_cmd` (manipulated variable) and lead vehicle speed `v_lead` (measured disturbance).

## Controller

MPC from MATLAB Model Predictive Control Toolbox:

- sample time 0.2 s, prediction horizon 40 steps (8 s), control horizon 10 steps (2 s)
- acceleration limits `-4 ... +2 m/s^2`, limit on the change of `a_cmd` between steps
- soft output constraint on the safe distance `d - t_gap * v >= d0`
- cost: tracking of `v_set` and penalty on control rate

## Scenario

The lead vehicle drives at 15 m/s and from t = 20 s brakes (-2 m/s^2) down to 8 m/s. The ego vehicle starts 40 m behind at 20 m/s with `v_set = 25 m/s`.

## How to run

Requirements: MATLAB with Control System Toolbox and Model Predictive Control Toolbox.