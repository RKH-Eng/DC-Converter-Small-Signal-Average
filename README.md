# DC-Converter-Small-Signal-Average
MATLAB implementation of state-space averaging and small-signal linearization for a DC-DC Buck converter, including parasitic components. Computes steady-state operating point, A/B/C/D matrices, and transfer functions (vo/vin and vo/d) for control design.


# DC Converter Small Signal Average

This repository contains a MATLAB implementation of state-space averaging and small-signal linearization for a DC-DC Boost Converter, including parasitic resistances and diode voltage drop.

## Features
- Calculates steady-state DC operating point
- Performs small-signal linearization using Jacobian matrices
- Generates state-space A/B/C/D matrices
- Extracts transfer functions:
  - vo/vin (input-to-output)
  - vo/d (control-to-output)
- Enables control design (feedback / bode / compensation)

## Requirements
- MATLAB (Control System Toolbox)
- Symbolic Math Toolbox

## Usage
Open MATLAB and run:
```matlab
run("model.m")
