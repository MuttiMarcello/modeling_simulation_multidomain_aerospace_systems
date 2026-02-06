# Modeling & Simulation of Aerospace Systems

Academic project performing casual and acasual simulation of multi-domain aerospace systems.

## Overview

This repository contains three simulation-based studies developed as part of a university modeling and simulation project.

The work includes:
1. Development and comparison of MATLAB-casual and Simscape-acasual models for rocket nozzle thermal simulation
2. Development and assessment of acasual Dymola aircraft propeller gearbox model for thermo-mechanical simulation
3. Development and assessment of acasual Dymola gearbox cooling system model for thermal simulation

The full methodology and results (including plots) are available in 'docs/report.pdf'.

## Repository structure

- 'src/' - MATLAB, Simscape, Dymola implementations of each study
- 'docs/' - Project report

## Reproducibility

'rocket_nozzle_acausal_thermal_simulation.slx' is run indirectly through 'rocket_nozzle_causal_thermal_simulation.m', and it is not to be run independently.