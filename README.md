# Modeling \& Simulation of Aerospace Systems

Individual project performing casual and acasual simulation of multi-domain aerospace systems.

## Overview

This repository contains three simulation-based studies developed as part of a university modeling and simulation project.

The work includes:

1. Development and comparison of MATLAB-casual and Simscape-acasual models for discreet-elements rocket nozzle thermal simulation
2. Development and assessment of acasual Dymola aircraft propeller motor, gearbox and controller model simulation, combining electric, thermal and mechanical elements
3. Development and assessment of acasual Dymola gearbox water cooling system control model for thermal simulation

## Results and Validation

Key results:

* Casual MATLAB simulation results consistent with acasual Simscape simulation results. The difference in the temperature profiles is negligible and << 1e-3 K. Increasing number of nodes in conductive layers introduces transient displacements with respect to single-node model, but increases simulation fidelity.
* PI controller tuned for fast transient tracking of angular velocity reference command, while successfully maintaining motor input voltage below desired threshold. The system reaches steady state conditions in < 5s.
* Controller designed as PI controller with hysteresis switch activation. Control logic successfully maintains gearbox between prescribed temperatures, and sink tank temperature below 10°C. Final simulation results hint at the possibility of a recirculating cooling system.

Representative outputs:

* Nozzle casual model history of layers temperatures, with comparison to acasual Simscape model
* Aircraft propeller gearbox electro-mechanical and thermal transients
* Cooling system temperature and heat flux histories

Representative figures are available in 'results/' (PNG format).
See 'results/results.txt' for figure-by-figure explanations.
The full methodology and results are documented in 'docs/report.pdf'.

## Repository structure

* 'src/' - MATLAB, Simscape, Dymola implementations of each study
* 'docs/' - Project report
* 'results/' - Key result figures (PNG)
* 'figures/' - Source figures (EPS)

## Development notes

This repository was uploaded after project completion.
Commit history does not reflect the original development timeline.

## Reproducibility

'rocket\_nozzle\_acausal\_thermal\_simulation.slx' is run indirectly through 'rocket\_nozzle\_causal\_thermal\_simulation.m', and it is not to be run independently.

