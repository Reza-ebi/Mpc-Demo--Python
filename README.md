# Simple Model Predictive Control in Python

This project demonstrates a basic implementation of Model Predictive Control (MPC) for a 1D linear system using Python and CVXPY. It solves an optimal control problem at each time step and applies the first control action.

## Features
- Finite horizon MPC
- State and control penalty
- Control constraints

## Requirements
- numpy
- cvxpy
- matplotlib

## How to Run
```bash
pip install -r requirements.txt
python mpc_controller.py
```

## Output
The controller drives the system state from 5.0 toward zero over time.
