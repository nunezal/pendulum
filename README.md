# Pendulum

## Overview

This project implements a Rapidly-exploring Random Tree (RRT) algorithm in MATLAB to find a series of torque inputs that enable an underactuated pendulum to perform a swing-up action from rest. The implementation demonstrates kinodynamic planning in state space while respecting physical constraints like velocity and torque limits.

## Problem Description

Controlling an underactuated pendulum to swing from a downward position (rest) to an upright position presents challenges in nonlinear control systems. While there are no obstacles in this implementation, finding the right sequence of torque inputs that satisfy specific constraints such as velocity and torque limits is non-trivial for nonlinear systems.

## RRT Algorithm

The Rapidly-exploring Random Tree (RRT) algorithm is used for motion planning in high-dimensional spaces. In this project, RRT:

- Explores the state space of the pendulum (angle and angular velocity)
- Generates random samples within the state space boundaries
- Extends the tree toward these samples using the pendulum dynamics model
- Finds a feasible path connecting the start state to the goal state

## Implementation Details

### Pendulum Model

- State variables: angle (θ) and angular velocity (θ̇)
- Control input: torque (u)
- System parameters:
  - Gravity (g = 9.81 m/s²)
  - Damping coefficient (b = 0.1)
  - Maximum torque (7 units)

### Algorithm Parameters

- State space boundaries: θ ∈ [-π, π], θ̇ ∈ [-10, 10]
- Time step (dt): 0.02 seconds
- Maximum samples: 20,000

### Key Functions

- `generateRRT`: Builds the RRT by sampling the state space
- `extend`: Chooses the best torque input to move toward a random state
- `dynamics`: Implements the pendulum's equation of motion
- `pick_path`: Extracts the solution path from the generated tree

## Visualization

The program generates two visualizations:

1. State space plot showing the RRT exploration and the final path
2. Pendulum animation displaying the swing-up motion

## Results

When successful, the algorithm finds a path from the downward position (-π/2, 0) to the upward position (π/2, 0). The resulting trajectory demonstrates a swing-up maneuver that respects the system's dynamic constraints.

## Limitations and Future Work

- The current implementation has fixed parameters. Future work could explore adaptive parameter selection.
- The solution is not guaranteed to be optimal in terms of energy or time.
- Additional constraints or obstacles could be incorporated to extend the problem.

## References

- LaValle, S. M. (1998). Rapidly-exploring random trees: A new tool for path planning.
- LaValle, S. M., & Kuffner, J. J. (2001). Randomized kinodynamic planning.
