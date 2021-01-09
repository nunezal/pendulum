# Pendulum
This project implements an RRT algorithm in MATLAB to find a series of torque inputs to allow an underactuated pendulum to do a swing up action from rest.

There are no obstacles in this implementation, but finding the right set of actions that also satisfies specific constraints such as velocity and torque limits is not trivial for nonlinear systems. RRT allows for planning in the state space while also maintaining the kinetic and dynamic constraints of the motion, commonly known as kinodymanic planning.
