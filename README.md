# ToyDIM
A library of MATLAB tools  for multibody simulation the  dynamic behavior of rigid and flexible body systems with arbitrary topology.

Current Features

The following components of a multibody system are currently supported:

Joints

    Spherical
    Revolute
    Rigid

Force types

    Spring force

Force Elements

    Gravity
    Translational (The mentioned force types are applied along the line connecting two points)


If as many drivers as degrees of freedom of the multibody system are specified, inverse dynamic simulation is performed. In this case, the output data are calculated by Newton-Raphson iteration. Otherwise, numerical integration is performed, still allowing the inclusion of driver elements.
Simulation results: Propagation of

    All body coordinates, their first and second derivatives
    The Jacobian and Lagrange multiplies of the position constraints
    The constraint and joint reaction forces acting on the bodies centers of mass
    The applied forces acting on the bodies centers of mass
    The output values of all user-defined functions
