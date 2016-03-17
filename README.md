﻿# ToyDIM
A library of MATLAB tools  used to simulate the dynamic behavior of a rigid and flexible body system with arbitrary topology.

Methodology

     The methodology used for rigid multibody systems dynamics follows (Parviz E. Nikravesh 1989), using a Augmented Lagrange Formulation and the DIM. To describe large relative rotations in deformable systems a finite element representation of each flexible component is used following the methodology presented by Jorge Ambrosio in DSM2007.

Current Features

The following components of a multibody system are currently supported:

Bodies:
     rigid 
     flexible

Joints

    Spherical
    Revolute
    Rigid

Force types

    Spring force

Force Elements

    Gravity
    Translational (The mentioned force types are applied along the line connecting two points)


Simulation results: Propagation of

    All body coordinates, their first and second derivatives
    The Jacobian and Lagrange multiplies of the position constraints
    The constraint and joint reaction forces acting on the bodies centers of mass
    The applied forces acting on the bodies centers of mass
    The output values of all user-defined functions

Simulation results: Propagation of

    All body coordinates, their first derivatives
    The Jacobian and Lagrange multiplies of the position constraints
    The constraint and joint reaction forces acting on the bodies centers of mass

Getting started

    See 

	Rrods3Revel2_v01.m - chain with 4 rigid bodies and 3 revolute joints
        Rrods3Sherical2_v01.m - chain with 4 rigid bodies and 3 spherical joints
	ElasticRod3_v01.m - single elastic body defined by a chain with 3 nodes
	ElasticRod16_v02.m - single elastic body defined by a chain with 16 nodes
	ElasticSurface16Modes_v02.m - single elastic body defined by a surface with 4x4 nodes
	ElasticBlock32Modes_v02.m - single elastic body defined by a block with 4x4x2 nodes

    use
	GraphMovie(file) -  with data from the elastic body simulations for avi generation
