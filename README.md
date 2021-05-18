# Satellite Trajectory Optimization: From GTO to GSO

## Info
Authors: Mitchell Fogelson, Alex Withers, Justin Morris

Date: 05-17-21

License: MIT

## Quick Start

'Satellite Trajectory Optimization.ipynb'


## Project Report

[Final Report](https://www.overleaf.com/read/zsczwjzgqwck)

## Pretty Plots
![alt](OOP2IP1)


## Description
This project was a final project completed as part of the 2021 Spring Semester of **16-745: Optimal Control for Robotics** at Carnegie Mellon Unviersity. 

In this repo we implement a solution to the control optimization problem of a satellite maneuvering from a Geo-Transition Orbit to a Geo-Stationary Orbit. We formulate the problem as a free final time optimization problem with the following assumptions and approximations. We model our satellite as a point model using Keplerian Dynamics. We implement inequality constraints on the maximum thrust for these maneuvers, based on a simplified Ion-Thruster model. We implemented higher order explicit integration (RK4,RK8) methods.

### Dynamics:
$\newcommand{\norm}[1]{\left\lVert#1\right\rVert}$

$$ 
\ddot{r} = F = \frac{-\mu}{\norm{r}^3}*r+u \\
x = [r, \dot{r}] \\
\norm{u} \le Tmax
$$
with the following model parameters:
* $\mu$: Gravitational Parameter = 63781
* $r$: distance to earth center
* $u$: control of ion thruster

### Optimization: 
\begin{equation}
    \text{minimize} \sum_{k=1}^{N-1} (||u_k||^2 + (r_k-\bar{r})^2 + \frac{1}{r_k} * h_k) + (r_N-\bar{r})^2+(\dot{r}_N-\bar{\dot{r}})^2 \\
    x_1=x_{init} \\
    r_N = \bar{r} \\
    \dot{r_N} = \bar{\dot{r}} \\
    x_N \cdot \dot{x}_N = 0 \\
    f(x_k, u_k) = x_{k+1} \forall k \\
    0\leq||u_k||\leq T_{max} \forall k \\
    h_{min}\leq h_k \leq h_{max} \forall k
\end{equation}

## Requirements
1. Julia >=1.53
2. ForwardDiff
3. Test
4. RobotDynamics
5. LinearAlgebra
6. StaticArrays
7. SparseArrays
8. Plots
9. Ipopt
10. MathOptInterface
11. TrajOptPlots
12. JLD
13. MathOptInterface
14. Printf

## Files
1. 16745_Final_Project.ipynb: This is the main project notebook 
