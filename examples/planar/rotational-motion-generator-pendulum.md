# Rotational Motion Generator on the Planar Pendulum

Status: verified component and kinematic-analysis example

## 1. Purpose

A rotational motion generator imposes a prescribed relative angle between two orientation markers. Adding one to the revolute pendulum removes its remaining degree of freedom. The pendulum is then a kinematic mechanism: its position, velocity, and acceleration are determined by constraints and prescribed motion rather than by applied forces.

At each requested time the solution sequence is:

1. kinematic position;
2. kinematic velocity;
3. kinematic acceleration; and
4. kinematic forces.

This follows the staged analysis sequence described in the author's earlier
local design notes.

## 2. Motion-generator variables

The generator retains four scalar variables:

$$
\theta_m,\qquad \omega_m,\qquad \alpha_m,\qquad \tau_m.
$$

The first three are the relative angle, angular velocity, and angular acceleration at the marker pair. The last is the torque required to impose the motion.

Let marker $a$ be fixed to the pendulum body and marker $b$ be fixed to ground. Their relative angle is evaluated with the planar two-argument angle definition,

$$
\theta_{ab}
=\operatorname{atan2}
\left(
\sin(\theta_a-\theta_b),
\cos(\theta_a-\theta_b)
\right).
$$

The retained relative coordinate is defined by

$$
\Theta=\theta_m-\theta_{ab}=0.
$$

The prescribed motion is a supplied function $g(t)$,

$$
\Gamma=\theta_m-g(t)=0.
$$

Retaining both equations distinguishes the definition of the marker-relative coordinate from the particular motion imposed on it.

## 3. Differentiated equations

The velocity-level equations are

$$
\dot\Theta
=\omega_m-(\omega_a-\omega_b)=0,
$$

$$
\dot\Gamma=\omega_m-\dot g(t)=0.
$$

The acceleration-level equations are

$$
\ddot\Theta
=\alpha_m-(\alpha_a-\alpha_b)=0,
$$

$$
\ddot\Gamma=\alpha_m-\ddot g(t)=0.
$$

For this example, marker $b$ is fixed to ground, so its angular velocity and acceleration are zero.

## 4. Analysis equation sets

The complete component model has 15 variables and 15 possible equations:

- nine rigid-body variables;
- two revolute-pin reactions;
- four motion-generator variables;
- three body-balance equations;
- six translational pin-constraint equations; and
- six rotational motion-generator equations.

Each kinematic analysis selects only the equations appropriate to its level.

| Analysis | Unknowns | Equations | Size |
|---|---|---|---:|
| Kinematic Position | $R^g$, $\theta$, $\theta_m$ | $\Phi$, $\Theta$, $\Gamma$ | 4 |
| Kinematic Velocity | $V^g$, $\omega$, $\omega_m$ | $\dot\Phi$, $\dot\Theta$, $\dot\Gamma$ | 4 |
| Kinematic Acceleration | $a^g$, $\alpha$, $\alpha_m$ | $\ddot\Phi$, $\ddot\Theta$, $\ddot\Gamma$ | 4 |
| Kinematic Forces | $\lambda_x$, $\lambda_y$, $\tau_m$ | $\sum F_x$, $\sum F_y$, $\sum T$ | 3 |

Gravity enters only the kinematic-force analysis. Changing gravity changes the required reactions and generator torque, but it does not change the prescribed kinematics.

## 5. Example motion and verification

The demonstration uses

$$
g(t)=\theta_0+A\sin(\nu t),
$$

with its analytical derivatives

$$
\dot g(t)=A\nu\cos(\nu t),
\qquad
\ddot g(t)=-A\nu^2\sin(\nu t).
$$

The four analyses are solved successively while sharing one canonical 15-variable context. Automated checks verify the prescribed angle and its derivatives, stationary-pin position, velocity and acceleration, all four selected equation sets, and the finite motion-generator torque. The maximum selected implicit-equation error is at machine precision.

The implementation is in [`rotational_motion_generator_pendulum.jl`](rotational_motion_generator_pendulum.jl).

The earlier generated browser animation was retired after completed simulations
moved to the common HDF5 result and SimpView workflow.
