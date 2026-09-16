# Driven Planar Four-Bar Linkage

Status: verified ideal closed-loop kinematic and force analysis

## 1. Purpose

The driven four-bar is the first model assembled as an ideal closed kinematic
loop. It reuses the planar rigid body, revolute joint, gravity, and rotational
motion-generator components without adding a special four-bar component. The
example tests whether the component-local implicit equations and automatic
analysis policies remain sufficient when the joint chain closes on ground.

The mechanism contains a crank, coupler, and rocker. The fixed ground pivots are

$$
A = \begin{bmatrix}0 & 0\end{bmatrix}^T,
\qquad
D = \begin{bmatrix}d & 0\end{bmatrix}^T.
$$

The moving joint locations are $B$ and $C$. The link lengths satisfy

$$
\lVert B-A\rVert=a,\qquad
\lVert C-B\rVert=b,\qquad
\lVert D-C\rVert=c.
$$

The default values are $a=0.6$ m, $b=1.4$ m, $c=1.0$ m, and $d=1.5$ m.

## 2. Component assembly

Each moving body retains its planar Cartesian center position, orientation,
velocity, angular velocity, acceleration, and angular acceleration. Four
revolute joints connect

1. ground to the crank;
2. the crank to the coupler;
3. the coupler to the rocker; and
4. the rocker back to ground.

The fourth joint closes the ideal loop. Each joint supplies two constraint
equations at position, velocity, and acceleration level and owns two reaction
components. A rotational motion generator between ground and the crank imposes
constant angular velocity,

$$
\theta_c(t)=\theta_0+\omega_c t,
\qquad \dot\theta_c=\omega_c,
\qquad \ddot\theta_c=0.
$$

Its reaction variable is the torque required to impose this motion. The default
speed is $2\pi/5$ rad/s, so the five-second example covers one complete crank
rotation. The prescribed relative-angle value is wrapped to its principal value,
while each body's orientation remains continuous through the revolution.

The canonical model has 39 variables and 39 available equations:

- 27 body variables and 9 body-balance equations;
- 8 joint-reaction variables and 24 joint-constraint equations; and
- 4 motion-generator variables and 6 motion-generator equations.

As in the earlier component examples, derivative levels of the same constraint
are alternatives for different analyses rather than simultaneous physical
conditions.

## 3. Sequential analyses

The existing metadata policies select four square equation sets:

| Analysis | Unknowns | Equations |
|---|---:|---:|
| Kinematic position | 10 | 10 |
| Kinematic velocity | 10 | 10 |
| Kinematic acceleration | 10 | 10 |
| Kinematic forces | 9 | 9 |

Position analysis solves the nine body configuration variables and the motion
generator's relative angle from eight joint closure equations, one relative
angle equation, and one prescribed-angle equation. Velocity and acceleration
analysis use the corresponding differentiated joint and motion equations.

The force analysis holds the solved kinematics fixed. Nine body force and torque
balances determine eight joint-reaction components and the driving torque.
Gravity contributes to the body balances but does not affect the prescribed
kinematics. Newton corrections are computed from the sparse Jacobian assembled
from the active component blocks and contributions.

## 4. Assembly branch and continuation

Two circle intersections can generally satisfy the coupler and rocker lengths.
An analytical circle intersection supplies only the initial estimate and chooses
one of these assembly branches. The component equations perform the position
solve. For a sequence of times, the preceding positions, velocities, and
accelerations form the second-order prediction

$$
R_{n+1}^{(0)}=R_n+hV_n+\frac{h^2}{2}a_n,
$$

$$
\theta_{n+1}^{(0)}=\theta_n+h\omega_n+\frac{h^2}{2}\alpha_n.
$$

The position analysis then corrects this estimate onto the constraint manifold.
This is the same prediction-and-correction idea used in an implicit time
integrator. Continuation preserves the selected assembly branch as long as the
path does not pass through a singular configuration.

This distinction will matter near a toggle position. A large condition estimate
for the position or velocity Jacobian is a mechanical warning that the selected
configuration is approaching a loss of instantaneous mobility information; it
is not merely a nonlinear-solver detail.

## 5. Verification

The regression tests verify:

- the selected variable and equation counts for all four analyses;
- convergence at several crank positions;
- predictor-corrector continuation through a complete crank revolution without
  changing branch;
- finite Jacobian condition estimates and driving torque;
- agreement of dense and sparse assembly for every analysis Jacobian; and
- every selected analytical Jacobian against directional finite differences.

At $t=0.7$ s with the default motion, the maximum implicit-equation errors are
below $3\times10^{-15}$ and the four analysis Jacobian condition estimates are
approximately six.

Run the example from the repository root with

```bash
julia --project=. examples/planar/driven_planar_four_bar.jl
```

The earlier special-purpose viewer was retired in favor of the common HDF5
result and SimpView workflow.

The next modeling comparison can replace the ideal loop-closing joint with a
stiff planar bushing. That compliant model should remain an open ideal-constraint
tree even though its force path closes mechanically.

The prescribed-motion generator has also been replaced by a constant applied
torque in the subsequent dynamic example
[`torque-driven-planar-four-bar.md`](torque-driven-planar-four-bar.md). The
prescribed version remains useful as the preceding kinematic verification.
