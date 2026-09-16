# Generalized BDF Step Prototype

Status: completed design experiment, superseded by `HistoricalDDASSL`

This fixed-order prototype established the separation between the complete
Newton system and the variables used for integration-error control. Its step
doubling and other deliberate limits are not properties of the current
variable-step integrator. See the
[Julia DDASSL conversion](../common/julia-ddassl-conversion.md) for current behavior.

## 1. Purpose

The existing DASSL interface assumes a square equation set

$$
F(t,y,\dot y)=0,
$$

and makes every current Newton unknown a component of $y$. It also uses the integration-error weights in the Newton correction norm.

The proposed mechanical formulation needs a different separation:

- the Newton solution contains all current mechanical variables;
- only variables integrated through time have BDF histories;
- integration-error control applies only to those integrated variables;
- Newton convergence uses its own variable scales; and
- satisfaction of the implicit equations uses separate equation scales.

## 2. Pendulum BDF1 unknowns

For the planar pendulum, define the current-step Newton unknown

$$
z_{n+1}=
\begin{bmatrix}
a^g & \alpha & \lambda^g & V^g & \omega & R^g & \theta
\end{bmatrix}_{n+1}^{T}.
$$

This contains eleven scalar unknowns. Accelerations and reactions are explicit unknowns, but they are not integration-history variables.

The six integrated variables are

$$
x=
\begin{bmatrix}
V^g & \omega & R^g & \theta
\end{bmatrix}^{T}.
$$

## 3. Eleven current-step equations

The mechanical equations are

$$
ma^g-\lambda^g-mg^g=0,
$$

$$
J\alpha-(d^g)^T\lambda^g=0,
$$

and the displacement constraint is

$$
R^g+r^g(\theta)-p_0^g=0.
$$

These provide five scalar equations. BDF1 supplies the remaining six:

$$
a_{n+1}^g-\frac{V_{n+1}^g-V_n^g}{h}=0,
$$

$$
\alpha_{n+1}-\frac{\omega_{n+1}-\omega_n}{h}=0,
$$

$$
V_{n+1}^g-\frac{R_{n+1}^g-R_n^g}{h}=0,
$$

$$
\omega_{n+1}-\frac{\theta_{n+1}-\theta_n}{h}=0.
$$

Thus the BDF formulas are equations assembled alongside the component equations rather than substitutions made by an integrator outside the mechanical system.

## 4. Three convergence decisions

The prototype deliberately keeps three tests distinct.

1. **Newton correction:** Is the scaled change in every component of $z$ small enough?
2. **Implicit equations:** Are the scaled mechanical, constraint, and BDF equations sufficiently close to zero?
3. **Integration error:** Is the estimated local error in the six components of $x$ acceptable?

Reaction and acceleration variables participate in the first two tests but not the third.

For this first prototype, the integration-error estimate comes from step doubling. One BDF1 step of length $h$ is compared with two BDF1 steps of length $h/2$. This is computationally expensive but keeps the error-control experiment independent of a more elaborate variable-order history implementation.

## 5. Initial experiment

The program [`generalized_bdf1_pendulum.jl`](../../examples/planar/generalized_bdf1_pendulum.jl) implements the eleven-equation Newton solve and adaptive step doubling.

The default experiment uses an integration relative tolerance of $10^{-3}$ over one second. BDF1 is only first order, and the displacement-level formulation transmits position accuracy requirements through two BDF relationships. Tight tolerances consequently require many steps. This is a useful baseline, not an efficiency result.

The initial run accepted 1,208 steps and rejected 7. The maximum displacement-constraint error was $1.91\times10^{-11}$, while the maximum Cartesian state difference from the reduced-coordinate reference was $1.78\times10^{-2}$. The contrast is instructive: solving the assembled equations accurately does not by itself make a first-order integration formula accurate. Integration-error estimation and equation convergence are genuinely different controls.

Run it with

```text
julia --project=. examples/planar/generalized_bdf1_pendulum.jl
```

The next development should replace step doubling with the BDF history and local-error estimator used by DASSL, while preserving the three separate convergence decisions above.
