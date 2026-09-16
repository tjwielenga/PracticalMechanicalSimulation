# Planar Pendulum: Baumgarte Constraint Stabilization

Status: internal working document

## 1. Stabilized constraint equation

The displacement, velocity, and acceleration constraints are combined in the two scalar equations

$$
\ddot\Phi+2\zeta\omega_c\dot\Phi+\omega_c^2\Phi=0.
$$

This does not impose six independent scalar constraint equations. It replaces the two acceleration-constraint equations with two weighted equations containing all three constraint levels.

For critical damping, $\zeta=1$, and therefore

$$
\ddot\Phi+2\omega_c\dot\Phi+\omega_c^2\Phi=0.
$$

The relative coefficients are consequently

$$
1:2\omega_c:\omega_c^2.
$$

It is useful to define the correction time

$$
\tau=\frac{1}{\omega_c}.
$$

The equation can then be written

$$
\ddot\Phi+\frac{2}{\tau}\dot\Phi+\frac{1}{\tau^2}\Phi=0.
$$

The coefficients have different physical units; they should not be interpreted as dimensionless proportions.

## 2. Pendulum equations

For the pin constraint,

$$
\Phi^g=R^g+r^g-p_0^g,
$$

$$
\dot\Phi^g=V^g+d^g\omega,
$$

and

$$
\ddot\Phi^g=a^g+d^g\alpha-r^g\omega^2.
$$

The stabilized equation is therefore

$$
a^g+d^g\alpha-r^g\omega^2
+\frac{2}{\tau}(V^g+d^g\omega)
+\frac{1}{\tau^2}(R^g+r^g-p_0^g)=0.
$$

This remains an eight-variable, eight-equation implicit system. It is an acceleration-level, deficit-zero formulation with feedback from the lower constraint levels.

## 3. Meaning of the correction time

If the numerical motion satisfies the stabilized equation exactly, each component of the constraint error obeys

$$
\ddot e+\frac{2}{\tau}\dot e+\frac{1}{\tau^2}e=0.
$$

For initial errors $e_0$ and $\dot e_0$, the critically damped solution is

$$
e(t)=\left[e_0+\left(\dot e_0+\frac{e_0}{\tau}\right)t\right]e^{-t/\tau}.
$$

A smaller $\tau$ removes constraint error more rapidly but introduces a faster numerical time scale. Thus very aggressive stabilization can reduce the integrator step size and make the Newton system less well conditioned.

## 4. Computer experiment

The program [`implicit_baumgarte_pendulum.jl`](implicit_baumgarte_pendulum.jl) intentionally starts with a small displacement-constraint error. It compares correction times of $0.2$, $0.1$, and $0.05$ seconds using the Julia DDASSL reconstruction, DASSL, and IDA at the same $10^{-5}$ controlled tolerance used in the deficit-zero experiment.

Run it with

```text
julia --project=. examples/planar/implicit_baumgarte_pendulum.jl
```

The comparison should address two separate questions:

1. Does the constraint error follow the intended critically damped decay?
2. How much does a shorter correction time increase the number of integration steps?

The correction time is a numerical modeling choice, not a physical property of the pin joint.

## 5. Initial results

The imposed initial displacement error is $10^{-3}$. The initial velocity error is zero.

| $\tau$ (s) | Solver | Accepted time points | Final $\lVert\Phi\rVert_\infty$ | Final $\lVert\dot\Phi\rVert_\infty$ |
|---:|---|---:|---:|---:|
| 0.20 | Julia reconstruction | 167 | $2.03\times10^{-5}$ | $1.65\times10^{-4}$ |
| 0.20 | DASSL | 506 | $8.58\times10^{-7}$ | $2.54\times10^{-7}$ |
| 0.20 | IDA | 483 | $9.23\times10^{-6}$ | $1.20\times10^{-5}$ |
| 0.10 | Julia reconstruction | 168 | $2.26\times10^{-5}$ | $2.07\times10^{-4}$ |
| 0.10 | DASSL | 478 | $6.03\times10^{-8}$ | $2.06\times10^{-6}$ |
| 0.10 | IDA | 521 | $9.54\times10^{-7}$ | $8.44\times10^{-6}$ |
| 0.05 | Julia reconstruction | 154 | $1.31\times10^{-5}$ | $2.24\times10^{-4}$ |
| 0.05 | DASSL | 378 | $7.99\times10^{-8}$ | $1.81\times10^{-6}$ |
| 0.05 | IDA | 534 | $1.25\times10^{-6}$ | $1.12\times10^{-5}$ |

All cases reduce the imposed position error substantially. The step counts do not establish a simple monotonic cost trend, particularly for DASSL, so a broader study would be needed before drawing an efficiency conclusion. The shorter correction times also create larger transient velocity-constraint errors, as predicted by the faster critically damped return.

## 6. Julia reconstruction details

The stabilized constraint row has level two because its highest terms contain acceleration. Its velocity- and displacement-level feedback terms are lower-level contributions to the same equation. The metadata are

$$
\text{variable levels}=
\begin{bmatrix}
0&0&0&1&1&1&2&2
\end{bmatrix},
$$

$$
\text{equation levels}=
\begin{bmatrix}
2&2&2&1&1&1&2&2
\end{bmatrix}.
$$

The formulation has deficit zero, so positions and velocities control integration error while the algebraic reactions do not.

| $\tau$ (s) | Accepted steps | Rejected steps | Newton iterations | Factorizations | Maximum critical-decay difference |
|---:|---:|---:|---:|---:|---:|
| 0.20 | 144 | 2 | 289 | 146 | $1.10\times10^{-4}$ |
| 0.10 | 144 | 2 | 289 | 146 | $8.09\times10^{-5}$ |
| 0.05 | 145 | 0 | 287 | 145 | $6.81\times10^{-5}$ |

All three runs reach BDF order five and satisfy the stabilized implicit equation to approximately $8\times10^{-9}$. The numerical constraint error follows the intended critically damped decay to within the integration tolerance. As in the earlier experiments, these counts characterize the present implementation and tolerance settings; they are not general efficiency claims.

## 7. First-order velocity-displacement stabilization

A related deficit-one formulation omits the acceleration constraint and combines only the velocity and displacement constraints:

$$
\dot\Phi+\omega_c\Phi=0,
$$

or

$$
\dot\Phi+\frac{1}{\tau}\Phi=0.
$$

These are two scalar equations replacing the original two pin-constraint rows. The displacement and velocity constraints are not imposed as four independent equations. If the stabilized equation is satisfied exactly, the constraint error follows the first-order decay

$$
e(t)=e_0e^{-t/\tau}.
$$

Consistent initial values must satisfy

$$
\dot\Phi_0=-\frac{1}{\tau}\Phi_0.
$$

Consequently, this experiment cannot combine a nonzero initial displacement error with zero initial velocity error as the second-order Baumgarte experiment does. For the imposed displacement error, the initial velocity is adjusted to satisfy the first-order stabilized equation. Differentiating that equation once supplies the acceleration relationship used to calculate consistent initial accelerations and reactions.

The stabilized constraint rows have level one, giving

$$
\text{equation levels}=
\begin{bmatrix}
2&2&2&1&1&1&1&1
\end{bmatrix}.
$$

The formulation has deficit one, so position and velocity variables control integration error and reactions do not.

| $\tau$ (s) | Accepted/rejected steps | Final $\lVert\Phi\rVert_\infty$ | Final $\lVert\dot\Phi\rVert_\infty$ | Maximum exponential-decay difference |
|---:|---:|---:|---:|---:|
| 0.20 | 144 / 2 | $1.85\times10^{-5}$ | $9.27\times10^{-5}$ | $5.02\times10^{-5}$ |
| 0.10 | 144 / 2 | $1.32\times10^{-5}$ | $1.32\times10^{-4}$ | $3.70\times10^{-5}$ |
| 0.05 | 144 / 1 | $6.40\times10^{-6}$ | $1.28\times10^{-4}$ | $2.95\times10^{-5}$ |

All runs reach BDF order five and satisfy the first-order stabilized implicit equation to better than $4\times10^{-10}$. The final constraint errors are somewhat smaller than in the second-order Baumgarte runs at the same tolerances. With the improved controller, the step counts are nearly independent of correction time in this small experiment. Because the two formulations require different consistent initial velocity errors, these results compare two stabilization behaviors rather than providing a strict solver-efficiency comparison.
