# Julia DDASSL Conversion

Status: shared production integrator; currently exercised by the planar modeler

## 1. Scope

[`src/common/HistoricalDDASSL.jl`](../../src/common/HistoricalDDASSL.jl) is a readable Julia reconstruction of the numerical structure in the historical Netlib DDASSL source. It is deliberately not a line-by-line transliteration.

The current implementation retains:

- the square implicit equation interface

  $$
  G(t,y,\dot y)=0;
  $$

- variable-step BDF formulas of orders one through five;
- simultaneous changes $\Delta\dot y=CJ\Delta y$ during Newton iteration;
- the combined iteration matrix

  $$
  G_y+CJ G_{\dot y};
  $$

- analytical or directional finite-difference Jacobians;
- modified Newton correction and convergence testing;
- adaptive step rejection, step-size selection, and order selection; and
- component selection for integration-error control.

The last item extends the original interface in a direction needed by the mechanical examples: an algebraic variable can participate in every residual and Newton solve without contributing to the integration-error norm.

## 2. Historical code eliminated

Julia's standard `LinearAlgebra` library replaces:

- `DGEFA` and `DGESL`;
- `DGBFA` and `DGBSL` from the historical banded implementation;
- `DAXPY`, `DSCAL`, `DDOT`, and `IDAMAX`; and
- explicit pivot-vector and factored-matrix storage in `RWORK` and `IWORK`.

Ordinary Julia facilities replace:

- `D1MACH` and `DUMSUM` with `eps`;
- SLATEC error-message routines with a symbolic return code and message;
- packed work arrays with named structures and ordinary arrays;
- computed `GO TO` control flow with functions and structured loops; and
- the fixed-form callback parameter arrays with a single arbitrary `parameter` object.

## 3. BDF coefficients

DDASSL recursively maintains modified divided differences and several coefficient arrays. The Julia version instead constructs the BDF derivative weights directly from the current and recent time points.

For nodes

$$
t_{n+1},t_n,\ldots,t_{n+1-k},
$$

the weights $a_j$ satisfy

$$
\sum_{j=0}^{k}a_j(t_{n+1-j}-t_{n+1})^m
=
\begin{cases}
1,&m=1,\\
0,&m\ne1,
\end{cases}
$$

for $m=0,\ldots,k$. Thus

$$
\dot y_{n+1}=\sum_{j=0}^{k}a_jy_{n+1-j},
\qquad
CJ=a_0.
$$

This requires solving a system no larger than $6\times6$. `LinearAlgebra` performs that work, making the variable-step formula easier to inspect than the historical coefficient recurrence.

The predictor is the Lagrange extrapolation of recent accepted values. When sufficient history is available it uses one more past value than the BDF order, so the predictor-corrector difference has the appropriate local-error order.

## 4. Interface

The residual callback is

```julia
residual!(out, t, y, yprime, parameter)
```

The optional Jacobian callback is

```julia
jacobian!(matrix, t, y, yprime, cj, parameter)
```

and fills

$$
G_y+CJ G_{\dot y}.
$$

A call has the form

```julia
solution = dassl(residual!, y0, yprime0, (t0, tf);
    parameter,
    jacobian!,
    options,
    differential_vars,
    error_control)
```

`differential_vars` classifies variables whose derivatives appear in the implicit equations. `error_control` is an optional Boolean override. All variables remain Newton unknowns, but only selected entries enter the integration-error norm.

### Complete solution history

The current interface carries the complete simultaneous solution vector in the BDF history. For an unreduced mechanical problem this vector might be

$$
y=
\begin{bmatrix}
a & \alpha & \lambda & v & \omega & R & \theta
\end{bmatrix}^{T}.
$$

Not every derivative $\dot y_i$ needs to appear in the implicit equations, and not every $y_i$ needs to participate in integration-error control. The BDF formula nevertheless predicts every component from its accepted history. For an acceleration or constraint reaction, the associated numerical derivative can simply be ignored by the mechanical equations.

Predicting these algebraic solution variables is generally useful. Accelerations and reactions normally vary smoothly between ordinary time steps, so their extrapolated values provide good initial estimates for Newton iteration and can improve convergence. Carrying their short histories requires little storage compared with storing and factoring the Jacobian.

The solver therefore distinguishes three properties without separating the solution vector:

- whether a variable's derivative appears in an implicit equation;
- whether the variable participates in integration-error control; and
- the variable's derivative level for equation and Jacobian scaling.

At a discontinuity, contact transition, or other event, extrapolation of an algebraic variable may be inappropriate. Its history can then be restarted, its prediction order reduced, or its last accepted value used as the initial estimate.

A replaceable step-solution method remains a possible future extension for specialized sparse solution, block elimination, iterative linear solution, contact equations, or genuinely rectangular systems. It is not required merely because some variables are excluded from integration-error control. The existing square Newton iteration is capable of solving the planned unreduced mechanical systems while predicting the complete set of solution variables.

### Variable and equation levels

The extended call can provide integer metadata:

```julia
solution = dassl(residual!, y0, yprime0, tspan;
    variable_levels,
    equation_levels,
    differential_vars,
    deficit)
```

The level convention is:

| Level | Typical variable | Typical equation |
|---:|---|---|
| 0 | Position | Position equation |
| 1 | Velocity | Velocity equation |
| 2 | Acceleration or reaction | Acceleration/force equation |

`variable_levels` records the level of the stored solution variable that forms a column of the Newton matrix. It does not record the level of that variable's BDF derivative. Differentiation raises the level by one. Thus $R$ has level zero, $V$ has level one, and $\dot V=a$ has level two. A reaction $\lambda$ has level two even though its numerical derivative may be formed for prediction and then ignored by the mechanical equations.

This distinction is necessary for the step-size scaling. Consider the level-two force equation

$$
F_F=m\dot V-\lambda-mg=0.
$$

The BDF relationship gives

$$
\dot V=CJ\,V+V'_H,
\qquad
CJ\approx\frac{1}{h},
$$

so the Newton coefficient in the stored $V$ column is approximately

$$
\frac{\partial F_F}{\partial V}=mCJ\approx\frac{m}{h}.
$$

Because the equation has level two and the stored variable $V$ has level one, the scaled coefficient is

$$
\widehat J_{FV}
=h^{2-1}\frac{m}{h}
=m.
$$

It therefore remains approximately constant as the step size changes. The direct reaction coefficient also remains constant because both the force equation and $\lambda$ have level two:

$$
\widehat J_{F\lambda}=h^{2-2}(-1)=-1.
$$

Assigning level two to the stored variable $V$ would leave the coefficient $m/h$ unscaled and would be incorrect under this convention.

The level-one kinematic equation provides the corresponding lower-level example:

$$
F_K=\dot R-V=0.
$$

The stored position $R$ has level zero, so its BDF coefficient scales as

$$
h^{1-0}CJ\approx h\frac{1}{h}=1.
$$

The direct coefficient of the level-one variable $V$ receives the factor $h^{1-1}=1$. Both leading coefficients therefore remain approximately independent of step size.

Variable role and variable level answer different questions. An ideal constraint reaction may have level two, but it is an algebraic solution variable rather than an evolving state. Its predictor-corrector difference can measure the quality of its extrapolation, but it is not a classical local integration error and should not control the step by default.

If an explicit `error_control` mask is omitted, define the maximum controlled level as

$$
L(d)=\max\left(0,\min(1,2-d)\right),
$$

where $d$ is the system deficit. For variable $i$, the default selection is

$$
c_i=\text{differential}_i\ \land\ \left(v_i\leq L(d)\right).
$$

The resulting mechanical policy is:

| Deficit | Default controlled levels |
|---:|---|
| 0 | Position and velocity, levels zero and one |
| 1 | Position and velocity, levels zero and one |
| 2 or greater | Position, level zero |

Algebraic reactions and explicit acceleration unknowns are excluded by `differential_vars`. A physical force governed by its own differential equation can still participate because it is then an evolving state rather than an ideal reaction.

An explicitly supplied `error_control` mask has highest precedence and is used exactly. If levels are absent but `differential_vars` is supplied, the differential classification becomes the default mask. If neither is supplied, all variables participate for backward compatibility.

For a step of magnitude $h$, define diagonal equation and variable scalings

$$
D_e=\operatorname{diag}(|h|^{e_i}),
\qquad
D_v=\operatorname{diag}(|h|^{v_j}).
$$

The physical Newton system

$$
J\,\Delta z=-F
$$

is rewritten using the scaled correction $\Delta\widehat z=D_v\Delta z$ as

$$
\underbrace{D_eJD_v^{-1}}_{\widehat J}
\Delta\widehat z
=-D_eF.
$$

Thus an entry coupling equation level $e_i$ to variable level $v_j$ becomes

$$
\widehat J_{ij}=|h|^{e_i-v_j}J_{ij}.
$$

Terms between equations and variables at the same level receive factor one and remain approximately independent of step size. A BDF term such as $-R/h$ in a level-one equation receives the factor $h^{1-0}$ and also remains approximately constant. This prevents the scaled Jacobian from losing its highest-level coefficients as $h$ becomes small.

### Modified-Newton matrix reuse

The same scaling also permits reuse of a numerical factorization across time
steps. With level metadata present, the integrator compares the dimensionless
leading coefficient $|h|CJ$ with the value represented by the stored factors.
Without level metadata it compares the ordinary $CJ$. The default DDASSL
window refreshes the matrix when their ratio falls below $0.6$ or rises above
$5/3$. It also refreshes after five attempted steps, since additional chord
iterations eventually cost more than updating the sparse numerical factors.

Inside that window the old factors define a chord, or modified-Newton,
iteration. Every correction requires only a triangular solve. The residual is
multiplied by DDASSL's factor $2/(1+C/C_{old})$, using the corresponding scaled
coefficient $C$, to improve convergence when the leading coefficient has
changed. If the old matrix stagnates or exhausts the correction limit, the
integrator returns to the same predictor and tries once with a freshly
evaluated and numerically factored matrix before rejecting the time step.

A force-law event invalidates the numerical values because stiffness may have
changed abruptly, but preserves UMFPACK's symbolic ordering. Repeated failures
with newly evaluated numerical matrices eventually discard the symbolic
factorization as well. A changed state selection replaces the closing implicit
equations inside the same integrator call. It retains the complete BDF history,
discards the old symbolic analysis, and factors the replacement sparse pattern.

The scaled variable correction also enters the Newton and integration-error norms. Physical nominal scales will still be needed for systems mixing different units and characteristic magnitudes; derivative level alone cannot provide complete nondimensionalization.

### Error and convergence measures

Three different numerical measures are used. They answer different questions and should not be combined into a single error measure.

#### Integration error

Let $y^p_{n+1}$ be the value predicted by Lagrange extrapolation and let $y^c_{n+1}$ be the value after the implicit equations have been solved by Newton iteration. For a BDF formula of order $k$, the present implementation estimates the local integration error of component $i$ by

$$
e_i=\frac{y^c_{i,n+1}-y^p_{i,n+1}}{k+1}.
$$

The derivative-level scaling for that component is

$$
s_i=|h|^{v_i},
$$

and its error-control weight is

$$
W_i=\mathrm{ATOL}_i+\mathrm{RTOL}_i
\max\left(\left|s_i y^c_{i,n+1}\right|,
           \left|s_i y^p_{i,n+1}\right|\right).
$$

If $\mathcal C$ is the set of variables selected by `error_control`, the scalar integration-error measure is the weighted root-mean-square value

$$
ERR=
\left[
\frac{1}{|\mathcal C|}
\sum_{i\in\mathcal C}
\left(\frac{s_i e_i}{W_i}\right)^2
\right]^{1/2}.
$$

The step is accepted when $ERR\leq1$. For any candidate order $j$, the proposed step-size multiplier is based on

$$
0.85\,ERR_j^{-1/(j+1)},
$$

with bounds imposed by the implementation. Variables omitted from $\mathcal C$ remain in the implicit equations and in every Newton solve; they are omitted only from integration-error control.

#### Step-size and order selection

After an accepted step at order $k$, the implementation forms predictor-corrector error estimates for the available candidate orders

$$
k-1,\qquad k,\qquad k+1.
$$

For candidate order $j$, the predictor is a degree-$j$ Lagrange extrapolation through $j+1$ accepted past values. Its difference from the corrected value supplies $ERR_j$. The controller calculates the proposed step factor for each available candidate and selects the order expected to permit the largest next step.

An order change is made only when the candidate factor is at least 20 percent better than the factor at the current order. The method also waits for at least

$$
\max(2,k+1)
$$

accepted steps at the current order before changing again. This provides enough history for the higher-order estimate and prevents rapid switching between adjacent orders.

A single rejected step reduces the step size but does not automatically reduce the order. After two consecutive failures, the order is reduced by one. Newton-convergence failure and integration-error failure use different step reductions, although either can eventually cause an order decrease after repeated failures.

This replaces the earlier policy that raised order only when $ERR<0.05$ and lowered it whenever $ERR>0.8$ or a step was rejected. That policy made it easy for a difficult interval to force the method to order one and unnecessarily hard for it to recover.

This predictor-corrector estimate is deliberately compact, but it is not DDASSL's exact error estimator. DDASSL forms its estimate from the accumulated correction and solution history and applies an order- and step-history-dependent coefficient. Reproducing and validating that estimator remains an important part of a closer historical conversion.

#### Newton convergence

Newton convergence is tested using the scaled correction

$$
\Delta\widehat z=D_v\Delta z.
$$

All unknowns participate in a weighted root-mean-square norm formed with the Newton absolute and relative tolerances. This test asks whether another Newton iteration would materially change the solution at the current time; it is not an estimate of the integration error.

#### Satisfaction of the implicit equations

The implicit-equation vector is scaled by equation level,

$$
\widehat F=D_eF,
$$

and all equation rows participate in its weighted root-mean-square norm. Newton iteration is declared successful only when both the correction norm and this implicit-equation norm are at most one. This confirms that the converged values satisfy the equations to the requested nonlinear-solution tolerance, independently of whether the time step satisfies the integration-error tolerance.

The returned `DASSLResult` follows the common SciML solution conventions:

```julia
solution.t                 # saved times
solution.u                 # saved states; alias for solution.y
solution.du                # saved derivatives; alias for solution.yprime
solution[variable, point]  # SciML-style indexing
solution(time)             # interpolated state
solution(time, Val{1})     # interpolated derivative
solution.retcode           # SciMLBase.ReturnCode
solution.differential_vars # supplied differential classification
solution.error_control     # mask actually used by the error estimator
solution.stats.nf          # residual evaluations
solution.stats.naccept     # accepted steps
```

Interpolation between accepted steps evaluates the corrected BDF history
polynomial for that step. Its order is the saved order at the right endpoint,
and its nodes are that endpoint and the preceding accepted history values. The
same polynomial supplies both $y$ and $\dot y$, as in DDASSL's `DDATRP`
routine. No additional implicit-equation solution is performed at an output
time. The DDASSL-specific `orders` and `steps` histories remain directly
available.

## 5. Verification

The example [`examples/planar/julia_ddassl_smoke_test.jl`](../../examples/planar/julia_ddassl_smoke_test.jl) includes three cases:

1. $y'=-y$ with a numerical Jacobian;
2. the same equation with an analytical iteration matrix; and
3. a semi-explicit DAE,

   $$
   \dot y_1+y_1=0,
   \qquad
   y_2-y_1^2=0,
   $$

   with $y_2$ excluded from integration-error control.

With relative tolerance $10^{-7}$, both scalar cases have an error of approximately $2.1\times10^{-7}$ at $t=1$. The algebraic equation in the DAE is satisfied to machine precision. Tightening the relative tolerance from $10^{-4}$ through $10^{-6}$ to $10^{-8}$ reduces the scalar final error monotonically.

### Pendulum formulation experiments

Five eight-variable planar rigid-body pendulum formulations exercise the level scaling, differential-variable classification, error-control policy, analytical Newton matrix, and prediction of algebraic reactions. The Fully Consistent formulation expands the simultaneous solution to eleven variables while retaining only two differential states. GearStableV uses thirteen simultaneous variables and retains six differential states; GearStableA is its complete position-, velocity-, and acceleration-level extension. The first five use the common solution vector

$$
y=
\begin{bmatrix}
R_x&R_y&\theta&V_x&V_y&\omega&\lambda_x&\lambda_y
\end{bmatrix}^{T},
$$

with variable levels

$$
\begin{bmatrix}
0&0&0&1&1&1&2&2
\end{bmatrix}.
$$

The first six variables are differential variables. The two reactions are algebraic, but remain in the BDF history so that they are predicted before Newton iteration.

| Formulation | Deficit | Retained constraint equation | Default error-controlled variables |
|---|---:|---|---|
| Acceleration constraint | 0 | $\ddot\Phi=0$ | Position and velocity |
| Velocity constraint | 1 | $\dot\Phi=0$ | Position and velocity |
| Displacement constraint | 2 | $\Phi=0$ | Position only |
| Baumgarte stabilization | 0 | $\ddot\Phi+2\omega_c\dot\Phi+\omega_c^2\Phi=0$ | Position and velocity |
| First-order stabilization | 1 | $\dot\Phi+\omega_c\Phi=0$ | Position and velocity |
| GearStableV | 1 | $\Phi=0$ and $\dot\Phi=0$ | Position and velocity |
| GearStableA | 0 | $\Phi=0$, $\dot\Phi=0$, and $\ddot\Phi=0$ | Position and velocity |
| Independent-state implicit system | 0 | $\ddot\Phi=0$, $\dot\Phi=0$, and $\Phi=0$ | $\omega$ and $\theta$ only |
| Euler-parameter extensions | 0 | All three $\Phi$ levels, $K_p$, and $N_p$ | $\omega$ and $\theta$; optionally $p$ |

The first three experiments start from the same consistent state, run for five seconds, and use relative tolerance $10^{-5}$ and absolute tolerance $10^{-7}$. The reduced-coordinate pendulum supplies an independent reference trajectory.

| Quantity | Acceleration, deficit 0 | Velocity, deficit 1 | Displacement, deficit 2 |
|---|---:|---:|---:|
| Accepted/rejected steps | 217 / 6 | 217 / 5 | 195 / 14 |
| Maximum $\lVert\Phi\rVert_\infty$ | $5.10\times10^{-4}$ | $1.05\times10^{-5}$ | $2.07\times10^{-14}$ |
| Maximum $\lVert\dot\Phi\rVert_\infty$ | $2.58\times10^{-4}$ | $3.83\times10^{-14}$ | $3.46\times10^{-4}$ |
| Maximum $\lVert\ddot\Phi\rVert_\infty$ | $7.66\times10^{-9}$ | $2.33\times10^{-3}$ | $2.89\times10^{-2}$ |
| Maximum state difference from reference | $5.13\times10^{-4}$ | $5.35\times10^{-4}$ | $8.14\times10^{-4}$ |
| Maximum reaction difference from reference | $3.79\times10^{-4}$ | $2.30\times10^{-3}$ | $2.88\times10^{-2}$ |
| Maximum energy error | $5.16\times10^{-3}$ | $1.87\times10^{-4}$ | $2.24\times10^{-4}$ |

Each formulation satisfies its retained constraint equation most accurately. That alone does not establish the accuracy of quantities requiring further numerical differentiation. The deficit-two formulation satisfies $\Phi=0$ to roundoff and retains a reasonable state trajectory, but its acceleration and reaction errors are appreciably larger than those of the lower-deficit formulations. Reaction accuracy deteriorates as the constraint-derivative deficit increases because the force equations balance reactions against numerical acceleration estimates obtained through additional differentiation.

The deficit-one experiment also tests the default error-control policy. Controlling both position and velocity produces smaller state, reaction, energy, and displacement-drift errors than controlling position alone, while reducing both accepted and rejected step counts. This supports controlling velocity values even though their numerically generated derivatives are not suitable error-control quantities.

The detailed formulation documents and programs are:

- [`planar-pendulum-implicit-acceleration-constraint.md`](../../examples/planar/planar-pendulum-implicit-acceleration-constraint.md) and [`implicit_acceleration_pendulum.jl`](../../examples/planar/implicit_acceleration_pendulum.jl);
- [`planar-pendulum-implicit-velocity-constraint.md`](../../examples/planar/planar-pendulum-implicit-velocity-constraint.md) and [`implicit_velocity_pendulum.jl`](../../examples/planar/implicit_velocity_pendulum.jl); and
- [`planar-pendulum-implicit-displacement-constraint.md`](../../examples/planar/planar-pendulum-implicit-displacement-constraint.md) and [`implicit_displacement_pendulum.jl`](../../examples/planar/implicit_displacement_pendulum.jl).

#### GearStableV and GearStableA constraint satisfaction

GearStableV retains both $\Phi=0$ and $\dot\Phi=0$. It introduces explicit acceleration variables and a second two-component multiplier $\mu^g$, and replaces the ordinary kinematic differential equations with

$$
\dot q-\nu+D^T\mu^g=0.
$$

The thirteen simultaneous variables are $a$, $\nu$, $q$, the physical reaction $\lambda^g$, and the constraint-satisfaction multiplier $\mu^g$. The six components of $\nu$ and $q$ are the differential and error-controlled variables. The acceleration constraint is not imposed, so the formulation has deficit one.

GearStableA adds $\ddot\Phi=0$, a second multiplier $\eta^g$, and the modified acceleration definitions

$$
a-\dot\nu+D^T\eta^g=0.
$$

It has fifteen simultaneous variables and is deficit zero. Both GearStable formulations retain the same six differential and error-controlled variables.

| Quantity | Velocity constraint | GearStableV | GearStableA | Fully Consistent |
|---|---:|---:|---:|---:|
| Simultaneous variables | 8 | 13 | 15 | 11 |
| Error-controlled variables | 6 | 6 | 6 | 2 |
| Retained constraint levels | $\dot\Phi$ | $\Phi$, $\dot\Phi$ | All three | All three |
| Accepted/rejected steps | 217 / 5 | 217 / 5 | 217 / 5 | 168 / 10 |
| Maximum $\lVert\Phi\rVert_\infty$ | $1.05\times10^{-5}$ | $1.11\times10^{-16}$ | $1.11\times10^{-16}$ | approximately roundoff |
| Maximum $\lVert\dot\Phi\rVert_\infty$ | $3.83\times10^{-14}$ | $3.83\times10^{-14}$ | $3.83\times10^{-14}$ | approximately roundoff |
| Maximum $\lVert\ddot\Phi\rVert_\infty$ | $2.33\times10^{-3}$ | $2.33\times10^{-3}$ | $3.83\times10^{-9}$ | approximately roundoff |
| Maximum state difference | $5.35\times10^{-4}$ | $5.40\times10^{-4}$ | $2.52\times10^{-4}$ | $1.54\times10^{-4}$ |
| Maximum reaction difference | $2.30\times10^{-3}$ | $2.30\times10^{-3}$ | $7.55\times10^{-4}$ | $5.77\times10^{-4}$ |
| Maximum energy error | $1.87\times10^{-4}$ | $1.46\times10^{-4}$ | $9.64\times10^{-5}$ | $7.46\times10^{-5}$ |

GearStableV removes the position drift of the velocity-only formulation, at the cost of a small discrepancy between $\dot q$ and $\nu$. It does not improve acceleration or reaction accuracy because $\ddot\Phi=0$ is not among its equations. GearStableA also satisfies the acceleration constraint, reducing the acceleration and reaction differences while introducing a second small discrepancy between $a$ and $\dot\nu$. Details are in [`planar-pendulum-gear-constraint-satisfaction.md`](../../examples/planar/planar-pendulum-gear-constraint-satisfaction.md) and [`gear_constraint_satisfaction_pendulum.jl`](../../examples/planar/gear_constraint_satisfaction_pendulum.jl).

For the complete Gear run, the largest absolute components of the velocity-level multiplier are $9.23\times10^{-5}$ for $\mu_x$ and $1.11\times10^{-4}$ for $\mu_y$. The corresponding acceleration-level values are $2.29\times10^{-3}$ for $\eta_x$ and $2.33\times10^{-3}$ for $\eta_y$. These are constraint-satisfaction variables rather than physical reactions. They quantify the corrections in $\dot q-\nu=-D^T\mu^g$ and $a-\dot\nu=-D^T\eta^g$.

#### Baumgarte stabilization

The Baumgarte experiment begins with displacement error $10^{-3}$ and zero velocity-constraint error. It uses relative and absolute tolerances of $10^{-5}$ and varies the critical-damping correction time $\tau=1/\omega_c$.

| $\tau$ (s) | Accepted/rejected steps | Final $\lVert\Phi\rVert_\infty$ | Final $\lVert\dot\Phi\rVert_\infty$ | Maximum difference from critical decay |
|---:|---:|---:|---:|---:|
| 0.20 | 144 / 2 | $2.03\times10^{-5}$ | $1.66\times10^{-4}$ | $1.10\times10^{-4}$ |
| 0.10 | 144 / 2 | $2.24\times10^{-5}$ | $1.95\times10^{-4}$ | $8.09\times10^{-5}$ |
| 0.05 | 145 / 0 | $1.31\times10^{-5}$ | $2.19\times10^{-4}$ | $6.81\times10^{-5}$ |

All three runs reach BDF order five, reduce the imposed displacement error, and follow the intended critically damped decay within the integration tolerance. The experiment shows that acceleration-level feedback can correct lower-level constraint drift. The step counts do not show a monotonic cost penalty for shorter correction time in this small problem and should not be generalized into an efficiency claim.

Details are in [`planar-pendulum-baumgarte-stabilization.md`](../../examples/planar/planar-pendulum-baumgarte-stabilization.md) and [`implicit_baumgarte_pendulum.jl`](../../examples/planar/implicit_baumgarte_pendulum.jl).

#### First-order stabilization

The related deficit-one experiment combines only velocity and displacement constraints:

$$
\dot\Phi+\omega_c\Phi=0.
$$

Its error should decay as $e(t)=e_0e^{-t/\tau}$. A nonzero initial displacement error requires the consistent initial velocity error $\dot\Phi_0=-\Phi_0/\tau$, so its initial conditions differ from the second-order Baumgarte experiment.

| $\tau$ (s) | Accepted/rejected steps | Final $\lVert\Phi\rVert_\infty$ | Maximum difference from exponential decay |
|---:|---:|---:|---:|
| 0.20 | 144 / 2 | $1.85\times10^{-5}$ | $5.02\times10^{-5}$ |
| 0.10 | 144 / 2 | $1.32\times10^{-5}$ | $3.70\times10^{-5}$ |
| 0.05 | 144 / 1 | $6.40\times10^{-6}$ | $2.95\times10^{-5}$ |

All runs reach order five and follow the intended exponential decay within the integration tolerance. The detailed comparison with second-order Baumgarte stabilization is in the same document and program linked above.

#### Eleven-variable independent-state system

The final experiment makes acceleration an explicit algebraic solution variable rather than the BDF derivative used by the force equations. Its solution vector is

$$
z=
\begin{bmatrix}
a_x&a_y&\alpha&V_x&V_y&\omega&R_x&R_y&\theta&\lambda_x&\lambda_y
\end{bmatrix}^{T}.
$$

All three constraint levels are included. Only $\omega$ and $\theta$ have derivatives that appear in the equations and only those two variables control integration error. Thus

$$
n_{\text{solution}}=n_{\text{equations}}=11,
\qquad
n_{\text{differential}}=n_{\text{error control}}=2.
$$

Over five seconds the maximum $[\theta,\omega]$ difference from the reduced-coordinate reference is $1.54\times10^{-4}$. The maximum Cartesian position, velocity, acceleration, and reaction differences are respectively $5.00\times10^{-5}$, $1.54\times10^{-4}$, $7.31\times10^{-4}$, and $5.77\times10^{-4}$. The run takes 168 accepted and 10 rejected steps and reaches BDF order five.

This verifies that the present integrator can solve and predict a much larger mechanical variable set while integrating and controlling only an independent subset. Details are in [`planar-pendulum-independent-state-implicit-system.md`](../../examples/planar/planar-pendulum-independent-state-implicit-system.md) and [`independent_state_pendulum.jl`](../../examples/planar/independent_state_pendulum.jl).

#### Euler-parameter orientation extensions

Three further experiments add planar Euler parameters $p=[p_0,p_3]^T$, their kinematic differential equation, and their normalization constraint to form a thirteen-variable system. In the first case the mechanics continue to use $A(\theta)$ and the Euler parameters provide a redundant controlled orientation description. In the second case the mechanics use $A(p)$, while $\theta$ and $\omega$ remain the error-controlled states and $p$ is excluded from integration-error control. In the third, the mechanics still use $A(p)$, but orientation error control is transferred from $\theta$ to $p$.

| Quantity | $A(\theta)$, $p$ controlled | $A(p)$, $\theta$ controlled | $A(p)$, $p$ controlled |
|---|---:|---:|---:|
| Accepted/rejected steps | 169 / 8 | 168 / 10 | 171 / 3 |
| Maximum Euler normalization error | $4.82\times10^{-14}$ | $8.42\times10^{-14}$ | $2.18\times10^{-14}$ |
| Maximum $\lVert A(\theta)-A(p)\rVert_\infty$ | $2.22\times10^{-5}$ | $3.87\times10^{-5}$ | $1.93\times10^{-5}$ |
| Maximum mechanical-orientation difference | $8.30\times10^{-5}$ | $6.33\times10^{-5}$ | $7.55\times10^{-5}$ |
| Maximum acceleration difference | $1.22\times10^{-3}$ | $9.31\times10^{-4}$ | $1.11\times10^{-3}$ |
| Maximum reaction difference | $9.67\times10^{-4}$ | $8.84\times10^{-4}$ | $8.86\times10^{-4}$ |

All cases reach BDF order five and preserve Euler-parameter normalization. The second verifies that Euler parameters can drive the mechanical transformation without themselves controlling time-step selection. The third also completes successfully, but the same componentwise tolerances on half-angle Euler parameters do not produce the same orientation accuracy as error control on $\theta$. Details are in [`planar-pendulum-euler-parameter-orientation.md`](../../examples/planar/planar-pendulum-euler-parameter-orientation.md) and [`euler_parameter_independent_state_pendulum.jl`](../../examples/planar/euler_parameter_independent_state_pendulum.jl).

The paper verification suite in
[`test/paper/ddassl_formulation_tests.jl`](../../test/paper/ddassl_formulation_tests.jl)
contains 273 checks covering the integrator and pendulum experiments. Run all
paper verification with `julia --project=. test/paper/run_all.jl`.

## 6. Current implementation status

The conversion now serves as the planar modeler's production integrator. It
accepts analytical sparse Jacobian prototypes, reuses UMFPACK symbolic
factorization while the pattern is unchanged, and may reuse a numerical
factorization for several modified-Newton corrections and steps. The modeler
performs its simultaneous consistent initialization before entering DDASSL.

The integrator uses the predictor-corrector estimates described above rather
than reproducing DDASSL's original fixed-leading-coefficient estimator
line-for-line. It also extends the historical interface with independently
selected integration-error and monitored-variable masks, root location with
hard, soft, or no history restart, and equivalent-equation reconfiguration for
state-selection recovery.

The remaining intentional limits are a square implicit Newton interface, no
nonnegativity option, and no low-level mutable SciML integrator interface. These
are not obstacles to the current mechanical program. Alternative paper
formulations remain isolated in `test/paper`; supported planar models use the
zero-deficit StateSelected formulation.
