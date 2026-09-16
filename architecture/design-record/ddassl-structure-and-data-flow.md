# DDASSL Structure and Data Flow

Status: historical Netlib solver reading used during the Julia conversion

This document describes the original Fortran implementation, not the present
Julia API or its current limitations. See the
[Julia DDASSL conversion](../common/julia-ddassl-conversion.md) for the maintained
integrator.

## 1. Purpose of this document

This document describes the historical Netlib DDASSL implementation before any translation to Julia. Its purposes are to:

- identify the numerical responsibilities of each routine;
- separate the BDF mathematics from Fortran work-array conventions;
- show where system equations, integration equations, Newton iteration, and error control interact; and
- provide a map for a staged translation and verification.

This is not a Julia API. It is an internal reading of the historical algorithm.

The original Netlib source used for this reading is kept in the author's local
historical archive rather than the public repository. It compiled and ran with
GFortran 16.2.0; its smoke test solved $y'=-y$ with an error of approximately
$4.6\times10^{-9}$ at $t=1$.

## 2. Mathematical problem accepted by DDASSL

DDASSL solves an implicit first-order system

$$
G(t,y,\dot y)=0,
$$

where $y$ and $\dot y$ have the same length, $n$. DDASSL does not distinguish differential variables from algebraic variables in its principal equation interface. That distinction is embodied in the dependence of each row of $G$ on $y$ and $\dot y$.

At a new time $t_{n+1}$, the BDF history gives a linear relationship between the new value and derivative:

$$
\dot y_{n+1}=c_{n+1}+CJ\,y_{n+1},
$$

where $c_{n+1}$ is fixed during a Newton iteration and $CJ$ is determined by the current step-size history and BDF order. Consequently, a correction $\Delta y$ implies

$$
\Delta\dot y=CJ\,\Delta y.
$$

Newton's iteration matrix is therefore

$$
PD
=\frac{\partial G}{\partial y}
+CJ\frac{\partial G}{\partial\dot y}.
$$

This matrix is the central connection between the user's implicit equations and the BDF discretization.

## 3. Top-level data flow

The principal flow is:

```text
calling program
    |
    v
DDASSL: validate options, initialize/restore state, manage output times
    |
    +-- optional DDAINI: obtain a consistent initial derivative
    |
    +-- repeated DDASTP: attempt one variable-step, variable-order BDF step
    |       |
    |       +-- RES: evaluate G(t,y,ydot)
    |       +-- DDAJAC: form and factor G_y + CJ G_ydot
    |       +-- DDASLV: solve a Newton correction
    |       +-- DDANRM: convergence and error norms
    |
    +-- DDATRP: interpolate y and ydot to the requested output time
    |
    v
return T, Y, YPRIME, IDID, and persistent history
```

The public driver and the one-step routine have different responsibilities. `DDASSL` manages calls, options, initialization, output times, stopping rules, and persistent state. `DDASTP` implements the numerical step.

## 4. Routine responsibilities

| Routine | Numerical responsibility | Likely Julia responsibility |
|---|---|---|
| `DDASSL` | Public driver; validates options; allocates regions within work arrays; initializes or resumes integration; loops over internal steps; returns at requested output conditions | Integrator interface and step controller |
| `DDAINI` | Attempts one backward-Euler step to make the initial $\dot y$ consistent with $G=0$ | Optional consistent-initial-derivative procedure |
| `DDASTP` | Performs one variable-step, variable-order BDF step, including prediction, modified Newton correction, error testing, rejection, and order/step selection | Core BDF stepping kernel |
| `DDAJAC` | Forms $PD=G_y+CJ G_{\dot y}$ analytically or by finite differences and factors it | Jacobian assembly and factorization policy |
| `DDASLV` | Solves the factored Newton system | Linear-solver abstraction |
| `DDATRP` | Evaluates the history polynomial and its derivative at an output time | Dense output/interpolation |
| `DDAWTS` | Forms componentwise error weights | Error-control policy |
| `DDANRM` | Computes a scaled root-mean-square norm | Weighted norm |
| `D1MACH`, `DUMSUM` | Determine machine roundoff in the historical environment | Replace with `eps(Float64)` |
| `XERMSG` family | Historical SLATEC diagnostics | Replace with returned status and Julia exceptions where appropriate |
| LINPACK/BLAS routines | Dense or banded LU factorization and solution | Replace with Julia linear algebra and sparse/direct solver interfaces |

## 5. Public driver: `DDASSL`

### 5.1 User-supplied equations

The residual callback has the effective contract

$$
\operatorname{RES}(t,y,\dot y)\longrightarrow G(t,y,\dot y).
$$

The historical callback can also return:

- `IRES = 0`: normal evaluation;
- `IRES = -1`: the current trial state is locally invalid, so the solver may reduce the step and retry; or
- `IRES = -2`: stop and return to the calling program.

The optional analytical Jacobian callback supplies

$$
PD=G_y+CJ G_{\dot y},
$$

not $G_y$ and $G_{\dot y}$ separately. This is efficient for the original solver but is an important interface decision for a future implementation. A component-based mechanical formulation may naturally assemble the two partial matrices separately and combine them after BDF discretization.

### 5.2 Initial and continuation calls

The first call performs the following work:

1. Check options and workspace sizes.
2. Select dense or banded matrix storage and analytical or numerical Jacobian evaluation.
3. Initialize counters and pointers into the work arrays.
4. Construct the error-weight vector.
5. Estimate an initial step size unless one was supplied.
6. Optionally invoke `DDAINI` to improve the initial derivative.
7. Initialize the first two history columns with

   $$
   \Phi_1=y,
   \qquad
   \Phi_2=h\dot y.
   $$

On a continuation call, the driver restores the current integration time, proposed step size, order, history, and counters from `RWORK` and `IWORK`. It then checks whether the requested output has already been reached or can be obtained by interpolation.

### 5.3 Internal time and requested output time

DDASSL distinguishes:

- $T$: the time returned to the user;
- $T_N$: the time reached internally by accepted steps; and
- `TOUT`: the requested output time.

The solver may step beyond `TOUT` and use `DDATRP` to interpolate back to it. This is why an output call does not necessarily correspond to an integration step boundary.

### 5.4 Driver loop

Before every attempted internal step, `DDASSL`:

1. refreshes the error weights;
2. rejects nonpositive weights;
3. checks whether the requested accuracy is below roundoff capability;
4. imposes minimum- and maximum-step restrictions; and
5. calls `DDASTP`.

It also limits each user call to 500 internal steps. Successful returns distinguish an intermediate internal step, a stop time, and the requested output time.

## 6. Consistent initial derivative: `DDAINI`

`DDAINI` is used only when requested by `INFO(11)`. It takes a backward-Euler step, possibly smaller than the proposed initial step, to find a state and derivative satisfying the implicit equations.

For a backward-Euler correction,

$$
CJ=\frac{1}{h},
$$

and every correction obeys

$$
y\leftarrow y-\Delta,
\qquad
\dot y\leftarrow\dot y-CJ\Delta.
$$

The routine uses a damped modified Newton iteration. It initially multiplies the residual by $0.75$, reuses the factored iteration matrix, and recomputes that matrix every five iterations if necessary. It permits at most ten Newton corrections for an attempt.

If Newton convergence or the backward-Euler error test fails, the original $t$, $y$, and $\dot y$ are restored, the step is reduced, and the attempt is repeated. Thus this routine changes both $y$ and $\dot y$; it is not merely a solve for $\dot y$ with $y$ fixed.

This behavior should be kept distinct from the mechanical initialization procedures already developed in this project, where positions and velocities are made constraint-consistent in separate weighted solves.

## 7. One BDF step: `DDASTP`

`DDASTP` uses modified divided differences and fixed-leading-coefficient BDF formulas of orders one through five. A step attempt has six conceptual phases.

### 7.1 Form coefficients

From the current step size $h$, past step sizes `PSI`, and order $k$, the routine forms arrays `ALPHA`, `BETA`, `GAMMA`, and `SIGMA`. These serve different purposes:

- `BETA` rescales divided differences when the step size changes;
- `GAMMA` converts the history representation into the predicted derivative;
- `SIGMA` scales error estimates at adjacent orders; and
- `ALPHA` contributes to the BDF leading coefficient and error constant.

The leading derivative coefficient is

$$
CJ=-\frac{\displaystyle\sum_{i=1}^{k}(-1/i)}{h}
=\frac{\displaystyle\sum_{i=1}^{k}1/i}{h}
$$

for constant step size. With variable steps, the computed `ALPHA` values modify the corresponding formula through the stored step history.

### 7.2 Predict $y$ and $\dot y$

The history array `PHI` contains scaled divided differences. The prediction is evaluated as

$$
y^{(0)}=\sum_{j=1}^{k+1}\Phi_j,
$$

$$
\dot y^{(0)}
=\sum_{j=2}^{k+1}\gamma_j\Phi_j.
$$

This makes both the state and its derivative explicit trial variables when the user's residual is evaluated, even though the Newton correction is parameterized by a change in $y$.

### 7.3 Modified Newton corrector

At each corrector iteration:

1. Evaluate $G(t,y,\dot y)$.
2. If required, form and factor

   $$
   PD=G_y+CJ G_{\dot y}.
   $$

3. Solve

   $$
   PD\,\Delta=G.
   $$

4. Update

   $$
   y\leftarrow y-\Delta,
   \qquad
   \dot y\leftarrow\dot y-CJ\Delta.
   $$

5. Accumulate the correction vector used for the local error estimate.

The factorization is reused while its convergence rate remains acceptable. A sufficiently large change in $CJ$ forces a new Jacobian. At most four corrector iterations are used in an ordinary BDF step.

### 7.4 Newton convergence test

The correction norm is measured with `DDANRM`. After the first correction, DDASSL estimates the asymptotic convergence rate from successive correction norms. If the estimated remaining correction satisfies

$$
\frac{r}{1-r}\lVert\Delta\rVert_W\leq 0.33,
$$

the iteration is accepted as converged. A rate above $0.9$ is treated as nonconvergence.

### 7.5 Error test and order selection

After Newton convergence, the accumulated correction is used to estimate errors at orders $k$, $k-1$, and, when available, $k-2$. The local error test has the form

$$
\mathrm{ERR}=C_k\lVert e\rVert_W\leq1.
$$

The algorithm may also estimate the error at order $k+1$. From these estimates it chooses among lowering, retaining, or raising the order. The next step size is then selected from an error-based power law, subject to bounded growth and shrinkage factors.

### 7.6 Accept or reject

On acceptance, the routine:

- increments the step counter;
- records the order and step size used;
- updates the divided-difference history; and
- returns a proposed order and step size for the next step.

On rejection, it restores time and history before retrying. The response depends on the cause:

- singular iteration matrix: reduce $h$ by four;
- Newton failure: refresh the matrix or reduce $h$;
- error-test failure: reduce $h$, possibly lower the order, and eventually return to order one; or
- user residual status: retry or return, according to `IRES`.

This rollback is part of the numerical method and should be represented explicitly in a future integrator state rather than reproduced through array mutation conventions.

## 8. Iteration matrix and linear solve

### 8.1 Analytical or numerical matrix

`DDAJAC` supports four active combinations:

| Matrix structure | Matrix evaluation | Historical method type |
|---|---|---:|
| Dense | User callback | 1 |
| Dense | Finite difference | 2 |
| Banded | User callback | 4 |
| Banded | Grouped finite difference | 5 |

For numerical differentiation, column $i$ is obtained by perturbing $y_i$ and $\dot y_i$ together:

$$
y_i^+=y_i+\delta_i,
\qquad
\dot y_i^+=\dot y_i+CJ\delta_i.
$$

Therefore,

$$
PD_{:i}
\approx
\frac{G(t,y^+,\dot y^+)-G(t,y,\dot y)}{\delta_i}.
$$

The perturbation is based on $\sqrt{\epsilon}$ and the magnitudes of $y_i$, $h\dot y_i$, and its error weight.

The banded finite-difference option perturbs groups of columns whose nonzero row bands do not overlap. This is an early coloring-like optimization, although the implementation describes it purely in band-matrix terms.

### 8.2 Factorization reuse

`DDAJAC` immediately factors the matrix using dense or banded LINPACK LU. `DDASLV` then performs only triangular solution. The stored object is therefore a factorization, not an unfactored Jacobian.

For a component-local mechanical implementation, these responsibilities should be separated into:

1. assemble/update the Newton matrix;
2. choose ordering and factorization;
3. solve one or more corrections; and
4. decide when the factorization is stale.

That separation permits sparse solvers without changing the BDF algorithm.

## 9. Error weights and norms

For each component,

$$
W_i=RTOL_i|y_i|+ATOL_i.
$$

The error norm is

$$
\lVert v\rVert_W
=\sqrt{\frac{1}{n}\sum_{i=1}^{n}
\left(\frac{v_i}{W_i}\right)^2}.
$$

DDASSL applies this same error-control structure to every entry in $y$. That fact is directly relevant to our mechanical-system experiments. Reaction variables and selected derivative variables may be necessary in the nonlinear solve while being inappropriate for integration-error control. The original `INFO(2)` permits scalar or componentwise tolerances, but it does not provide a distinct nonlinear-solution norm and integration-error norm.

This is one of the clearest places where our prospective method may intentionally differ from DDASSL.

## 10. Interpolation: `DDATRP`

`DDATRP` evaluates the accepted history polynomial and its derivative at an arbitrary output time. It uses:

- the current internal time;
- the requested output time;
- the order of the last accepted step;
- the divided differences `PHI`; and
- the step history `PSI`.

No residual or Newton solve is performed during interpolation. Consequently, interpolated values satisfy the history polynomial but are not independently corrected against $G=0$ at the output time. This distinction is important for constrained mechanical systems and relates to the historical WSTIFF practice of recalculating velocities, accelerations, and forces at output points.

## 11. Historical work-array layout

### 11.1 Persistent scalar state in `RWORK`

| Entry | Meaning |
|---:|---|
| 1 | Optional stopping time |
| 2 | Optional maximum step size |
| 3 | Proposed/current step size $h$ |
| 4 | Internal integration time $T_N$ |
| 5 | Current $CJ$ |
| 6 | $CJ$ used for the stored factorization |
| 7 | Last accepted step size |
| 8 | Newton convergence-rate quantity |
| 9 | Unit roundoff |
| 11–16 | `ALPHA` coefficients |
| 17–22 | `BETA` coefficients |
| 23–28 | `GAMMA` coefficients |
| 29–34 | `PSI` step history |
| 35–40 | `SIGMA` error coefficients |

Following those scalars are vectors for the correction, accumulated error, error weights, divided-difference history, and matrix workspace.

### 11.2 Persistent integer state in `IWORK`

| Entry | Meaning |
|---:|---|
| 1–2 | Lower and upper bandwidths |
| 3 | Maximum BDF order |
| 4 | Matrix method type |
| 5 | Jacobian/factorization refresh flag |
| 6 | Startup phase flag |
| 7 | Proposed/current order |
| 8 | Last accepted order |
| 9 | Number of steps at unchanged history conditions |
| 10 | Step count at beginning of current user call |
| 11 | Total accepted steps |
| 12 | Residual evaluations |
| 13 | Jacobian evaluations |
| 14 | Error-test failures |
| 15 | Convergence-test failures |
| 16 | Matrix-storage length |
| 21 onward | LU pivot indices |

### 11.3 Interpretation for translation

The packed arrays are an interface and memory-management strategy, not part of the BDF mathematics. A Julia translation should initially replace them with named structures such as:

```text
BDFHistory
    divided differences
    past step sizes
    current and previous orders

StepController
    current, previous, minimum, and maximum step sizes
    error coefficients
    acceptance/failure counters

NewtonState
    CJ and factored-CJ
    residual and correction work vectors
    convergence-rate estimate
    factorization validity

IntegratorStatistics
    accepted steps
    residual evaluations
    Jacobian evaluations
    error-test failures
    convergence failures
```

Named state will make the translation auditable while preserving the original state transitions.

## 12. What is general and what is historical

### 12.1 General numerical structure worth preserving

- implicit equation interface $G(t,y,\dot y)=0$;
- linear BDF relationship between corrections in $y$ and $\dot y$;
- Newton matrix $G_y+CJ G_{\dot y}$;
- divided-difference history for variable step size;
- factorization reuse tied to changes in $CJ$ and convergence behavior;
- rollback after a rejected step;
- adjacent-order error estimates;
- interpolation from the accepted history polynomial; and
- explicit status reporting for residual-domain failures.

### 12.2 Historical implementation details not automatically worth preserving

- one-based pointer arithmetic inside `RWORK` and `IWORK`;
- computed `GO TO` dispatch;
- LINPACK-specific dense and banded storage;
- fixed maximum order encoded in six-element coefficient arrays;
- SLATEC message machinery;
- a single tolerance mechanism serving every numerical purpose; and
- a Jacobian callback that must return the already-combined matrix.

## 13. Relationship to the proposed mechanical formulation

The present DDASSL interface requires the number of equations to equal the number of integrated variables. It treats $y$ and $\dot y$ as paired vectors and eliminates $\dot y$ corrections through

$$
\Delta\dot y=CJ\Delta y.
$$

Our eventual unreduced mechanical formulation may instead include explicit acceleration, velocity, position, and reaction variables in a larger implicit equation system, with selected BDF relationships embedded among the system equations. That is a more general algebraic structure than DDASSL's public interface.

Nevertheless, DDASSL provides reusable ideas at three levels:

1. **History representation and prediction:** the divided-difference BDF machinery.
2. **Nonlinear stepping policy:** modified Newton iteration, factorization reuse, rejection, and recovery.
3. **Integration error control:** order and step-size selection based on the history correction.

These levels should be separated during translation. We should first reproduce DDASSL's behavior for square $G(t,y,\dot y)=0$ problems. Only after that reference implementation is verified should we generalize the Newton unknowns and distinguish equation-solution control from integration-error control.

## 14. Proposed translation sequence

1. Preserve the current Fortran smoke test as the behavioral reference.
2. Translate the weighted norm and error-weight construction.
3. Translate the history representation and interpolation, with direct unit tests against Fortran values.
4. Translate a single fixed-order BDF step using a dense numerical Jacobian.
5. Add modified Newton convergence and factorization reuse.
6. Add adaptive step size and order.
7. Add consistent-initial-derivative behavior as a separate optional stage.
8. Compare complete trajectories, accepted step counts, orders, residual calls, and Jacobian calls with the Fortran implementation.
9. Replace dense linear algebra with a solver-neutral factorization interface.
10. Only then investigate the larger unreduced mechanical equation set and separate error-control policies.

This sequence keeps a working reference at every stage and avoids confusing algorithm translation with mechanical-system generalization.
