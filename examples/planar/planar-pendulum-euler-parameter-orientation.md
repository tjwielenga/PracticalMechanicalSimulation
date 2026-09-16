# Planar Pendulum: Euler-Parameter Orientation Experiments

Status: internal working document

## 1. Purpose

These experiments extend the eleven-variable independent-state pendulum with the two planar Euler parameters

$$
p=
\begin{bmatrix}
p_0\\
p_3
\end{bmatrix}
=
\begin{bmatrix}
\cos(\theta/2)\\
\sin(\theta/2)
\end{bmatrix}.
$$

They anticipate the four Euler parameters required for nonsingular three-dimensional orientation. The first experiment retains $A(\theta)$ in the mechanical equations and uses the Euler parameters as a redundant kinematic check. The second evaluates the mechanical transformation from $p$, while retaining $\theta$ and $\omega$ as the two error-controlled states. The third also uses $A(p)$, but transfers orientation error control from $\theta$ to the Euler parameters.

## 2. Variables and equations

The thirteen-variable solution vector is

$$
z=
\begin{bmatrix}
a_x&a_y&\alpha&V_x&V_y&\omega&R_x&R_y&\theta&
\lambda_x&\lambda_y&p_0&p_3
\end{bmatrix}^{T}.
$$

The first eleven equations are unchanged from the independent-state formulation:

$$
\sum F,quad
\sum T,quad
\ddot\Phi,quad
\dot\Phi,quad
\Phi,quad
\alpha-\dot\omega,quad
\omega-\dot\theta.
$$

The two new equations are the kinematic differential equation

$$
\omega-B(p)\dot p=0,
$$

where

$$
B(p)=2
\begin{bmatrix}
-p_3&p_0
\end{bmatrix},
$$

and the normalization constraint

$$
p^Tp-1=0.
$$

The variable levels are

$$
\begin{bmatrix}
2&2&2&1&1&1&0&0&0&2&2&0&0
\end{bmatrix},
$$

and the equation levels are

$$
\begin{bmatrix}
2&2&2&2&2&1&1&0&0&2&1&1&0
\end{bmatrix}.
$$

The Euler parameters are differential variables because $\dot p$ appears in the kinematic equation. Their normalization equation supplies the second scalar relationship needed to determine two parameter rates from the scalar angular velocity.

## 3. Transformation matrix

The planar body-to-global transformation is

$$
A^{gb}(p)=
\begin{bmatrix}
p_0^2-p_3^2&-2p_0p_3\\
2p_0p_3&p_0^2-p_3^2
\end{bmatrix}.
$$

For normalized parameters initialized consistently with $\theta$,

$$
A^{gb}(p)=A^{gb}(\theta).
$$

The maximum matrix difference between these two evaluations is recorded in both experiments.

## 4. Experiment 1: redundant Euler parameters

The first experiment continues to evaluate marker position and its angular direction from $A(\theta)$. The Euler parameters do not affect the mechanical equations. They are governed by their kinematic differential equation and normalization constraint and participate in integration-error control along with $\omega$ and $\theta$.

The differential and error-control masks are therefore

$$
\begin{bmatrix}
0&0&0&0&0&1&0&0&1&0&0&1&1
\end{bmatrix}.
$$

This case checks that the additional differential coordinates and algebraic normalization equation can be added without changing the physical motion.

## 5. Experiment 2: transformation from Euler parameters

The second experiment evaluates $A^{gb}$, $r^g$, and $d^g$ from $p$. The scalar angle remains a differential variable through

$$
\omega-\dot\theta=0
$$

and remains under error control. The Euler parameters remain differential variables because their derivatives occur in $\omega-B(p)\dot p=0$, but they are explicitly excluded from integration-error control. Thus

$$
\text{differential variables}=
\begin{bmatrix}
0&0&0&0&0&1&0&0&1&0&0&1&1
\end{bmatrix},
$$

while

$$
\text{error control}=
\begin{bmatrix}
0&0&0&0&0&1&0&0&1&0&0&0&0
\end{bmatrix}.
$$

This deliberately separates the variables that appear through derivatives from the variables used to choose BDF step size and order.

## 6. Experiment 3: Euler parameters under error control

The third experiment continues to evaluate the mechanical transformation from $p$. Angular velocity and both Euler parameters control integration error, while $\theta$ remains a differential tracking variable but is excluded from error control:

$$
\text{error control}=
\begin{bmatrix}
0&0&0&0&0&1&0&0&0&0&0&1&1
\end{bmatrix}.
$$

This is closer to the anticipated three-dimensional use of Euler parameters, where a separate scalar angle is unavailable to control orientation accuracy. The retained $\theta$ provides an independent planar reference for measuring drift between the two orientation descriptions.

## 7. Results

Both cases were run for five seconds with relative tolerance $10^{-5}$ and absolute tolerance $10^{-7}$. The implementation uses the integrator's numerical iteration matrix for these experiments, avoiding a lengthy analytical differentiation of the Euler-parameter transformation.

| Quantity | $A(\theta)$, $p$ controlled | $A(p)$, $\theta$ controlled | $A(p)$, $p$ controlled |
|---|---:|---:|---:|
| Accepted/rejected steps | 169 / 8 | 168 / 10 | 171 / 3 |
| Maximum BDF order | 5 | 5 | 5 |
| Maximum implicit-equation error | $3.83\times10^{-9}$ | $3.83\times10^{-9}$ | $3.83\times10^{-9}$ |
| Maximum normalization error | $4.82\times10^{-14}$ | $8.42\times10^{-14}$ | $2.18\times10^{-14}$ |
| Maximum $\lVert A(\theta)-A(p)\rVert_\infty$ | $2.22\times10^{-5}$ | $3.87\times10^{-5}$ | $1.93\times10^{-5}$ |
| Maximum mechanical-orientation difference | $8.30\times10^{-5}$ | $6.33\times10^{-5}$ | $7.55\times10^{-5}$ |
| Maximum $[\theta,\omega]$ difference | $2.36\times10^{-4}$ | $3.29\times10^{-4}$ | $3.11\times10^{-4}$ |
| Maximum Cartesian position difference | $8.47\times10^{-5}$ | $7.98\times10^{-5}$ | $5.75\times10^{-5}$ |
| Maximum Cartesian velocity difference | $2.36\times10^{-4}$ | $3.29\times10^{-4}$ | $3.11\times10^{-4}$ |
| Maximum acceleration difference | $1.22\times10^{-3}$ | $9.31\times10^{-4}$ | $1.11\times10^{-3}$ |
| Maximum reaction difference | $9.67\times10^{-4}$ | $8.84\times10^{-4}$ | $8.86\times10^{-4}$ |

All three experiments preserve normalization and reproduce the reduced-coordinate motion. In the second case the transformation used by every mechanical and constraint equation comes from Euler parameters that do not participate in integration-error control. The normalization and kinematic equations nevertheless keep those parameters consistent with the controlled scalar angle to the requested accuracy.

The third case also completes successfully. Under the improved step and order controller, none of the three error-control choices is uniformly most accurate across all reported quantities. The components of $p$ are half-angle quantities with a normalization constraint, so applying the same scalar tolerances used for $\theta$ still does not create an equivalent orientation-error test. A three-dimensional formulation will need an error measure or tolerance scaling appropriate to orientation rather than an unexamined componentwise parameter tolerance.

These planar results do not by themselves establish the best error-control policy for four-component Euler parameters in three dimensions. They do show that differential status, orientation use, and integration-error control can be assigned independently without changing the integrator interface.

The program is [`euler_parameter_independent_state_pendulum.jl`](euler_parameter_independent_state_pendulum.jl).
