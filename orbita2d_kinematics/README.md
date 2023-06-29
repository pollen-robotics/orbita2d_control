# Orbita 2D kinematics model

Implements forward/inverse computation for:
* [x] position
* [x] velocity
* [x] torque

## Model definition

Given two motors $M_A$ and $M_B$ with their respective reduction $r_A$ and $r_B$.

We note $\theta_A$, $\omega_A$ and $\tau_A$ resp. the angle, angular velocity and torque of the motor $M_A$. Idem for $M_B$.

We note $\theta_{pitch}$ and $\theta_{roll}$ the angle of the pitch and roll orientation of the output (similarly for $\omega_{pitch}$, $\tau_{pitch}$, $\omega_{roll}$, $\tau_{roll}$).

$$
\begin{equation}
Position:
\begin{bmatrix}
  \theta_{pitch} \\
  \theta_{roll}
\end{bmatrix} = 
\begin{bmatrix}
  \frac{1}{2*R_A} & \frac{1}{2*R_B} \\
  \frac{1}{2*R_A} & -\frac{1}{2*R_B}
\end{bmatrix}
\begin{bmatrix}
  \theta_A \\
  \theta_B
\end{bmatrix}
\end{equation}
$$

$$
\begin{equation}
Velocity:
\begin{bmatrix}
  \omega_{pitch} \\
  \omega_{roll}
\end{bmatrix} = 
\begin{bmatrix}
  \frac{1}{2*R_A} & \frac{1}{2*R_B} \\
  \frac{1}{2*R_A} & -\frac{1}{2*R_B}
\end{bmatrix}
\begin{bmatrix}
  \omega_A \\
  \omega_B
\end{bmatrix}
\end{equation}
$$

$$
\begin{equation}
Torque:
\begin{bmatrix}
  \tau_{pitch} \\
  \tau_{roll}
\end{bmatrix} = 
\begin{bmatrix}
  2*R_A & 2*R_B \\
  2*R_A & -2*R_B
\end{bmatrix}
\begin{bmatrix}
  \tau_A \\
  \tau_B
\end{bmatrix}
\end{equation}
$$