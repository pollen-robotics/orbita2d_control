# Orbita 2D kinematics model

Implements forward/inverse computation for:
* [x] position
* [x] velocity
* [x] torque

## Model definition

Given two motors $M_A$ and $M_B$ with their respective reduction $r_A$ and $r_B$.

We note $\theta_A$, $\omega_A$ and $\tau_A$ resp. the angle, angular velocity and torque of the motor $M_A$. Idem for $M_B$.

We note $\theta_{ring}$ and $\theta_{center}$ the angle of the ring and center orientation of the output (similarly for $\omega_{ring}$, $\tau_{ring}$, $\omega_{center}$, $\tau_{center}$).

_Please refer to the [README](../README.md) for more details on the model definition._

$$
\begin{equation}
Position:
\begin{bmatrix}
  \theta_{ring} \\
  \theta_{center}
\end{bmatrix} = 
\begin{bmatrix}
  \frac{1}{2R_A} & \frac{1}{2R_B} \\
  \frac{1}{2R_A} & -\frac{1}{2R_B}
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
  \omega_{ring} \\
  \omega_{center}
\end{bmatrix} = 
\begin{bmatrix}
  \frac{1}{2R_A} & \frac{1}{2R_B} \\
  \frac{1}{2R_A} & -\frac{1}{2R_B}
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
  \tau_{ring} \\
  \tau_{center}
\end{bmatrix} = 
\begin{bmatrix}
  2R_A & 2R_B \\
  2R_A & -2R_B
\end{bmatrix}
\begin{bmatrix}
  \tau_A \\
  \tau_B
\end{bmatrix}
\end{equation}
$$