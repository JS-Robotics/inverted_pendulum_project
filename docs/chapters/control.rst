Control
=======
The inverted pendulum on a cart problem is a classic example because it highlights the importance of feedback
control in stabilizing an unstable system. It has been used as a benchmark problem in control theory and robotics,
and it has also been implemented in various control applications, such as self-balancing robots and segways.

This problem is a challenging control problem because the pendulum is an unstable system, meaning that small
disturbances can cause the pendulum to fall over. As a result, various control strategies have been proposed to stabilize
the pendulum, such as proportional-derivative (PD) control, linear-quadratic regulator (LQR) control, and model predictive control (MPC).

The LQR method will be explored and applied in this project to stabilize the inverted pendulum on a cart system.

LQR
-----

Previously, the equations of motion where derived to the following for the cart and pendulum respectively:

.. math::

    \ddot{x}_{cx} = \frac{F_m - F_f - m_pL\ddot{\theta}\cos(\theta) + m_pL\dot{\theta}^2\sin(\theta)}{m_p + m_c}

.. math::

    \ddot{\theta}  = \frac{-M_f -m_pL\ddot{x}_{cx}\cos(\theta) - m_pLg\sin(\theta)}{I_p + m_pL^2}

In order to apply the LQR technique, it is necessary to linearize the system equations. This is done by linearizing the
equations around the vertically upward equilibrium point where :math:`\theta = \pi`, assuming that the system operates within
a small deviation from this point. Let :math:`\phi` denote the deviation of the pendulum's position from equilibrium,
such that :math:`\theta = \pi + \phi`. Given a small deviation from equilibrium, we can use the following small-angle
approximations of the nonlinear functions in the system equations.

.. math::

    \cos(\theta) = \cos(\pi+\phi) \approx -1

.. math::

    \sin(\theta) = \sin(\pi+\phi) \approx -\phi

.. math::

    \dot\theta^2 = \dot\phi^2 \approx 0

This results in the following linearized equations of motion

.. math::

    \ddot{x}_{cx} = \frac{F_m - b_c\dot{x}_{cx} + m_pL\ddot{\theta}}{m_p + m_c}

.. math::

    \ddot{\theta}  = \frac{-b_p\dot{\theta} + m_pL\ddot{x}_{cx} + m_pLg(\theta-\pi)}{I_p + m_pL^2}

In order to transform the equations of motion into a state space model in the form :math:`\mathbf{\dot x} = \mathbf{A x} + \mathbf{B u}`
where

.. math::

    \mathbf{\dot x} = \begin{bmatrix}
        \dot{x}_{cx} &
        \ddot{x}_{cx} &
        \dot \theta &
        \ddot \theta
    \end{bmatrix}^T

.. math::

    \mathbf{x} = \begin{bmatrix}
            {x}_{cx} &
            \dot{x}_{cx} &
            \theta &
            \dot \theta
    \end{bmatrix}^T

The :math:`\ddot\theta` and :math:`\ddot{x}_{cx}` on the right-hand side in each of the linearized equations of motion, has to be
eliminated. This is achieved by substituting each of the equations into each other.

.. math::

    \ddot{x}_{cx} = \frac{F_m - b_c\dot{x}_{cx} + m_pL(\frac{-b_p \dot\theta + m_pL\ddot{x}_{cx} + m_pLg(\theta-\pi)}{I_p + m_pL^2})}{m_p + m_c}

.. math::

    \ddot{\theta}  = \frac{-b_p \dot\theta + m_pL(\frac{F_m - b_c\dot{x}_{cx} + m_pL\ddot{\theta}}{m_p + m_c}) + m_pLg(\theta-\pi)}{I_p + m_pL^2}

TODO add solved eq2uation set

This yields the following state space model

.. math::

    \begin{bmatrix}
    \dot{x}_{cx}   \\
    \ddot{x}_{cx}  \\
    \dot{\theta}   \\
    \ddot{\theta}  \\
    \end{bmatrix}
    =
    \begin{bmatrix}
    0 & 1 & 0 & 0 \\
    0 & \frac{-b_c(m_pL_p^2+I_p)}{{k}_{den}} & \frac{m_p^2L_p^2g}{{k}_{den}} & \frac{-b_pm_pL_p}{{k}_{den}}\\
    0 & 0 & 0 & 1\\
    0 & \frac{-b_cm_pL_p}{{k}_{den}} & \frac{m_pL_pg(m_c+m_p)}{{k}_{den}} & \frac{-b_p(m_c+m_p)}{{k}_{den}}
    \end{bmatrix}
    \begin{bmatrix}
    {x}_{cx}        \\
    \dot{x}_{cx}  \\
    \theta        \\
    \dot{\theta}  \\
    \end{bmatrix}
    +
    \begin{bmatrix}
    0 \\
    \frac{m_pL_p^2+I_p}{{k}_{den}} \\
    0 \\
    \frac{m_pL_p}{{k}_{den}}
    \end{bmatrix}
    u(t)

where

.. math::

    {k}_{den} = (m_c+m_p)I_p+m_cm_pL_p^2

