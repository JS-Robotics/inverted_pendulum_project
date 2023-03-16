Introduction
============

The inverted pendulum on a cart problem is a classical example in control theory and robotics, which involves
controlling the motion of a pendulum attached to a cart that moves on a horizontal surface.
The goal is to keep the pendulum upright by moving the cart back and forth in response to the pendulum's motion.

The system consists of a cart with a motorized wheel that can move along a horizontal surface and a pendulum attached
to the cart through a hinge joint. The pendulum is free to rotate around the hinge joint.
The motion of the cart is controlled by applying a force to the cart.

The dynamics of the system can be modeled using the principles of mechanics, and the equations of motion can be derived
using Newtonian or Lagrangian mechanics. The equations of motion describe the motion of the cart and the pendulum
in response to the applied force.

This problem is a challenging control problem because the pendulum is an unstable system, meaning that
small disturbances can cause the pendulum to fall over. As a result, various control strategies have been proposed
to stabilize the pendulum, such as proportional-derivative (PD) control,
linear-quadratic regulator (LQR) control, and model predictive control (MPC).

The inverted pendulum on a cart problem is a classic example because it highlights the importance of
feedback control in stabilizing an unstable system. It has been used as a benchmark problem in control
theory and robotics, and it has also been implemented in various control applications, such as self-balancing robots and segways.

Section
-------

asdasdsad

New section
-----------

1. ss
2. aa
3. gg

* bob
* james
* test

The Kalman filter is a widely used algorithm for state estimation in control systems, robotics, and other applications. The choice of whether to use raw sensor data or filtered sensor data for state measurement in a Kalman filter depends on the specific application and the characteristics of the sensor data.

In general, the Kalman filter is designed to work with noisy sensor data, and its effectiveness depends on the accuracy of the sensor measurements. If the raw sensor data is noisy and unreliable, it may be beneficial to apply a low-pass filter to smooth out the data before feeding it into the Kalman filter. This can help to reduce the impact of sensor noise on the state estimate, and improve the overall performance of the filter.

On the other hand, if the sensor data is already pre-processed or filtered, using the filtered sensor data directly for state measurement may be appropriate. In some cases, using the raw sensor data may introduce additional noise or inaccuracies into the state estimate, particularly if the sensor data is highly variable or subject to significant drift.

Ultimately, the choice of whether to use raw or filtered sensor data for state measurement in a Kalman filter should be based on the specific characteristics of the sensor data and the requirements of the application. It may be useful to experiment with both approaches and compare the performance of the filter in each case to determine the best approach for a particular situation.




To calculate the linear distance traveled by a toothed belt and pulley system, you should use the effective diameter of the pulley with the belt. This is sometimes called the pitch diameter or the datum diameter.

The effective diameter takes into account the thickness of the belt and the depth of the teeth on the pulley. The actual diameter of the pulley itself may not accurately represent the distance traveled by the belt.

To calculate the effective diameter of the pulley with the belt, you can use the following formula:

Effective diameter = Actual diameter + 2 x (tooth height x number of teeth)/π

Once you have determined the effective diameter of the pulley, you can use it along with the number of teeth on the pulley to calculate the linear distance traveled by the belt. This can be done using the following formula:

Distance traveled = (number of teeth on pulley) x (pitch of belt)

The pitch of the belt is the distance between the teeth on the belt, and can usually be found in the belt manufacturer's specifications.



Pully effective diamter = 19.184

Soft stop maximum distance = # Soft max = 6 turns

Pully diamter = 18.5
Pully + belt = 20.5

Dead zone compensation: 0.015 [Nm]
