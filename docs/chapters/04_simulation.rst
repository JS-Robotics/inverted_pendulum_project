

Simulation
==========

The dynamics of the system can be modeled using either Newtonian or Lagrangian mechanics,
which are two different approaches to describe the motion of physical systems. In the Newtonian approach,
the equations of motion are derived from the fundamental laws of mechanics, such as Newton's laws of motion and
the principle of conservation of energy. In the Lagrangian approach, the equations of motion are derived from the
Lagrangian, which is a function that summarizes the energy of the system in terms of generalized coordinates and
their time derivatives.

The two approaches are equivalent, meaning that they provide the same physical predictions for a given system.
The Lagrangian approach, however, often provides a more elegant and concise way of deriving the equations of motion.
In this chapter, both approaches will be applied in order to derive the equations of motion for the inverted pendulum
on a cart system. First the Newtonian approach will be applied to derive the equations of motion, and then the
Lagrangian approach to obtain the same set of equations.

After the equations of motion have been derived, the system will be simulated using numerical methods. Specifically,
the Forward Euler and Fourth order Runge-Kutta methods will be explored. Both the Euler and Runge-Kutta methods have
their advantages and disadvantages, and the choice of which method to use will depend on the specific requirements of
the simulation. In general, the Euler method is simpler and faster, but may not be accurate enough for some applications.
The Runge-Kutta method, on the other hand, is more accurate and stable, but may be slower and more computationally expensive.

By using these numerical methods to simulate the inverted pendulum on a cart system, the dynamics of the system can be explored
and different control strategies can be tested.

Pole Cart Model
***************

asdsadd

Newtonian Approach
------------------

asdasdsadsadasd

Lagrangian Approach
-------------------

asdasdasdsad

Forward Euler
*************

The forward Euler can be considered the The simplest method for numerical integration. It involves using the current
state of the system to estimate its next state. The Euler method can be prone to numerical instability and may
require a very small time step to accurately simulate the system.

Runge-Kutta
***********

The fourth-order Runge-Kutta method is a numerical integration scheme commonly used to solve ordinary differential equations.
It is a higher-order method than the forward Euler method, meaning that it is more accurate and can use a larger time
step while maintaining stability.

The method works by evaluating the state of the system at multiple intermediate time steps, using a weighted average of
these evaluations to determine the next state of the system. The weights used in the averaging process are chosen to
minimize the error in the approximation, resulting in a more accurate simulation.

Although the fourth-order Runge-Kutta method is more computationally expensive than the forward Euler method, it is a
popular choice for simulating physical systems due to its accuracy and stability. It is also a versatile method that can
be adapted to different types of differential equations, making it a valuable tool for many applications in science and engineering.
