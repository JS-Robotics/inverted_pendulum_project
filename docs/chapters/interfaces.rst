===========
Interfaces
===========

ROS2 Topics
************

torque_setpoint
----------------
Set the requested torque setpoint of the motor.

* Source type: :code:`SUBSCRIBER`
* Message type: :code:`std_msgs/msg/Float32`
* :code:`msg.x` = pendulum angular position :code:`[rad]`
* :code:`msg.y` = pendulum angular velocity :code:`[rad/s]`
* :code:`msg.z` = Not in use :code:`[-]`
* QOS: :code:`rcl::SensorDataQoS`

pendulum_state
---------------
Publishes the current pendulum angular position and angular velocity.

* Source type: :code:`PUBLISHER`
* Message type: :code:`geometry_msgs/msg/Vector3`
* :code:`msg.x` = pendulum angular position :code:`[rad]`
* :code:`msg.y` = pendulum angular velocity :code:`[rad/s]`
* :code:`msg.z` = Not in use :code:`[-]`
* QOS: :code:`rcl::SensorDataQoS`

turns
------
Publishes the current number of turns that the motor has performed form center.

* Source type: :code:`PUBLISHER`
* Message type: :code:`std_msgs/msg/Float32`
* :code:`msg.data` = pendulum turns :code:`[turn]`
* QOS: :code:`rcl::SensorDataQoS`


status_cart
------------
Publishes the current status of the cart.

* Source type: :code:`PUBLISHER`
* :code:`msg.data` = cart state :code:`[-]`
    * :code:`0` = CONFIGURED
    * :code:`1` = ACTIVE
    * :code:`2` = HEARTBEAT_ERROR
* Message type: :code:`std_msgs/msg/UInt8`
* QOS: :code:`status_qos`

    .. code-block:: Python

            self.status_qos = qos.QoSProfile(depth=1,
                                             reliability=qos.QoSReliabilityPolicy.RELIABLE,
                                             history=qos.QoSHistoryPolicy.KEEP_LAST,
                                             durability=qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
