===========
Interfaces
===========

ROS2 Topics
************

force_setpoint
----------------
Set the requested torque setpoint of the motor.

* Source type: :code:`SUBSCRIBER`
* Message type: :code:`std_msgs/msg/Float32`
* :code:`msg.data` = Cart applied linear force :code:`[N]`
* QOS: :code:`rcl::SensorDataQoS`
* Frequency : TBD

pendulum_state
---------------
Publishes the current pendulum angular position and angular velocity.

* Source type: :code:`PUBLISHER`
* Message type: :code:`geometry_msgs/msg/Vector3`
* :code:`msg.x` = pendulum angular position :code:`[rad]`
* :code:`msg.y` = pendulum angular velocity :code:`[rad/s]`
* :code:`msg.z` = Not in use :code:`[-]`
* QOS: :code:`rcl::SensorDataQoS`
* Frequency: 50[Hz] - Period: 20[ms]

cart_state
--------------
Publishes the current cart position from center.

* Source type: :code:`PUBLISHER`
* Message type: :code:`geometry_msgs/msg/Vector3`
* :code:`msg.x` = cart position :code:`[m]`
* :code:`msg.y` = cart velocity :code:`[m/s]`
* :code:`msg.z` = Not in use :code:`[-]`
* QOS: :code:`rcl::SensorDataQoS`
* Frequency: 100[Hz] - Period: 10[ms]


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
* Frequency: Only published on change, changed at 100[Hz]