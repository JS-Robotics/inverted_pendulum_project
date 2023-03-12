# ivp_cart
ivp_cart is the node that concerns all data and control going to and from the ODrive V3.6:
* Reading the estimated cart position
* Writing the requested motor torque


### Disable setup.py deprecation warning
To disable the deprecation warning:

    --- stderr: ivp_cart                   
        /usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
        warnings.warn(    
    ---

Simply export the following enrivonment variable

    export PYTHONWARNINGS="ignore:setup.py install is deprecated::setuptools.command.install"





## Get ROS2 autocomplete in Pycharm

1. Open Pycharm from terminal where ros2 has been sourced
2. Set the workspace folder's src folder to source root
3. Add the following path to the project structure: `/opt/ros/humble/local/lib/python3.10/dist-packages`


## Testing CLI publishers ##

* Heartbeat testing:

        ros2 topic pub /ivp/torque_setpoint std_msgs/msg/Float32 '{data: 0.017}' -r 15
