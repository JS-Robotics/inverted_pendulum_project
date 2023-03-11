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