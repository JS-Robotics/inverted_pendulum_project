from setuptools import setup
import os

# Disable build warning: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
os.environ["PYTHONWARNINGS"] = "ignore:setup.py install is deprecated::setuptools.command.install"

package_name = 'ivp_cart'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ICraveSleep',
    maintainer_email='todo@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ivp_cart = ivp_cart.main:main'  # executable_name --> set to file nem
        ],
    },
)
