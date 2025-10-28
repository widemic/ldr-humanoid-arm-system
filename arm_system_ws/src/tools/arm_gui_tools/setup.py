from setuptools import setup

package_name = 'arm_gui_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='PyQt-based GUI tools for arm control and monitoring',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_controller_gui = arm_gui_tools.joint_controller_gui:main',
            'trajectory_recorder_gui = arm_gui_tools.trajectory_recorder_gui:main',
            'motor_monitor_gui = arm_gui_tools.motor_monitor_gui:main',
        ],
    },
)
