from setuptools import setup

package_name = 'manipulation_library'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Reusable Python library for manipulation tasks',
    license='MIT',
    tests_require=['pytest'],
)
