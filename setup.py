from setuptools import find_packages, setup

package_name = 'eval'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omercahit',
    maintainer_email='o.cahitozdemir@gmail.com',
    description='evaluation of ros2 pkgs',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eval = eval.eval:main',
        ],
    },
)
