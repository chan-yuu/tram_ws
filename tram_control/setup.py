from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'tram_control',
        'tram_control.strategies',
        'tram_control.strategies.pi_lqr',
        'tram_control.strategies.mpc',
    ],
    package_dir={'': 'src'},
)
setup(**d)
