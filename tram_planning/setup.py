from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['tram_planning', 'tram_planning.scenarios'],
    package_dir={'': 'src'},
)
setup(**d)
