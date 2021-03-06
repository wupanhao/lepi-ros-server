# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['apriltag_detector', 'camera_utils', 'face_recognizer',
              'keras_transfer', 'line_detector', 'barcode_scanner', 'image_processor'],
    package_dir={'': 'include'},
)

setup(**setup_args)
