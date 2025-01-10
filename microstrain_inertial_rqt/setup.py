import os
import sys
import glob
import subprocess
from setuptools import setup, find_packages

package_name = 'microstrain_inertial_rqt'

common_dir = 'microstrain_inertial_rqt_common'
common_src_dir = os.path.join(common_dir, 'src')
common_resource_dir = os.path.join(common_dir, 'resource')
common_icon_resource_dir = os.path.join(common_resource_dir, 'icons')

dest_share_dir = os.path.join('share', package_name)
dest_common_share_dir = os.path.join(dest_share_dir, common_dir)
dest_common_resource_dir = os.path.join(dest_common_share_dir, 'resource')
dest_common_icon_resource_dir = os.path.join(dest_common_resource_dir, 'icons')

# If the submodule does not exist, initialize it
if not os.path.isdir(common_resource_dir):
    submodule_update_result = subprocess.run(['git submodule update --init --recursive'], capture_output=True, shell=True)
    if submodule_update_result.returncode != 0:
        print("Unable to initialize submodule. Error:", file=sys.stderr)
        print(submodule_update_result.stderr, file=sys.stderr)
        sys.exit(1)

setup(
    name=package_name,
    version='4.3.0',
    package_dir={'': common_src_dir},
    packages=find_packages(common_src_dir),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (dest_share_dir, ['package.xml']),
        (dest_share_dir, glob.glob('launch/*')),
        (dest_common_share_dir, [os.path.join(common_dir, 'plugin.xml')]),
        (dest_common_resource_dir, [f for f in glob.glob(os.path.join(common_resource_dir, '*')) if os.path.isfile(f)]),
        (dest_common_icon_resource_dir, glob.glob(os.path.join(common_icon_resource_dir, '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rob Fisher',
    maintainer_email='rob.fisher@parker.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='The microstrain_inertial_rqt package provides several RQT widgets to view the status of Microstrain devices',
    license='MIT',
    tests_require=['pytest'],
)
