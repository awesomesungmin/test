from setuptools import setup
from glob import glob
import os

package_name = 'morai'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # launch 파일 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kroad',
    maintainer_email='bong7121@koreatech.ac.kr',
    description='Package to handle GPS, IMU, and camera data for the Morai system.',
    license='BSD',  # License declaration 수정
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'morai_gps = morai.morai_gps:main',
            'morai_imu = morai.morai_imu:main',
            'gps_to_utm = morai.gps_to_utm:main',
            'morai_camera = morai.morai_camera:main',  # morai_camera 추가
        ],
    },
)
