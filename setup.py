from setuptools import find_packages, setup

package_name = 'quycaros_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adrybcampo',
    maintainer_email='adrybcampo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "emotion_control = quycaros_py_pkg.emotion_control:main",
            "main_control = quycaros_py_pkg.main_control:main",
            "navigation = quycaros_py_pkg.navigation:main",
            "states_server = quycaros_py_pkg.states_server:main",
            "control_msg_publisher = quycaros_py_pkg.control_msg_publisher:main",
            "video_capture = quycaros_py_pkg.video_capture:main"
        ],
    },
)
