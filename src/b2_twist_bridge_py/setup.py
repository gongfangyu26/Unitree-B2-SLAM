from setuptools import find_packages, setup

package_name = "b2_twist_bridge_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description="宇树 B2 Python 安全速度桥与有监督基础运动测试",
    license="Apache-2.0",
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'twist_bridge = b2_twist_bridge_py.twist_bridge:main',
            'safe_motion_test = b2_twist_bridge_py.safe_motion_test:main',
        ],
    },
)
