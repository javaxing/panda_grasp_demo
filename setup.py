from setuptools import setup

package_name = 'panda_grasp_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    package_data={package_name: ['*.py']},  # 添加这一行，确保python脚本被安装
    include_package_data=True,
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='you@example.com',
    description='Auto grasp demo with MoveIt 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'auto_grasp = panda_grasp_demo.auto_grasp:main',
        ],
    },
)
