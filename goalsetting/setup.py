from setuptools import find_packages, setup

package_name = 'goalsetting'

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
    maintainer='yebin',
    maintainer_email='syb6886@naver.com',
    description='Building goal setting action package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'goalsetting_server = goalsetting.goalsetting_server:main',
            'goalsetting_client = goalsetting.goalsetting_client:main',
        ],
    },
)
