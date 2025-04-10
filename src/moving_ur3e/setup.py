from setuptools import setup, find_packages

package_name = 'moving_ur3e'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), # This will find all packages in your directory
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='INDEPTashmu',
    maintainer_email='your_email@example.com',
    description='Description of your package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'troll_mover = moving_ur3e.scripts.trollmovement:main',
        ],
    },
)