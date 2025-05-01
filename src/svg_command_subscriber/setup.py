from setuptools import find_packages, setup

package_name = 'svg_command_subscriber'

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
    maintainer='ashmu',
    maintainer_email='ashanthan.a.mudaly@student.uts.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'svg_command_subscriber = svg_command_subscriber.command_subscriber:main'
 ],
    },
)
