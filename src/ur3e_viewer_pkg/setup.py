from setuptools import find_packages, setup

package_name = 'ur3e_viewer_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    	('share/' + package_name + '/launch', ['launch/ur3e_viewer.launch.py']),
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
        ],
    },
)
