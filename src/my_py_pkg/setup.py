from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
    maintainer='eric',
    maintainer_email='Eric_Schmidt_99@gmx.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "first_node = my_py_pkg.my_first_node:main",
            "number_publisher = my_py_pkg.number_publisher:main",
            "number_counter = my_py_pkg.number_counter:main",
            "number_counter_srv = my_py_pkg.number_counter_srv:main",
            "reset_counter = my_py_pkg.reset_counter_client:main", 
            "count_until_action_server_no_feedback = my_py_pkg.count_until_action_server_no_feedback:main",    
            "count_until_action_client_no_feedback = my_py_pkg.count_until_action_client_no_feedback:main",
            "count_until_action_server = my_py_pkg.count_until_action_server:main",
            "count_until_action_client = my_py_pkg.count_until_action_client:main",       
        ],
    },
)
