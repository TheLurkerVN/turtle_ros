from setuptools import find_packages, setup

package_name = 'main_module'
submodules = "main_module/utils"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    #packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amc',
    maintainer_email='amc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'keymodule = main_module.key_module:main',
            'mapmodule = main_module.map_module:main',
            'navmodule = main_module.navigation_module:main',
            'launchmodule = main_module.launch_module:main',
            'testthread = main_module.thread_test:main'
        ],
    },
)
