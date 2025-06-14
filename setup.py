from setuptools import setup

package_name = 'susumu_dummy_agi'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A simple relay node that forwards messages from /from_human to /to_human',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'relay_node = susumu_dummy_agi.relay_node:main',
            'claude_processor_node = susumu_dummy_agi.claude_processor_node:main',
        ],
    },
)