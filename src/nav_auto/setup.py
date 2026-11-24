from setuptools import setup

package_name = 'nav_auto'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoniodicillo',
    maintainer_email='antoniodicillo@users.noreply.github.com',
    description='Navegação autônoma no labirinto do culling_games',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'part1 = nav_auto.part1_node:main',
        ],
    },
)
