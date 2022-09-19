from setuptools import setup

package_name = 'lufft_wsx'

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
    maintainer='molisens',
    maintainer_email='christoph.gaisberger@uni-graz.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "wsxxx = lufft_wsx.wsxxx:main"
        ],
    },
)
