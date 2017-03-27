from setuptools import setup, Extension

setup(
    name='pg_mixed',
    version='0.0.0',
    package_dir={ '': '${CMAKE_CURRENT_SOURCE_DIR}' },
    packages=[],
    py_modules=[
        'pg_mixed.listener',],
    install_requires=['setuptools'],
    author='Siddharth Srivatsa',
    author_email='ssrivatsa@houstonmechatronics.com',
    maintainer='Siddharth Srivatsa',
    maintainer_email='ssrivatsa@houstonmechatronics.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Mixed Ament package  for python and cpp files.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'pg_mixed_listener = pg_mixed.listener:main',
        ],
    },
)
