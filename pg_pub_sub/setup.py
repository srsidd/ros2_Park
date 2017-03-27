from setuptools import setup, Extension

setup(
    name='pg_pub_sub',
    version='0.0.0',
    packages=[],
    py_modules=[
        'pg_subscriber',
        'pg_publisher',
        'utilities'],
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
    description='Place holder for command line tools for topics and nodes',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'pg_sub = pg_subscriber:main',
            'pg_pub = pg_publisher:main'
        ],
    },
)
