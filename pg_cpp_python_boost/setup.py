from setuptools import setup, Extension

setup(
    name='pg_cpp_python_boost',
    version='0.0.0',
    packages=['src'],
    ext_modules=[
        Extension('hi',
                ['src/talker.cpp'],
                 include_dirs=['/usr/include', '/home/sidd/Downloads/boost_1_63_0'],
                 library_dirs=['/usr/lib/', '/home/sidd/Downloads/boost_1_63_0/stage/lib'],
                 libraries=['boost_python', 'avcodec', 'swscale'],
                 extra_compile_args=['-std=c++11']
                )
    ],
    py_modules=[
        'pg_python',],
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
            'playground_boost_python = pg_python:run',
        ],
    },
)
