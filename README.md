[![CircleCI](https://circleci.com/gh/duckietown/duckietown-cslam.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-cslam)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-cslam/badge.svg?branch=master18)](https://coveralls.io/github/duckietown/duckietown-cslam?branch=master18)

[![PyPI status](https://img.shields.io/pypi/status/duckietown_cslam.svg)](https://pypi.python.org/pypi/duckietown_cslam/)


[![PyPI pyversions](https://img.shields.io/pypi/pyversions/duckietown_cslam.svg)](https://pypi.python.org/pypi/duckietown_cslam/)


# cSLAM

A contralized slam

## Prerequisite
!! True as long as the graph part has not yet been dockerized !!

In order for this to work, you will need to install g2opy on the computer that runs the graph part.

g2opy : https://github.com/uoip/g2opy 
Warning : you will need to run  `cmake -DPYBIND11_PYTHON_VERSION=2.7 ..` to have it build for python2

## Installation from source

This is the way to install within a virtual environment created by
using `pipenv`:

    $ pipenv install
    $ pipenv shell
    $ cd lib-cslam
    $ pip install -r requirements.txt
    $ python setup.py develop --no-deps


## Unit tests

Run this:

    $ make -C lib-cslam tests-clean tests

The output is generated in the folder in `lib-cslam/out-comptests/`.

## To test ongoing work with g2o
To test the ongoing work on g2o, do this :

    $ cd lib-cslam
    $ python setup.py develop --no-deps --user
    the --user is because I dont use a virtual env right now
    $ cd ../ros-cslam
    $ catkin_make install
    $ source devel/setup.bash
    $ roslaunch pose_graph_builder transform_listener.launch


This will launch a dummy publisher that publishes 6 duckies and relative pose randomly.  
The transform_listener_ros.py listens to it, and creates a graph using duckietown_graph_builder (that itself uses g2o_graph_builder, both are in lib-cslam)

To visualize, run rviz on another terminal and set the frame from "map" to "world". Add the topic type TF. Et voilà!  
