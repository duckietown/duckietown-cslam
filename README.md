[![CircleCI](https://circleci.com/gh/duckietown/duckietown-cslam.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-cslam)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-cslam/badge.svg?branch=master18)](https://coveralls.io/github/duckietown/duckietown-cslam?branch=master18)

[![PyPI status](https://img.shields.io/pypi/status/duckietown_cslam.svg)](https://pypi.python.org/pypi/duckietown_cslam/)


[![PyPI pyversions](https://img.shields.io/pypi/pyversions/duckietown_cslam.svg)](https://pypi.python.org/pypi/duckietown_cslam/)


# cslam

A contralized slam


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
