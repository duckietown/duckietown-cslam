# coding=utf-8
__version__ = '0.1.0'

import logging

logging.basicConfig()
logger = logging.getLogger('dt-cslam [image_rectifier]')
logger.setLevel(logging.DEBUG)

logger.info('duckietown_cslam [image_rectifier] %s' % __version__)

from .image_rectifier import ImageRectifier
