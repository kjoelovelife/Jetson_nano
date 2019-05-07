
import logging
logging.basicConfig()
logger = logging.getLogger('led_detection')
logger.setLevel(logging.DEBUG)


from .api import *
from .unit_tests import *
from .algorithms import *
