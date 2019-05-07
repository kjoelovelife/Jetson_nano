from. import logger
from duckietown_utils.expand_variables import expand_environment
import os

def check_years():
    import datetime
    now = datetime.datetime.now()
    if now.year < 2016:
        msg = 'The date is not set correctly. This will screw up building.'
        raise Exception(msg)
    return '%s' % now.year

def check_import(package):
    def f():
        try:
            __import__(package, fromlist=['dummy'])
        except ImportError as e:
            msg = 'Cannot import package %r: %s\n' % (package, e)
            raise ValueError(msg)

    f.__name__ = 'import-%s' % package
    return f

def check_import_messages():
    try:
        from duckietown_msgs.msg import (AntiInstagramTransform, BoolStamped, Segment,
    SegmentList, Vector2D)  # @UnresolvedImport @UnusedImport
    except ImportError as e:
        msg = str(e)
        msg += '\n\n This can usually be fixed by building everything ("make build").'
        msg += '\nOr in extreme cases, "make catkin-clean build".'
        raise Exception(msg)

def check_failure():
    raise ValueError('error')

def on_duckiebot():
    import platform
    proc = platform.processor()
    on_the_robot = not('x86' in proc)
    # armv7l
    return on_the_robot

def check_environment_variables():
    vs = {
    'DUCKIETOWN_ROOT': """
DUCKIETOWN_ROOT should be set.
    """,
          'DUCKIETOWN_DATA': """
The environment variable DUCKIETOWN_DATA must either:
1) be set to "n/a"
2) point to an existing path corresponding to Dropbox/duckietown-data.
    (containing a subdirectory 'logs')
                """, 
        'VEHICLE_NAME':
            "The environment variable VEHICLE_NAME must be the name of your robot \n"
            " (if you are on the robot). Please add this line to ~/.bashrc: \n"
            " \n"
            "  export VEHICLE_NAME=<your vehicle name>\n"
    }

    # Only check this if we are on the robot

    if not on_duckiebot():
        del vs['VEHICLE_NAME']

    # do not check DUCKIETOWN_DATA on robot
    if on_duckiebot():
        del vs['DUCKIETOWN_DATA']

    errors = []
    for v in vs:
        if not v in os.environ:
            e = 'Environment variable %r not defined.' % v
            errors.append(e + '\n' + vs[v])
        
    if not on_duckiebot():
        if 'DUCKIETOWN_DATA' in os.environ:
            path = os.environ['DUCKIETOWN_DATA']
            if path != 'n/a':
                f = expand_environment(path)
                logs = os.path.join(f, 'logs')
                if not os.path.exists(f) or not os.path.exists(logs):
                    e = vs['DUCKIETOWN_DATA']
                    errors.append(e)

    if errors:
        raise Exception('\n---\n'.join(errors))

def do_all_checks():
    """ Returns the names of the failures  """

    checks = [
        check_years,
        check_environment_variables,

        check_import('scipy'),
        check_import('scipy.io'),
        check_import('sklearn'),

        check_import_messages,
    ]

    results = []
    for c in checks:
        logger.debug('Checking %s...' % c.__name__)
        try:
            res = c()
            if res is None: res = '()'
            results.append((True, res))
        except Exception as e:
            r = False, str(e)
            results.append(r)
            logger.error(e)

    failures = []
    for c, (status, s) in zip(checks, results):
        if status:
            mark = 'OK'
        else:
            mark = 'failed'

#         if len(s) > 25:
#             s = s[:22] + ' [...]'
        s = '%20s: %s: %s' % (c.__name__, mark, s)
        if not status:
            failures.append(c.__name__)
            logger.error(s)
        else:
            logger.info(s)
    return failures
