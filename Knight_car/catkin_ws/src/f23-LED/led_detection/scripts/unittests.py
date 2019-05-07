#!/usr/bin/env python
# colored logging
import os
import sys

from duckietown_utils import col_logging
from led_detection import logger
from led_detection.dummy import DummyLEDDetector
from led_detection.LEDDetector import LEDDetector
from led_detection.LEDDetector_forloops import LEDDetector_forloops
from led_detection.unit_tests import load_tests
from duckietown_utils.wildcards import expand_string
from duckietown_utils.wrap_main import wrap_main

W = 0
H = 0

def main():
    script_name = os.path.basename(sys.argv[0])
    args = sys.argv[1:]
    if len(args) != 2:
        msg = """
Usage:

    rosrun led_detection <script> <tests> <algorithms>

where:

    <tests> = comma separated list of algorithms. May use "*".
    <algorithms> = comma separated list of algorithms. May use  "*".

For example, this runs all tests on all algorithms:


    rosrun led_detection <script> '*' '*'

The default algorithm is called "baseline", and the tests are invoked using:

    rosrun led_detection <script> '*' 'baseline'

"""

        msg = msg.replace('<script>', script_name)
        logger.error(msg)
        sys.exit(2)

    which_tests0 = sys.argv[1]
    which_estimators0 = sys.argv[2]

    root = os.environ['DUCKIETOWN_ROOT']
    dirname = 'catkin_ws/src/f23-LED/led_detection/scripts/'
    #filename = 'all_tests.yaml'
    filename = 'dp45_tests.yaml'
    filename = os.path.join(root, dirname, filename)

    alltests = load_tests(filename)
    estimators = {  'baseline' :
                    LEDDetector(ploteverything=False, verbose=True, plotfinal=False),
                    'LEDDetector_plots' : LEDDetector(True, True, True)}
                 #,'LEDDetector_forloops' : LEDDetector_forloops(True, True, True)}

    which_tests = expand_string(which_tests0, list(alltests))
    which_estimators = expand_string(which_estimators0, list(estimators))

    logger.info('     tests: %r |-> %s' % (which_tests0, which_tests))
    logger.info('estimators: %r |-> %s' % (which_estimators0, which_estimators))

    # which tests to execute
    test_results = {}
    for id_test in which_tests:
        for id_estimator in which_estimators:
            result = run_test(id_test, alltests[id_test], id_estimator, estimators[id_estimator])
            test_results[(id_test, id_estimator)] = result

    nfailed = list(test_results.values()).count(False)
    if not nfailed:
        logger.info('All tests passed')
    else:
        which = [k for k, v in test_results.items() if not v]
        logger.error('These tests failed: %s ' % which)
        sys.exit(3)

def is_match(detection, expected):
    # Determines whether a detection matches with an expectation
    # if either the frequency or the position match but something
    # else doesn't, it warns about what it is
    global W, H

    print('shape is %s, %s'% (W, H) )
    predicates = dict({
    'position': abs(1.0*detection.pixels_normalized.x*W-expected['image_coordinates'][0])<expected['image_coordinates_margin']
    and abs(1.0*detection.pixels_normalized.y*H-expected['image_coordinates'][1])<expected['image_coordinates_margin'],
    'frequency': detection.frequency == expected['frequency'],
    #'timestamps': abs(detection.timestamp1-expected['timestamp1'])<0.1 and
    #              abs(detection.timestamp2-expected['timestamp2'])<0.1
    })

    unsatisfied = [n for n in predicates if not predicates[n]]
    if(unsatisfied and (predicates['position'] or predicates['frequency'])):
        logger.warning('\nAlmost a match - (%s mismatch) - between detection: \n%s \nand expectation: \n%s'
                        % (unsatisfied, detection, expected))

    return not unsatisfied

def find_match(detection, expected_set):
    # return index (in expected) of the first match to detection
    # or -1 if there is no match
    try:
        return (n for n in range(len(expected_set)) if \
        (is_match(detection, expected_set[n]))).next()
    except StopIteration:
        return -1

def run_test(id_test, test, id_estimator, estimator):
    global W, H

    logger.info('     id_test: %s' % id_test)
    logger.info('id_estimator: %s' % id_estimator)
    from led_detection.unit_tests import LEDDetectionUnitTest
    assert isinstance(test, LEDDetectionUnitTest)
    query = test.get_query()
    print( query['images']['rgb'][0].shape)
    H, W, _ = query['images']['rgb'][0].shape
    print('shape is %s, %s'%(W, H))
    result = estimator.detect_led(**query)

    # We are testing whether the expected detections are a subset of
    # the returned ones, we will accept duplicate detections of the
    # same LED
    match_count = [0]*len(test.expected)
    for r in result.detections:
        m = find_match(r, test.expected)
        if(m != -1):
            match_count[m]+=1

    missedLEDs = [test.expected[i] for i in range(0,  len(match_count)) if match_count[i]==0]
    if(missedLEDs):
        logger.error('missed LED detections (%s): \n %s' % (len(missedLEDs),missedLEDs))

    return not 0 in match_count

if __name__ == '__main__':
    wrap_main(main)
