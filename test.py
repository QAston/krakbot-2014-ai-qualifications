#
# Program automatyzujacy testy robota
#
import os
import sys
import argparse
import json
import minify_json
# dodaj folder krak do pythonpath
sys.path.append(os.path.abspath("krak"))
import simulator
from robot_controller import compile_robot

parser = argparse.ArgumentParser()
parser.add_argument("test_file", help="path to a file with tests specification")
parser.add_argument("robot_file", help="path to a RobotController file")
#parser.add_argument("--maps_base_path", help="path to a RobotController file") TODO
args = parser.parse_args()

tests_file = open(args.test_file)
minified = minify_json.json_minify(tests_file.read())

robot_class = compile_robot(args.robot_file)[0]

tests = json.loads(minified);
# todo: run tests in parallel
test_id = 0
failed_tests = []
for test in tests:
    test["map"] = str(test["map"]) # because of stupid check in simulator
    sim = simulator.KrakrobotSimulator(robot_controller_class=robot_class, simulation_dt=0.0, visualisation=False, **test)
    result = sim.run();
    print "Test{} has finished. Results: {}".format(test_id, result)
    # todo: store results

    if result["goal_achieved"] == False:
        failed_tests.append(test_id)

    ++test_id

if len(failed_tests) > 0:
    print "Failed tests: ", ",".join(failed_tests)
else:
    print "All test were successful"
