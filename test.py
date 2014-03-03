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
# todo: run tests in parallel - or not - it's bad for debugging
test_id = 0
num_tests = len(tests)
failed_tests = []
results = []
for test in tests:
    test_id += 1
    print "--+Running test ({},{}) of {}:".format(test_id, test["map"], num_tests)
    test["map"] = str(test["map"]) # because of stupid check in simulator
    sim = simulator.KrakrobotSimulator(robot_controller_class=robot_class, simulation_dt=0.0, visualisation=False, iteration_write_frequency=1000000, **test)
    result = sim.run();
    print "--+Test{} has finished. Results: {}".format(test_id, result)
    results.append(result)

    if result["goal_achieved"] == False:
        failed_tests.append((test_id, test["map"]))

test_id = 0
print "--+All Results:"
for test in tests:
    test_id += 1
    results[test_id-1]["test_id"] = test_id
    print "--+Test{} Result: {}".format(test_id, results[test_id-1])


if len(failed_tests) > 0:
    print "--+Failed tests: ",str(failed_tests)
else:
    print "--+All test were successful"

#store results
results_file_name = args.test_file[0:-5]+"_results.json"
result_file = open(results_file_name, "w")
json.dump(results, result_file, indent=4);
result_file.close()

print "--+Results are written to :{}".format(results_file_name)

