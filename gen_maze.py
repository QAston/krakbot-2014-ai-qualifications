#
# Wrapper dla maze_gen.py umozliwiajacy wywolanie z linii komend z podanymi opcjami
#
import os
import sys
# dodaj folder krak do pythonpath
sys.path.append(os.path.abspath("krak"))
import maze_gen

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("output_file", help="file to which map will be saved")
parser.add_argument("-n", type=int, default=20,
                    help="vertical map size")
parser.add_argument("-m", type=int, default=30,
                    help="horizontal map size")
parser.add_argument("--count_direction", type=int, default=0,
                    help="how many direction hints to add")
parser.add_argument("--count_distance", type=int, default=0,
                    help="how many distance hints to add")
parser.add_argument("--count_optimal", type=int, default=0,
                    help="how many optimal turn hists to add")
parser.add_argument("--prunning_prob", type=float, default=0,
                    help="After generating map the function will iterate\
                         over each field and prune (that is make\
                         it empty if it is a wall) with probability prunning_prob")
args = parser.parse_args()

maze_gen.generate_map(args.n, args.m, args.output_file, "",
                      count_direction = args.count_direction,
                      count_distance = args.count_distance,
                      count_optimal = args.count_optimal,
                      prunning_prob = args.prunning_prob);
