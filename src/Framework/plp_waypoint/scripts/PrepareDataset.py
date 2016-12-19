#!/usr/bin/env python
#
#  Scans a directory of scenarios and creates a CSV file to be used as an input for machine learning.
#

import rospy
import os
import sys
import pickle
import yaml

# From Glue
from robil_msgs.msg import Map, Path, AssignNavTask, AssignMission
from geometry_msgs.msg import PoseWithCovarianceStamped

# Because PLP messages
from std_msgs.msg import String, Header  # TODO replace with the plp message
from plp_waypoint.msg import PlpMessage

# Imported generated classes
from PlpWaypoint import *
from PlpWaypointClasses import *


############################################
# Constants
############################################
TARGET_FILENAME = "grades.txt"
PARAMETERS_FILENAME = "capture.txt"
TARGET_TITLES = ["minimal_distance_to_obstacle", "roll", "pitch", "distance_to_destination"]
RECORD_DELIMITER = "= "


############################################
# Helper Classes
############################################
class CaseResults(object):
    """
    Parsed results of a single case.
    """
    def __init__(self, data, target, source):
        self.data = data
        self.target = target
        self.source = source
        self.variables = None  # To be filled later, e.g. in calculate_variables.
        self.success = None  # To be filled later, e.g. in calculate_variables.


class GenericMessage(object):
    """
    Used to re-create the message objects from the yaml dictionaries
    """
    def __init__(self, data):
        for name, value in data.iteritems():
            setattr(self, name, self._wrap(value))

    def _wrap(self, value):
        if isinstance(value, (tuple, list, set, frozenset)):
            return type(value)([self._wrap(v) for v in value])
        else:
            return GenericMessage(value) if isinstance(value, dict) else value


############################################
# Procedures
############################################
def traverse_directory(path):
    """
    Go over a folder, parse all cases.
    :param path: Path of root folder
    :return: List of CaseResults
    """
    ret_val = list()
    for dirName, subdirList, fileList in os.walk(path):
        log_info("scanning: %s" % dirName)
        if not os.path.isfile(dirName + "/" + TARGET_FILENAME):
            log_warn("target file '%s' is missing. Skipping directory %s" % (TARGET_FILENAME, dirName))
            continue
        if not os.path.isfile(dirName + "/" + PARAMETERS_FILENAME):
            log_warn("feature file '%s' is missing. Skipping directory %s" % (PARAMETERS_FILENAME, dirName))
            continue

        ret_val.append(CaseResults(parse_parameters_file(dirName + "/" + PARAMETERS_FILENAME),
                                   parse_target(dirName + "/" + TARGET_FILENAME),
                                   dirName))

    return ret_val


def parse_target(path):
    """
    Parses the target files, which contain the simulation results
    :param path: A path to results file
    :return: An array of the results.
    """
    target_file = open(path, "r")
    line = target_file.read().split("---")[0]  # Sometimes there's more than a single result, we load only the first.
    target_file.close()
    target = yaml.load(line)
    return target["data"]


def parse_parameters_file(path):
    """
    Parses the PLP harness' dump files into a dictionary with the parameters contained in the dump.
    :param path: A path to the dump file
    :return: The PlpParameters object
    """
    features_file = open(path, "r")
    parameters = pickle.load(features_file)
    features_file.close()
    return parameters


def calculate_variables(cases):
    """
    Takes a list of cases (with the parameters), turns them to a list of variables and results.
    :param cases: parsed dump and result files
    :return: calculated variables, ready to be written to CSV.
    """
    plp_params = PlpWaypointParameters
    plp = PlpWaypoint(PlpWaypoint.constants_map(), plp_params, PlpWaypointDevNullCallback())
    for case in cases:
        log_info("Parsing case %s" % case.source)
        plp.parameters = case.data
        plp.parameters_updated()
        plp.calculate_variables()
        case.variables = plp.variables()

        case.success = 1 if is_considered_success(case) else 0
        print(repr(case.variables))

    return cases


def is_considered_success(case):
    """
    Calculates weather a case can be considered a success.
     This function should be re-written per PLP test.
    :param case: The case we judge
    :return: True iff (definitions from Daniel)
                1. minimal distance from obstacle should be > 0.5m
                2. distance to destination: < 5m
    """
    return case.target[0] > 0.5 and case.target[3] < 5


def print_csv(cases, out_file):
    # 1. Decide on titles
    sample_case = cases[0]
    feature_titles = [e for e in dir(sample_case.variables) if not e.startswith("_")]
    feature_titles.sort()

    # 2. Print title line
    title_row = list()
    for ttl in feature_titles:
        title_row.append("f:" + ttl)
    for ttl in TARGET_TITLES:
        title_row.append("t:" + ttl)
    title_row.append("t:success")
    out_file.write(", ".join(title_row))
    out_file.write("\n")

    # 3. Print body
    for case in cases:
        line_buffer = list()
        for ft in feature_titles:
            line_buffer.append(getattr(case.variables, ft))
        line_buffer.extend(case.target)
        line_buffer.append(case.success)

        out_file.write(", ".join(map(str, line_buffer)))
        out_file.write("\n")


############################################
# Logging
############################################
def log_info(msg):
    print("[INFO] %s" % msg)


def log_warn(msg):
    print("[WARN] %s" % msg)


############################################
# Main
############################################
# NOTE If we end up running as a ROS node, use rospy.myargv(argv=sys.argv) rather than sys.argv
if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)
    # argv = sys.argv
    rootDir = argv[1]
    outFilePath = None

    log_info("Reading directory %s" % rootDir)

    if len(sys.argv) > 2:
        outFilePath = sys.argv[2]
        outFile = open(outFilePath, "w")
        log_info("output file: %s" % outFilePath)
    else:
        outFile = sys.stdout

    # 1. Collect the results into a list of parameter dump objects
    results = traverse_directory(rootDir)

    # 2. Calculate the variables from the parameters
    calculate_variables(results)

    # 3. Print the CSV to stdout.
    print_csv(results, outFile)

    if outFilePath is not None:
        outFile.close()

    log_info("Done.")
