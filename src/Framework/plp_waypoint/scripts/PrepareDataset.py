#
#  Scans a directory of scenarios and creates a CSV file to be used as an input for machine learning.
#

import os
import sys
import yaml
from PlpWaypoint import PlpWaypoint
from PlpWaypointClasses import *

############################################
# Constants
############################################
TARGET_FILENAME = "results.txt"
TARGET_TITLES = ["minimal_distance_to_obstacle", "roll", "pitch", "distance_to_destination"]
PARAMETERS_FILENAME = "out.txt"
RECORD_DELIMITER = "###"

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
        self.variables = None # To be filled later, e.g. in calculate_variables.


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
    line = target_file.read()
    target = yaml.load(line)
    target_file.close()
    return target


def parse_parameters_file(path):
    """
    Parses the PLP harness' dump files into a dictionary with the parameters contained in the dump.
    :param path: A path to the dump file
    :return: Dictionary with the topic name and the last messaege sent on the topic when the dump occurred.
    """
    features_file = open(path, "r")
    lines = features_file.readlines()
    # Stage 1: Separate to dictionary of names and yaml strings
    initial_dict = {}
    cur_record = None
    cur_title = None
    for line in lines:
        if line.startswith(RECORD_DELIMITER):
            if cur_record is not None:
                initial_dict[cur_title] = "".join(cur_record)
            cur_title = line[3:].strip()
            cur_record = list()
        else:
            cur_record.append(line)

    initial_dict[cur_title] = "\n".join(cur_record)
    features_file.close()

    # Stage 2: Parse the yaml strings and make an object out of it.
    parameters = {}
    for key in initial_dict.keys():
        parameters[key] = yaml.load(initial_dict[key])

    # Stage 3: Compute the values of the topics into features.
    return parameters


def calculate_variables(cases):
    """
    Takes a list of cases (with the parameters), turns them to a list of variables and results.
    :param cases: parsed dump and result files
    :return: calculated variables, ready to be written to CSV.
    """
    plp_params = PlpWaypointParameters
    plp = PlpWaypoint(PlpWaypoint.constants_map(), plp_params, None)
    for case in cases:
        # placeholder: setting PLP parameters from case.data
        # placeholder: calling plp.calculate_variables()
        # placeholder: vars = plp.variables
        variables = {"idx": case.data["Object1"]["indexx"], "case": case.data["Object1"]["case"], "b": case.data["Object1"]["b"]}
        case.variables = variables

    return cases


def print_csv(cases, outFile):
    # 1. Decide on titles
    sample_case = cases[0]
    feature_titles = list(sample_case.variables.keys())
    feature_titles.sort()

    # 2. Print title line
    title_row = list()
    for ttl in feature_titles:
        title_row.append("f:" + ttl)
    for ttl in TARGET_TITLES:
        title_row.append("t:" + ttl)
    outFile.write(", ".join(title_row))
    outFile.write("\n")

    # 3. Print body
    for case in cases:
        line_buffer = list()
        for ft in feature_titles:
            line_buffer.append(case.variables[ft])
        line_buffer.extend(case.target)

        outFile.write(", ".join(map(str, line_buffer)))
        outFile.write("\n")


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
if __name__ == '__main__':
    rootDir = sys.argv[1]
    outFilePath = None

    log_info("Reading directory %s" % rootDir)

    if len(sys.argv) > 2:
        outFilePath = sys.argv[2]
        outFile = open(outFilePath, "w")
        log_info("dataset file: %s" % outFilePath)
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
