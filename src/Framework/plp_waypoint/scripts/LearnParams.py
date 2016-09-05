##
# This script takes a .csv file created by, e.g. PrepareDataset.py, and learns a predictive model for each target.
# Requires scikit-learn!

# 1. Load file
# 2. Detect targets and features
# 3. Create and print a model for each target, based on the models proposed by Revital(?)


class Dataset(object):

    def __init__(self, csv_lines):
        """
        Splits the lines to create a 2D array of data, and an array of features.
        :param csv_lines: Lines from a CSV file.
        :return:
        """
        raw_arrays = map((lambda line: line.split(",")), csv_lines)
        clean_arrays = map((lambda line: map((lambda cell: cell.strip()), line)), raw_arrays)
        self.headers = clean_arrays[0]
        self.target_titles = [e[2:] for e in self.headers if e.startswith("t:")]
        self.feature_titles = [e[2:] for e in self.headers if e.startswith("f:")]
        feature_count = len(self.feature_titles)
        self.data = map((lambda line: line[:feature_count]), clean_arrays[1:])
        self.targets = {}
        idx = feature_count
        for t in self.target_titles:
            trimmed_arrays = map((lambda line: line[idx:idx+1]), clean_arrays[1:])
            self.targets[t] = map((lambda line: line[0]), trimmed_arrays)
            idx += 1

        self._clean_arrays = clean_arrays  # FIXME remove


def load_file(path):
    f = open(path)
    lines = f.readlines()
    f.close()
    return Dataset(lines)



ds = load_file("Out.csv")