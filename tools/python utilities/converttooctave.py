from datasethandler import DatasetHandler
import argparse
parser = argparse.ArgumentParser(description="Dataset handler")
parser.add_argument("--path", type = str,help="dataset.txt path")
parser.add_argument("--header_length",type = int, default=10, help="Number of header lines to skip")
parser.add_argument("--dest", type = str,help="destination save path")

args = parser.parse_args()

pd = DatasetHandler(args.path,args.header_length)
#pd.view_traj()
pd.convertToOctave(args.dest)
