from datasethandler import DatasetHandler

pd = DatasetHandler("dataset.txt",header_length=9)
pd.view_traj()
pd.convertToOctave()