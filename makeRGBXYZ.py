import pandas as pd
from pyntcloud import PyntCloud
import sys

if len(sys.argv) == 2:
	f = sys.argv[1]
	cloud = PyntCloud.from_file(f,
	                            sep=" ",
	                            names=["x","y","z","red","green","blue"])
	print(cloud)

	cloud.to_file("out_file.ply", as_text=True)
else:
	print("Error: makeRGBXYZ.py requires an argument\nPlease call file to generate .ply and .pcd from")