import csv
import os

# 数据文件路径，通过环境变量 DATA_ROOT 指定数据根目录，默认为当前目录
_DATA_ROOT = os.environ.get("DATA_ROOT", os.path.dirname(os.path.abspath(__file__)))
file_location = os.path.join(_DATA_ROOT, "ViconRoom101/state_groundtruth_estimate0/data.csv")
with open(os.path.join(_DATA_ROOT, "ViconRoom101/state_groundtruth_estimate0/data.txt"), 'w') as txt_f:
	with open(file_location) as f:
	    f_csv = csv.reader(f)
	    headers = next(f_csv)
	    for row in f_csv:
	    	txt_f.write('%lf\n'% (float(row[0]) / 1000000000.0) )
	    	txt_f.write('%lf\n'% (float(row[1])) )
	    	txt_f.write('%lf\n'% (float(row[2])) )
	    	txt_f.write('%lf\n'% (float(row[3])) )
	    	txt_f.write('%lf\n'% (float(row[4])) )
	    	txt_f.write('%lf\n'% (float(row[5])) )
	    	txt_f.write('%lf\n'% (float(row[6])) )
	    	txt_f.write('%lf\n'% (float(row[7])) )