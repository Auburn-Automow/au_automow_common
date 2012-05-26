#!/usr/bin/env python

import os

os.chdir("bin")

for dir, root, file in os.walk(os.getcwd()):
	for f in file:
		if f.endswith("dylib") or f == 'ground_filter':
			import subprocess
			r = subprocess.check_output(["otool", "-L", f])
			r = r.split('\n')
			for l in r:
				l = l.strip()
				l = l.split()
				if len(l) == 0:
					continue
				l = l[0]
				if l.find("flann") != -1:
					old_l = str(l)
					l = l.split("/")
					l = l[-1]
					print f, l
					cmd = ["install_name_tool", "-change", old_l, l, f]
					subprocess.call(cmd)
				if not l.startswith("/") and not l.startswith(".."):
					rt = os.getcwd()
					if l.find("flann") != -1:
						rt = "/Users/william/devel/ros/perception_pcl/flann/lib"
					if l.find("pcl") != -1:
						rt = "/Users/william/devel/ros/perception_pcl/pcl/lib"
					cmd = ["install_name_tool", "-change", l, os.path.join(rt, l), f]
					print cmd
					subprocess.call(cmd)