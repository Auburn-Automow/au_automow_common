#!/usr/bin/env python

"""
Script to convert the (old) csv field format to the new yaml format.
"""

import sys, os, csv
from yaml import dump

def usage():
    print("map_csv2yaml.py </path/to/src_field.csv>")
    print("  The resulting yaml file is put in the same ")
    print("  directory with the name, but different extension.")
    sys.exit(0)

def main():
    if len(sys.argv) < 2:
        usage()
    csv_file_name = sys.argv[1]
    if not os.path.exists(csv_file_name):
        print("Error: file %s does not exist."%csv_file_name)
        usage()
    csv_file = csv.reader(open(csv_file_name, 'r'))
    result = []
    for index, row in enumerate(csv_file):
        if len(row) != 3:
            print("Line %i in csv file %s invalid"%(index,csv_file_name))
            continue
        result.append({'easting': row[0],
                       'northing': row[1],
                       'fix_type': row[2]})
    # put the result into a yaml file
    yaml_file_name = csv_file_name[:-4]+'.yaml'
    yaml_file = open(yaml_file_name, 'w')
    yaml_file.write(dump(result))

if __name__ == '__main__':
    main()
