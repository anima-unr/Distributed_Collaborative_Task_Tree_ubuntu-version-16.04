#!/usr/bin/env python
from struct import *
import argparse

def main():
    parser = argparse.ArgumentParser(description='Process File input names')
    parser.add_argument('-a', type=str, required=True)
    parser.add_argument('-b', type=str, required=True)
    parser.add_argument('-c', type=str, required=True)

    args = parser.parse_args()
    af_poses = []
    bf_poses = []
    cf_poses = []
    with open(args.a, 'rb') as af:
        header_a = af.read(256)
        while True:
            key = af.read(256)
            if key:
                af_poses.append(key)
            else:
                break
    with open(args.b, 'rb') as bf:
        header_b = bf.read(256)
        while True:
            key = bf.read(256)
            if key:
                bf_poses.append(key)
            else:
                break

    with open(args.c, 'wb') as cf:
        # Merge files
        cf.write(header_a)
        for i in xrange(len(af_poses)):
            key = str(af_poses[i][0:128]).split('\0')[0]
            while True:
                input_text = raw_input('Take: ' + key + ' from FILE A or B?: ')
                if input_text == "a":
                    print "Merging from File A into File C"
                    cf.write(af_poses[i])
                    break
                elif input_text == "b":
                    print "Merging from File B into File C"
                    cf.write(bf_poses[i])
                    break
                else:
                    print "Unknown File choice Try again"


if __name__ == '__main__':
    main()