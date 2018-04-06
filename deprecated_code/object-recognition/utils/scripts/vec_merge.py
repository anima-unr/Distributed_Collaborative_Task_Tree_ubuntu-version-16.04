#!/usr/bin/env python
'''
Execute vec_merge.cc for each vec file in a given folder 
and output a single vec file
'''

import os
import shutil
import argparse
import weakref
from os import listdir
from os.path import isfile, join

def Merge(file_1, file_2, vec_file):
    os.system('./vec-merge -a {0} -b {1} -c{2}'.format(file_1, file_2, vec_file))
    print './vec-merge -a {0} -b {1} -c {2}'.format(file_1, file_2, vec_file)

class DoubleFileBuffer(object):
    def __init__(self, filename):
        self.filename = filename
        self.input = "a.buf"
        self.output = "b.buf"
        file = open(self.input, 'w')
        file.close()
        file = open(self.output, 'w')
        file.close()
    def __del__(self):
        shutil.copy(self.input, self.filename)
        os.remove(self.input)
        os.remove(self.output)
        print "deleting Files and closeing shop"

    def SwapBuffers(self):
        temp = self.input
        self.input = self.output
        self.output = temp

    def GetInput(self):
        return self.input
    def GetOutput(self):
        return self.output



def MergeVecFiles(directory, vec_file_name):
    directory = os.path.abspath(directory)
    vec_file_name = os.path.abspath(vec_file_name)
    print directory

    dir, filename = os.path.split(__file__)
    cwd = os.getcwd()

    os.chdir(dir)

    dir_list = [ f for f in listdir(directory) if isfile(join(directory,f)) ]
    init_file =  '{0}/{1}'.format(directory, dir_list[0])
    fbuffer = DoubleFileBuffer(vec_file_name)
    shutil.copy(init_file, fbuffer.GetInput())
    dir_list.pop(0)
    for file in dir_list:
        Merge(fbuffer.GetInput(), os.path.join(directory, file), fbuffer.GetOutput())
        fbuffer.SwapBuffers()


    del fbuffer
    os.chdir(cwd)

def main():

    parser = argparse.ArgumentParser(description='Process Input')
    parser.add_argument('-d', '--dir', type=str, required=True, help='File Directory for image files must be specified')
    parser.add_argument('-v', '--vec', type=str, required=True, help='.vec file name must be specified')
    args = parser.parse_args()

    MergeVecFiles(args.dir, args.vec)

if __name__ == '__main__':
    main()
