#!/usr/bin/env python

''' 

Use os.system to automate opencv create samples command line argument 
for every file in a directory and pass the command to the command line
with that file specified and a new .vec file name

'''
import os
import argparse
import shutil
from vec_merge import MergeVecFiles


def CreateCommand(vec_file, img_file, bg_file, num_samples, bg_color, bg_thresh, w, h):
    command = 'opencv_createsamples -vec {0} -img {1} -bg {2} -num {3} -bgcolor {4} -bgthresh{5} -w {6} -h {7}'.format(vec_file, img_file, bg_file, num_samples, bg_color, bg_thresh, w, h)
    print command
    return command

def CreateVecFile(command):
    os.system(command)


def main(): 
    #specify command line arguments
    parser = argparse.ArgumentParser(description='Process Input')
    parser.add_argument('-d', '--imgdir', type=str, required=True, help='File Directory for image files must be specified')
    parser.add_argument('-vec', type=str, required=True, help='.vec file name must be specified')
    parser.add_argument('-bg', type=str, required=True, help='Background Images File must be specified')
    parser.add_argument('-num', type=int, required=True, help='Desired number of positive samples')
    parser.add_argument('-bgcolor', type=int, required=True, help='Backgound image color (0-255)')
    parser.add_argument('-bgthresh', type=int, required=True, help='Background threshold (0-255)')
    parser.add_argument('-x', '--width', type=int,required=True, help='Desired output image width:x')
    parser.add_argument('-y', '--height', type=int, required=True, help='Desired output image height:y')
    args = parser.parse_args()

    images_file_list = os.listdir(args.imgdir)

    new_dir = args.imgdir + "_vec_files"

    if not os.path.exists(new_dir):
       os.mkdir(new_dir)


    #run opencv_createsamples for each image in directory
    for image_name in images_file_list:
        image_file = os.path.join(args.imgdir,image_name)
        (name, ext) = os.path.splitext(image_name)
        vec_file = args.vec + "_" + name + '.vec'
        command = CreateCommand(vec_file, image_file, args.bg, args.num, args.bgcolor, args.bgthresh, args.width, args.height)
        CreateVecFile(command)
        
        #move all vec files created into one directory
        shutil.move(vec_file, new_dir)

    MergeVecFiles(new_dir, args.vec)

if __name__ == '__main__':
    main()