#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements: 
# sudo apt-get install python-argparse

"""
The Kinect provides the color and depth images in an un-synchronized way. This means that 
the set of time stamps from the color images do not intersect with those of the depth images. 
Therefore, we need some way of associating color images to depth images. For this purpose,
you can use the ''associate.py'' script. It reads the time stamps from the rgb.txt file and 
the depth.txt file, and joins them by finding the best matches.

FELICE Edit: We have edited the script to include the accelerometer and gyroscope measurements
Run as: 
python assoc_rgbdi.py $DATASET_DIR
"""

import argparse
import sys
import os
import numpy


def read_file_list(filename):
    """
    Reads a trajectory from a text file. 
    
    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp. 
    
    Input:
    filename -- File name
    
    Output:
    dict -- dictionary of (stamp,data) tuples
    
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    #HERE USED * to get the actual value
    list = [(float(l[0]),str(*l[1:])) for l in list if len(l)>1]
    return dict(list)

def associate(first_list, second_list,offset,max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim 
    to find the closest match for every input tuple.
    
    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
    
    """
    first_keys = list(first_list)#.keys()
    second_keys = list(second_list)#.keys()
    potential_matches = [(abs(a - (b + offset)), a, b) 
                         for a in first_keys 
                         for b in second_keys 
                         if abs(a - (b + offset)) < 20]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
    matches.sort()
    return matches

def associate_no_remove(first_list, second_list,offset,max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim 
    to find the closest match for every input tuple.
    
    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
    
    """
    first_keys  = list(first_list)#.keys()
    second_keys = list(second_list)#.keys()
    potential_matches = [(abs(a - (b + offset)), a, b) 
                         for a in first_keys 
                         for b in second_keys 
                         if abs(a - (b + offset)) < 20]
    potential_matches.sort()
    matches = []
    #INFO: This a>b is to ensure that accleration-gyro measurements will  have timestamps less than the current frame
    #i.e. ensure that acceleration-gyro measurements are inserted as they have come until the depth frame appeared
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys and a>b:
            #first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
    matches.sort()
    return matches

if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script takes four data files with timestamps and associates them   
    ''')
    parser.add_argument('dataset_directory', help='the directory of your dataset (format: string)')

    parser.add_argument('--first_only', help='only output associated lines from first file', action='store_true')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    args = parser.parse_args()

    if not os.path.exists(args.dataset_directory):
        print("Directory does not exist.")
        quit()

    if not os.path.isfile(args.dataset_directory + "/depth.txt"):
        print("Depth index not found.")
        quit()

    if not os.path.isfile(args.dataset_directory + "/rgb.txt"):
        print("RGB index not found.")
        quit()

    if not os.path.isfile(args.dataset_directory + "/acc.txt"):
        print("Accelerometer index not found.")
        quit()

    if not os.path.isfile(args.dataset_directory + "/gyr.txt"):
        print("Gyroscope index not found.")
        quit()

    depth_list = read_file_list(args.dataset_directory + "/depth.txt")
    rgb_list   = read_file_list(args.dataset_directory + "/rgb.txt")
    acc_list   = read_file_list(args.dataset_directory + "/acc.txt")
    gyr_list   = read_file_list(args.dataset_directory + "/gyr.txt")

    print("Synchronizing the RGB, Depth, Accelerometer and Gyroscope measurements, based on their timestamps.")
    print("The new indexes for the synchronized frames are: rgb_aligned.txt, depth_aligned.txt and imu_aligned.txt.")
    #For preintegration we need only one acceleration - gyro pair so associate one-to-one accel gyro. We want the maximum pairs of accel-gyro for each depth
    #frame so use associate_no_remove for this purpose. 
    matches_depthrgb = associate		    (depth_list,    rgb_list,   float(args.offset),float(args.max_difference))    #Associate depth with rgb images
    matches_depthacc = associate_no_remove  (depth_list,    acc_list,   float(args.offset),float(args.max_difference))    #Associate one depth with multiple Acceleration Frames
    matches_accgyr   = associate		    (acc_list,      gyr_list,   float(args.offset),float(args.max_difference))    #Associate one acceleration frame with one gyro

    # print(matches_accgyr)
    depth_keys  = list(depth_list)#.keys()
    rgb_keys    = list(rgb_list)#.keys()
    acc_keys    = list(acc_list)#.keys()
    gyr_keys    = list(gyr_list)#.keys()

    matches_acc     = []
    depth_aligned   = open(args.dataset_directory + "/depth_aligned.txt","w")
    rgb_aligned     = open(args.dataset_directory + "/rgb_aligned.txt","w")
    imu_aligned     = open(args.dataset_directory + "/imu_aligned.txt","w")

    if not os.path.exists(args.dataset_directory + "/imu/"):
        os.mkdir(args.dataset_directory + "/imu/")
    index = 0

    for d,r in matches_depthrgb: 
        depth_aligned.write('%f %s\n' % (d, depth_list.get(d)))
        rgb_aligned.write('%f %s\n' % (r, rgb_list.get(r)))
        imu_aligned.write('%f imu/i%d.csv\n' % (d,index))
        
        txt = args.dataset_directory + "/imu/i{index:d}.csv"
        #print(txt.format(index = index))
        imu_frame_file = open(txt.format(index = index),"w")
        print("Matched ... Depth Timestamp: %f Depth file: %s RGB Timestamp: %f RGB file: %s"%(d,depth_list.get(d),r, str(rgb_list.get(r))))
        for d2,a in matches_depthacc: 
            if d == d2: 
                for a2,g in matches_accgyr:
                    if a2 == a:
                        gyr_file = open(args.dataset_directory + "/" + gyr_list.get(g),"r")
                        str_gyr = gyr_file.readline().strip()
                        acc_file = open(args.dataset_directory + "/" + acc_list.get(a),"r")
                        str_acc = acc_file.readline().strip()
                        #print("%f %s %s"%(a2,str_acc,str_gyr))

                        imu_frame_file.write('%f %s %s\n'%(a2, str_acc, str_gyr))
                        print("\t\tAcc Timestamp: %f Accelorometer file: %s Gyroscope Timestamp: %f Gyroscope file: %s"%(a2,acc_list.get(a2),g, gyr_list.get(g)))

        index = index+1

    depth_aligned.close()
    rgb_aligned.close()
    imu_aligned.close()
        