#! /usr/bin/env python
# -*- coding=utf-8 -*-

# /*******************************************************
#  * Copyright (C) 2019, Intelligent Positioning and Navigation Lab, Hong Kong Polytechnic University,
#  * 
#  * This file is part of UrbanLoco,
#  * 
#  * Licensed under the GNU General Public License v3.0;
#  * you may not use this file except in compliance with the License.
#  *
#  * Author: Weisong Wen (17902061r@connect.polyu.hk)
#  *******************************************************/
"""
    Function: output the trajectory from span-cpt as .kml file

"""
from lxml import etree  # output the kml node as string
import rospy
from pykml.factory import KML_ElementMaker as KML # use the factory module
import csv # csv reading needed library
from novatel_msgs.msg import BESTPOS # span-cpt message

class spancpt2kml():
    def __init__(self):
        rospy.Subscriber('/novatel_data/bestpos', BESTPOS, self.callcptBestPos_llh)
        self.lat_ = [] # used to save latitude
        self.lon_ = [] # used to save longitude
        self.GPS_Week_Second = 0.0
        self.writeToKML = 0.0

    def callcptBestPos_llh(self,data):
        self.bestPos_ = BESTPOS()
        self.bestPos_ = data
        self.lat_.append(float(self.bestPos_.latitude))
        print 'len(self.lat_)',len(self.lat_)
        self.lon_.append(float(self.bestPos_.longitude))
        self.GPS_Week_Second = self.bestPos_.header.gps_week_seconds
        print 'GPS_Week_Second',self.GPS_Week_Second


if __name__ == '__main__':
    rospy.init_node('spancpt2kmluGt', anonymous=True)
    spancpt2kml_ =spancpt2kml()
    rate = rospy.Rate(0.002)#
    preTim = 0.0
    while not rospy.is_shutdown():
        #rate.sleep()
        print 'GPS Time ',preTim,spancpt2kml_.GPS_Week_Second
	if( len(spancpt2kml_.lon_)>5):
            print 'write llh to kml'
            spancpt2kml_.writeToKML = 1
            # generate folder based on first point
            fold = KML.Folder(KML.Placemark(
                KML.Point(KML.coordinates(str(spancpt2kml_.lon_[0]) + ',' + str(spancpt2kml_.lat_[0]) + ',0'))
            )
            )
            # push the rest point to the folder
            for i in range(1, len(spancpt2kml_.lon_)):
                fold.append(KML.Placemark(
                    KML.Point(KML.coordinates(str(spancpt2kml_.lon_[i]) + ',' + str(spancpt2kml_.lat_[i]) + ',0')))
                )
            #  output string data from kml node using etree
            content = etree.tostring(etree.ElementTree(fold), pretty_print=True)
            # save kml file
            with open('span-cpt.kml', 'w') as fp:
                fp.write(content)

        preTim = spancpt2kml_.GPS_Week_Second