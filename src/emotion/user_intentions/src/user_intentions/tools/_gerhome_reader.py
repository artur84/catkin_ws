#! /usr/bin/env python
import sys
import re  # Package to work with regular expressions
import rospy
import numpy as np
import yaml
import math
from copy import deepcopy 
from Crypto.Util.number import str2long

import xlwt
from datetime import datetime

#==============================================================================
#==============================================================================
class Excel_Writer():
    """ This is to take data froma  scenario_goals.yaml and print it as excel file """
    def __init__(self):
        self.goals_file = '/home/artur/ros_works/pal/teams/emotion/wheelchairconf/world/gerhome_goals.yaml'
        self.excel_file = '/home/artur/Documents/videos_wheelchair/gerhome_dataset_videos/2008_03_27/gerhome_goals.xls'
        self.init_data_flag = False
        self.read_dictionary()
        
    def read_dictionary(self):
        """ Reads the data of typical destinations in the selected scenario
        """
        self.init_data_flag = True
        try:
            with open(self.goals_file) as opened_file:
                self.__goal_dictionary__ = yaml.load(opened_file)
        except IOError as exc:
            print "The file CAN'T BE READ: ", self.goals_file
        opened_file.close()  
             
    def write_excel(self):
        font0 = xlwt.Font()
        font0.name = 'Times New Roman'
        font0.colour_index = 2
        font0.bold = True
        p_g_xc0 = self.__goal_dictionary__['p_g_xc0']
        poses = self.__goal_dictionary__['goals']
        tags = self.__goal_dictionary__['tags'] 
        style0 = xlwt.XFStyle()
        style0.font = font0
        
        print poses

        style1 = xlwt.XFStyle()
        style1.num_format_str = 'D-MMM-YY'

        wb = xlwt.Workbook()
        ws1 = wb.add_sheet('Poses')
        ws2 = wb.add_sheet('Probability')
        row = 2
        
        # POSES
        ws1.write(0, 0, 'Goals(x,y,theta)', style0)
        ws1.write(1, 0, datetime.now(), style1)
        for pose in poses:
            print pose[0]
            print row
            ws1.write(row, 0, pose[0])
            ws1.write(row, 1, pose[1])
            ws1.write(row, 2, pose[2])
            row += 1
            
        # PROBABILITIES TABLE
        ws2.write(0, 0, 'Probability', style0)
        ws2.write(1, 0, datetime.now(), style1)
        row = 2
        for p_row in p_g_xc0:
            col = 2
            for p_col in p_row:
                ws2.write(row, col, p_col)
                col += 1
            row += 1
            
        print "Saving file:" + self.excel_file
        wb.save(self.excel_file)
#==============================================================================
#==============================================================================
class GerhomeReader():
    """ @package ui
    
        Reads the sensor logs taken from gerhome house, and computes the statistics of typical destinations
        in the scene to be used as apriori info for the user intentions algorithm.
        @author: Jesus Arturo Escobedo Cabello
        @contact: jesus.escobedo-cabello@inria.fr
        @organization: INRIA Grenoble, Emotion-team
    """
    
    def __init__(self):
        self.log_file = '/home/artur/Documents/videos_wheelchair/gerhome_dataset_videos/2008_03_27/total.evt-log'
        self.goals_file = '/home/artur/Documents/videos_wheelchair/gerhome_dataset_videos/2008_03_27/PRUEBA_GERHOME.yaml'
        self.data = []  # A list of dictionaries containing the sorted information of the log_file
        self.__goal_dictionary__ = {'tags':[], 'goals':[], 'p_g_xc0':[], 'p_desired_measured':[]} 
        self.__init_dictionary_flag__ = False
        self.local_goals = {}
        self.global_goals = {}
        self.__read_data__()
        self.excel_writer = Excel_Writer()

    def __read_data__(self):
        """ Reads the data in self.log_file, 
            converts it into a list of dictionaries
            stores the list in self.data
        """
        try:
            print "Trying read data from File:", self.log_file
            with open(self.log_file, "r") as file:
                print "Data read from ", self.log_file
                for line in file:
                    # print "for"
                    dictionary = self.get_dictionary_from_line(line)
                    if dictionary is not None:
                        self.data.append(dictionary)
                self.merge_same_places()
                # print "file was succesfully read"   
                file.close()
        except IOError as exc:
            print "The file CAN'T BE ACCESSED: ", self.log_file
        

    
    def get_dictionary_from_line(self, str_line):
        """ Extract statistics from self.data
            looks for strings with a pattern similar to 080414-081045.460/OPENCLOSE/KITCHEN.CUPBOARD.UPPER.RIGHT.1/open
            @param str_line: The string to look for the pattern 
            @return dictionary = {"time":time, "activity":activity, "room":room, "location":location_complete, "flag":flag}
        """
        self.file_pattern = "([\d]+-[\d]+.[\d]+)/([\w]+)/([\w]+).(.+)/([\w]+)"  # pattern of the type (digits-digits.digits/strings) eg. (080327-081045.712/PRESENCE)
        m = re.search(self.file_pattern, str_line)  # Look for the first time the pattern appears in the self.data string
        if m is not None:
            time = m.group(1)
            activity = m.group(2)
            room = m.group(3)
            location_complete = m.group(4)
            flag = m.group(5)
            dictionary = {"time":time, "activity":activity, "room":room, "location":location_complete, "flag":flag}
        else:
            dictionary = None
        return dictionary
        # print re.findall(self.patterns[0], self.data)
    def merge_same_places(self):
        """In the file there are locations that should be the same because they are just diferent for the action  KITCHEN.SINK.HOT and KITCHEN.SINK.COLD
        """
        pattern = "SINK"  # pattern 
        for element in self.data:
            m = re.search(pattern, element["location"])  # Look for the first time the pattern appears in the self.data string
            """If COLD is found delete it """
            # print "element:", element["location"]
            if m is not None:
                element["location"] = pattern
        
        pattern = "WASHBOWL"  # pattern 
        for element in self.data:
            m = re.search(pattern, element["location"])  # Look for the first time the pattern appears in the self.data string
            """If COLD is found delete it """
            # print "element:", element["location"]
            if m is not None:
                element["location"] = pattern
        
        pattern = "DRAWER"  # pattern 
        for element in self.data:
            m = re.search(pattern, element["location"])  # Look for the first time the pattern appears in the self.data string
            """If COLD is found delete it """
            # print "element:", element["location"]
            if m is not None:
                element["location"] = pattern
        
        pattern = "CUPBOARD.UPPER.RIGHT"  # pattern 
        for element in self.data:
            m = re.search(pattern, element["location"])  # Look for the first time the pattern appears in the self.data string
            """If COLD is found delete it """
            # print "element:", element["location"]
            if m is not None:
                element["location"] = "SINK"
                
        pattern = "STOVE"  # pattern 
        for element in self.data:
            m = re.search(pattern, element["location"])  # Look for the first time the pattern appears in the self.data string
            """If COLD is found delete it """
            # print "element:", element["location"]
            if m is not None:
                element["location"] = "REFRIGERATOR"
        
        pattern = "CUPBOARD.UPPER.CENTER"  # pattern 
        for element in self.data:
            m = re.search(pattern, element["location"])  # Look for the first time the pattern appears in the self.data string
            """If COLD is found delete it """
            # print "element:", element["location"]
            if m is not None:
                element["location"] = "REFRIGERATOR"
                
        pattern = "DRAWER"  # pattern 
        for element in self.data:
            m = re.search(pattern, element["location"])  # Look for the first time the pattern appears in the self.data string
            """If COLD is found delete it """
            # print "element:", element["location"]
            if m is not None:
                element["location"] = "MICRO_WAVE_OWEN"
        
        pattern = "CUPBOARD.UPPER.LEFT"  # pattern 
        for element in self.data:
            m = re.search(pattern, element["location"])  # Look for the first time the pattern appears in the self.data string
            """If COLD is found delete it """
            # print "element:", element["location"]
            if m is not None:
                element["location"] = "MICRO_WAVE_OWEN"
        
        pattern = "CHAIR"  # pattern 
        for element in self.data:
            m = re.search(pattern, element["location"])  # Look for the first time the pattern appears in the self.data string
            """If COLD is found delete it """
            # print "element:", element["location"]
            if m is not None:
                element["location"] = pattern
                

    def set_typical_loc_counters(self):
        """ Counts how many times each location in the data is visited by the user
        """
        starting_point_local = "START"
        last_location = "START"
        starting_point_global = "START"
        self.local_goals = {}
        self.global_goals = {}
        """Counts the number of times that the user moves from one place to another"""
        for element in self.data:
            # print "element", element['location']
            # print 'last', last_location
            if element["location"] != last_location:
                # print "if"
                change = True
                final_point_local = element["room"] + "." + element["location"]
                final_point_global = element["room"]
                """ local goals """
                if starting_point_local in self.local_goals:  # if the element is already in the list inc the counter
                    if final_point_local in self.local_goals[starting_point_local]:
                        self.local_goals[starting_point_local][final_point_local]['counter'] += 1.0
                    else:
                        self.local_goals[starting_point_local].setdefault(final_point_local, {'counter':1.0, 'proba':0})
                else:
                    self.local_goals.setdefault(starting_point_local, {final_point_local:{'counter':1.0, 'proba':0}})
                """ global goals """
                if starting_point_global in self.global_goals:
                    if final_point_global in self.global_goals[starting_point_global]:
                        self.global_goals[starting_point_global][final_point_global]['counter'] += 1.0
                    else:
                        self.global_goals[starting_point_global].setdefault(final_point_global, {'counter':1.0, 'proba':0})
                else:
                    self.global_goals.setdefault(starting_point_global, {final_point_global:{'counter':1.0, 'proba':0}})
            else:
                change = False
            last_location = deepcopy(element["location"])
            starting_point_local = element["room"] + "." + element["location"]
            starting_point_global = element["room"]
        """ Compute probabilities """
        total_global_count = 0.0
        total_local_count = 0.0
        # For global positions
        # Total number of changes of position
        for (init, final) in self.global_goals.iteritems():
            for (final_goal, value) in final.iteritems():
                total_global_count = total_global_count + value['counter']
            for (final_goal, value) in final.iteritems():
                value['proba'] = value['counter'] / total_global_count
            total_global_count = 0
        # For LOCAL positions
        # Total number of changes of position
        for (init, final) in self.local_goals.iteritems():
            for (final_goal, value) in final.iteritems():
                total_local_count = total_local_count + value['counter']
            for (final_goal, value) in final.iteritems():
                value['proba'] = value['counter'] / total_local_count
            total_local_count = 0
            

    def set_goal_dictionary(self):
        """ Write the probability tables to a text file """
        tags = []
        poses = []
        # Extracting all the goals and put them in the dictionary
        for (local_key, local_value) in self.local_goals.iteritems():
            if local_key not in tags:
                tags.append(local_key)
                poses.append([0, 0, 0])
            for (final_goal_key, final_value) in local_value.iteritems():
                if final_goal_key not in tags:
                    tags.append(final_goal_key)
                    poses.append([0, 0, 0])
        # init the matrix of probabilities with the same size as the number of elements in labels
        size = len(tags)
        matrix = np.zeros((size, size))
        
        for (local_goal_key, local_goal_value) in self.local_goals.iteritems():
            local_goal_index = tags.index(local_goal_key)
            for (final_goal_key, final_goal_value) in local_goal_value.iteritems():
                final_goal_index = tags.index(final_goal_key)
                matrix[local_goal_index, final_goal_index] = final_goal_value["proba"]
        # Filling the dictionary
        self.__goal_dictionary__['p_g_xc0'] = matrix.tolist()
        self.__goal_dictionary__['goals'] = poses
        self.__goal_dictionary__['tags'] = tags
        print "************  self.__goal_dictionary__ **********"
        print self.__goal_dictionary__
        
        self.__init_dictionary_flag__ = True  # to indicate that the goal_dictionary was initialized
                 
                
    def write_goals_file(self):
        if self.__init_dictionary_flag__:
            try:
                print "Trying open File: ", self.goals_file
                with open(self.goals_file, "w+") as file:
                    print "File was opened successfully", self.goals_file
                    yaml.dump(self.__goal_dictionary__, file)
                file.close()
            except IOError as exc:
                print "The file CAN'T BE ACCESSED: %s", self.goals_file
        else:
            print "WARNING: the dictionary was not initialized please call the set_goal_dictionary() function first"
    
    def time_at_this_location(self, location):
        """ gets how much time the user spent in this location before moving """
        print "do something here"
        
    def read_dictionary(self):
        """ Reads the data of typical destinations in the selected scenario
        """
        try:
            with open(self.goals_file) as opened_file:
                self.data = yaml.load(opened_file)
        except IOError as exc:
            print "The file CAN'T BE READ: ", self.goals_file
        self.static_goals_X_Y_THETA_array = np.asfarray(self.data["goals"])  # The matrix containing [x,y,theta] for each goal
        self.goals_X_Y_THETA_array = deepcopy(self.static_goals_X_Y_THETA_array)
        self.p_g_xc0 = np.asfarray(self.data["p_g_xc0"])  # Prior probability
        print self.p_g_xc0
        self.static_goals_prior = deepcopy(self.p_g_xc0) 
        self.static_tag_list = self.data['tags']  # The tag for each goal
        self.p_desired_measured = np.asfarray(self.data["p_desired_measured"])
        self.number_of_static_goals = deepcopy(np.shape(self.static_goals_X_Y_THETA_array)[0])
        # print self.p_desired_measured
        opened_file.close()  


        
        
if __name__ == "__main__":
    # print "init gerhome_reader"
    gr = GerhomeReader()
    # print "finished"
    # print gr.data[6]
    gr.set_typical_loc_counters()
    gr.set_goal_dictionary()
    gr.write_goals_file()
    gr.read_dictionary()
    gr.excel_writer.write_excel()
    
    
    # print gr.global_goals
    # print gr.local_goals
