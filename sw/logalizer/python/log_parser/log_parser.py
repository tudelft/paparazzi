import numpy as np

# Imports for opening a file from a file dialog
import tkinter as tk
from tkinter import filedialog

# xml parser includes
import xml.etree.ElementTree as ET

# Functions for plotter
import matplotlib.pyplot as plt

from lxml import etree

class LogParser(object):
    def __init__(self, t_start = None, t_end = None):
        '''
        Init function for the python LogParser class
        '''
        # Copy t_start and t_end
        self.t_start = t_start
        self.t_end = t_end

        # If creating a LogParser object, open a file dialog box to open a logile (.data)
        root = tk.Tk()
        root.withdraw()

        self.data_path = filedialog.askopenfilename(filetypes=[("data log files", ".data")])
        self.log_path  = self.data_path[:-4] + "log"

        # Parse telemetry message definitions
        self.msg_definition = {}
        self.import_message_definitions()

        # Parse log after log_path is known
        self.log_dict = {}
        self.parse_log()

    def import_message_definitions(self):
        '''
        Function that imports message definitions from .log file
        '''
        print(self.log_path)
        parser = etree.XMLParser(recover=True)
        xmlfile= open(self.log_path, 'r')
        xmlstring = xmlfile.read()
        xmlfile.close()
        tree = etree.fromstring(xmlstring, parser=parser)
        #tree = ET.parse(self.log_path)
        protocol = tree.find('protocol')

        for message in protocol.iter('message'):
            self.msg_definition[message.attrib['NAME']] = {}
            for field in message.iter('field'):
                self.msg_definition[message.attrib['NAME']][field.attrib['NAME']] = {}
                self.msg_definition[message.attrib['NAME']][field.attrib['NAME']]['type'] = field.attrib['TYPE']

                # Add non mandatory fields regarding units
                if ('ALT_UNIT_COEF' in field.attrib) and ('ALT_UNIT' in field.attrib):
                    self.msg_definition[message.attrib['NAME']][field.attrib['NAME']]['alt_unit_coef'] = field.attrib['ALT_UNIT_COEF']
                    self.msg_definition[message.attrib['NAME']][field.attrib['NAME']]['alt_unit'] = field.attrib['ALT_UNIT']
                if ('UNIT' in field.attrib):
                    self.msg_definition[message.attrib['NAME']][field.attrib['NAME']]['unit'] = field.attrib['UNIT']
    
    def parse_log(self):
        '''
        Function that reads the log lines and parses a log dictionary for each message in the log
        '''
        # Open log file and read lines
        log_lines = open(self.data_path, "r")
        
        # Loop through log lines
        for log_line in log_lines:
            # First split line at spaces
            splitted_line = log_line.split(" ")
            # remove /n from last element
            splitted_line[-1] = splitted_line[-1].replace('\n', '')

            ##########################
            # Order data in log_dict #
            ##########################

            # Check if msg_name already exists
            msg_name = splitted_line[2]
            if msg_name not in self.log_dict:
                # create time array
                self.log_dict[msg_name] = {'t' : []}
                # copy variables from message definitions
                for variable, attributes in self.msg_definition[msg_name].items():
                    self.log_dict[msg_name][variable] = {} 
                    self.log_dict[msg_name][variable]['data'] = []
                    self.log_dict[msg_name][variable]['type'] = attributes['type']
                    # Express variables in alt_unit if alt_unit is defined
                    if 'alt_unit' in attributes:
                        self.log_dict[msg_name][variable]['alt_unit_coef'] = float(attributes['alt_unit_coef'])
                        self.log_dict[msg_name][variable]['unit'] = attributes['alt_unit']
                    elif 'unit' in attributes:
                        self.log_dict[msg_name][variable]['unit'] = attributes['unit']

            ########################
            # Add data to log dict #
            ########################

            # Append timestamp-
            if self.t_start is not None:
                if float(splitted_line[0]) < self.t_start:
                    continue

            if self.t_end is not None:
                if float(splitted_line[0]) > self.t_end:
                    continue

            self.log_dict[msg_name]['t'].append(float(splitted_line[0]))

            # Append data
            i = 3 # first data index in splitted line
            for variable, attributes in self.log_dict[msg_name].items():
                # continue when variable is timestamp
                if variable == 't':
                    continue

                # check type for data conversion
                # If it is a list
                if '[]' in attributes['type']:
                    if 'int' in attributes['type']:
                        self.log_dict[msg_name][variable]['data'].append([])
                        variable_list = splitted_line[i].split(',')
                        if 'alt_unit_coef' in self.log_dict[msg_name][variable]:
                            for item in variable_list:
                                self.log_dict[msg_name][variable]['data'][-1].append(float(item) * self.log_dict[msg_name][variable]['alt_unit_coef'])
                        else:
                            for item in variable_list:
                                self.log_dict[msg_name][variable]['data'][-1].append(int(item))

                    elif 'float' in attributes['type']:
                        self.log_dict[msg_name][variable]['data'].append([])
                        variable_list = splitted_line[i].split(',')
                        if 'alt_unit_coef' in self.log_dict[msg_name][variable]:
                            for item in variable_list:
                                self.log_dict[msg_name][variable]['data'][-1].append(float(item) * self.log_dict[msg_name][variable]['alt_unit_coef'])
                        else:
                            for item in variable_list:
                                self.log_dict[msg_name][variable]['data'][-1].append(float(item))
                        
                    elif 'char' in attributes['type']:
                        self.log_dict[msg_name][variable]['data'].append(splitted_line[i])
                        
                # if it is  a single data value
                elif 'int' in attributes['type']:
                    if 'alt_unit_coef' in self.log_dict[msg_name][variable]:
                        self.log_dict[msg_name][variable]['data'].append(float(splitted_line[i]) * self.log_dict[msg_name][variable]['alt_unit_coef'])
                    else:
                        self.log_dict[msg_name][variable]['data'].append(int(splitted_line[i]))

                elif 'float' in attributes['type']:
                    if 'alt_unit_coef' in self.log_dict[msg_name][variable]:
                        self.log_dict[msg_name][variable]['data'].append(float(splitted_line[i]) * self.log_dict[msg_name][variable]['alt_unit_coef'])
                    else:
                        self.log_dict[msg_name][variable]['data'].append(float(splitted_line[i]))
            
                i += 1

        log_lines.close()

    def get_message_dict(self, msg_name):
        '''
        Function that returns the dict of a specific message
        '''
        message_dict = self.log_dict[msg_name]
        return message_dict

    def plot_variable(self, msg_name, variable_names = [], idx = []):
        '''
        Function that can plot a variable from the log
        '''
        # Create figure for plot
        plt.figure(msg_name)

        # Loop through variables to be plotted
        variable_counter = 0
        for variable in variable_names:
            if '[]' in self.log_dict[msg_name][variable]['type']:
                for i in idx[variable_counter]:
                    if 'unit' in self.log_dict[msg_name][variable]:
                        label = variable +'[' + str(i) + ']' + ' ' + self.log_dict[msg_name][variable]['unit']
                    else:
                        label = variable +'[' + str(i) + ']'
                    plt.plot(self.log_dict[msg_name]['t'], np.array(self.log_dict[msg_name][variable]['data'])[:,i], label=label)

            else:
                # define label
                if 'unit' in self.log_dict[msg_name][variable]:
                    label = variable + ' ' + self.log_dict[msg_name][variable]['unit']
                else:
                    label = variable

                plt.plot(self.log_dict[msg_name]['t'], self.log_dict[msg_name][variable]['data'], label=label)

            variable_counter += 1

        plt.legend()
