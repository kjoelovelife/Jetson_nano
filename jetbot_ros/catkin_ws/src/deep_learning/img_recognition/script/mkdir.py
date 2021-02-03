#!/usr/bin/env python3

import os , sys , argparse ,errno , yaml
import rospy


class mkdir(object):
    
    def __init__(self):
        self.package = "img_recognition"
        self.parser = argparse.ArgumentParser(description="Make a folder in ~/ROSKY/catkin_ws/src/" + self.package +"/image",epilog="save your image")
        self.parser.add_argument("--name" , "-n" , type=str,required=True,help="Please type you want the name of folder.")
        self.args = self.parser.parse_args()
        self.path = os.path.abspath(os.path.join(os.path.dirname(__file__),os.pardir)) # "~/ROSKY/catkin_ws/src/" + self.package
        self.make = False
        action_1 = self.try_make(self.args.name)
        action_2 = self.read_param_from_file()
        action_3 = self.write_to_file(self.make,self.args.name)

    def try_make(self,name):
        try:
            save_path = self.path + "/image/" + name
            os.makedirs(save_path)
            self.make = True
            print("Done! Make a directory: {}".format(save_path))            
        except OSError as e:
            if e.errno == errno.EEXIST:
                print("Note! Directory not created because it already exit. ")
            else:
                raise

    def read_param_from_file(self):
        fname = self.path + "/param/image_label.yaml"
        with open(fname, 'r') as in_file:
            try:
                self.yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                print(" YAML syntax error. File: {}".format(fname))
        if self.yaml_dict != None: 
            for label_name in self.yaml_dict:
                image_count = 0
                for dir_path, dir_names, file_names in os.walk(self.path + "/image/" + str(label_name) + "/"):
                    for image in file_names:
                        if image.endswith('jpg') or image.endswith('jpeg') :
                            image_count += 1  
                self.yaml_dict[label_name] = image_count
        else:
            self.yaml_dict = {}    

    def write_to_file(self,make,name):
        fname = self.path + "/param/image_label.yaml"
        if make == True:
            self.yaml_dict[name] = 0
        data = self.yaml_dict
        with open(fname, 'w') as outfile:
            outfile.write(yaml.dump(data, default_flow_style=False)) 
           

if __name__ == "__main__" :
    make = mkdir()
    
    
