#!/usr/bin/env python3

import os , sys , argparse ,errno , yaml
import rospy


class RVDIR(object):
    
    def __init__(self):
        self.package = "img_recognition"
        self.parser = argparse.ArgumentParser(description="remove a folder in ~/ROSKY/catkin_ws/src/" + self.package +"/image",epilog="removeyour image")
        self.parser.add_argument("--name", "-rm", type=str,required=True, help="Please type you want to remove the folder name.")
        self.args = self.parser.parse_args()
        self.path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)) # "~/ROSKY/catkin_ws/src/" + self.package
        self.remove = False
        action_1 = self.read_param_from_file()
        action_2 = self.try_remove(self.args.name)
        action_3 = self.write_to_file(self.remove, self.args.name)

    def try_remove(self, name):
        remove_path = self.path + "/image/" + name
        if os.path.isdir(self.path + "/image/" + name):
            if self.yaml_dict[name] == 0:
                try:
                    os.rmdir(remove_path)
                    self.remove = True
                    print("Done! Remove folder : {}".format(remove_path))            
                except OSError as e:
                    if e.errno == errno.EEXIST:
                        print("Note! Directory not exit. ")
                    else:
                        raise
            else:
                msg = "There are {} images in folder [{}]. Do you want to remove?(y/N): ".format(self.yaml_dict[name], name)
                remove = input(msg)
                if remove == "y" or remove == "Y":
                    os.rmdir(remove_path)
                    self.remove = True
                    print("Done! Remove folder: {}".format(remove_path))
                else:
                    print("Skip remove [{}]".format(self.path + "/image/" + name))   

        else:
            print("[{}] is not exist.".format(remove_path))
            sys.exit(00)


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

    def write_to_file(self, remove, name):
        fname = self.path + "/param/image_label.yaml"
        if remove == True:
            self.yaml_dict.pop(name)
        data = self.yaml_dict
        with open(fname, 'w') as outfile:
            outfile.write(yaml.dump(data, default_flow_style=False)) 

if __name__ == "__main__" :
    remove = RVDIR()
    
    
