#!/usr/bin/env python

import os, sys, argparse, errno, yaml, time, datetime
import cv2, numpy 
import rospy, rospkg, threading
import torch, torchvision
import torch.optim as optim
import torch.nn.functional as function
import torchvision.datasets as datasets
import torchvision.models as models
import torchvision.transforms as transforms

class Train_Model_Node(object):
 
    ####
    # local param :
    # ros param   : ~model, 
    # service     : 
    ####  

    def __init__(self):
        self.package = "img_recognition"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        rospy.loginfo("{}  Initializing train_model.py......".format(self.node_name))

        # set/get ros param
        self.loader_batch_size  = rospy.get_param("~loader/batch_size",8)
        self.loader_shuffle = rospy.get_param("~loader/shuffle",True)
        self.loader_num_workers = rospy.get_param("~loader/num_workers",0)
        self.use_cuda = rospy.get_param("~train/use_cuda",True)
        self.param_model = rospy.get_param("~train/model","alexnet")
        self.train_save_model_name  = rospy.get_param("~train/save_model_name","best")
        self.train_epochs = rospy.get_param("~train/epochs",30)
        self.train_lr = rospy.get_param("~train/learning_rate",0.001)
        self.train_momentum = rospy.get_param("~train/momentum",0.9)
        
        # set local param
        self.yaml_dict = {}
        self.kind_of_classifier = 0

        # read label
        self.yaml_dict, self.kind_of_classifier = self.read_param_from_file() # will get self.yaml_dict, self.kind_of_classifier
        
        # initial model
        self.train_save_model_name  = self.compare_model_name(self.train_save_model_name)
        _data = self.rule_for_datasets(batch_size=self.loader_batch_size, shuffle=self.loader_shuffle, num_workers=self.loader_num_workers)
        _model = self.neural_network(model=self.param_model, param_pretrained=True, kind_of_classifier=self.kind_of_classifier)
        _cuda = self.cuda(use=self.use_cuda) 

        # train model
        self.train(epochs=self.train_epochs, best_model_path=self.train_save_model_name, learning_rate=self.train_lr, momentum=self.train_momentum)


    def rule_for_datasets(self, batch_size=8, shuffle=True, num_workers=0):
        folder = rospkg.RosPack().get_path(self.package) + "/image" 
        self.dataset = datasets.ImageFolder(
                           folder,
                           transforms.Compose([
                               transforms.ColorJitter(0.1, 0.1, 0.1, 0.1),
                               transforms.Resize((224, 224)),
                               transforms.ToTensor(),             
                               transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
                                           ]) 
                        )
        # split dataset
        self.train_dataset, self.test_dataset = torch.utils.data.random_split(self.dataset, [len(self.dataset)-50, 50])

        # create data loaders to load data in batches
        self.train_loader = torch.utils.data.DataLoader(self.train_dataset, batch_size=batch_size, shuffle=shuffle, num_workers=num_workers)
        self.test_loader = torch.utils.data.DataLoader(self.test_dataset, batch_size=batch_size, shuffle=shuffle, num_workers=num_workers)

    def neural_network(self, model="alexnet", param_pretrained=False, kind_of_classifier=2):
        # reference : https://pytorch.org/docs/stable/torchvision/models.html
        model_list = [
                      "resnet18", "alexnet", "squeezenet", "vgg16", 
                      "densenet", "inception", "googlenet", "shufflenet", 
                      "mobilenet", "resnet34", "wide_resnet50_2", "mnasnet" 
                     ]

        if model in model_list:
            rospy.loginfo("You use model [{}]. Need some time to load model...".format(model))
            start_time = rospy.get_time()
            if model == "resnet18":
                self.model = models.resnet18(pretrained=param_pretrained)
                self.model.fc = torch.nn.Linear(512,kind_of_classifier)
            elif model == "alexnet":
                self.model = models.alexnet(pretrained=param_pretrained)
                self.model.classifier[-1] = torch.nn.Linear(self.model.classifier[-1].in_features, int(kind_of_classifier))
            elif model == "squeezenet":
                self.model = models.squeezenet1_1(pretrained=param_pretrained)
                self.model.classifier[1] = torch.nn.Conv2d(self.model.classifier[1].in_features, kind_of_classifier, kernel_size=1)
                self.num_classes = kind_of_classifier
            elif model == "vgg16":
                self.model = models.vgg16(pretrained=param_pretrained)
            elif model == "densenet":
                self.model = models.densenet161(pretrained=param_pretrained)
            elif model == "inception":
                self.model = models.inception_v3(pretrained=param_pretrained)
            elif model == "googlenet":
                self.model = models.googlenet(pretrained=param_pretrained)
            elif model == "shufflenet":
                self.model = models.shufflenet_v2_x1_0(pretrained=param_pretrained)
            elif model == "mobilenet":
                self.model = models.mobilenet_v2(pretrained=param_pretrained)
            elif model == "resnext50_32x4d":
                self.model = models.resnext50_32x4d(pretrained=param_pretrained)
            elif model == "resnet34": 
                self.model = models.resnet34(pretrained=param_pretrained)
                self.model.fc = torch.nn.Linear(512, kind_of_classifier)
            elif model == "wide_resnet50_2":
                self.model = models.wide_resnet50_2(pretrained=param_pretrained)
            elif model == "mnasnet":
                self.model = models.mnasnet1_0(pretrained=param_pretrained)
            interval = rospy.get_time() - start_time
            rospy.loginfo("Done with loading modle! Use {:.2f} seconds.".format(interval))
            rospy.loginfo("There are [{}] objects you want to recognize.".format(kind_of_classifier))
        else:
            rospy.loginfo("Your classifier is wrong. Please check out image label!")

    def cuda(self,use=False):
        if use == True:
            rospy.loginfo("Using cuda! Need some time to start...")
            self.device = torch.device('cuda')
            start_time = rospy.get_time()
            self.model = self.model.to(self.device)
            interval = rospy.get_time() - start_time
            rospy.loginfo("Done with starting! Can use cuda now! Use {:.2f} seconds to start.".format(interval)) 
        else:
            rospy.loginfo("Do not use cuda!")
    
    def train(self,epochs=30, best_model_path="best_model", learning_rate=0.001, momentum=0.9):
        self.NUM_EPOCHS = epochs
        self.BEST_MODEL_PATH = rospkg.RosPack().get_path(self.package) + '/model/' + best_model_path + '.pth'
        self.best_accuracy = 0.0
        optimizer = optim.SGD(self.model.parameters(), lr=learning_rate, momentum=momentum)
        start_time = rospy.get_time()
        rospy.loginfo("Start training! Don't stop this process... ")
        for epoch in range(self.NUM_EPOCHS):
            epoch_start = rospy.get_time()
            for images, labels in iter(self.train_loader):
                images = images.to(self.device)
                labels = labels.to(self.device)
                optimizer.zero_grad()
                outputs = self.model(images)
                loss = function.cross_entropy(outputs, labels)
                loss.backward()
                optimizer.step()
                
            test_error_count = 0.0
            for images, labels in iter(self.test_loader):
                images = images.to(self.device)
                labels = labels.to(self.device)
                outputs = self.model(images)
                test_error_count += float(torch.sum(torch.abs(labels - outputs.argmax(1))))

                
            test_accuracy = 1.0 - float(test_error_count) / float(len(self.test_dataset))
            if test_accuracy > self.best_accuracy:
                torch.save(self.model.state_dict(),self.BEST_MODEL_PATH)
                self.best_accuracy = test_accuracy
            epoch_interval = rospy.get_time() - epoch_start
            rospy.loginfo("Epoch: {}, accuracy: {}, loss: {}, time: {:.2f}.".format(epoch + 1, test_accuracy, loss, epoch_interval))
        interval = rospy.get_time() - start_time
        rospy.loginfo("Done! Use {:.2f} seconds to train model.".format(interval))
        self.recording(model_name=best_model_path, train_time=round(interval, 2), accuracy=self.best_accuracy, labels=self.yaml_dict)
        rospy.loginfo("Please check out you model and recording in [{}]".format(rospkg.RosPack().get_path(self.package) + '/model/'))

    def getFilePath(self,name ,folder="image"):
        rospack = rospkg.RosPack()
        return rospack.get_path(self.package) + "/" + folder + "/" + name   

    def read_param_from_file(self):
        fname = rospkg.RosPack().get_path(self.package) + "/param/image_label.yaml"
        folder = os.listdir(rospkg.RosPack().get_path(self.package)+"/image")
        with open(fname, 'r') as in_file:
            try:
                self.yaml_dict = yaml.load(in_file)
                for key in list(self.yaml_dict.keys()) :
                    if key not in folder :
                        rospy.loginfo("Please checkout folder [image] and label in [/param/image_label.yaml]. They are different.")
                        rospy.loginfo("train_model.py will shutdown.")
                        self.on_shutdown()
                self.kind_of_classifier = len(list(self.yaml_dict.keys()))
            except yaml.YAMLError as exc:
                print(" YAML syntax  error. File: {}".format(fname))
        if self.yaml_dict != None: 
            for label_name in self.yaml_dict:
                image_count = 0
                for dir_path, dir_names, file_names in os.walk(self.getFilePath(label_name)+ "/"):                   
                    for image in file_names:
                        if image.endswith('jpg') or image.endswith('jpeg') :
                            image_count += 1  
                self.yaml_dict[label_name] = image_count

            for keys in self.yaml_dict:
                if self.yaml_dict[keys] == 0:
                    rospy.logwarn("No image in folder [{}].".format(rospkg.RosPack().get_path(self.package) + "/" + keys))
                    rospy.logwarn("Please checkout the folder or use [rosun {} mkdir.py -rm {}] to remove folder.".format(self.package, keys))
                    self.on_shutdown()
        else:
            rospy.logwarn("Please use  [{}]  to make the folder for saving data ".format(rospkg.RosPack().get_path(self.package) + "/script/mkdir.py"))
            self.on_shutdown()
        return self.yaml_dict, self.kind_of_classifier

    def compare_model_name(self, model_name):
        fname = rospkg.RosPack().get_path(self.package) + "/model/"
        if (model_name + ".pth") in os.listdir(fname):
            time_format = '%Y-%m-%d-%H-%M-%S'
            now = datetime.datetime.now().strftime(time_format)
            name = self.train_save_model_name + "-" + str(now)
            rospy.logwarn("Model name repeat nad will be reset! Now the name is: {}".format(name))
            return name
        return self.train_save_model_name

    def recording(self, model_name, train_time, accuracy, labels):
        fname = rospkg.RosPack().get_path(self.package) + "/model/recording.yaml"
        time_format = '%Y-%m-%d-%H-%M-%S'
        now = datetime.datetime.now().strftime(time_format)
        pre_recording_yaml = {}
        model_name = model_name + ".pth"
        with open(fname, 'r') as in_file:
            try:
                pre_recording_yaml = yaml.load(in_file)
            except yaml.YAMLError as exc:
                print(" YAML syntax  error. File: {}".format(fname))
        recording_now = {
            model_name: {
                "build_time": now,
                "labels"    : labels,
                "loader"    : {
                    "batch_size": self.loader_batch_size, 
                    "shuffle": self.loader_shuffle, 
                    "num_workers": self.loader_num_workers,             
                },
                "train"     : {
                    "use_cuda": self.use_cuda,
                    "model": self.param_model,
                    "epochs": self.train_epochs,
                    "learning_rate": self.train_lr,
                    "momentum": self.train_momentum,
                    "train_time": train_time,
                    "accuracy"  : accuracy,
                },
            }
        }
        with open(fname, 'w') as outfile:
            if pre_recording_yaml != None:
                outfile.write(yaml.dump(pre_recording_yaml, default_flow_style=False))
                outfile.write("\n")
            outfile.write(yaml.dump(recording_now, default_flow_style=False))


    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def on_shutdown(self): 
        rospy.loginfo("{} Close.".format(self.node_name))
        rospy.loginfo("{} shutdown.".format(self.node_name))
        #rospy.logwarn("Now you can press [ctrl] + [c] ro close the launch file.")
        rospy.sleep(1)
        rospy.is_shutdown=True
        try:
            sys.exit(0)
        except:
            rospy.loginfo("Now you can press [ctrl] + [c] to shutdwon the lauch file.")

if __name__ == "__main__" :
    rospy.init_node("train_model",anonymous=False)
    train_model_node = Train_Model_Node()
    rospy.on_shutdown(train_model_node.on_shutdown)   
    #rospy.spin()
