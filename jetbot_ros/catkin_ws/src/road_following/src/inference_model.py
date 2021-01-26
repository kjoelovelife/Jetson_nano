#!/usr/bin/env python
import os, sys, argparse, errno, yaml, time, datetime
import rospy, rospkg
import torch, torchvision, cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from img_recognition.msg import Inference
from jetcam_ros.utils import bgr8_to_jpeg

class Inference_Model_Node(object):
    def __init__(self):
        self.package = "img_recognition"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        self.start = rospy.wait_for_message("/" + self.veh_name +"/jetson_camera/image/raw", Image)
        rospy.loginfo("{}  Initializing inference_model.py......".format(self.node_name))

        # set/get ros param
        self.model_name  = self.setup_parameter("~model_pth","best.pth")
        self.use_cuda  = self.setup_parameter("~use_cuda",True)

        # read information
        self.recording = self.read_param_from_file(file_name="recording.yaml", file_folder="model")

        # check model
        if self.check_model_exist(self.model_name):
            #configure model
            self.labels = sorted(list(self.recording[self.model_name]["labels"].keys()))
            self.kind_of_classifier = len(self.labels)
            self.model_struct = self.recording[self.model_name]["train"]["model"]
            self.model = self.load_model(model=self.model_struct, param_pretrained=False, kind_of_classifier=self.kind_of_classifier) # configure: self.model 
            self.cuda(use=self.use_cuda) # configure: self.device, self.model

            # configure parameter with processing image
            self.process_img_mean  = 255.0 * np.array([0.485, 0.456, 0.406])
            self.process_img_stdev = 255.0 * np.array([0.229, 0.224, 0.225])
            self.process_img_normalize = torchvision.transforms.Normalize(self.process_img_mean, self.process_img_stdev)

            # CV_bridge
            self.bridge = CvBridge()
     
            # configure subscriber
            self.first_sub = True
            self.sub_msg = rospy.Subscriber("~image/raw", Image, self.convert_image_to_cv2,queue_size=1)

            # configure Publisher
            self.pub_msg = rospy.Publisher("~inference", Inference, queue_size=1)

    def check_model_exist(self, name):
        if not name in self.recording.keys():
            rospy.logwarn("[{}] does not exist in folder [model]. Please check your model_pth in [{}].".format(name,self.getFilePath(name="inference_model.yaml", folder="param") ))
            rospy.logwarn("There are model_pth you can use below:")
            rospy.logwarn(list(self.recording.keys()))
            return False
        return True

    def load_model(self, model="alexnet", param_pretrained=False, kind_of_classifier=2):
        # reference : https://pytorch.org/docs/stable/torchvision/models.html
        model_list = [
                      "resnet18", "alexnet", "squeezenet", "vgg16", 
                      "densenet", "inception", "googlenet", "shufflenet", 
                      "mobilenet", "resnet34", "wide_resnet50_2", "mnasnet" 
                     ]

        if model in model_list:
            rospy.logwarn("[{0}]] use [{1}]. Need some time to load model [{0}]...".format(self.model_name, model))
            start_time = rospy.get_time()
            if model == "resnet18":
                self.model = torchvision.models.resnet18(pretrained=param_pretrained)
                self.model.fc = torch.nn.Linear(512,kind_of_classifier)
            elif model == "alexnet":
                self.model = torchvision.models.alexnet(pretrained=param_pretrained)
                self.model.classifier[-1] = torch.nn.Linear(self.model.classifier[-1].in_features, int(kind_of_classifier))
            elif model == "squeezenet":
                self.model = torchvision.models.squeezenet1_1(pretrained=param_pretrained)
                self.model.classifier[1] = torch.nn.Conv2d(self.model.classifier[1].in_features, kind_of_classifier, kernel_size=1)
                self.num_classes = kind_of_classifier
            elif model == "vgg16":
                self.model = torchvision.models.vgg16(pretrained=param_pretrained)
            elif model == "densenet":
                self.model = torchvision.models.densenet161(pretrained=param_pretrained)
            elif model == "inception":
                self.model = torchvision.models.inception_v3(pretrained=param_pretrained)
            elif model == "googlenet":
                self.model = torchvision.models.googlenet(pretrained=param_pretrained)
            elif model == "shufflenet":
                self.model = torchvision.models.shufflenet_v2_x1_0(pretrained=param_pretrained)
            elif model == "mobilenet":
                self.model = torchvision.models.mobilenet_v2(pretrained=param_pretrained)
            elif model == "resnext50_32x4d":
                self.model = torchvision.models.resnext50_32x4d(pretrained=param_pretrained)
            elif model == "resnet34": 
                self.model = torchvision.models.resnet34(pretrained=param_pretrained)
                self.model.fc = torch.nn.Linear(512, kind_of_classifier)
            elif model == "wide_resnet50_2":
                self.model = torchvision.models.wide_resnet50_2(pretrained=param_pretrained)
            elif model == "mnasnet":
                self.model = torchvision.models.mnasnet1_0(pretrained=param_pretrained)
            model_pth = self.getFilePath(name=self.model_name, folder="model")
            self.model.load_state_dict(torch.load(model_pth))
            interval = rospy.get_time() - start_time
            rospy.loginfo("Done with loading model! Use {:.2f} seconds.".format(interval))
            rospy.loginfo("There are {} objects you want to recognize.".format(kind_of_classifier))
            return self.model
        else:
            rospy.loginfo("Your classifier is wrong. Please check out model struct!")
            self.on_shutdown()

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

    def getFilePath(self,name ,folder="image"):
        rospack = rospkg.RosPack()
        return rospack.get_path(self.package) + "/" + folder + "/" + name  


    def read_param_from_file(self, file_name, file_folder):
        fname = self.getFilePath(name=file_name,folder=file_folder)
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                print(" YAML syntax  error. File: {}".format(fname))
        return yaml_dict

    def convert_image_to_cv2(self,img_msg):
        try:
            # Convert your ROS Image ssage to opencv2
            cv2_img = cv2.resize(self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8"), (224, 224))
            #jpeg_img = cv2.resize(bgr8_to_jpeg(cv2_img), (224, 224))
            self.inference(img=cv2_img, labels=self.labels)
        except CvBridgeError as e:
            print(e)

    def preprocess(self, camera_value): 
        img = camera_value
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = img.transpose((2, 0, 1))
        img = torch.from_numpy(img).float()
        img = self.process_img_normalize(img)
        img = img.to(self.device)
        img = img[None, ...]
        return img   

    def inference(self, img, labels):
        start_time = rospy.get_time()
        if self.first_sub == True:
            rospy.loginfo("Deploy model to gpu. Please wait...")
        img = self.preprocess(img)
        predict = self.model(img)
        if self.first_sub == True:
            interval = rospy.get_time() - start_time
            self.inference_information(interval)
            self.first_sub = False
        # we apply the 'softmax' function to normalize the output vector so it sums to 1 (which makes ti a probability distribution)
        predict = torch.nn.functional.softmax(predict, dim=1)
        local_confidence = {}
        pub_confidence = Inference()
        for text in self.labels:
            local_confidence[text] = float(predict.flatten()[self.labels.index(text)])
        pub_confidence.labels = local_confidence.keys()
        pub_confidence.confidence = local_confidence.values()
        self.pub_msg.publish(pub_confidence)
        time.sleep(0.001)

    def inference_information(self, interval):
        rospy.loginfo("Deployment complete! Use {:.2f} seconds.".format(interval))
        rospy.loginfo("Start to recognitize image! Theer are {} object you can recognitize: {}".format(len(self.labels), self.labels))
        rospy.loginfo("You can listen the topic to see how much the confidence about object: {}".format( self.node_name + "/inference"))
        rospy.loginfo("More information about {} :\n{}".format(self.model_name, self.recording[self.model_name]))


    def on_shutdown(self): 
        rospy.loginfo("{} Close.".format(self.node_name))
        rospy.loginfo("{} shutdown.".format(self.node_name))
        rospy.sleep(1)
        rospy.is_shutdown=True

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == "__main__" :
    rospy.init_node("inference_model", anonymous=False)
    inference_model_node = Inference_Model_Node()
    rospy.on_shutdown(inference_model_node.on_shutdown)   
    rospy.spin()
