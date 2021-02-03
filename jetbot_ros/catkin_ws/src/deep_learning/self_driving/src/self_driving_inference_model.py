#!/usr/bin/env python
import os, sys, argparse, errno, yaml, time, datetime, threading
import rospy, rospkg
import torch, torchvision, cv2, PIL.Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from img_recognition.msg import Inference as img_recognition_inference_msg
from road_following.msg import Inference as road_following_inference_msg
from jetcam_ros.utils import bgr8_to_jpeg

class Inference_Model_Node(object):
    def __init__(self):
        self.package = "self_driving"
        self.road_following = "road_following"
        self.img_recognition = "img_recognition"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        #_start = rospy.wait_for_message("/" + self.veh_name +"/jetson_camera/image/raw", Image)
        rospy.loginfo("[{}]  Initializing self_driving_inference_model.py......".format(self.node_name))

        ## set/get ros param
        self.inference_model_information = self.read_param_from_file(package=self.package, folder="param", file_name="inference_model.yaml")
        self.img_model_name = self.setup_parameter("~img_recognition_model_pth", self.inference_model_information["img_recognition_model_pth"])
        self.road_model_name  = self.setup_parameter("~road_following_model_pth", self.inference_model_information["road_following_model_pth"])
        self.use_cuda  = self.setup_parameter("~use_cuda", self.inference_model_information["use_cuda"])
        #### setup parameter with processing image
        self.road_preprocess_mean  = self.setup_parameter("~road_preprocess_mean", self.inference_model_information["road_preprocess_mean"])    
        self.road_preprocess_stdev = self.setup_parameter("~road_preprocess_stdev", self.inference_model_information["road_preprocess_stdev"])
        self.img_preprocess_mean  = self.setup_parameter("~img_preprocess_mean", self.inference_model_information["img_preprocess_mean"])    
        self.img_preprocess_stdev = self.setup_parameter("~img_preprocess_stdev", self.inference_model_information["img_preprocess_stdev"])

        ## read information
        self.img_model_information = self.read_param_from_file(package=self.img_recognition, folder="model", file_name="recording.yaml")[self.img_model_name]
        self.road_model_information = self.read_param_from_file(package=self.road_following, folder="model", file_name="recording.yaml")[self.road_model_name]

        ## set local parameter
        self.labels = sorted(list(self.img_model_information["labels"].keys()))
        self.initialization = True
        self.road_inference_stop = threading.Event()
        self.img_inference_stop = threading.Event()

        ## check model
        if self.check_model_exist(self.inference_model_information):
            self.img_model = self.load_model(self.img_model_information)
            self.road_model = self.load_model(self.road_model_information)
            self.cuda(use=self.use_cuda)  # depoly model to GPU(cuda) or not
            #### setup parameter with processing image
            self.road_preprocess_mean  = torch.Tensor(self.road_preprocess_mean).cuda().half()
            self.road_preprocess_stdev = torch.Tensor(self.road_preprocess_stdev).cuda().half()
            self.img_preprocess_mean = 255.0 * np.array(self.img_preprocess_mean)
            self.img_preprocess_stdev = 255.0 * np.array(self.img_preprocess_stdev)
            self.img_recognition_normalize = torchvision.transforms.Normalize(self.img_preprocess_mean, self.img_preprocess_stdev)

            ## setup parameter with inference  
            self.angle = 0.0
            self.angle_last = 0.0


            ## CV_bridge
            self.bridge = CvBridge()
     
            ## configure subscriber
            self.first_sub = True
            self.sub_msg = rospy.Subscriber("~image/raw", Image, self.convert_image_to_cv2,queue_size=1)

            ## configure Publisher
            self.pub_road_following_msg = rospy.Publisher("~road_following_inference", road_following_inference_msg, queue_size=1)
            self.pub_img_recognition_inference_msg = rospy.Publisher("~img_recognition_inference", img_recognition_inference_msg, queue_size=1)

        else:
            self.on_shutdown()

    def getFilePath(self, package, folder, file_name):
        rospack = rospkg.RosPack()
        return rospack.get_path(package) + "/" + folder + "/" + file_name  

    def read_param_from_file(self, package, folder, file_name):
        fname = self.getFilePath(package, folder, file_name)
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                print(" YAML syntax  error. File: {}".format(fname))
        return yaml_dict

    def check_model_exist(self, inference_model):
        road_following_model_pth = self.getFilePath(package=self.road_following, folder="model", file_name=inference_model["road_following_model_pth"])
        img_recognition_model_pth = self.getFilePath(package=self.img_recognition, folder="model", file_name=inference_model["img_recognition_model_pth"])
        if os.path.isfile(road_following_model_pth) and os.path.isfile(img_recognition_model_pth):
            rospy.loginfo("[{}] Load model \"{}\" and \"{}\"".format(self.node_name, inference_model["road_following_model_pth"], inference_model["img_recognition_model_pth"]))
            return True
        else:
            rospy.logwarn("[{}] {} and {} does not exist. Please check your model_pth in [\"package\"/model.]".format(self.node_name, inference_model["road_following_model_pth"], inference_model["img_recognition_model_pth"]))
            return False

    def load_model(self, information):
        # reference : https://pytorch.org/docs/stable/torchvision/models.html
        model_list = [
                      "resnet18", "alexnet", "squeezenet", "vgg16", 
                      "densenet", "inception", "googlenet", "shufflenet", 
                      "mobilenet", "resnet34", "wide_resnet50_2", "mnasnet" 
                     ]
        model_struct = information["train"]["model"]
        model_purpose = information["purpose"]
        param_pretrained = False
        if model_purpose == "classifier":
            kind_of_classifier = len(information["labels"])
            if model_struct in model_list:
                rospy.loginfo("[{}] Try to load classifier model".format(self.node_name))
                rospy.loginfo("[{0}] [{1}]] use [{2}]. Need some time to load model [{1}]...".format(self.node_name, self.img_model_name, model_struct))
                start_time = rospy.get_time()
                if model_struct == "resnet18":
                    model = torchvision.models.resnet18(pretrained=param_pretrained)
                    model.fc = torch.nn.Linear(512,kind_of_classifier)
                elif model_struct == "alexnet":
                    model = torchvision.models.alexnet(pretrained=param_pretrained)
                    model.classifier[-1] = torch.nn.Linear(model.classifier[-1].in_features, int(kind_of_classifier))
                elif model_struct == "squeezenet":
                    model = torchvision.models.squeezenet1_1(pretrained=param_pretrained)
                    model.classifier[1] = torch.nn.Conv2d(model.classifier[1].in_features, kind_of_classifier, kernel_size=1)
                    num_classes = kind_of_classifier
                elif model_struct == "vgg16":
                    model = torchvision.models.vgg16(pretrained=param_pretrained)
                elif model_struct == "densenet":
                    model = torchvision.models.densenet161(pretrained=param_pretrained)
                elif model_struct == "inception":
                    model = torchvision.models.inception_v3(pretrained=param_pretrained)
                elif model_struct == "googlenet":
                    model = torchvision.models.googlenet(pretrained=param_pretrained)
                elif model_struct == "shufflenet":
                    model = torchvision.models.shufflenet_v2_x1_0(pretrained=param_pretrained)
                elif model_struct == "mobilenet":
                    model = torchvision.models.mobilenet_v2(pretrained=param_pretrained)
                elif model_struct == "resnext50_32x4d":
                    model = torchvision.models.resnext50_32x4d(pretrained=param_pretrained)
                elif model_struct == "resnet34": 
                    model = torchvision.models.resnet34(pretrained=param_pretrained)
                    model.fc = torch.nn.Linear(512, kind_of_classifier)
                elif model_struct == "wide_resnet50_2":
                    model = torchvision.models.wide_resnet50_2(pretrained=param_pretrained)
                elif model_struct == "mnasnet":
                    model = torchvision.models.mnasnet1_0(pretrained=param_pretrained)
                else:
                    rospy.loginfo("[{}] Your classifier is wrong. Please check out model struct in {}!".format(self.node_name, model_path))
                    self.on_shutdown()
                model_pth = self.getFilePath(package=self.img_recognition, folder="model", file_name=self.img_model_name)
                model.load_state_dict(torch.load(model_pth))
                interval = rospy.get_time() - start_time
                rospy.loginfo("[{}] Loading \"{}\" done! Use {:.2f} seconds.".format(self.node_name, self.img_model_name, interval))
                rospy.loginfo("[{}] There are {} objects you want to recognize.".format(self.node_name, kind_of_classifier))
        elif model_purpose == "regression":
            if model_struct in model_list:
                rospy.loginfo("[{}] Try to load regression model".format(self.node_name))
                rospy.loginfo("[{0}] [{1}] use [{2}]. Need some time to load model [{1}]...".format(self.node_name, self.road_model_name, model_struct))
                start_time = rospy.get_time()
                if model_struct == "resnet18":
                    model = torchvision.models.resnet18(pretrained=param_pretrained)
                    model.fc = torch.nn.Linear(512, 2)
                elif model_struct == "alexnet":
                    model = torchvision.models.alexnet(pretrained=param_pretrained)
                    model.classifier[-1] = torch.nn.Linear(self.model.classifier[-1].in_features, int(kind_of_classifier))
                elif model_struct == "squeezenet":
                    model = torchvision.models.squeezenet1_1(pretrained=param_pretrained)
                    #model.classifier[1] = torch.nn.Conv2d(self.model.classifier[1].in_features, kind_of_classifier, kernel_size=1)
                    #num_classes = kind_of_classifier
                elif model_struct == "vgg16":
                    model = torchvision.models.vgg16(pretrained=param_pretrained)
                elif model_struct == "densenet":
                    model = torchvision.models.densenet161(pretrained=param_pretrained)
                elif model_struct == "inception":
                    model = torchvision.models.inception_v3(pretrained=param_pretrained)
                elif model_struct == "googlenet":
                    model = torchvision.models.googlenet(pretrained=param_pretrained)
                elif model_struct == "shufflenet":
                    model = torchvision.models.shufflenet_v2_x1_0(pretrained=param_pretrained)
                elif model_struct == "mobilenet":
                    model = torchvision.models.mobilenet_v2(pretrained=param_pretrained)
                elif model_struct == "resnext50_32x4d":
                    model = torchvision.models.resnext50_32x4d(pretrained=param_pretrained)
                elif model_struct == "resnet34": 
                    model = torchvision.models.resnet34(pretrained=param_pretrained)
                    model.fc = torch.nn.Linear(512, kind_of_classifier)
                elif model_struct == "wide_resnet50_2":
                    model = torchvision.models.wide_resnet50_2(pretrained=param_pretrained)
                elif model_struct == "mnasnet":
                    model = torchvision.models.mnasnet1_0(pretrained=param_pretrained)
                else:
                    rospy.loginfo("[{}] Your regression is wrong. Please check out model struct in {}!".format(self.node_name, model_path))
                    self.on_shutdown()
                model_pth = self.getFilePath(package=self.road_following, folder="model", file_name=self.road_model_name)
                model.load_state_dict(torch.load(model_pth))
                interval = rospy.get_time() - start_time
                rospy.loginfo("[{}] Loading \"{}\" done! Use {:.2f} seconds.".format(self.node_name, self.road_model_name, interval))
        else:
            rospy.loginfo("[{}] Your model struct is wrong. Please check out model struct!".format(self.node_name))
            self.on_shutdown()
        return model

    def cuda(self,use=False):
        if use == True:
            rospy.loginfo("[{}] Using cuda! Need some time to start...".format(self.node_name))
            self.device = torch.device('cuda')
            start_time = rospy.get_time()
            self.img_model = self.img_model.to(self.device)
            self.road_model = self.road_model.to(self.device)
            self.road_model = self.road_model.eval().half()
            interval = rospy.get_time() - start_time
            rospy.loginfo("[{}] Using cuda Done! Can use cuda now! Use {:.2f} seconds to start.".format(self.node_name, interval)) 
        else:
            self.device = torch.device('cpu')
            rospy.loginfo("[{}] Do not use cuda!".format(self.node_name))

    def convert_image_to_cv2(self, img_msg):
        try:
            # Convert your ROS Image sage to opencv2
            cv2_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
            self.inference(img=cv2_img)
            #road_following = threading.Thread(target=self.road_inference)
            #img_recognition = threading.Thread(target=self.img_inference)
            #jpeg_img = cv2.resize(bgr8_to_jpeg(cv2_img), (224, 224))
            #if self.initialization:
            #    road_following.start()
            #    img_recognition.start()
            #    initialization = False
            
        except CvBridgeError as e:
            print(e)

    def road_img_preprocess(self, camera_value): 
        img = PIL.Image.fromarray(cv2.cvtColor(camera_value, cv2.COLOR_BGR2RGB))
        img = torchvision.transforms.functional.to_tensor(img).to(self.device).half()
        img.sub_(self.road_preprocess_mean[:, None, None]).div_(self.road_preprocess_stdev[:, None, None])
        return img[None, ...] 
 
    def img_recognition_preprocess(self, camera_value):
        img = cv2.cvtColor(camera_value, cv2.COLOR_BGR2RGB)
        img = img.transpose((2, 0, 1))
        img = torch.from_numpy(img).float()
        img = self.img_recognition_normalize(img)
        img = img.to(self.device)
        img = img[None, ...]
        return img   

    def inference(self, img):        
        start_time = rospy.get_time()
        if self.first_sub == True:
            rospy.loginfo("[{}] Deploy model to gpu. Please wait...".format(self.node_name))
        road_img = self.road_img_preprocess(img)
        recognition_img = self.img_recognition_preprocess(img)     
        xy = self.road_model(road_img).detach().float().cpu().numpy().flatten()
        predict = self.img_model(recognition_img)
        if self.first_sub == True:
            interval = rospy.get_time() - start_time
            self.inference_information(interval)
            self.first_sub = False
        
        ## road_following
        inference_msg_road_following = road_following_inference_msg()
        x = xy[0]
        y = (0.5 - xy[1]) / 2.0
        self.angle = np.arctan2(x, y)
        self.angle_last = self.angle
        #print("angle: {}, last_angle: {}".format(self.angle, self.angle_last))
        inference_msg_road_following.angle = self.angle
        inference_msg_road_following.angle_last = self.angle_last

        ## img_recognition
        predict = torch.nn.functional.softmax(predict, dim=1)
        local_confidence = {}
        inference_msg_img_recognition = img_recognition_inference_msg()
        for text in self.labels:
            local_confidence[text] = float(predict.flatten()[self.labels.index(text)])
        inference_msg_img_recognition.labels = local_confidence.keys()
        inference_msg_img_recognition.confidence = local_confidence.values()

        ## Publisher publish msg
        self.pub_road_following_msg.publish(inference_msg_road_following)
        self.pub_img_recognition_inference_msg.publish(inference_msg_img_recognition)
        time.sleep(0.001)

    def inference_information(self, interval):
        rospy.loginfo("[{}] Deployment complete! Use {:.2f} seconds.".format(self.node_name, interval))
        rospy.loginfo("[{}] You can listen the topic to see how much the confidence: {}, {}".format(self.node_name , self.node_name + "/road_following_inference", self.node_name + "/img_recognition_inference"))
        rospy.loginfo("[{}] More information about {} :\n{}".format(self.node_name, self.img_model_name, self.img_model_information))
        rospy.loginfo("[{}] More information about {} :\n{}".format(self.node_name, self.road_model_name, self.road_model_information))


    def on_shutdown(self): 
        #self.road_inference_stop.set()
        #self.img_inference_stop.set()
        rospy.loginfo("[{}] Close.".format(self.node_name))
        rospy.loginfo("[{}] shutdown.".format(self.node_name))
        rospy.loginfo("[{}] Now you can press [ctrl] + [c] to close this launch file.".format(self.node_name))
        #self.img_recognition.set()
        rospy.sleep(1)
        rospy.is_shutdown = True
        #sys.exit()

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == "__main__" :
    rospy.init_node("self_driving_inference_model", anonymous=False)
    self_driving_inference_model = Inference_Model_Node()
    rospy.on_shutdown(self_driving_inference_model.on_shutdown)   
    rospy.spin()
