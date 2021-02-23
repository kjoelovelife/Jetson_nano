#!/usr/bin/env python

import os, sys, argparse, errno, yaml, time, datetime, glob, PIL.Image
import cv2, numpy 
import rospy, rospkg, threading
import torch, torchvision
import torch.optim as optim
import torch.nn.functional as function
import torchvision.datasets as datasets
import torchvision.models as models
import torchvision.transforms as transforms

class XYDataset(torch.utils.data.Dataset):

    def __init__(self, directory, random_hflips=False):
        self.directory = directory
        self.random_hflips = random_hflips
        self.image_paths = glob.glob(os.path.join(self.directory, '*.jpg'))
        self.color_jitter = transforms.ColorJitter(0.3, 0.3, 0.3, 0.3)
        self.transforms_functional_resize = (224, 224)

    def get_x(self, path):
        return (float(int(path[3:6])) - 50.0 ) / 50.0

    def get_y(self, path):
        return (float(int(path[7:10])) - 50.0 ) / 50.0

    def __len__(self):
        return len(self.image_paths)

    def __getitem__(self, idx):
        image_path = self.image_paths[idx]
        image = PIL.Image.open(image_path)
        x = float(self.get_x(os.path.basename(image_path)))
        y = float(self.get_y(os.path.basename(image_path)))
        if float(numpy.random.rand(1)) > 0.5:
            image = transforms.functional.hflip(image)
            x = -x
        image = self.color_jitter(image)
        image = transforms.functional.resize(image, self.transforms_functional_resize)
        image = transforms.functional.to_tensor(image)
        image = image.numpy()[::-1].copy()
        image = torch.from_numpy(image)
        image = transforms.functional.normalize(image, [0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        return image, torch.tensor([x, y]).float()

class Train_Model_Node(object):
 
    def __init__(self):
        self.package = "road_following"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        rospy.loginfo("[{}]  Initializing train_model.py......".format(self.node_name))

        ## Set/get ros param
        self.folder_name = self.setup_parameter("~dataset", "dataset_xy")
        self.test_percent = self.setup_parameter("~test_percent", 0.1)
        self.model_name = self.setup_parameter("~model", "resnet18")
        self.pre_train = self.setup_parameter("~pre_train", True)
        self.use_cuda = self.setup_parameter("~use_cuda", True)
        self.loader_batch_size  = self.setup_parameter("~loader/batch_size", 16)
        self.loader_shuffle = self.setup_parameter("~loader/shuffle", True)
        self.loader_num_workers = self.setup_parameter("~loader/num_workers", 4)
        self.train_save_model_name = self.setup_parameter("~train/save_model_name", "best_steering_model_xy")
        self.train_epochs = self.setup_parameter("~train/epochs", 30)
        self.train_lr = self.setup_parameter("~train/learning_rate", 0.001)
        self.train_betas = self.setup_parameter("~train/betas", (0.9, 0.999))
        self.train_eps = self.setup_parameter("~train/eps", 1e-8) 
        self.train_weight_decay = self.setup_parameter("~train/weight_decay", 0)
        
        ## Set local param
        self.image_folder = self.check_folder(self.folder_name)
        self.dataset = XYDataset(self.image_folder, random_hflips=False)
        self.train_save_model_name = self.compare_model_name(self.train_save_model_name) 

        ## Split dataset into train and test sets and create data loaders to load data in batches
        self.deal_with_dataset()

        ## Define Neural Network Model
        _model = self.neural_network(self.model_name, self.pre_train)
        _cuda = self.cuda(self.use_cuda)
        ## train model
        _train = self.train(epochs=self.train_epochs, best_model_name=self.train_save_model_name, learning_rate=self.train_lr, betas=self.train_betas, eps=self.train_eps, weight_decay=self.train_weight_decay)

    def check_folder(self, name):
        rospack = rospkg.RosPack()
        folder = rospack.get_path(self.package) + "/image/" + name
        if not os.path.isdir(folder):
            time_format = "%Y_%m_%d_%H_%M_%S"
            now = datetime.datetime.now().strftime(time_format)
            rospy.logwarn("[{}] Can't find folder: {}.\nWill make [dataset_xy_{}] in {}.".format(self.node_name, folder, now,rospack.get_path(self.package + "/image")))
            self.on_shutdown()
        return folder

    def deal_with_dataset(self):
        num_test = int(self.test_percent * len(self.dataset))
        self.train_dataset, self.test_dataset = torch.utils.data.random_split(self.dataset, [len(self.dataset) - num_test, num_test])
        self.train_loader = torch.utils.data.DataLoader(
            self.train_dataset,
            batch_size=self.loader_batch_size,
            shuffle=self.loader_shuffle,
            num_workers=self.loader_num_workers,
        )
        self.test_loader = torch.utils.data.DataLoader(
            self.test_dataset,
            batch_size=self.loader_batch_size,
            shuffle=self.loader_shuffle,
            num_workers=self.loader_num_workers,
        )

    def neural_network(self, model="resnet18", pre_trained=True):
        # reference : https://pytorch.org/docs/stable/torchvision/models.html
        model_list = [
                      "resnet18", "alexnet", "squeezenet", "vgg16", 
                      "densenet", "inception", "googlenet", "shufflenet", 
                      "mobilenet", "resnet34", "wide_resnet50_2", "mnasnet" 
                     ]
        if model in model_list:
            rospy.loginfo("[{}] You use model [{}]. Need some time to load model...".format(self.node_name, model))
            start_time = rospy.get_time()
            if model == "resnet18":
                self.model = models.resnet18(pretrained=pre_trained)
                self.model.fc = torch.nn.Linear(512, 2)
            elif model == "alexnet":
                self.model = models.alexnet(pretrained=pre_trained)
            elif model == "squeezenet":
                self.model = models.squeezenet1_1(pretrained=pre_trained)
            elif model == "vgg16":
                self.model = models.vgg16(pretrained=pre_trained)
            elif model == "densenet":
                self.model = models.densenet161(pretrained=pre_trained)
            elif model == "inception":
                self.model = models.inception_v3(pretrained=pre_trained)
            elif model == "googlenet":
                self.model = models.googlenet(pretrained=pre_trained)
            elif model == "shufflenet":
                self.model = models.shufflenet_v2_x1_0(pretrained=pre_trained)
            elif model == "mobilenet":
                self.model = models.mobilenet_v2(pretrained=pre_trained)
            elif model == "resnext50_32x4d":
                self.model = models.resnext50_32x4d(pretrained=pre_trained)
            elif model == "resnet34": 
                self.model = models.resnet34(pretrained=pre_trained)
            elif model == "wide_resnet50_2":
                self.model = models.wide_resnet50_2(pretrained=pre_trained)
            elif model == "mnasnet":
                self.model = models.mnasnet1_0(pretrained=pre_trained)
            interval = rospy.get_time() - start_time
            rospy.loginfo("[{}] loading modle done! Use {:.2f} seconds.".format(self.node_name, interval))
        else:
            rospy.loginfo("[{}] We don't have the model in this rpoject. Please choose one of below: ".format(self.node_name))
            rospy.loginfo("[{}] {}".format(self.node_name, model_list))
            self.on_shutdown()

    def cuda(self, use=False):
        if use == True:
            rospy.loginfo("[{}] Using cuda! Need some time to start...".format(self.node_name))
            self.device = torch.device('cuda')
            start_time = rospy.get_time()
            self.model = self.model.to(self.device)
            interval = rospy.get_time() - start_time
            rospy.loginfo("[{}] Done with starting! Can use cuda now! Use {:.2f} seconds to start.".format(self.node_name, interval)) 
        else:
            self.device = torch.device('cpu')
            rospy.loginfo("[{}] Do not use cuda!".format(self.node_name))
    
    def train(self, epochs=70, best_model_name="best_model", learning_rate=1e-3, betas=(0.9, 0.999), eps=1e-08, weight_decay=0):
        NUM_EPOCHS = epochs
        BEST_MODEL_PATH = rospkg.RosPack().get_path(self.package) + '/model/' + best_model_name + '.pth'
        best_loss = 1e9
        optimizer = optim.Adam(self.model.parameters(), lr=learning_rate, betas=betas, eps=eps, weight_decay=weight_decay)
        start_time = rospy.get_time()
        rospy.loginfo("[{}] Start training! Don't stop this process... ".format(self.node_name))
        for epoch in range(NUM_EPOCHS):
            epoch_start = rospy.get_time()
            self.model.train()
            train_loss = 0.0
            for images, labels in iter(self.train_loader):
                if self.use_cuda == True:
                    images = images.to(self.device)
                    labels = labels.to(self.device)
                optimizer.zero_grad()
                outputs = self.model(images)
                loss = function.mse_loss(outputs, labels)
                train_loss += loss
                loss.backward()
                optimizer.step()
            train_loss /= len(self.train_loader)

            self.model.eval()
            test_loss = 0.0
            for images, labels in iter(self.test_loader):
                if self.use_cuda == True:
                    images = images.to(self.device)
                    labels = labels.to(self.device)
                outputs = self.model(images)
                loss = function.mse_loss(outputs, labels)
                test_loss += loss
            test_loss /= len(self.test_loader)
    
            if test_loss < best_loss:
                torch.save(self.model.state_dict(), BEST_MODEL_PATH)
                best_loss = test_loss
            epoch_interval = rospy.get_time() - epoch_start
            rospy.loginfo("[{}] Epoch: {}, train_loss: {}, test_loss: {}, time: {:.2f}.".format(self.node_name, epoch + 1, train_loss, test_loss, epoch_interval))
        interval = rospy.get_time() - start_time
        rospy.loginfo("[{}] Done! Use {:.2f} seconds to train model.".format(self.node_name, interval))
        self.recording(model_name=best_model_name, train_time=round(interval, 2), loss=(float(train_loss), float(test_loss)))
        rospy.loginfo("[{}] Please check out you model and recording in [{}]".format(self.node_name, rospkg.RosPack().get_path(self.package) + '/model/'))

    def compare_model_name(self, model_name):
        fname = rospkg.RosPack().get_path(self.package) + "/model/"
        if (model_name + ".pth") in os.listdir(fname):
            time_format = '%Y_%m_%d_%H_%M_%S'
            now = datetime.datetime.now().strftime(time_format)
            name = self.train_save_model_name + "_" + str(now)
            rospy.logwarn("[{}] Model name repeat nad will be reset! Now the name is: {}".format(self.node_name, name))
            return name
        return model_name

    def recording(self, model_name, train_time, loss):
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
                "loader"    : {
                    "batch_size": self.loader_batch_size, 
                    "shuffle": self.loader_shuffle, 
                    "num_workers": self.loader_num_workers,             
                },
                "train"     : {
                    "use_cuda": self.use_cuda,
                    "model": self.model_name,
                    "epochs": self.train_epochs,
                    "train_time": train_time,
                    "test_loss"  : loss[0],
                    "train_loss" : loss[1],
                    "optimizer": {
                        "name": "Adam",
                        "learning_rate": self.train_lr,
                        "betas": list(self.train_betas),
                        "eps": self.train_eps,
                        "weight_decay": self.train_weight_decay,
                    },
                
                },
                "purpose": "regression",
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
        rospy.loginfo("[{}] Close.".format(self.node_name))
        rospy.loginfo("[{}] shutdown.".format(self.node_name))
        rospy.logwarn("[{}] Now you can press [ctrl] + [c] ro close the launch file.".format(self.node_name))
        rospy.sleep(1)
        rospy.is_shutdown=True
        #try:
        #    sys.exit(0)
        #except:
        #    rospy.logwarn("Now you can press [ctrl] + [c] to shutdwon the lauch file.")

if __name__ == "__main__" :
    rospy.init_node("train_model",anonymous=False)
    train_model_node = Train_Model_Node()
    rospy.on_shutdown(train_model_node.on_shutdown)   
    #rospy.spin()
