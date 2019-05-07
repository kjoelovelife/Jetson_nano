root=local

install:
	sudo apt-get install -y python-virtualenv mplayer mencoder
	sudo add-apt-repository ppa:mc3man/trusty-media
	sudo apt-get update 
	sudo apt-get install ffmpeg gstreamer0.10-ffmpeg -y

clean:
	rm -rf $(root) tmp
	
environment:
	bash -c "source ~/duckietown/environment.sh && virtualenv --system-site-packages $(root)"	
	bash -c "virtualenv --system-site-packages $(root)"	
	bash -c "source environment.sh && easy_install pip" 
	bash -c "source environment.sh && pip install SystemCmd==1.2 ros_node_utils==1.0 ConfTools==1.8 QuickApp==1.2.2 Procgraph==1.10"	

environment2:
	pip install SystemCmd==1.2 ros_node_utils==1.0 ConfTools==1.8 QuickApp==1.2.2 Procgraph==1.10

videos-clean:
	rm -rf out-make_videos.py
	
videos:
	bash -c "source environment.sh && ./make_videos.py --dir ~/duckietown-data/logs/ -c 'clean; rparmake n=2'"

videos-console:
	# bash -c "source environment.sh && ./make_videos.py --dir ~/duckietown-data/logs/ --console"
	./make_videos.py --dir ~/duckietown-data/logs/ --console

#bash -c "source environment.sh && cd procgraph && python setup.py develop"
# bash -c "source environment.sh && pip install https://github.com/AndreaCensi/procgraph/archive/env_fault.zip"
# git clone -b duckietown2016 git@github.com:AndreaCensi/procgraph.git
# git clone -b duckietown2016 git@github.com:AndreaCensi/procgraph.git
# git clone -b duckietown2016 git@github.com:AndreaCensi/conf_tools.git
# log=~/duckietown-data/logs/Phase_2/F23/Meeting_1_Logs/allblinking_test1/20160312-allblinking_test1-argo.bag
# go1:
# 	./go1.sh $(log) argo  
