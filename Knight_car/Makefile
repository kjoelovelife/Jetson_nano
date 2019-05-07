
catkin_ws := catkin_ws
scuderia := scuderia.yaml
machines := $(catkin_ws)/src/duckietown/machines

all: $(machines)


$(machines): $(scuderia)
	python setup/create-machines-file.py $(scuderia) > $(machines)

fix-time:
	echo "Calling ntpdate to fix time"
	sudo ntpdate -u us.pool.ntp.org 

fix-time2:
	sudo ntpdate -s time.nist.gov

clean-pyc:
	find catkin_ws/src/ -name '*.pyc' | xargs rm 

catkin-clean: clean-pyc
	rm -rf $(catkin_ws)/build

build-parallel:
	catkin_make -C $(catkin_ws) --make-args "-j4"

build:
	catkin_make -C $(catkin_ws) 

# Unit tests
# Teddy: make it so "make unittests" runs all unit tests

unittests-environment: $(machines)
	bash -c "source environment.sh; source set_vehicle_name.sh; python setup/sanity_checks"

unittests:
	$(MAKE) unittests-environment
	bash -c "source environment.sh; catkin_make -C $(catkin_ws) run_tests; catkin_test_results $(catkin_ws)/build/test_results/"


unittests-anti_instagram:
	$(MAKE) unittests-environment
	bash -c "source environment.sh; rosrun anti_instagram annotation_tests.py"

# HW testing 

test-camera:
	echo "Testing Camera HW by taking a picture (smile!)."
	raspistill -t 1000 -o test-camera.jpg


test-led: 
	echo "Calibration blinking pattern"
	bash -c "source environment.sh; rosrun rgb_led blink test_all_1"

test-turn-right:
	echo "Calibrating right turn"
	bash -c "rostest indefinite_navigation calibrate_turn.test veh:=$(vehicle_name) type:=right"

test-turn-left:
	echo "Calibrating left turn"
	bash -c "rostest indefinite_navigation calibrate_turn.test veh:=$(vehicle_name) type:=left"

test-turn-forward:
	echo "Calibrating forward turn"
	bash -c "rostest indefinite_navigation calibrate_turn.test veh:=$(vehicle_name) type:=forward"


# Basic demos

vehicle_name=$(shell hostname)

demo-joystick: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh;  roslaunch duckietown joystick.launch veh:=$(vehicle_name)"

demo-joystick-high-speed: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh;  roslaunch duckietown joystick.launch veh:=$(vehicle_name) joy_mapper_param_file_name:=high_speed "

demo-joystick-camera: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh;  roslaunch duckietown joystick_camera.launch veh:=$(vehicle_name)"

demo-joystick-camera-high-speed: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh;  roslaunch duckietown joystick_camera.launch veh:=$(vehicle_name) joy_mapper_param_file_name:=high_speed "

demo-line_detector: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; roslaunch duckietown line_detector.launch veh:=$(vehicle_name)"



demo-joystick-perception: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos master.launch fsm_file_name:=joystick"

demo-lane_following-%: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos lane_following.launch line_detector_param_file_name:=$*"

demo-led-fancy1: unittests-environment
	bash -c "source environment.sh; rosrun rgb_led fancy1"

demo-led-fancy2: unittests-environment
	bash -c "source environment.sh; rosrun rgb_led fancy2"

demo-led-blink-%: unittests-environment
	bash -c "source environment.sh; rosrun rgb_led blink $*"

demo-line_detector-default:     demo-line_detector-quiet-default
demo-line_detector-guy:         demo-line_detector-quiet-guy
demo-line_detector-universal:   demo-line_detector-quiet-universal
demo-line_detector-default_ld2: demo-line_detector-quiet-default_ld2

demo-line_detector-quiet-%: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; roslaunch duckietown line_detector.launch veh:=$(vehicle_name) line_detector_param_file_name:=$* verbose:=false"

# traffic lights
traffic-light:
	bash -c "source environment.sh; source set_ros_master.sh; roslaunch traffic_light traffic_light_node.launch veh:=$(vehicle_name)"

#traffic-light-%:
#	bash -c "source environment.sh; source set_ros_master.sh $*; roslaunch traffic_light traffic_light_node.launch veh:=$*"

# ==========
# openhouse demos
 
openhouse-dp1: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos parallel_autonomy.launch"

openhouse-dp1-%: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos parallel_autonomy.launch line_detector_param_file_name:=$*"

openhouse-dp1-veh: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos parallel_autonomy.launch vehicle_avoidance:=true"

openhouse-dp2: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos obstacle_vehicle_avoid.launch"

openhouse-dp2-%: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos obstacle_vehicle_avoid.launch line_detector_param_file_name:=$*"

openhouse-dp2-vehicle: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos vehicle_avoid.launch"

openhouse-dp2-obstacle: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos obstacle_avoid.launch"

openhouse-dp2-vehicle-no-wheels: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos vehicle_avoid_nowheels.launch"

openhouse-dp2-obstacle-no-wheels: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos obstacle_avoid_nowheels.launch"

openhouse-dp6a-generate-map-%: unittests-environment
	bash -c 'echo -n "Enter full path to the tags .csv file: "; read tag_map; echo -n "Enter full path to the tiles .csv file: "; read tile_map;\
	source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; \
	roslaunch duckietown_description csv2xacro_node.launch tag_map_csv:=$$tag_map  tile_map_csv:=$$tile_map map_name:=$*'

openhouse-dp6a-%: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; \
	roslaunch duckietown_demos localization.launch map_name:=$*"

openhouse-dp6a_laptop-%: unittests-environment
	bash -c "source set_ros_master.sh $*; roslaunch duckietown_demos localization_frontend.launch"

openhouse-dp6b: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos mission_planning.launch"

openhouse-dp6b-laptop-%: unittests-environment
	bash -c "source set_ros_master.sh $*; rqt --force-discover"

openhouse-dp6: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos localization_navigation.launch"

openhouse-dp6-laptop-%: unittests-environment
	bash -c "source set_ros_master.sh $*; rqt --force-discover"

openhouse-dp3: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos indefinite_navigation.launch"

openhouse-dp3-%: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos indefinite_navigation.launch  line_detector_param_file_name:=$*"

openhouse-dp5: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos stop_sign_coordination.launch"

openhouse-dp4: unittests-environment
	bash -c "source environment.sh; source set_ros_master.sh; source set_vehicle_name.sh; roslaunch duckietown_demos traffic_light_coordination.launch"


