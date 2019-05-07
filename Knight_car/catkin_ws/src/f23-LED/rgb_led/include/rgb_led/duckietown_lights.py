import random
# __all__ = [
# 	'DuckietownLights',
# 	'cycle_LEDs_named',
# 	'cycle_LEDs',
# 	'cycle_LEDs_fancy1',
# ]

# names for lights
TOP = 'top'
BACK_LEFT = 'bl'
BACK_RIGHT = 'br'
FRONT_LEFT = 'fl'
FRONT_RIGHT = 'fr'

class DuckietownLights():

	# Configuration (calibration) - needs to be in a yaml.file
	name2port = {
		TOP: 2,
		BACK_LEFT: 1,
		BACK_RIGHT: 3,
		FRONT_LEFT: 0,
		FRONT_RIGHT: 4,
	}

	# name -> sequence
	sequences = {}

	car_all_lights = [TOP, BACK_LEFT, BACK_RIGHT, FRONT_LEFT, FRONT_RIGHT]

def add_pattern(name, pattern):
	DuckietownLights.sequences[name] = pattern 

def create_patterns():

	s = 1.0
	RED = [s, 0, 0]
	GREEN = [0, s, 0]
	BLUE = [0, 0, 1]
	YELLOW = [0, s, s]
	WHITE = [s, s, s]
	GREEN2 = [0, 0.3, 0]

	OFF = [0, 0, 0]

	s = 0.5

	s = 0.8
	WHITE2 = [s, s, s]

	add_pattern('blinking1', [
		(0.5, {TOP: GREEN2, BACK_LEFT:RED, BACK_RIGHT:RED, FRONT_LEFT:WHITE2,FRONT_RIGHT:WHITE2}),	
		(0.5, {TOP: OFF, BACK_LEFT:OFF, BACK_RIGHT:OFF, FRONT_LEFT:WHITE2,FRONT_RIGHT:WHITE2}),
	])

	add_pattern('blinking2',[
		(0.25, {TOP: GREEN2, BACK_LEFT:RED, BACK_RIGHT:RED, FRONT_LEFT:WHITE2,FRONT_RIGHT:WHITE2}),	
		(0.25, {TOP: OFF, BACK_LEFT:OFF, BACK_RIGHT:OFF, FRONT_LEFT:WHITE2,FRONT_RIGHT:WHITE2}),
	])

	add_pattern('blinking3',[
		(0.25, {TOP: GREEN, BACK_LEFT:GREEN2, BACK_RIGHT:[0,1,1], FRONT_LEFT:YELLOW,FRONT_RIGHT:WHITE}),	
		(0.25, {TOP: RED, BACK_LEFT:GREEN2, BACK_RIGHT:RED, FRONT_LEFT:RED,FRONT_RIGHT:RED}),
	])
	
	add_pattern('mar12special',[
		(0.50, {TOP: BLUE, BACK_LEFT: WHITE, BACK_RIGHT:WHITE, FRONT_LEFT:WHITE,FRONT_RIGHT:WHITE}),	
		(0.50, {TOP: OFF, BACK_LEFT: WHITE, BACK_RIGHT:WHITE, FRONT_LEFT:WHITE,FRONT_RIGHT:WHITE}),	
	])

	add_pattern('trafficlight4way',
		[
			{TOP: GREEN, BACK_LEFT:RED, BACK_RIGHT:RED, FRONT_LEFT:RED, FRONT_RIGHT:RED},
			{TOP: RED, BACK_LEFT:RED, BACK_RIGHT:GREEN, FRONT_LEFT:RED, FRONT_RIGHT:RED},
			{TOP: RED, BACK_LEFT:RED, BACK_RIGHT:RED, FRONT_LEFT:GREEN, FRONT_RIGHT:RED},
			{TOP: RED, BACK_LEFT:RED, BACK_RIGHT:RED, FRONT_LEFT: RED, FRONT_RIGHT:GREEN},
		]
	)

	conf_all_off = {
		TOP: OFF, 
		BACK_LEFT:OFF, 
		BACK_RIGHT:OFF, 
		FRONT_LEFT:OFF,
		FRONT_RIGHT:OFF,
	}

	conf_static_car = {
		TOP: OFF, 
		BACK_LEFT:RED, 
		BACK_RIGHT:RED, 
		FRONT_LEFT:WHITE,
		FRONT_RIGHT:WHITE,
	}
	
	def conf_all_on(color):
		x = dict(**conf_all_off)
		for k in x:
			x[k] = color
		return x

	def blink_one(which, color, period, others=conf_all_off):
		r = dict(**others)
		r[which] = color
		return [
			(period/2, others),
			(period/2, r),
		]
	
	def blink_all(color, period):
		return [
			(period/2, conf_all_off),
			(period/2, conf_all_on(color)),
		]
	
	def cat(*patterns):
		p = []
		for ps in patterns:
			for a in ps:
				#print(a)
				p.append(a)

		return p

	colors = {
		'white': [1, 1, 1],
		'red': [1, 0, 0],
		'green': [0, 1, 0],
		'blue': [0,0,1],
		'yellow': [1,1,0],
		'orange': [1,0.4,0],
	}
		
	frequencies = [1, 1.1, 1.2, 1.3, 1.4, 1.5, 
					1.6, 1.7, 1.8, 1.9, 2, 2.1,2.2,2.3,2.4,
					 2.5, 3, 3.5, 4.5, 5, 6, 7, 8, 9,
					10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
	
	for name in DuckietownLights.car_all_lights:
		for color, rgb in colors.items():
			for freq in frequencies:
				comb = '%s-%s-%1.1f' % (name, color, freq)
				add_pattern(comb, blink_one(name, rgb, 1.0/freq))

	for color, rgb in colors.items():
		for freq in frequencies:
			comb = 'wr-%s-%1.1f' % (color, freq)
			add_pattern(comb, blink_one(TOP, rgb, 1.0/freq,
				others=conf_static_car))

	for color, rgb in colors.items():
		for freq in frequencies:
			comb = 'all-%s-%1.1f' % (color, freq)
			add_pattern(comb, blink_all(rgb, 1.0/freq))
 
	static = {TOP: WHITE, FRONT_LEFT:RED, FRONT_RIGHT:GREEN,
	BACK_LEFT:colors['orange'], BACK_RIGHT:colors['blue']}

	# Do not change this one; the test in "test1" refers to it.
	add_pattern('test_all_1', cat(
		blink_all(RED, 1.0),
		blink_all(GREEN, 1.0/2)*2,
		blink_all(BLUE, 1.0/3)*3,
		[(5.0, static)],
		))

create_patterns()

def cycle_LEDs_named(sequence_name):
	if not sequence_name in DuckietownLights.sequences:
		msg = 'Could not find the sequence %r.' % sequence_name
		msg += '\n\nThese are some I know:'
		avail = list(DuckietownLights.sequences)
		random.shuffle(avail)
		N = 20
		if len(avail) > N:
			avail = avail[:N]
			
		msg += ' ' + ", ".join(avail) + '.'
		raise ValueError(msg)
	sequence = DuckietownLights.sequences[sequence_name]
	cycle_LEDs(sequence)

def get_current_step(t, t0, sequence):
	""" returns i, (period, color) """
	period = sum(s[0] for s in sequence)

	tau = t - t0
	while tau - period > 0:
		tau -= period
	i = 0
	while True: 
		current = sequence[i][0]
		if tau < current:
			break
		else:
			i += 1
			tau -= current

	return i, sequence[i]

def cycle_LEDs(sequence):
	import time
	from time import sleep
	from .rgb_led import RGB_LED
	led = RGB_LED()

	t0 = time.time()
	# last = all_off
	last = {
		TOP: None, 
		BACK_LEFT:None, 
		BACK_RIGHT:None, 
		FRONT_LEFT:None,
		FRONT_RIGHT:None,
	}
	while True:
		t = time.time()
		_, (_, config) = get_current_step(t, t0, sequence)

		for name, col in config.items():
			k = DuckietownLights.name2port[name]
			
			if last[name] != col:
				led.setRGB(k, col)
				print(name, k, col)
				last[name] = col

		sleep(0.01)


