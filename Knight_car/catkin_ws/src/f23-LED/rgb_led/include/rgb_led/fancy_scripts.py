def turn_off_LEDs(speed=5):
	from .duckietown_lights import DuckietownLights, TOP, BACK_LEFT, BACK_RIGHT, FRONT_LEFT, FRONT_RIGHT
	import time
	from time import sleep
	from .rgb_led import RGB_LED
	led = RGB_LED()

	from math import sin, cos

	t0 = time.time()
	def get_config(t):
		t = t * speed
		a=0
		b=0
		c=0
		return {
			TOP: [a,b,c],
			BACK_LEFT: [b,c,a],
			BACK_RIGHT: [a, c,b],
			FRONT_LEFT: [b,a,c],
			FRONT_RIGHT: [c,a,b],
		}

	for i in range(1):
		t = time.time()
		config = get_config(t)
		for name, col in config.items():
			k = DuckietownLights.name2port[name]
			led.setRGB(k, col)		
		sleep(0.01)
	del led

def cycle_LEDs_fancy1(speed=5):
	from .duckietown_lights import DuckietownLights, TOP, BACK_LEFT, BACK_RIGHT, FRONT_LEFT, FRONT_RIGHT
	import time
	from time import sleep
	from .rgb_led import RGB_LED
	led = RGB_LED()

	from math import sin, cos

	t0 = time.time()
	def get_config(t):
		t = t * speed
		a = 0.5 + 0.5 * cos(t)
		b = 0.5 + 0.5 * sin(t)
		c = 0.5 + 0.5 * cos(2 * t)

		return {
			TOP: [a,b,c],
			BACK_LEFT: [b,c,a],
			BACK_RIGHT: [a, c,b],
			FRONT_LEFT: [b,a,c],
			FRONT_RIGHT: [c,a,b],
		}

	while True:
		t = time.time()
		config = get_config(t)
		for name, col in config.items():
			k = DuckietownLights.name2port[name]
			led.setRGB(k, col)		
		sleep(0.01)
	del led


def cycle_LEDs_fancy2(speed=1):
	from time import sleep
	from .duckietown_lights import DuckietownLights, TOP, BACK_LEFT, BACK_RIGHT, FRONT_LEFT, FRONT_RIGHT
	import time

	from .rgb_led import RGB_LED
	led = RGB_LED()

	from math import sin, cos

	def get_config(t):
		slow = 0.00005*t
		a = []
		for i in range(15):
			period = speed * (sin(i) + 0.00001 * cos(slow+i))
			x = 0.5 + 0.5 * cos(t * period)
			a.append(x)

		# nonlinear
		f = lambda x : x*x*x *0.9 + 0
		s = sum([f(x) for x in a])
		a = [f(x)/s for x in a]
		return {
			TOP: [a[0], a[1], a[2]],
			BACK_LEFT: [a[3],a[4],a[5]],
			BACK_RIGHT: [a[6],a[7],a[8]],
			FRONT_LEFT: [a[9],a[10],a[11]],
			FRONT_RIGHT: [a[12],a[13],a[14]],
		}

	while True:
		t = time.time()
		config = get_config(t)
		for name, col in config.items():
			k = DuckietownLights.name2port[name]
			led.setRGB(k, col)		
		sleep(0.01)
	del led


