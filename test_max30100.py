import os
import sys
import time
import max30100


def main():
	try:
		mx30 = max30100.MAX30100()
		# mx30.set_mode(max30100.MODE_SPO2)

		# mx30.reset()
		# time.sleep(1)
		# mx30.startup()
		# time.sleep(1)


		i = 0
		while i < 30:
			# mx30.set_mode(max30100.MODE_HR)
			# mx30.read_sensor()
			# # The latest values are now available via .ir and .red
			# print("HeartRate sensor .ir: {} and .red: {}".format(mx30.ir, mx30.red))

			# time.sleep(2)
			# mx30 = max30100.MAX30100()
			mx30.reinit()
			mx30.set_mode(max30100.MODE_SPO2)
			# time.sleep(1)

			mx30.read_sensor()
			# The latest values are now available via .ir and .red
			print("HRate sensor .ir: {} and SpO2 .red: {}".format(mx30.ir, mx30.red))


			# temp = mx30.get_temperature()
			# print("Temperature {}\n".format(temp))
			mx30.reset()
			time.sleep(1)

			i = i + 1
		mx30.reset()
		# mx30.shutdown()


	except Exception as e:
		print("Max30100 got exception {}".format(e))


if __name__ == "__main__":
  main()

