import os
import sys
import time
import max30100


def main():
	try:
		mx30 = max30100.MAX30100(
					debug=False,
					led_current_red=33.8,
                	led_current_ir=33.8,
				)
		mx30.set_mode(max30100.MODE_SPO2)

		i = 0
		while i < 60:
			# mx30.read_sensor()
			mx30.update()
			# The latest values are now available via .ir and .red
			# print("HRate sensor .ir: {} and SpO2 .red: {}".format(mx30.ir, mx30.red))
			# print("\n")
			time.sleep(0.010)
			i = i + 1

		print("Current BPM {}".format(mx30.currentBPM))
		print("values of BPM {}".format(mx30.valuesBPM))
		mx30.reset()


	except Exception as e:
		print("Max30100 got exception {}".format(e))


if __name__ == "__main__":
  main()

