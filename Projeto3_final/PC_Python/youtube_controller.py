import pyautogui
import serial
import argparse
import time
import logging
import osascript

from urllib.request import urlopen
import requests
import json
import threading
import time

global status_check_stream

ser = serial.Serial("/dev/cu.usbmodem14202", baudrate=115200)

super
def is_live_stream():
	while 1:
		url = "https://api.twitch.tv/kraken/streams/veguinho98?client_id=uf2ck23nxqeaohmgy79zamz0p65o1e"
		streamer_html = requests.get(url)
		streamer = json.loads(streamer_html.content)
		time.sleep(5)
		#MANDA STATUS DA LIVE
		if streamer["stream"] is not None:
			mandarLiveString = ("L" + str(streamer["stream"]["viewers"]) + "X")
			ser.write(mandarLiveString.encode())
			print(mandarLiveString)
			print("ESTA LIVE (L)")
		else:
			mandarNotLiveString = str("N0X")
			ser.write(mandarNotLiveString.encode())
			print(mandarNotLiveString)
			print("NAO ESTA LIVE (N)")
		status_check_stream = 0


class MyControllerMap:
	def __init__(self):
		self.button = {'L':['ctrl','shift','s'],'M': ['ctrl','shift','o'], 'J':['ctrl','shift','m']} # Fast forward (10 seg) pro Youtubem
	  # Fast forward (10 seg) pro Youtube
class SerialControllerInterface:
	# Protocolo 
	# byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
	# byte 2 -> EOP - End of Packet -> valor reservado 'X'

	def __init__(self, port, baudrate):
		self.mapping = MyControllerMap()
		self.incoming = '0'
		pyautogui.PAUSE = 0  ## remove delay
	
	def update(self):
		## Sync protocol
		while self.incoming != b'X':
			self.incoming = ser.read()
			logging.debug("Received INCOMING: {}".format(self.incoming))

		data = ser.read()
		logging.debug("Received DATA: {}".format(data))

		if data == b'1':
			data = ser.read()
			if data == b'L':
				logging.info("KEYDOWN L")
				pyautogui.hotkey('ctrl','shift','s')
			if data == b'M':
				logging.info("KEYDOWN B")
				pyautogui.hotkey('ctrl','shift','o')
			if data == b'J':
				logging.info("KEYDOWN J")
				pyautogui.hotkey('ctrl','shift','m')

		if data == b'A':
			data = ser.read()
			for k in range(3):
				data += ser.read()
			logging.info("KEYDOWN V")
			print("set volume output volume " + data.decode("utf-8"))
			data = int(data.decode("utf-8"))
			data = str(100*(int(data)/4095))
			osascript.osascript("set volume output volume " + data)
			
		# elif data == b'0':
		# 	logging.info("KEYUP A")
		# 	pyautogui.keyUp('A')
		# 	logging.info("KEYUP B")
		# 	pyautogui.keyUp('M')

		self.incoming = ser.read()


class DummyControllerInterface:
	def __init__(self):
		self.mapping = MyControllerMap()

	def update(self):
		pyautogui.keyDown(self.mapping.button['L'])
		time.sleep(0.1)
		pyautogui.keyUp(self.mapping.button['L'])
		logging.info("[Dummy] Pressed L button")
		time.sleep(1)


if __name__ == '__main__':
	interfaces = ['dummy', 'serial']
	argparse = argparse.ArgumentParser()
	argparse.add_argument('serial_port', type=str)
	argparse.add_argument('-b', '--baudrate', type=int, default=115200)
	argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
	argparse.add_argument('-d', '--debug', default=False, action='store_true')
	args = argparse.parse_args()

	if args.debug:
		logging.basicConfig(level=logging.DEBUG)

	print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
	if args.controller_interface == 'dummy':
		controller = DummyControllerInterface()
	else:
		controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

	status_check_stream = 0
	while True:
		if(status_check_stream == 0):
			status_check_stream = 1
			threading.Thread(target=is_live_stream).start()
		controller.update()
