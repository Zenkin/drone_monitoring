import socket


class EvenGenerator:
	"""Генерирует события из видеопотока и отправляет их на сервер.

	fps - количество кадров в секунду в видеопотоке

	eventTimeout - минимальный интервал между генерацией двух событий

	ip - адрес сервера

	port - порт сервера
	"""

	def __init__(self, fps=30, eventTimeout=60, ip='176.122.25.59', port=12345):
		"""Генерирует события из видеопотока и отправляет их на сервер.

		fps - количество кадров в секунду в видеопотоке

		eventTimeout - минимальный интервал между генерацией двух событий

		ip - адрес сервера

		port - порт сервера
		"""
		self.period = fps * eventTimeout
		self.server = (ip, port)
		self.counter = 0
		self.canSend = True
		self.buffer = None
		self.score = -1
		self.event_timeout = eventTimeout

	def grab(self, frame, score):
		"""Отправить кадр видеопотока в генератор событий.

		frame - кадр, массив байт

		score - вероятность обнаружения [0;1]



		with open('img.jpg', "rb") as file:
			generator.grab(file.read(),0.5)"""
		self.counter += 1
		if self.counter == 0 or score > self.score:
			self.score = score
			self.buffer = frame
		if self.counter == self.period:
			self.canSend = True
		if self.canSend:
			self._send_event()

	def _send_event(self):
		sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		sock.settimeout(self.event_timeout)
		sock.connect(self.server)
		sock.send(self.buffer)
		sock.close()
		self.canSend = False
		self.score = -1
		self.buffer = None
		self.counter = 0


# Tests
if __name__ == '__main__':
	fps = 1
	timeout = 5
	gen = EvenGenerator(fps=fps, eventTimeout=timeout)
	# gen = EvenGenerator(fps=fps, eventTimeout=timeout, ip='127.0.0.1')
	with open('img.jpg', "rb") as file:
		img = file.read()

	import random, time
	from datetime import datetime

	# input('Input to start')
	n = 10
	for i in range(fps + fps * timeout * (n - 1)):
		score = random.random()
		print('Grabbing score ' + str(score) + ' at ' + str(datetime.now()))
		gen.grab(img, score)
		time.sleep(1)
