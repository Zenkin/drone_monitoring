import json
import socket
import sys
import time
from io import BytesIO
from datetime import datetime
from telegram import Bot
from telegram.utils import request


class Server:
	"""Сервер событий и ботов управления."""

	def __init__(self, bot_timeout=1, event_timeout=1, ip='176.122.25.59', port=12345):
		"""Создать новый сервер событий и ботов управления.чия события Поддерживает только одно событие в один момент времени!

		bot_timeout - период проверки наличия сообщений боту.

		event_timeout - период проверки наличия событий.

		ip - прослушиваемый адрес сервера событий.

		port - прослушиваемый порт сервера событий."""
		self._operators = []
		# Load auth settings and create the telegram bot
		with open('auth.json', 'r', encoding='utf-8') as f:
			auth = json.loads(f.read())
			self._bot = Bot(token=auth['token'], request=request.Request(
				proxy_url=auth['request_kwargs']['proxy_url'],
				urllib3_proxy_kwargs=auth['request_kwargs']['urllib3_proxy_kwargs']))
			self._bot_last_update = 0
			self._bot_timeout = bot_timeout
		# Create the event server
		self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self._socket.bind((ip, port))
		self._socket.settimeout(event_timeout)
		# Set server frequency
		self._timeout = bot_timeout + event_timeout
		# Set log file as <module name>.log
		self._log_path = sys._getframe(0).f_code.co_filename.replace('<', '').replace('>', '') + '.log'
		# Set bot messages handlers
		self._bot_handlers = {}
		self.handle('/start', self._cmd_start)
		self.handle('/stop', self._cmd_stop)
		self.handle('/list', self._cmd_list)

	def log(self, msg):
		"""Записать в лог сообщение."""
		msg = str(datetime.now()) + '>> ' + msg
		print(msg)
		with open(self._log_path, mode='a') as file:
			file.write(msg + '\n')

	def handle(self, text, function):
		"""Добавить боту обработчик текста (команды).

		function (bot, update) - функция обработки команды
			bot - телеграм-бот

			update - объект update для обрабатываемого сообщения"""
		if text in self._bot_handlers:
			raise KeyError('Сообщения ' + text + ' уже обрабатываются другой функцией.')
		self._bot_handlers[text] = function

	def _receive_data(self, connection):
		"""Считать данные из сокета с буфером 1024 байта."""
		data = bytearray(0)
		chunk = [1]  # instead of do...while
		while len(chunk) != 0:
			chunk = connection.recv(1024)
			for b in chunk:
				data.append(b)
		return data

	def _check_bot_updates(self):
		"""Обработать все входящие боту сообщения."""
		updates = self._bot.get_updates(offset=self._bot_last_update, timeout=self._bot_timeout)
		if len(updates) == 0:
			return
		for update in updates:
			if update.message.text in self._bot_handlers:
				self._bot_handlers[update.message.text](self._bot, update)
		self._bot_last_update = updates[len(updates) - 1].update_id + 1

	def _check_events(self):
		"""Обработать отправленное серверу событие. Поддерживается только одно событие в один момент времени!"""
		try:
			client_socket, client_address = self._socket.accept()
			# TODO Внести в отдельный поток для паралельных подключений
			self.log('Входящее событие от ' + str(client_address) + '.')
			data = self._receive_data(client_socket)
		except socket.timeout:
			# Just no event was sent to the server
			pass
		else:
			self.log('Получено ' + str(len(data)) + ' байт(а).')
			# Telegram accepts a bytes stream for uploading photos
			stream = BytesIO()
			stream.write(data)
			for op in self._operators:
				stream.seek(0)  # Update stream pointer
				self._bot.send_photo(op, stream, caption='У нас беда!')
			self.log('Фотография отправлена операторам.')
			client_socket.close()

	def run(self):
		"""Запустить сервер в текущим потоке с блокировкой."""
		self._socket.listen(1)
		self.log('Сервер запущен.')
		while True:
			self._check_bot_updates()
			self._check_events()
			time.sleep(self._timeout)

	def _cmd_start(self, bot, update):
		if update.message.chat.id not in self._operators:
			self._operators.append(update.message.chat.id)
			self.log('Добавлен оператор ' + str(update.message.chat.id) + '.')
		update.message.reply_text(
			'Привет! Я буду оперативно высылать тебе события. Чтобы перестать их получать, напиши мне команду /stop. Чтобы посмотреть список операторов, отправь мне команду /list.')

	def _cmd_stop(self, bot, update):
		if update.message.chat.id in self._operators:
			self._operators.remove(update.message.chat.id)
			self.log('Удалён оператор ' + str(update.message.chat.id) + ' .')
		update.message.reply_text('Я больше не стану тебя беспокоить. Пока.')

	def _cmd_list(self, bot, update):
		self.log('Запрос на получение списка операторов от ' + str(update.message.chat.id) + '.')
		update.message.reply_text('Вот список моих операторов: ' + ', '.join(map(str, self._operators)) + '.')


if __name__ == '__main__':
	# ip='127.0.0.1'
	ip = '176.122.25.59'
	server = Server(ip=ip)
	while True:
		try:
			server.run()
		except:
			server.log('Ошибка времени выполнения. Сервер перезапущен.')
