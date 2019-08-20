import json
import sys
from datetime import datetime
from io import BytesIO
from typing import Dict, Union

from telegram import Bot
from telegram.utils import request


class Manager(object):
	"""Телеграмм-бот."""

	def _load_json(self, file):
		"""Загрузить json из файла в кодировке UTF-8."""
		with open(file, 'r', encoding='utf-8') as f:
			return json.loads(f.read())

	def _save_json(self, file, data):
		"""Save object as json into the file."""
		with open(file, 'w', encoding='utf-8') as f:
			f.write(json.dumps(data))

	def __init__(self, config: Dict[str, int] = None):
		"""Создать нового телеграмм-бота.

		config - сопоставление командам id. {'command':id}"""
		self._operator = None
		auth = self._load_json('auth.json')
		self._bot = Bot(token=auth['token'], request=request.Request(
			proxy_url=auth['request_kwargs']['proxy_url'],
			urllib3_proxy_kwargs=auth['request_kwargs']['urllib3_proxy_kwargs']))
		self._bot_last_update = 0
		# Set log file as <module name>.log
		self._log_path = sys._getframe(0).f_code.co_filename.replace('<', '').replace('>', '') + '.log'
		# Config
		if not config:
			config = {'/start': 1}
		self._config = config

	def log(self, msg: str) -> None:
		"""Записать в лог сообщение."""
		msg = str(datetime.now()) + '>> ' + msg
		print(msg)
		with open(self._log_path, mode='a') as file:
			file.write(msg + '\n')

	def request(self) -> int:
		"""Возвращает id последней полученной команды, 0 для неопазнанной команды или -1 при осутствии команд (и удаляет её)."""
		updates = self._bot.get_updates(offset=self._bot_last_update)
		if len(updates) == 0:
			return -1
		cmd = updates[-1]
		self._bot_last_update = cmd.update_id + 1
		self._operator = cmd.message.chat.id
		self.log('Received ' + cmd.message.text + ' command.')
		if cmd.message.text in self._config:
			return self._config[cmd.message.text]
		return 0

	def response(self, data: Union[str, bytes, bytearray]) -> None:
		"""Отправить ответ на последнюю полученную команду (и удалить её). При отсутствии команд ничего не делать."""
		if not self._operator:
			return
		if isinstance(data, str):
			self._bot.send_message(self._operator, data)
		else:
			stream = BytesIO()
			stream.write(data)
			stream.seek(0)
			self._bot.send_photo(self._operator, stream)
		self._operator = None
		self.log('Sent data to operator.')

	def listen(self) -> int:
		"""Ожидать получения команды, а затем выполнить request()."""
		self.log('Listening for a command.')
		result = -1
		while result == -1:
			result = self.request()
		return result


def tests():
	manager = Manager()
	while True:
		try:
			print(str(manager.listen()))
			manager.response('OK')
			# with open('img.jpg', 'rb') as f:manager.response(f.read());
		except KeyboardInterrupt:
			return
		except Exception as e:
			manager.log('Runtime error:\n' + str(e))


# test
if __name__ == '__main__':
	tests()
