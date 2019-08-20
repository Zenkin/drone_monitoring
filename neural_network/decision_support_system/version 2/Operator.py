import json
import logging
from datetime import datetime, timedelta
from enum import IntEnum
from io import BytesIO
from random import randint
from typing import Callable, List
from time import sleep

from telegram import Bot, InlineKeyboardButton, InlineKeyboardMarkup
from telegram.utils import request

from IOperator import IOperator


class States(IntEnum):
	Disconnected = 0
	Authentication = 1
	Connected = 2


class Operator(IOperator):
	"""Икапсулирует работу СППР и взаимодействие с человеком-оператором."""

	_photo: bytes = None
	_last_photo: bytes = None
	_score: float = -1
	_post_time: datetime

	_events: List[List[Callable[[], None]]] = []
	_decisions = {}
	_bot: Bot

	_id: int = None
	_offset: int = 0
	_state: States = States.Disconnected

	def __init__(self):
		"""Икапсулирует работу СППР и взаимодействие с человеком-оператором."""
		# Load config
		with open('config.json', 'r', encoding='utf-8') as f:
			self._config = json.loads(f.read())
		# Init fields
		self._post_time = datetime.now() - timedelta(seconds=10 * self._config['timeouts']['detection'])
		self.__register_events(2)
		# Create bot
		self._bot = Bot(token=self._config['telegram']['token'], request=request.Request(
			proxy_url=self._config['telegram']['proxy']['url'],
			urllib3_proxy_kwargs=self._config['telegram']['proxy']['auth']))
		# Set log file as <module name>.log
		logging.basicConfig(format=u'%(levelname)-8s [%(asctime)s] %(message)s', level=logging.INFO)  # filename=sys._getframe(0).f_code.co_filename.replace('<', '').replace('>', '') + '.log'

	def __register_events(self, count):
		"""Зарегистрировать указанное количество событий. Их ID будут начинаться с 0 до count-1."""
		self._events = [[] for x in range(count)]

	def __generate_decision_id(self) -> str:
		"""Сгенерировать уникальный ID для фотографии."""
		decision_id = randint(1, 1000000)
		while decision_id in self._decisions:
			decision_id = randint(1, 1000000)
		return str(decision_id)

	def __callback(self, index):
		"""Вызвать событие index."""
		if index < 0 or index >= len(self._events):
			raise 'Unknown event ' + str(index) + ' .'
		for cb in self._events[index]:
			cb()

	def _process(self):
		"""Выполнить такт процесса - цикл главного потока СППР."""
		self._update()
		self._post()
		self._ignore()

	def _update(self):
		"""Обработать входящие сообщения."""
		updates = self._bot.get_updates(offset=self._offset)  # TODO: проверить offset=-1
		if len(updates) == 0:
			return
		for update in updates:
			if update.message:
				if self._state == States.Disconnected:
					if updates.index(update) == len(updates) - 1:
						update.message.reply_text('Вы пытаетесь подключиться к дрону Альфа. Отправьте пароль.')
					self._id = update.message.chat.id
					self._state = States.Authentication
				else:
					if update.message.chat.id != self._id:
						if updates.index(update) == len(updates) - 1:
							update.message.reply_text('Дроном Альфа уже управляет другой оператор. Попробуйте подключиться позже.')
					else:
						if self._state == States.Authentication:
							if update.message.text != self._config['password']:
								if updates.index(update) == len(updates) - 1:
									update.message.reply_text('Неверный пароль. Попробуйте ещё раз.')
							else:
								if updates.index(update) == len(updates) - 1:
									update.message.reply_text('Вы успешно подключились. Теперь вы можете отправлять команды и получать уведомления от бота Альфа.')
								self._state = States.Connected
								self.__callback(0)
						else:
							if update.message.text == '/stop':
								if updates.index(update) == len(updates) - 1:
									update.message.reply_text('Вы успешно отключились от бота Альфа.')
								self._state = States.Disconnected
								self._id = None
								self.__callback(1)
							else:
								# TODO: Make a command to get the last photo not any text.
								stream = BytesIO()
								stream.write(self._last_photo)
								stream.seek(0)
								self._bot.send_photo(self._id, stream, caption='Текущее изображение')
			elif update.callback_query:
				data = update.callback_query.data.split('-')
				type = data.pop(0)
				if type == 'decision':
					self._decide(*data)
		self._offset = updates[len(updates) - 1].update_id + 1

	def _post(self):
		"""Отправить распознанную фотографию оператору."""
		if self._state == States.Connected and self._photo is not None and (datetime.now() - self._post_time).seconds > self._config['timeouts']['detection']:
			stream = BytesIO()
			stream.write(self._photo)
			stream.seek(0)
			# Create new decision
			decision_id = self.__generate_decision_id()
			callback_prefabs = ['decision-' + decision_id + '-' + x for x in ['ignore', 'alert']]
			decision = {'time': datetime.now(),
						'id': decision_id,
						'message': {
							'id': self._bot.send_photo(self._id, stream, caption='Обнаружена опасность!', reply_markup=InlineKeyboardMarkup([
								[
									InlineKeyboardButton("Ложное срабатывание", callback_data=callback_prefabs[0])
								], [
									InlineKeyboardButton("Вызвать спасателей", callback_data=callback_prefabs[1])
								]])).message_id,
							'operator': self._id}
						}
			self._decisions[decision_id] = decision
			self._photo = None
			self._score = -1
			self._post_time = datetime.now()

	def _ignore(self):
		"""Игнорировать все неотмеченные оператором фотографии."""
		if len(self._decisions) == 0:
			return
		for decision in self._decisions.copy().values():  # copy due to possibility of an deleting event (ignore)
			if (datetime.now() - decision['time']).seconds >= self._config['timeouts']['decision']:
				self._decide(decision['id'], 'ignore')

	def _decide(self, decision_id, action):
		"""Обработать реакцию на ЧС."""
		if decision_id not in self._decisions or action not in ['ignore', 'alert']:
			return
		text = 'Событие проигнорировано.' if action == 'ignore' else 'Вызваны службы МЧС.'
		self._bot.edit_message_caption(chat_id=self._decisions[decision_id]['message']['operator'], message_id=self._decisions[decision_id]['message']['id'], caption=text)  # delete buttons
		del self._decisions[decision_id]

	def send(self, photo: bytes, score: float) -> None:
		"""Отправить фотографию (кадр) в СППР.

		photo - байтовое представление фотографии (формат задаётся телеграммом).

		score - вероятность распознания на фотографии черезвычайного события."""
		self._last_photo = bytes(photo)
		if score > self._score:
			self._photo = self._last_photo
		self._process()

	def subscribe(self, event: int, callback: Callable[[], None], unique: bool = False) -> None:
		"""Подписаться на событие.

		event - номер события.

		callback - функция, которая будет вызвана при наступлении события.

		unique - запретить подписываться одной функции несколько раз?"""
		if event < 0 or event >= len(self._events):
			raise 'Unknown event ' + str(event) + ' .'
		if unique and callback in self._events[event]:
			raise 'Callback has been already subscribed to event ' + str(event) + '.'
		self._events[event].append(callback)

	def unsubscribe(self, event: int, callback: Callable[[], None]) -> None:
		"""Отписаться от события. Выбрасывает исключение, если функция не была подписана на указанное событие.

		event - номер события.

		callback - функция, которая подписана на событие."""
		if event < 0 or event >= len(self._events):
			raise 'Unknown event ' + str(event) + ' .'
		self._events[event].remove(callback)

	def wait(self, loop=True):
		"""Заблокировать поток и ожидать подключения оператора.

		loop - ожидать следующего оператора после отключения последнего?"""
		while True:
			while self._state != States.Connected:
				self._process()
				sleep(1)
			if not loop:
				break
