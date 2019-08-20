from typing import Callable


class IOperator:
	"""Прототип СППР"""

	def send(self, photo: bytes, score: float) -> None:
		"""Отправить фотографию (кадр) в СППР.

		photo - байтовое представление фотографии.

		score - вероятность распознания на фотографии черезвычайного события."""
		pass

	def subscribe(self, event: int, callback: Callable[[], None], unique: bool = False) -> None:
		"""Подписаться на событие СППР.

		event - номер события.

		callback - функция, которая будет вызвана при наступлении события.

		unique - запретить подписываться одной функции несколько раз?"""
		pass

	def unsubscribe(self, event: int, callback: Callable[[], None]) -> None:
		"""Отписаться от события СППР. Выбрасывает исключение, если функция не была подписана на указанное событие.

		event - номер события.

		callback - функция, которая подписана на событие."""
		pass


