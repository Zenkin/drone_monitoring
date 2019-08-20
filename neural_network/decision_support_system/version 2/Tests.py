from time import sleep

from Operator import Operator

operator = Operator()  # Создаём экземпляр оператора (СППР).

# Загружаем статический кадр в память
with open('img.jpg', "rb") as file:
	img = file.read()

# Подменяем видеопоток с камеры на статический кадр.
while True:
	# Анализируем поток, отбрасываем ненужные кадры
	operator.send(img, 1)  # Отправляем фотографию пожара/дыма/человека с поддельной вероятностью обнаружения в 100%.
	sleep(1)  # 1 FPS для теста хватит.
