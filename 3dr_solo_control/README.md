# Класс управления дроном solo3dr
## Основные функции

|Функция|Возвращаемое значение|Описание|Статус разработки|
|:----------:|:----------:|:----------:|:----------:|
|bool auPX4::connect()|True/False|При потере связи возвращает False, в противном случае True|DONE|
|IMU auPX4::getIMU();|struct auPX4::IMU|Возвращает все данные с IMU датчиков|DONE|
|Vector3* auPX4::getOdometry(std::string sourceData);|struct auPX4::Vector3|Возвращает позицию квадрокоптера. В параметр передается какую одометрию будем снимать (local/GPS). Также если мы выбрали GPS, но он не установлен (не публикуется сообщение в топик), то надо оповестить пользователя и выбрать режим local.|DONE|
|float auPX4::getHeight()|float|Возвращает высоту дрона, используя дальномер, если он отсутствует, то оповещаем об этом пользователя.|DONE|
|bool auPX4::takeoff(float altitude)|True/False|Дрон взлетает на высоту, которую мы указали в параметрах. Также реализация проверки на наличие сонара, если он есть, то по нему, если его нету, то по GPS. В любом другом случае выводить предупреждение о том, что взлет невозможен|DONE|
|bool auPX4::land()|True/False|Функция, которая реализует посадку квадрокоптера|DONE|
|void auPX4::cmdVel(float linearX, float linearY, float linearZ, float angularZ)||Управление скоростями квадрокоптера|DONE|
|void auPX4::control(int controlType)||Выбираем тип контроля, те если мы хотим управлять с джойстика, то надо вызвать с 1, в таком случае автономные функции будут недоступны. Аналогично с 2|Mostly DONE (Здесь могут быть баги и возможно нужны корректировки)|
|bool auPX4::arm()|True/False|Завести винты|DONE|
|bool auPX4::emergency()|True/False|Экстренное выключение винтов дрона. (Disarm)|DONE|

## Управление с джойстика

**Привязка кнопок (XBox)**

|Кнопка|Функция|
|:----:|:-----:|
|A|Arm|
|X|Takeoff 3m|
|Y|Land|
|B|Emergency(Disarm)|
|RB|Switch control|

**Для работы джойстика необходимо изменить его параметры в системе (если мх уже не изменяли)**

```
sudo chmod a+rw /dev/input/js0
```

## Запуск

Запускаем скрипт

```
./start_fight_sim.sh
```

Запускаем две ноды (каждую в своём терминале): нода драйвера для джойстика и нода управления(Предварительно прописав source, если ноды не находятся в catkin_ws системы)

```
rosrun quadrocopter_joystick_driver quadrocopter_joystick_driver
rosrun quadrocopter_control quadrocopter_control_node
```

*Изменяем систему отсчёта скорости на систему дрона, а не земли*


```
rosservice call /mavros/setpoint_velocity/mav_frame "mav_frame: 8"
```
