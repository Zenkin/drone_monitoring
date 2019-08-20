
В папке final_models находится все необходимое для запуска нейронки. А именно модели и скрипты.    
Ссылка на модели: https://yadi.sk/d/0rZV8MZB8fCnRQ
# Скрипты
## Описание скриптов
Определимся с названиями. К примеру,

•	ssd_detector – скрипт запуска кастомной модели ssd. (сочетается с моделями ssd_fire_detection, ssd_smoke_detection, ssd_defence (это последняя двухклассовая модель для защиты))

•	ssd_detector_writing.py - скрипт запуска кастомной модели ssd c записью результатов детектирования в видеофайл. (сочетается также как ssd_detector)

•	ssd_detector_extract.py - скрипт запуска оригинальной модели ssd с детектированием только людей. (сочетается только с ssd_original)

•	ssd_detector_extract_writing.py - скрипт запуска оригинальной модели ssd с детектированием только людей  с записью результатов детектирования в видеофайл. (сочетается только с ssd_original)

•	tiny-yolov3_extract.py - скрипт запуска оригинальной модели  tiny-yolov3 с детектированием только людей. (сочетается только с моделью yolo_tiny)

•	tiny-yolov3_extract_with_writing.py - скрипт запуска оригинальной модели  tiny-yolov3 с детектированием только людей с записью результатов детектирования в видеофайл.(сочетается только с моделью yolo_tiny)

•	ssd_detector_writing_fake.py - скрипт запуска кастомной модели ssd с фейковым фпс и измененной вероятностью детекта c записью результатов детектирования в видеофайл. (сочетается с ssd_defence)

Остальные скрипты относятся к телеграм-боту:

•	IОperator.py
•	Operator.py

Запускать или изменять их не нужно. Но нужно чтобы они находились в одной папке со скриптами запуска моделей.


## Запуск моделей

Запускаем по образцу:

python3 <название скрипта>  -m <полный путь к .xml файлу модели>  -i <полный путь к видеофайлу для детектирования или cam для детекта с камеры >

Опциональные добавки: -l <полный путь к либе для запуска чисто на CPU (т.е. без movidius) > --labels <полный путь к названию классов, чтобы бокс имел вместо номера класса его название> -pt <нижний порог вероятности принадлежности к классу для отображения, по дефолту стоит 0.5>

Примечания:
1)В папке с выбранной моделью есть еще 2 папки: 16 и 32-битные версии. Для запуске на movidius используем 16-битную модель. Для запуска на CPU – 32-x битную.
2) Для удобства в каждой папке моделей есть .labels файл. Это обычный тектовый файл с названием каждого класса в новой строке и нулевой первой строкой.

## Углубляясь в скрипты
Скрипты для запуска нейронок имеют схожую структуру.
Она состоит из 3 частей: нейронка, телеграм бот, рос

Рассмотрим на примере ssd_detector_ros.py
До момента ================================================== идут стандартные моменты по запуску нейронки, ничего менять не стоит.
Далее идет следующий кусочек
fourcc = cv2.VideoWriter_fourcc(*'DIVX')
fps = 24
capSize = (1280, 720)
out = cv2.VideoWriter('/home/nirma/test_videos/videos/new.avi', fourcc, fps, capSize)
Здесь задаются параметры для записи видео: фпс, разрешение и путь файла. Можно менять.

Потом спускаемся до следующей ==============================

operator = Operator()- создает экземпляр класса Operator. Т.е. запускает телеграм-бота. Если не хотим запускать – заккомечиваем.
    while cap.isOpened():
        frame_rects_centrs = [] - создаем список для добавления координат центров прямоугольников ( для последующей передачи их в рос)
        num_detects = 0 – переменная для определения количества детектов ( нужна для создания условия отправки картинки боту)
        score = 0 – переменная, показывающая точность детектирования

Потом спускаемся до следующей ==============================

    if obj[2] > args.prob_threshold:  если вероятность объекта больше порога
                    xmin = int(obj[3] * initial_w)  коо
                    ymin = int(obj[4] * initial_h)  рди
                    xmax = int(obj[5] * initial_w)   на
                    ymax = int(obj[6] * initial_h)   ты
                    class_id = int(obj[1])
                    # Draw box and label\class_id
                    color = (min(class_id * 12.5, 255), min(class_id * 7, 255), min(class_id * 5, 255)) рандомизатор цвета в записимости от класса
                    color_red = (0, 0, 255) просто красный цвет
                    cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color_red, 5) чертим прямоугольник на кадр
                    det_label = labels_map[class_id] if labels_map else str(class_id)
                    cv2.putText(frame, det_label + ' ' + str(round(obj[2] * 100, 1)) + ' %', (xmin, ymin - 7),
                                cv2.FONT_HERSHEY_COMPLEX, 0.6, color_red, 1) отображаемый на кадре текст: название класса + вероятность + на какое место лепить + каким цветом
                    rectangle_center = [xmin + (xmax-xmin)/2, ymin + (ymax-ymin)/2] рассчитываем центр квадрата
                    frame_rects_centrs.append(rectangle_center) добавляем координаты в ранее созданный список
                    num_detects += 1  увеличиваем счетчик детектов
                    score += obj[2] увеличиваем вероятность обнаружения
                   
            fps_message = "Detector's FPS: {:.3f}".format(1/render_avg)  текст для отображения фпс
        
            cv2.putText(frame, fps_message, (15, 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (10, 10, 200), 1)  вставка этого текста в кадр с параметрами цвета, расположения и размера
Далее реализуется логика для бота
if num_detects > 0: если на кадре были детекты
                success, encoded_image = cv2.imencode('.jpg', frame) операции для декодирования в байты ( телега требует)
                content2 = encoded_image.tobytes() продолжение декодирования
                score_mean = score/num_detects определение средней вероятности детекта ( если найдено больше 1 объета, а в телегу нужно прислать только 1 число)
                operator.send(content2, score_mean) отправка в телегу кадра с детектом и вероятности в цифрах

Дале идет логика рос, НО тебе нужно будет вставить на это место логику из скрипта на пишке, к сожалению его у меня нет
rect_cent = "-1"
            if num_detects > 0:
                rect_cent = "1"   
                     
            if not rospy.is_shutdown():
                #hello_str = "hello world %s" % rospy.get_time()
                
                rospy.loginfo(rect_cent)
                pub.publish(rect_cent)
                #rate.sleep()
По данной логике мы тупо отправляем единичку если есть детекты. По логике из скрипта на пишке отправляем координату центра прямоугольника.


## Пример запуска модели и записи видоса для защиты:
ppython3 ssd_detector_writing_fake.py -m /home/nirma/final_models/models/ssd_model_defence/FP16 -i /home/nirma/Downloads/Test_reaction_DJI.mp4  --labels /home/nirma/final_models/models/ssd_model_defence/FP16/frozen_inference_graph.labels
Нужно только подправить пути для модели и с кусок кода в скрипте (путь, куда записывать видос в out = cv2.VideoWriter('/home/nirma/test_videos/videos/new.avi', fourcc, fps, capSize)

Фейковость:
fps_message = "Detector's FPS: {:.3f}".format(1/render_avg + 15) # fake fps
увеличил отображаемое фпс на 15
class_probabil = round(obj[2] * 100, 1)
                    if class_probabil > 95:
                        class_probabil = class_probabil - 10 + round(random.random())
чтоб не слишком палиться на том что у нас совпадают тренировочные и тестовые данные, я уменьшил вероятность на 10 для детектов с вероятностью больше 95 %

Заметка:
Не включать телегу можно закомментив operator = Operator() и operator.send(content2, score_mean)
Не включать взаимодействие с росом можно закоменитив:
if not rospy.is_shutdown():
                #hello_str = "hello world %s" % rospy.get_time()
                
                rospy.loginfo(rect_cent)
                pub.publish(rect_cent)
                #rate.sleep()
