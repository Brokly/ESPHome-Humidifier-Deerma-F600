# ESPHome-Humidifier-Deerma-F600

   Небольшая переделка очень глупого , но мощного и надежного увлажнителя Deerma F600 в умный.

   ![image](https://user-images.githubusercontent.com/11642286/189353849-2060d076-47fb-478e-9930-5c90630df8e1.png)<br>Понадобится сам увлажнитель, еспшка, датчик влажности, DC/DC преобразователь, несколько резисторов, конденсатор, диод, провода и руки с паяльником.
   Датчик влажности рекомендую только такой , он имеет преобразователи уровней на борту, что избавляет от проблем.Питать его можно от 3.3 вольт, того же источника питания, что и есп.
	 ![image](https://user-images.githubusercontent.com/11642286/189356175-6a7d286f-064a-456c-bbbe-8b1e27c7d2a3.png)<br>Преобразователь питания подключен к блоку питания увлажнителя, хоть в увлажнителе и есть 5 вольт, мощности пяти вольтового источника не хватает есп, поэтому нужно использовать преобразователь, например такой:
	 ![image](https://user-images.githubusercontent.com/11642286/189357050-0a3804dd-8987-4186-8fbd-713e1569bfda.png)<br>
 
 Уровни напряжения увлажнителя и мозгов, которые мы встраиваем в устройство отличаются, поэтому нужны резистивные делители. Можно использовать резисторы в любом исполнении. Я использую SMD на плате от левел шифтера. Кстати при желании можно установить и левелшифтер, только не забыть подать на него 5 вольт. К сенсорной кнопке увлажнителя подключаемся через любой диод и неполярный конденсатор 1 мкф, дод для отсечения возможного напряжения идущего с кнопки, конденсатор - как эмулятор пальца :).
  
  ![image](https://user-images.githubusercontent.com/11642286/189538284-31dc6291-6e28-41a9-b1cf-5d8eb61044ba.png)![image](https://user-images.githubusercontent.com/11642286/189358416-db09d632-1bef-486a-af7d-7be225979b3f.png)
  
  Разобрать увлажнитель просто, выкрутить четыре самореза, три из них под резиновыми ножками.
  ![image](https://user-images.githubusercontent.com/11642286/189359149-d9150256-acf7-4b4c-9e92-a3765a73f577.png)

У меня получилось так:
  
![image](https://user-images.githubusercontent.com/11642286/189359517-b182d86c-0289-4478-af59-be2430f8cad3.png)

Но у меня есть самодельная платка еспшки с DC/DC преобразователем на борту.

По конфигу:

Обязательно задать ножки подключения к увлажнителю (pin)<br>
     <strong>disp_sync_pin: <br>
    disp_read0_pin: <br>
    disp_read1_pin: <br>
    sensor_control_pin:  </strong><br><br>
Обязательно указать id датчика влажности по которому поумневший увлажнитель будет понимать текущую влажность (sensor)<br>
     <strong>sensor_humidity_id:  </strong><br><br>
Обязательно - селектор режимов работы (Off, Low, Medium, High, Auto) (select)<br>
     <strong>current_preset: </strong><br><br>
Текстовое описание текущего режима (text_sensor)<br>
    <strong> current_state: </strong><br><br>
Ползунок установки желаемой влажности (number)<br>
     <strong>humidity_destination: </strong><br><br>
    Много писать не буду, будут вопросы спрашивайте тут - https://github.com/Brokly/ESPHome-Humidifier-Deerma-F600/issues<br>
    
<strong>UPD:</strong>
При пользовании устройством оказалось, что при переключении режимов звук встроенной пищалки довольно громкий, особенно ночью. А яркость дисплея, опять же , ночью, чрезмерна. 
Поскольку встроенных механизмов управления этими свойствами устройство не имеет, пришлось добавить несколько деталей.
    
![схема](https://github.com/Brokly/ESPHome-Humidifier-Deerma-F600/assets/11642286/beb301c0-697c-4552-9800-6d659ecc7290)

Тут я нарисовал полную схему "допиливания", с указанием точек на фото выше. 
Все элементы, имеющие цифру в обозначении (например R1 или R5), являются добавленными к схеме устройства. А детали не имеющие циффры (например R или P) - "родные" установленные на плате,
указаны в схеме для упрощения внесения модификаций. Естественно дисплей и процессор (MCU) это детали на плате устройства с завода. По факту нужно добавить три оптрона, диод, конденсатор и девять
резисторов.
	Конфиг дополненный регулировкой яркости дсплея и включателем бизера добавлен в папку с конфигами.
	Для полноты восприятия, еще несколько картинок.
![image](https://github.com/Brokly/ESPHome-Humidifier-Deerma-F600/assets/11642286/109486f5-2d4f-44a3-b879-fabafce3b4ed)
	Порезанные дорожки.
![image](https://github.com/Brokly/ESPHome-Humidifier-Deerma-F600/assets/11642286/528036f3-ff6e-4b76-99de-f388c6d8ad53)
	Оптроны управления яркостью дисплея
![image](https://github.com/Brokly/ESPHome-Humidifier-Deerma-F600/assets/11642286/fdbb4cf9-c146-43d5-b348-eb6692b9dd5b)
	Оптрон управления пищалкой.


