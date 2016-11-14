/*
    РОБОТЫ и НАБОРЫ ПО РОБОТОТЕХНИКЕ на МРобот! mrobot.by
    http://www.mrobot.by

    Кухня Роботов <maxxlife>
    http://www.vk.com/cookrobot
    Copyright (c) Кухня роботов <maxxlife.by> All rights reserved.
    Copyright (c) Макслайф Робот Maxxlife Robot All rights reserved.
    Наши лаборатории по робототехнике:
    Ленинская РОС ДОСААФ, ул. Рокоссовского 63/2, Минск, Беларусь
    Подробнее в нашей группе Вконтакте http://www.vk.com/cookrobot
    И на сайте http://www.maxxlife.by
    ****************************************************
    Мы всегда рады новым членам нашего сообщества Кухня Роботов.
    У нас есть вводные курсы, где мы объясняем
    как работать с нашими образовательными наборами по робототехнике и электронике.
    ****************************************************
    Название набора: РобоРовер М1 4WD CodeWheels Edition
    Программа создана и протестирована разработчиком:
    Имя: Максим Массальский
    Ник: maxxlife
    E-mail: maxxliferobot@gmail.com
*/

/*В роботе используются пины (для Arduino Uno-подобной платы)
   Мотордрайвер использует пины:
   D2, D3, D5, D8, D10, D11
   Сервопривод использует пины:
   D9
   Ультразвуковой датчик использует пины:
   D4, D7
   Свободные цифровые пины:
   D0, D1, D6, D12, D13
   Аналоговые ИК-датчики используют пины:
   A0, A1
   Свободные аналоговые пины:
   A2, A3, A4, A5
*/

//Подключаем библиотеку по работе с акселерометром-гироскопом платы
#include <CurieIMU.h>
//Подключаем библиотеку по работе с Bluetooth LE
#include <CurieBLE.h>
//Подключаем библиотеку которая вычисляет углы Эйлера. Madgwick Filter 
#include <MadgwickAHRS.h>

Madgwick filter;

unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float roll, pitch, heading;
unsigned long microsNow;


//Самодельный класс Servo, чтобы не использовать стандартную библиотеку Servo
//Т.к. она мешает управлению пинами мотордрайвера L298N
class Servo // имя класса
{
  public:
    int write(int data)
    {
      analogWrite(9, data);
    }
};
//Создаем экземпляр класса Servo под именем radar
Servo radar;

//Пины ультразвукового датчика
int trigPin = 7;
int echoPin = 4;

//Константа voltConst -это значение опорного напряжения в милливольтах
//Нужна для того, чтобы переводить значение напряжения с ИК-датчика в расстояние.
double voltConst = 0.0033;

//Определение номеров пинов мотордрайвера
int enA = 5;
int in1 = 2;
int in2 = 8;
// motor two
int enB = 3;
int in3 = 11;
int in4 = 10;

//Переменные по хранению расстояний с переднего левого и правого ИК-датчиков
volatile int rFront = 0;
volatile int lFront = 0;

// Поворотный сервопривод radar
volatile int posR;
//Значение для угла в 90 градусов (от 0 до 255)
int radarFront = 185;
//Значение для левого положения
int radarLeft = 140;
//Значение для правого положения
int radarRight = 230;

// Ультразвуковой сенсор
//Переменная по хранению расстояния с УЗ-датчика
volatile long usRange = 0;
//Максимальный порого
int usHigh = 100;
//Минимальный порог
int usLow = 30;
volatile long usRangeLeft = 0;
volatile long usRangeRight = 0;

// Датчики инфракрасные Передние
//Переменная, которая позволяет определить расстояние спереди робота, зная расстояние и то, что датчики расстояния стоят под углом в 45 градусов.
//double angleFront = cos((45 * PI) / 180); // Угол 45 градусов
float angleFront = 1;
//Левый ИК-датчик подключен к аналоговому пину 0
int irPinFrontLeft = 0;
//Правый ИК-датчик подключен к аналоговому пину 1
int irPinFrontRight = 1;

//Переменные по настройке алгоритма передних ИК-датчиков под условия и расстояния
//Максимальный порог
int frontHigh = 60;
//Минимальный порог
int frontLow = 40;

//Класс для измерения расстояния с ИК-датчиков робота
class irSharpMeasure
{
  public:
    int irMeasure(int irPin)
    {
      int distance;
      // Измеряем значение с датчика в милливольтах.
      //Значение, которое было считано с аналогового порта умножается на переменную voltConst
      //И получается значение в вольтах, которое есть на аналоговом пине
      double volts = analogRead(irPin) * voltConst;

      //Serial.println(volts);
      //Экспериментальные значения в вольтах для определенного расстояния
      //Определены экспериментальным путем и с использованием даташита к ИК-датчику Sharp (10-80cm)
      double val[40] = { 2.73, 2.34, 1.99, 1.76, 1.57, 1.42, 1.29, 1.20, 1.07, 1.00, 0.94, 0.88, 0.82, 0.79, 0.74, 0.71, 0.68, 0.65, 0.63, 0.61, 0.58, 0.55, 0.53, 0.50, 0.51, 0.49, 0.48, 0.47, 0.45, 0.44, 0.42, 0.41, 0.40, 0.39, 0.38, 0.37, 0.36, 0.33, 0.28, 0.26};
      int cm[40] = {8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 90, 100, 110};
      //Алгоритм по сопоставлению значения в вольтах расстоянию
      for (int i = 0; i < 40; i++)
      {
        if (val[i] <= volts)
        {
          //Находим расстояние
          //Если датчик находится под углом, то можно узнать расстояние до объекта по теореме Пифагора, если это необходимо
          //Умножив на angleFront
          distance = cm[i] * angleFront;
          break;
        }
        //Если volts меньше определенного порога, то считаем, что расстояние больше 120 см
        if (volts < 0.26)
        {
          distance = 120 * angleFront;
        }
      }
      //Если volts больше определенного порогоа, то считаем, что расстояние меньше 10 см
      if (volts > 3.00)
      {
        distance = 6 * angleFront;
      }
      //Возвращаем значение distance
      return distance;
    }
};
//Создаем экземпляр класса под именем IR
irSharpMeasure ir;

//Функция по измерению расстояний с переднего левого и правого ИК-датчиков
void irMeasure()
{
  //Меряем расстояние с правого ИК
  rFront = ir.irMeasure(irPinFrontRight);
  //Меряем расстояние с левого ИК
  lFront = ir.irMeasure(irPinFrontLeft);
}

//Функция по измерению расстояния с УЗ-датчика HCSR-04
void usMeasure () {

  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  usRange = (duration / 2) / 29.1;
  delay(50);
}

//Функция по выезду из тупика при помощи ИК-датчиков и УЗ-датчика
void irFrontGoBack()
{
  //Если с двух передних ИК-датчиков расстояния одинаковые,
  //То используем ультразвуковой датчик
  if (lFront == rFront)
  {
    //Поворачиваем сначала УЗ-датчик вправо
    radar.write(radarRight);
    delay(200);
    //Делаем измерением
    usMeasure();
    //Записываем расстояние справа в отдельную переменную
    usRangeRight = usRange;
    //Поворачиваем влево
    radar.write(radarLeft);
    delay(200);
    //Делаем измерение
    usMeasure();
    //Записываем расстояние слева в отдельную переменную
    usRangeLeft = usRange;
    //Если расстояние слева меньше расстояния справа
    if (usRangeLeft < usRangeRight)
    {
      //То поворачиваем вправо
      go_RightFull(200, 100);
      delay(500);
      go_Stop();
      //Поворачиваем УЗ влево
      radar.write(radarLeft);

      //Едем назад до тех пор пока расстояния с ИК-датчиков не будут удовлетворять условиям
      //Можешь настраивать как тебе необходимо
      while ((rFront < frontLow) && (lFront < frontLow))
      {
        irMeasure();
        go_Backward(220, 220);
      }
      //Едем назад, чтобы и с переднего УЗ-датчика, расстояние было больше необходимого
      usMeasure();
      while ( usRange < usLow)
      {
        usMeasure();
        go_Backward(220, 220);
      }
      //Поворачиваем робота вправо
      go_RightFull(200, 100);
      delay(500);
      go_Stop();
    }
    //Здесь как и выше описано, только все операции наоборот.
    if (usRangeRight < usRangeLeft)
    {
      go_LeftFull(100, 200);
      delay(500);
      go_Stop();
      radar.write(radarRight);
      while ((rFront < frontLow) && (lFront < frontLow))
      {
        irMeasure();
        go_Backward(220, 220);
      }
      usMeasure();
      while (usRange < usLow)
      {
        usMeasure();
        go_Backward(220, 220);
      }
      go_LeftFull(100, 200);
      delay(500);
      go_Stop();
    }
    //Если у нас произошло, что и УЗ-датчика одинаковое расстояние(!), робот просто едет назад
    //До тех пор пока не будут удовлетворены условия
    if (usRangeRight == usRangeLeft)
    {
      radar.write(radarFront);
      while ((rFront < frontLow) && (lFront < frontLow))
      {
        irMeasure();
        go_Backward(220, 220);
      }
      usMeasure();
      while ((usRange < usLow))
      {
        usMeasure();
        go_Backward(220, 220);
      }
    }
    radar.write(radarFront);
  }
  //Если с правого ИК-датчика расстояние меньше чем с левого ИК-датчика
  if (rFront < lFront)
  {
    //Поворачиваем робота влево
    go_LeftFull(100, 200);
    delay(500);
    go_Stop();
    //Едем назад до тех пор пока расстояния с ИК-датчиков не будут удовлетворять условиям
    //Можешь настраивать как тебе необходимо.
    while ((rFront < frontLow) && (lFront < frontLow))
    {
      irMeasure();
      go_Backward(220, 220);
    }
    usMeasure();
    //Едем назад, чтобы и с переднего УЗ-датчика, расстояние было больше необходимого
    while ((usRange < usLow))
    {
      usMeasure();
      go_Backward(220, 220);
    }
    //Поворачиваем робота влево
    go_LeftFull(100, 200);
    delay(500);
    go_Stop();
  }
  //Если с левого ИК-датчика расстояние меньше чем с правого ИК-датчика
  if (lFront < rFront)
  {
    //Поворачиваем робота вправо
    go_RightFull(200, 100);
    delay(500);
    go_Stop();
    //Едем назад до тех пор пока расстояния с ИК-датчиков не будут удовлетворять условиям
    //Можешь настраивать как тебе необходимо.
    while ((rFront < frontLow) && (lFront < frontLow))
    {
      go_Backward(220, 220);
      irMeasure();
    }
    usMeasure();
    //Едем назад, чтобы и с переднего УЗ-датчика, расстояние было больше необходимого
    while ((usRange < usLow))
    {
      usMeasure();
      go_Backward(220, 220);
    }
    //Поворачиваем робота вправо
    go_RightFull(200, 100);
    //Задержка обуславливает длительность поворота
    delay(500);
    //Остановка
    go_Stop();
  }
}
//Функция по измерению расстояния с УЗ-датчика
//В трех пололожениях. По центру, крайнее правое и крайнее левое.
void usRadar()
{
  radar.write(radarRight);
  delay(250);
  usMeasure();
  usRangeRight = usRange;
  radar.write(radarLeft);
  delay(250);
  usMeasure();
  usRangeLeft = usRange;
}

//Функция для мотордрайвера по движению робота прямо
//A-левый
//B-правый
int go_Forward(int a, int b)
{
  //Левый мотор
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  //Настройка скорости вращения мотора, используя ШИМ (от 0 до 255)
  analogWrite(enA, a);

  //Правый мотор
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  //Настройка скорости вращения мотора, используя ШИМ (от 0 до 255)
  analogWrite(enB, b);
}

//Функция, которая поворачивает влево
//Правые колеса едут на вперед
//Левые колеса едут на назад
int go_LeftFull(int a, int b)
{
  //Левый мотор
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, a);

  //Правый мотор
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, b);
}
// Функция легкого поворота влево.
//Просто замедляется скорость вращения левых колес
void go_LeftHalf(int a, int b)
{
  //Левый мотор
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, a);

  //Правый мотор
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, b);
}

//Функция, которая поворачивает вправо
//Левые колеса едут вперед
//Правые колеса едут назад
void go_RightFull(int a, int b)
{
  //Левый мотор
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, a);

  //Правый мотор
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, b);
}
// Функция легкого поворота вправо
//Просто замедляется скорость вращения правых колес
void go_RightHalf(int a, int b)
{
  //Левый мотор
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, a);

  //Правый мотор
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enB, b);
}
//Функция по движению назад
int go_Backward(int a, int b)
{
  //Левый мотор
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, a);

  //Правый мотор
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, b);
}
void go_Stop()
{
  //Левый мотор
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 200);

  //Правый мотор
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enB, 200);
}

//***********************Начало Настройка Bluetooth. Ничего не трогать тут.***********************//
BLEPeripheral blePeripheral;  // BLE Peripheral Device (the board you're programming)
BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEUnsignedCharCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
//***********************Конец Настройка Bluetooth. Ничего не трогать тут.***********************//

// Функция setup() выполняется каждый раз, когда будет перезапущена подача питания
// или будет произведена перезагрузка платы
void setup() {
  //Можно инициализровать Serial чтобы производить отладку.
  Serial.begin(9600);

//***********************Начало Настройка Bluetooth. Ничего не трогать тут, кроме имени робота***********************//
//Поменять имя робота в строке ниже. Написать тот номер, который будет выдан участнику
  // set advertised local name and service UUID:
  blePeripheral.setLocalName("1");
  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());

  // add service and characteristic:
  blePeripheral.addAttribute(ledService);
  blePeripheral.addAttribute(switchCharacteristic);

  // set the initial value for the characeristic:
  switchCharacteristic.setValue(0);

  // begin advertising BLE service:
  blePeripheral.begin();
//***********************Конец Настройка Bluetooth***********************//
  //Установка сервопривода в центральное положение
  radar.write(radarFront);
  //Инициализация пинов УЗ-датчика на входи и на выход
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  delay(5);
  //Инициализация пинов мотордрайвера на выход
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  //Для отладки можно еще настроить встроенный светодиод на 13 пине
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
//***********************Начало Настройка IMU робота. Ничего не трогать тут***********************//
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize device
  Serial.println("Initializing IMU device...");
  CurieIMU.begin();

  // verify connection
  Serial.println("Testing device connections...");
  if (CurieIMU.begin()) {
    Serial.println("CurieIMU connection successful");
  } else {
    Serial.println("CurieIMU connection failed");
  }

  // use the code below to calibrate accel/gyro offset values
  Serial.println("Internal sensor offsets BEFORE calibration...");
  Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
  Serial.print("\t"); // -76
  Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
  Serial.print("\t"); // -235
  Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
  Serial.print("\t"); // 168
  Serial.print(CurieIMU.getGyroOffset(X_AXIS));
  Serial.print("\t"); // 0
  Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
  Serial.print("\t"); // 0
  Serial.println(CurieIMU.getGyroOffset(Z_AXIS));

  Serial.println("About to calibrate. Make sure your board is stable and upright");
  //Начало калибровки акселерометра-гироскопа робота. 
  //Не трогай робота после загрузки скетча либо включения питания, до тех пор пока не зажжется маленький зеленый светодиод на плате.
  delay(2000);
  // The board must be resting in a horizontal position for
  // the following calibration procedure to work correctly!
  Serial.print("Starting Gyroscope calibration and enabling offset compensation...");
  CurieIMU.autoCalibrateGyroOffset();
  Serial.println(" Done");

  Serial.print("Starting Acceleration calibration and enabling offset compensation...");
  CurieIMU.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  CurieIMU.autoCalibrateAccelerometerOffset(Z_AXIS, 1);
  Serial.println(" Done");

  Serial.println("Internal sensor offsets AFTER calibration...");
  Serial.print(CurieIMU.getAccelerometerOffset(X_AXIS));
  Serial.print("\t"); // -76
  Serial.print(CurieIMU.getAccelerometerOffset(Y_AXIS));
  Serial.print("\t"); // -2359
  Serial.print(CurieIMU.getAccelerometerOffset(Z_AXIS));
  Serial.print("\t"); // 1688
  Serial.print(CurieIMU.getGyroOffset(X_AXIS));
  Serial.print("\t"); // 0
  Serial.print(CurieIMU.getGyroOffset(Y_AXIS));
  Serial.print("\t"); // 0
  Serial.println(CurieIMU.getGyroOffset(Z_AXIS));
  digitalWrite(13, HIGH);
//***********************Конец Настройка IMU робота***********************//
}

void readIMU()
{
  // read raw data from CurieIMU
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

  // convert from raw data to gravity and degrees/second units
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  // update the filter, which computes orientation
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // print the heading, pitch and roll
  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();
  Serial.print("Orientation: ");
  Serial.print(heading);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);

}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

//Главный бесконечный цикл, в котором находятся управляющие операторы и циклы
//В роботе четыре колеса без резиновых колец
void loop() {

//***********************Начало подключения Bluetooth. Ничего не трогать тут. ***********************//
  // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();
  // if a central is connected to peripheral:
  if (central) {
    // Serial.print("Connected to central: ");
    // print the central's MAC address:
    // Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (switchCharacteristic.written()) {

        while (1)
        {
          if (switchCharacteristic.value())
          {
//***********************Конец подключения Bluetooth. Ничего не трогать тут.***********************//

            //***********************Начало кода по движению робота. Ниже пиши свой код по управлению роботом**********************//

            //Вызов функции по измерению расстояний с ИК-датчиков
            //Необходимо всегда ее вызывать, чтобы обновлялись значения
            irMeasure();

            //Вызов функции по измерению расстояний с УЗ-датчика
            //Необходимо всегда ее вызывать, чтобы обновлялись значения
            usMeasure();

            //Вызов функции по чтению данных с акселерометра-гироскопа.
            //После вызова функции обновляются все переменные данной функции. См. функцию readIMU()
            //Необходимо всегда ее вызывать, чтобы обновлялись значения
            readIMU();
            //Все функции вызываний повторно, когда хочешь обновить данные и получить новые значения для нового положения робота

            //Принятие решения на основе показаний с ИК-датчиков
            //УЗ-датчик в алгоритме здесь не используется. Подключи сам:)
            
            //Едем вперед по двум ИК-датчикам, если (с правого датчика расстояние ниже верхнего порога либо с левого датчика расстояние ниже верхнего порога) 
            //либо с правого и левого датчика расстояние ниже верхнего порога, то переходим на следующий уровень 
            if (((rFront <= 60) || (lFront <= 60)) || ((rFront <= 60) && (lFront <= 60)))
            {
              //Если с правого и левого ИК-датчика расстояния меньше нижнего порога, 
              //то считаем, что робот врезался в стену и вызываем функцию по отъезду от препятствия
              if ((rFront <= 30) && (lFront <= 30))
              {
                //Функция по выезду назад от препятствия. В функции определяется в какую именно сторону выезжать. 
                //В функции можно изменять, силу с которой робот поворачивает
                irFrontGoBack();
              }
              else
              {
                //Если с правого датчика расстояние меньше, чем с левого
                if (rFront <= lFront)
                {
                  //Если расстояние с правого датчика больше 40, но меньше 60
                  //то плавно поворачиваем влево. Замедляем левые колеса, они вращаются вперед, до скорости 50 (минимум 0 стоп-максимум 255)
                  //Правые колеса вращаются вперед на скорости 220 (минимум 0 стоп-максимум 255)
                  if (rFront >= 40)
                  {
                    go_LeftHalf(50, 220);
                  }
                  //Если расстояние с правого датчика меньше 40, но больше 30
                  //то резко поворачиваем влево. Вращаем левые колеса в обратную сторону на скорости 50 (минимум 0 стоп-максимум 255), 
                  //правые колеса вращаются вперед на скорости 220 (минимум 0 стоп-максимум 255). Танковый-резкий поворот.
                 else
                  {
                    go_LeftFull(50, 220);
                  }
                }
                if (lFront < rFront)
                {
                  //Делаем все тоже самое, что и выше по поворотам, только наоборот.
                  if (lFront >= 40)
                  {
                    go_RightHalf(220, 50);
                  }
                  else
                  {
                    go_RightFull(220, 50);
                  }
                }
              }
            }
            else
            {
              //Если на обоих датчиках расстояние больше FrontLow, то надо определить не едет ли робот под горку
              //Если pitch тангаж робота отрицательный -5 градусов, то робот едет на горку, поддадим жару в котлы! Эгегегей!
              if (pitch <= -5)
              {
                //Больше ЖОГОВА!
                go_Forward(250, 250);
              }
              else
              {
                 //Если pitch тангаж робота положительный 5 градусов, то робот едет с горки, уменьшим скорость.
                if (pitch >= 5)
                {
                  go_Forward(100, 100);
                }
                else
                {
                  //В любом другом случае pitch тангаж робота находится в пределах от -5 до 5 градусов, считаем, что робот едет по ровной поверхности
                  go_Forward(220, 220);
                }
              }
            }

            //***********************Конец кода по движению робота. Ниже не пиши свой код**********************//
          }
          else {
            //Если ты отправил по Bluetooth  команду на остановку робота, то останавливаем робота
            go_Stop();
          }
        }
      }
      // when the central disconnects, print it out:
      Serial.print(F("Disconnected from central: "));
      Serial.println(central.address());
    }
  }
}

