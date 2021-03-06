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
    Название набора: РобоРовер М1 4WD CodeWheels Edition (M1-02)
    Программа создана и протестирована разработчиком:
    Имя: Максим Массальский
    Ник: maxxlife
    E-mail: maxxlife@mrobot.by
*/

/*В роботе используются пины (для Arduino 101 платы)
 * Мотордрайвер использует пины:
 * D2, D3, D5, D8, D10, D11
 * Сервопривод использует пины:
 * D9
 * Ультразвуковой датчик использует пины:
 * D4, D7
 * Свободные цифровые пины:
 * D0, D1, D6, D12, D13
 * Аналоговые ИК-датчики используют пины:
 * A0, A1
 * Свободные аналоговые пины:
 * A2, A3, A4, A5
 */
 
#include <Servo.h>
//Создаем экземпляр класса Servo под именем radar
Servo radar;

//Пины ультразвукового датчика
int trigPin = 7;
int echoPin = 4;

//Константа voltconst -это значение опорного напряжения в милливольтах
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
int radarLeft = 120;
//Значение для правого положения
int radarRight = 250;

// Ультразвуковой сенсор
//Переменная по хранению расстояния с УЗ-датчика
volatile long usRange = 0;
//Максимальный порог
int usHigh = 100;
//Минимальный порог
int usLow = 40;
volatile long usRangeLeft = 0;
volatile long usRangeRight = 0;

// Датчики инфракрасные Передние
//double angleFront=cos((30*PI)/180);// Угол 30 градусов
float angleFront = 1;
//Левый ИК-датчик подключен к аналоговому пину 0
int irPinFrontLeft = 0;
//Правый ИК-датчик подключен к аналоговому пину 1
int irPinFrontRight = 1;

//Переменные по настройке алгоритма передних ИК-датчиков под условия и расстояния
//Максимальный порог
int frontHigh = 40;
//Минимальный порог
int frontLow = 20;

void rotateServo(int servoValue)
{
  if(servoValue>130)
  servoValue=130;
  if(servoValue<50)
  servoValue=50;
  
  int angle=radar.read();
    if(angle<servoValue)
    {
        for(angle;angle<servoValue;angle++)
        {
             radar.write(angle);
             delay(20);
        }
    }
    if(angle>servoValue)
    {
           for(angle;angle>servoValue;angle--)
        {
             radar.write(angle);
             delay(20);
        }
    }
    if(angle==servoValue)
        radar.write(angle);
  
}

//Класс для измерения расстояния с ИК-датчиков робота
class irSharpMeasure
{
  public:
    int irMeasure(int irPin)
    {
      int distance;
      // Измеряем значение с датчика в милливольтах.
      //Значение, которое было считано с аналогового порта умножается на переменную voltconst
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
          //Умножив на anglefront
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
        distance = 6* angleFront;
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

// Функция setup() выполняется каждый раз, когда будет перезапущена подача питания
// или будет произведена перезагрузка платы
void setup() {
  //Инициализация Монитора последовательного порта Serial на скорости 9600 бод
  Serial.begin(9600);
  //Установка сервопривода в центральное положение
  radar.attach(9);
  rotateServo(radarFront);
  delay(5);
  //Инициализация пинов УЗ-датчика на входи и на выход
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  delay(1000);
}

void loop() {
 usMeasure();
  Serial.print("Front Ultrasonic (US) sensor, distance ");
  Serial.print(usRange);
  Serial.println(" cm");

  irMeasure();
  Serial.print("Left Infared (IK) sensor, distance ");
  Serial.print(lFront);
  Serial.println(" cm");

  Serial.print("Right Infared (IK) sensor, distance ");
  Serial.print(rFront);
  Serial.println(" cm");
  
  Serial.println(" ");

  delay(1000);
}






