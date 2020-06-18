#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>;
#include <Servo.h>


#define DHTPIN 2     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
#define buzzerPin 3
#define SERVO_PIN 5
#define RELAY_PIN 4


//declarations
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino
Servo myservo;

//prototypes for the functions
void beep(unsigned char delayms);
boolean isItTime ();
void turnServo();

byte drop[8] = {
  B00000,
  B00100,
  B01110,
  B11111,
  B11111,
  B11111,
  B01110,
};

byte temperature[8] = {
  B00100,
  B01010,
  B01010,
  B01010,
  B10101,
  B10101,
  B01110,
};

byte relogio[8] = {
  B00000,
  B00000,
  B01110,
  B10011,
  B10101,
  B10001,
  B01110,
  B00000
};

byte onOff[8] = {
  B00000,
  B00100,
  B01110,
  B10101,
  B10101,
  B10001,
  B01110,
  B00000
};


// Inicializa o display no endereco 0x27
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3, POSITIVE);

//temperature and humidity
int chk;
float hum;  //Stores humidity value
float temp; //Stores temperature value

//turn from time to time
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long interval = 14400000; //4 hours

//Servo position
int position1Servo = 0;
int position2Servo = 45;
int servoPosition = 0;
 
void setup()
  {
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(buzzerPin, OUTPUT);
    dht.begin();
    myservo.attach(SERVO_PIN);  // attaches the servo on pin SERVO_PIN to the servo object
    myservo.write(0); //INICIA A POSICAO A ZERO
    lcd.begin(16, 2);         // put your LCD parameters here
    lcd.createChar(0, temperature);
    lcd.createChar(1, drop);
    lcd.createChar(2, relogio);
    lcd.createChar(3, onOff);
    Serial.begin(9600);
    myservo.detach();  // detaches the servo
  }

 
void loop()
{  
  hum = dht.readHumidity();
  temp= dht.readTemperature();
  Serial.println("Temperature: " + String(temp));
  Serial.println("Humidity: " + String(hum));
  
  lcd.setCursor(0,0);
  lcd.write((uint8_t)0);
  lcd.print(temp);
  lcd.setCursor(10,0);
  lcd.write((uint8_t)1);
  lcd.print(hum);
  
  if (temp > float(38)) {
    digitalWrite(RELAY_PIN, HIGH);
    delay(1000);
    lcd.setCursor(10,1);
    lcd.write((uint8_t)3);
    lcd.write("DES");
  }

  else {
    digitalWrite(RELAY_PIN, LOW);
    delay(1000);
    lcd.setCursor(10,1);
    lcd.write((uint8_t)3);
    lcd.write("LIG");
  }

  if (hum < 55) {
    beep(1000);
  }
  
  if (isItTime()) {
    turnServo();
  }
  
  delay(1000);
}


void beep(unsigned char delayms) { //creating function
  analogWrite(buzzerPin, 20); //Setting pin to high
  delay(delayms); //Delaying
  analogWrite(buzzerPin ,0); //Setting pin to LOW
  delay(delayms); //Delaying
}

boolean isItTime () {
  unsigned long currentMillis = millis();
  lcd.setCursor(0,1);
  lcd.write((uint8_t)2);
  lcd.print((interval - float(currentMillis - previousMillis))/float(3600000));
  if (currentMillis - previousMillis >= interval) {
    // save the last time you turned it
    Serial.println("Changing millis, TRUE");
    previousMillis = currentMillis;
    return true;
  } else {
    return false;
  }
}

void turnServo() {
  int PositionToTurn;
  myservo.attach(SERVO_PIN);  // attaches the servo on pin SERVO_PIN to the servo object
  myservo.write(servoPosition);
  if (servoPosition > position1Servo) {
    for (; servoPosition >= position1Servo; servoPosition -= 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(servoPosition);              // tell servo to go to position in variable 'pos'
      Serial.println("-1 : " + String(servoPosition));
      delay(100);                       // waits 15ms for the servo to reach the position
    }
  }
else {
     for (; servoPosition <= position2Servo; servoPosition += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(servoPosition);              // tell servo to go to position in variable 'pos'
      Serial.println("+1 : " + String(servoPosition));
      delay(100);                       // waits 15ms for the servo to reach the position
    }
  }
  myservo.detach();  // detaches the servo

 
}
