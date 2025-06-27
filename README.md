# DC-Motor-Speed-Direction-Controller
Controls a DC motor's speed and direction using Arduino and L298N

**components**
Arduino Uno 
L298N Motor Driver 
12V DC Motor
16×2 LCD Display 
Push Buttons (x3)
18650 Battery Holder (2-Cell) 
18650 Battery Cell (3.7V x2) 
On/Off Switch 
10k Variable Resistor
100Ω Resistor
4.7kΩ Resistor
1kΩ Resistor
Breadboard
Jumper Wires 


## 📸 Circuit Diagram
![circuit1](https://github.com/user-attachments/assets/d0893200-4a8c-4d29-bd86-55f0652d023c)

![circuit](https://github.com/user-attachments/assets/eb8c8d6e-cb03-4e03-8bab-607c17ed4646)




**Working Principle**
The DC motor is controlled using an Arduino UNO and an L298N motor driver module.

Speed Control:
A potentiometer is connected to the Arduino’s analog input.
Arduino reads the analog value (0–1023) and maps it to a PWM value (0–255).
This PWM signal is sent to the ENA pin of the L298N to adjust motor speed.

Direction Control:
Three push buttons are used:
One for clockwise rotation.
One for counterclockwise rotation.
One to stop the motor.
Buttons set the logic levels on IN1 and IN2 pins of the L298N to change the motor direction:
IN1 = HIGH, IN2 = LOW → Clockwise.
IN1 = LOW, IN2 = HIGH → Counterclockwise.
Both LOW or HIGH → Motor stops or brakes.
A 16x2 LCD display is connected to the Arduino:
Displays the current duty cycle (PWM value) and motor direction in real-time.
Provides clear user feedback.
The entire system is powered by a battery pack, ensuring portability.
The Arduino handles all control logic, reading user inputs and sending appropriate signals to the motor driver.

**Code**

#include <LiquidCrystal.h>
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

#define potentiometer  A0  //10k Variable Resistor
#define bt_F A1 // Clockwise Button
#define bt_S A2 // Stop Button
#define bt_B A3 // Anticlockwise Button

#define M1_Ena 11 // Enable1 L298 for PWM
#define M1_in1 10 // In1  L298 for Clockwise
#define M1_in2 9  // In2  L298 for Anticlockwise

int read_ADC =0;
int duty_cycle;
int duty_cycle_lcd;
int set = 0;

void setup(){
Serial.begin(9600);// initialize serial communication at 9600 bits per second:

pinMode(potentiometer, INPUT);

pinMode(bt_F, INPUT_PULLUP);
pinMode(bt_S, INPUT_PULLUP);
pinMode(bt_B, INPUT_PULLUP);
lcd.begin(16,2);  
lcd.setCursor(0,0);
lcd.print(" WELCOME To  Our ");
lcd.setCursor(0,1);
lcd.print(“Group Project");
delay(2000); // Waiting for a while
lcd.clear();
}

void loop(){ 
read_ADC = analogRead(potentiometer);
duty_cycle = map(read_ADC, 0, 1023, 0, 255);  
duty_cycle_lcd = map(read_ADC, 0, 1023, 0, 100); 

analogWrite(M1_Ena, duty_cycle);

lcd.setCursor(0,0);
lcd.print("Duty Cycle: ");
lcd.print(duty_cycle_lcd); 
lcd.print("%  ");

if(digitalRead (bt_F) == 0){set = 1;}
if(digitalRead (bt_S) == 0){set = 0;}
if(digitalRead (bt_B) == 0){set = 2;}


lcd.setCursor(0,1);

if(set==0){ lcd.print("      Stop      ");
digitalWrite(M1_in1, LOW);  
digitalWrite(M1_in2, LOW);
}

if(set==1){ lcd.print("    Clockwise   ");
digitalWrite(M1_in1, HIGH);  
digitalWrite(M1_in2, LOW);
}

if(set==2){ lcd.print(" Anticlockwise  ");
digitalWrite(M1_in1, LOW);  
digitalWrite(M1_in2, HIGH);
}

delay(50); 
}
[DC MOTOR SPEED AND DIRECTION CONTROLLER PROJECT REPORT.pdf](https://github.com/user-attachments/files/20942866/DC.MOTOR.SPEED.AND.DIRECTION.CONTROLLER.PROJECT.REPORT.pdf)







