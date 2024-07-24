#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>
float Vsupply = 3.3; //power supply voltage (3.3 V rail) -STM32 ADC pin is NOT 5 V tolerant
float Vout; //Voltage divider output
float R_NTC; //NTC thermistor resistance in Ohms
float R_10k = 98400; //10k resistor measured resistance in Ohms (other element in the voltage divider)
float B_param = 3700; //B-coefficient of the thermistor
float T0 = 298.15; //25°C in Kelvin
float Temp_K; //Temperature measured by the thermistor (Kelvin)
float Temp_C; //Temperature measured by the thermistor (Celsius)
const int VoutPin = PB1 ; //ADC0 pin of STM32
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  pinMode(PB15, OUTPUT);
 display.begin(SSD1306_SWITCHCAPVCC, 0x3c); // initialize with the I2C addr 0x3D (for the 128x64)
  Serial.begin(9600);
}
// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  Vout = analogRead(VoutPin)* (3.3/4095); //4095 ' 12 bit resolution of the blue pill
  Serial.print("vout: ");
  Serial.println(Vout);
  R_NTC = (Vout * R_10k) /(Vsupply - Vout); //calculating the resistance of the thermistor
  Serial.print("RNTC: ");
  Serial.println(R_NTC);
  Temp_K = (T0*B_param)/(T0*log(R_NTC/R_10k)+B_param); //Temperature in Kelvin
  Temp_C = Temp_K - 273.15; //converting into Celsius
  Serial.print("Temp : ");
  Serial.println(Temp_C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(10,0);
  display.println("TEMPERATURE:");
  display.setCursor(80,50);
  display.println("Celsius");
  display.display();
  //delay(2000);
  //display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(WHITE);
  display.setCursor(10,20);
  display.print(Temp_C);
  display.display();
  delay(500);
  if (Temp_C >= 37) {
    digitalWrite(PB15, HIGH);
    delay(500);
  } 
  else {
    digitalWrite(PB15, LOW);
    delay(500);
  }
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V)
  // print out the value you read:
  // print out the value you read:
  Serial.println(Temp_C);

}