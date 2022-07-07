/* This code for PP22 Device, An Air Quality Monitoring System for Indonesia Goverment.
 *  The main propose of this project is : To make Modbus RS485 >>> Modbus TCP/IP 
 *  In Some Project i have found :https://github.com/budulinek/arduino-modbus-rtu-tcp-gateway, this project same like this. But the different are :  
 *  
 *  1. We use STM32F7 as a gateway,
 *  2. We use different Ethernet Chip  
 *  3. Gate way with additional sensor collecting like adc, spi, i/o etc. 
 *  So i hope may Allah help this project to be done with good ending. Aamin  
 *  Wassalam 
 *  Best Regard 
 *  -M. Darfyma Putra-
 *  06-Jully-2022
 */
#include <LwIP.h>
#include <STM32Ethernet.h>
//#include <ArduinoRS485.h> 
#include <ArduinoModbus.h>

HardwareSerial Serial1(PD6, PD5);//Hardware Serial STm32 Use Pin PD6 as RX and PD5 as TX

//Modbus TCP Peripheral
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 7, 20);
IPAddress myDns (192, 168, 7, 1);
IPAddress gateway (192, 168, 7, 1);
IPAddress subnet (255, 255, 0, 0);

EthernetServer ethServer(502);

ModbusTCPServer modbusTCPServer;

//As a Analog Input 
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached in the right 
const int analogInPin2 = A1;  // Analog input pin that the potentiometer is attached in midle position
const int analogInPin3 = A2;  // Analog input pin that the potentiometer is attached in left position

//As a Coil(Digital) or Analog Output(PWM)
const int DigitalOutPin = PB0; // Analog output pin that the LED is YELOW
const int DigitalOutPin2 = PB7; // Analog output pin that the LED is BLUE
const int DigitalOutPin3 = PB14; // Analog output pin that the LED is RED

// As a Button or Discrete Input
const int DigitalInputPin = PC13; // User Button
const int DigitalInputPin2 = PF15;//D2 Jumper No.1
const int DigitalInputPin3 = PE13;//D3 Jumper No.2 
const int DigitalInputPin4 = PF14;//D4 Jumper No.3
const int DigitalInputPin5 = PE11;//D5 Jumper No.4
 
void setup() {  
  Serial.begin(9600);
  while (!Serial) {;                                        //wait serial port connect. Need Native USB Port Only
  }
  Ethernet.begin(mac, ip, subnet, gateway, myDns);                                  //Start The Ethernet connection and the server
  Serial.print("Ethernet Modbus TCP-Server Start with IP: ");
  Serial.println(Ethernet.localIP());
/*
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
  Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
  while (true) {
    delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
*/
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");  
  }
  
  //Start The Server
  ethServer.begin();
  if (!modbusTCPServer.begin()) {
  Serial.println("Failed to start Modbus TCP Server!");
  while (1);
  }

  // put your setup code here, to run once:
  pinMode(DigitalOutPin, OUTPUT);                           //YELOW
  pinMode(DigitalOutPin2, OUTPUT);                          //BLUE
  pinMode(DigitalOutPin3, OUTPUT);                          //RED
  
  pinMode(DigitalInputPin,INPUT);                            //User Button
  pinMode(DigitalInputPin2, INPUT_PULLUP);                  //Jumper No.1
  pinMode(DigitalInputPin3, INPUT_PULLUP);                  //Jumper No.2
  pinMode(DigitalInputPin4, INPUT_PULLUP);                  //Jumper No.3
  pinMode(DigitalInputPin5, INPUT_PULLUP);                  //Jumper No.4

  //Resgister Config
  modbusTCPServer.configureCoils(0x00,3);             //in this case we use 3 LEDS =YELOW, BLUE, RED 
  modbusTCPServer.configureDiscreteInputs(0x00,5);    //button usr, jumper No.1, Jumper No.2, Jumper No.3, Jumper No.4.
  modbusTCPServer.configureInputRegisters(0x00,3);    //in this case we config 3 register for store the ADC value(AnalogPin-AnalogPin3) 
  modbusTCPServer.configureHoldingRegisters(0x00,3);  
}
void loop() 
{
  // put your main code here, to run repeatedly:
  EthernetClient client = ethServer.available();

  if (client)
  {
  Serial.println("New Client");
  modbusTCPServer.accept(client);
    while(client.connected())
    {
    modbusTCPServer.poll();
    updateCoil();
    updateButton();
    updateADC();
    updateHolding();
    }
    Serial.println("Client Disconnected");
  }

}

void updateCoil() 
{
  // read the current value of the coil
  int coilValue = modbusTCPServer.coilRead(0x00);
  int coilValue2 = modbusTCPServer.coilRead(0x01);
  int coilValue3 = modbusTCPServer.coilRead(0x02);

  //Push On Method  ---> True When you only first touch, and False if you touch again.
  if (coilValue) {
    // coil value set, turn LED on
    digitalWrite(DigitalOutPin, HIGH);
  } else {
    // coild value clear, turn LED off
    digitalWrite(DigitalOutPin, LOW);
  }
  if (coilValue2) {
    // coil value set, turn LED on
    digitalWrite(DigitalOutPin2, HIGH);
  } else {
    // coild value clear, turn LED off
    digitalWrite(DigitalOutPin2, LOW);
  }
  if (coilValue3) {
    // coil value set, turn LED on
    digitalWrite(DigitalOutPin3, HIGH);
  } else {
    // coild value clear, turn LED off
    digitalWrite(DigitalOutPin3, LOW);
  }
}

void updateButton()
{
  modbusTCPServer.discreteInputWrite(0x00,(digitalRead(DigitalInputPin)));
  modbusTCPServer.discreteInputWrite(0x01,(!digitalRead(DigitalInputPin2)));
  modbusTCPServer.discreteInputWrite(0x02,(!digitalRead(DigitalInputPin3)));
  modbusTCPServer.discreteInputWrite(0x03,(!digitalRead(DigitalInputPin4)));
  modbusTCPServer.discreteInputWrite(0x04,(!digitalRead(DigitalInputPin5)));
}

void updateADC()
{
  modbusTCPServer.inputRegisterWrite(0x00,(analogRead(analogInPin)) ); 
  modbusTCPServer.inputRegisterWrite(0x01,(analogRead(analogInPin2)));
  modbusTCPServer.inputRegisterWrite(0x02,(analogRead(analogInPin3)));
}

void updateHolding()
{
  int fromserver = 15000;
  modbusTCPServer.holdingRegisterWrite(0x00,fromserver);
  Serial.print("Holding Reg1(R): ");
  Serial.print(modbusTCPServer.holdingRegisterRead(0x00)); 
  Serial.print(" Holding Reg2: ");
  Serial.print(modbusTCPServer.holdingRegisterRead(0x01)); 
  Serial.print(" Holding Reg3: ");
  Serial.println(modbusTCPServer.holdingRegisterRead(0x02));
}


void notes()
{
  /*
  //No Program 
  // Just In Case want to recycle ol notes
  //Loop Button -->> True when you only push it, and False for Release
  modbusTCPServer.coilWrite(0x03,(digitalRead(DigitalInputPin)));
  modbusTCPServer.coilWrite(0x04,(digitalRead(DigitalInputPin2)));
  modbusTCPServer.coilWrite(0x05,(digitalRead(DigitalInputPin3)));
  modbusTCPServer.coilWrite(0x06,(digitalRead(DigitalInputPin4)));
  modbusTCPServer.coilWrite(0x07,(digitalRead(DigitalInputPin5)));   
  */
  }
