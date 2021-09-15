#include <PubSubClient.h>

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <SPI.h>
#include <Ethernet.h>
#include <BlynkSimpleEthernet.h>
#include <Wire.h>
#include <DFRobot_SHT20.h>

DFRobot_SHT20    sht20;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = "14778d10326a41a8bcb96b2265d4c56d";

#define W5100_CS  10
#define SDCARD_CS 4

const char* mqtt_server = "192.168.68.65";

PubSubClient client(EthernetClient);

unsigned long Tempdelay =millis();  //for delaying temp/humidity virtualWrite
unsigned long rainDelay =millis();  //for delaying rain virtualWrite.

 int pir = 0;

 char t2[16];
 char h2[16];
                                  //door sensors
 int Rdoor1down = 0;
 int Rdoor1up = 0;
 int Rdoor2down = 0;
 int Rdoor2up = 0;
                                  //door counters
 int door1 = 1;
 int lastdoor1 = 2;
 int door2 = 1;
 int lastdoor2 = 2;
                                  //PIR counter
 int count = 1;
                                  //Rain sensor state
 bool RainSensor = 0;   
 bool LastRainSensor = 0;  
 int  Debounce = 0;                           

 const int  rainPin = 2;    // the pin that the rain is attached to
 int rainCounter = 0;       // counter for the number of rain pulses
 //int rainState = 0;         // current state of the sensor
 int lastRainState = 0;     // previous state of the button
 int rainYesterday = 0;     //location of yesterdays rainCount
 int Total = 0;             //running rain total counter
 float conversion = 2.8;     //Value to convert rain pulses to mm
 int resetdelay = -60000;    //timer widget reset delay pre set to approx = millis at start up
 int SensorLight = 0;        //SensorLight set to off

void callback(String topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

void reconnect() 
{
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
     /*
     YOU  NEED TO CHANGE THIS NEXT LINE, IF YOU'RE HAVING PROBLEMS WITH MQTT MULTIPLE CONNECTIONS
     To change the ESP device ID, you will have to give a unique name to the ESP8266.
     Here's how it looks like now:
       if (client.connect("ESP8266Client")) {
     If you want more devices connected to the MQTT broker, you can do it like this:
       if (client.connect("ESPOffice")) {
     Then, for the other ESP:
       if (client.connect("ESPGarage")) {
      That should solve your MQTT multiple connections problem

     THE SECTION IN loop() function should match your device name
    */
    if (client.connect("EthernetClient")) {
      Serial.println("connected");  
      // Subscribe or resubscribe to a topic
      // You can subscribe to more topics (to control more LEDs in this example)
      //client.subscribe("homeassistant/switch1");
      //client.subscribe("homeassistant/switch2");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
}
 
void setup()
{

  pinMode(3, INPUT_PULLUP);           //PIR pin

  pinMode(4, OUTPUT);                 //Make output and high for use as pull up resitor
  digitalWrite(4, HIGH);
  pinMode(5, INPUT_PULLUP);           //Roller Door 2 down

  pinMode(6, INPUT_PULLUP);           //Roller Door 2 up

  pinMode(7, OUTPUT);                 //Make output and high for use as pull up resitor
  digitalWrite(7, HIGH);
  
  pinMode(8, OUTPUT);                 //Make output and high for use as pull up resitor
  digitalWrite(8, HIGH);
  
  pinMode(9, INPUT_PULLUP);            //Roller Door 1 down
 
  pinMode(10, INPUT_PULLUP);            //Roller Door 1 up
  
  pinMode(11, OUTPUT);                 //Make output and high for use as pull up resitor
  digitalWrite(11, HIGH);

  pinMode(13, INPUT);                 //Sensor light, from transformer 5v

  pinMode(19, INPUT_PULLUP);          //Pin 19 rain sensor (pull high)
  digitalWrite(19, HIGH);
    
                                       // Debug console
  Serial.begin(9600);

  Serial.println("SHT20 Example!");
    sht20.initSHT20();                                  // Init SHT20 Sensor
    delay(100);
    sht20.checkSHT20();                                 // Check SHT20 Sensor
    
  pinMode(SDCARD_CS, OUTPUT);
  digitalWrite(SDCARD_CS, HIGH);      // Deselect the SD card

  Blynk.begin(auth);
                                      // You can also specify server:
                                      //Blynk.begin(auth, "blynk-cloud.com", 80);
                                      //Blynk.begin(auth, IPAddress(192,168,1,100), 8080);
                 
 }

void loop()
{
  //****************** Sensor Light  ***********************
  if (!client.connected()) {
    reconnect();
  }
  if(!client.loop())
  client.connect("EthernetClient");
  
  SensorLight = digitalRead(13);
  Blynk.virtualWrite(V13, SensorLight);

  client.publish("Garage/SensorLight", SensorLight);
  
  //****************** Temp/Humidity ***********************

   if(millis() > Tempdelay + 2000){    //delay virtualWrite by 2sec
      Tempdelay = millis(); 
  
  float h = sht20.readHumidity();                  // Read Humidity
  float t = sht20.readTemperature();               // Read Temperature
 Serial.println("Test after Read Temp and Hum:");
 Serial.println(t, 1);
  //dtostrf(t, 1, 1, t2);               //convert to 2 dec places
  //dtostrf(h, 1, 1, h2);               //convert to 2 dec places
                                     
  Blynk.virtualWrite(V5, h);
  Blynk.virtualWrite(V6, t);
  
  }
  
//********************* Rain Counter ******************************

if(millis() > rainDelay + 2000){    //delay virtualWrite by 2sec
      RainSensor = digitalRead(19);
  
  if(RainSensor != LastRainSensor){
      Debounce++;
      
  if (Debounce >10){
      rainCounter++;                  // increment rain counter
      LastRainSensor = RainSensor;
      Debounce = 0;
      rainDelay = millis();
      }
     }
    }
    

 if(rainCounter!=lastRainState){
       Blynk.virtualWrite(V20, rainCounter);              //pulses
       Blynk.virtualWrite(V21, rainCounter/conversion);   //convert counter to mm of rain
       lastRainState = rainCounter;
  }
  
//********************* Roller Door *****************************  
                                  //Roller Door 1 is house side Roller door 2 is Middle
                                  //Read the reed switches,create three levels for blynk level display to look like a roller door
                                  //Test for change and send to blynk.
  Rdoor1down = digitalRead(9);   
  Rdoor1up = digitalRead(10);
       
  if (Rdoor1up == LOW){            
  door1 = 1;
  }
  if (Rdoor1down == HIGH && Rdoor1up == HIGH){
  door1 = 2;
  }
  if (Rdoor1down == LOW){            
  door1 = 3;   
  }

  if(door1 != lastdoor1){
  Blynk.virtualWrite(V9, door1);
  lastdoor1 = door1;
  }

  Rdoor2down = digitalRead(5);   
  Rdoor2up = digitalRead(6);
       
  if (Rdoor2up == LOW){            
  door2 = 1;
  }
  if (Rdoor2down == HIGH && Rdoor2up == HIGH){
  door2 = 2;
  }
  if (Rdoor2down == LOW){            
  door2 = 3;   
  }

  if(door2 != lastdoor2){
  Blynk.virtualWrite(V8, door2);
  lastdoor2 = door2;
}                                
//************************************* PIR **************************************                                  
  {
  pir = digitalRead(3);              //read PIR, If it changes send result.wait for chnage
  if (pir == HIGH && count == 0) {
        count = 1;                 
                                                           
        Blynk.virtualWrite(V3, 255);
                
  }
  
  if (pir == LOW && count == 1) {
        count = 0;                           
                              
        Blynk.virtualWrite(V3, 1);
               
  }
  }
    
  Blynk.run();
  }

//************************** end of void loop ********************** 
 
BLYNK_WRITE(V22){
 
      Blynk.virtualWrite(V20, rainCounter);              //pulses
      Blynk.virtualWrite(V21, rainCounter/conversion);   //convert counter to mm of rain
      Blynk.virtualWrite(V24, rainYesterday/conversion); //convert rainYesterday to mm of rain
      Blynk.virtualWrite(V25, Total/conversion);         //convert Total to mm of rain
     rainCounter=0;
 
 }
 BLYNK_WRITE(V23){
 
 int pinValue = param.asInt();
 
 if((pinValue==1)&&(millis() > (resetdelay + 60000))){   //delay 1 min for to allow widget timer to finish
      resetdelay = millis();                             //move todays rainCount to rainYesterday bassed on blynk timer 
      rainYesterday = rainCounter;
      Total = rainCounter + Total;                       //Calc Total
      rainCounter=0;}                                    //reset counter
      Blynk.virtualWrite(V20, rainCounter);              //pulses
      Blynk.virtualWrite(V21, rainCounter/conversion);   //convert counter to mm of rain
      Blynk.virtualWrite(V24, rainYesterday/conversion); //convert rainYesterday to mm of rain
      Blynk.virtualWrite(V25, Total/conversion);         //convert Total to mm of rain
 
 }      
