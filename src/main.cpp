/*
Design Synthesis.net -r.young 6/3/2019 v1.0
Skylight Sub-Controller [LSCTRL1 / LSCTRL2]
Control two motors with potentiometer limits
UP and DOWN limit switches for damage control
(-----------------------------------------------------------)
UnitName = LFCTRL1
Controls Motor 1-2
(-----------------------------------------------------------)
*/
#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Automaton.h>

#define motorPin1   2
#define motorPin2   3
#define dirPin      4
//  RED WIRE COMMON HIGH NO
#define upLimitPin1    6 //BLUE
#define downLimitPin1  5 //YELLO
#define upLimitPin2    7 //BLUE
#define downLimitPin2  8 //YELLOW

#define potPin1   A0
#define potPin2   A1
#define ledPin 9
// M1 ------------------------------
const int UpperLimit = 445;
const int LowerLimit = 65;
// M2 -----------------------------
const int UpperLimit2 = 445;
const int LowerLimit2 = 65;
//  --------------------------------
int action = 0;
// Automaton Objects ----------------------------------------
Atm_button upSwitch1;
Atm_button downSwitch1;
Atm_button upSwitch2;
Atm_button downSwitch2;

// State Machine functions for Potentiometer--
Atm_analog pot1;
Atm_analog pot2;
// Controller machine for monitoring the Potenciometers

Atm_led statusLED;

const char* mqtt_mqttServer = "192.198.10.22";
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.
// gateway and subnet are optional:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xEE
};
IPAddress ip(192, 168, 10, 161);
IPAddress myDns(192, 168, 10, 199);
IPAddress gateway(192, 168, 10, 199);
IPAddress subnet(255, 255, 255, 0);
IPAddress mqttServer (192, 168, 10, 22);

EthernetClient ethclient;
PubSubClient client(ethclient);

unsigned long previousMillis;
unsigned long polling_interval = 2000;
int position = 0;

Atm_led motor1;
Atm_led motor2;

uint16_t avgPot1Buffer[16];
uint16_t avgPot2Buffer[16];


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  
  Serial.println();

   if ((char)payload[0] == '0') {
   
    Serial.println("MQTT_STOP");
    action=0;
    motor1.trigger(motor1.EVT_OFF);
    motor2.trigger(motor2.EVT_OFF);
    client.publish("STATUS", "10");
    client.publish("STATUS", "20");
    
    digitalWrite(dirPin,LOW);
      
  } 

  if ((char)payload[0] == '1') {
   
    Serial.println("MQTT_CLOSING");
    action = 1;
    motor1.trigger(motor1.EVT_ON);
    motor2.trigger(motor2.EVT_ON);
    client.publish("STATUS", "13");
    client.publish("STATUS", "23");
    digitalWrite(dirPin,LOW);
     
  } 

  if ((char)payload[0] == '2') {

    Serial.println("MQTT_OPEN");
   
    action = 2;
    motor1.trigger(motor1.EVT_ON);
    motor2.trigger(motor2.EVT_ON);
    client.publish("STATUS", "13");
    client.publish("STATUS", "23");
    digitalWrite(dirPin,HIGH);
    
  }
  

}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("LF1")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("STATUS", "15");
      client.publish("STATUS", "25");

      // ... and resubscribe
      client.subscribe("SIGNAL");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
 

void pot1_callback( int idx, int v, int up ) {
 
  
  if (v < LowerLimit  && action==1)
  { 
  
    motor1.trigger(motor1.EVT_OFF);
    client.publish("STATUS", "11");

   }else if(v > UpperLimit && action==2){
     motor1.trigger(motor1.EVT_OFF);
     client.publish("STATUS", "12");
     
   }
  
}
void pot2_callback( int idx, int v, int up ) {
 
  if (v < LowerLimit2  && action==1)
  {  
   motor2.trigger(motor2.EVT_OFF);
   motor2.trace(Serial);
    client.publish("STATUS", "21");
   }else if(v > UpperLimit2 && action==2){
     motor2.trigger(motor2.EVT_OFF);
     client.publish("STATUS", "22");
   }
  
}

void setup() {
  // Directional PIN ------------------------------------------
  pinMode(dirPin,OUTPUT);
  // Limit Switches Init ------------------------------------------
  downSwitch1.begin(downLimitPin1).onRelease(motor1,motor1.EVT_OFF);
  //downSwitch1.trace(Serial);
  upSwitch1.begin(upLimitPin1).onRelease(motor1,motor1.EVT_OFF);
  //upSwitch1.trace(Serial);
  //---------------------------------------------------------------
  upSwitch2.begin(upLimitPin2).onRelease(motor2, motor2.EVT_OFF);
  //upSwitch2.trace(Serial);
  downSwitch2.begin(downLimitPin2).onRelease(motor2, motor2.EVT_OFF);
  //downSwitch2.trace(Serial);
  //--------------------------------------------------------------
  pot1.begin(potPin1,50)
      .average(avgPot1Buffer, sizeof(avgPot1Buffer))
        .onChange(pot1_callback);
  pot2.begin(potPin2,50)
      .average(avgPot2Buffer, sizeof(avgPot2Buffer))
          .onChange(pot2_callback);
  // -------------------------------------------------------------
  // Motors Controls
  motor1.begin(motorPin1);
  motor2.begin(motorPin2);// Throttle back dominant motor-Good Luck!?!

  Serial.begin(9600);
  // print your local IP address: 
  // Allow the hardware to sort itself out
  delay(1500);

  client.setServer(mqttServer, 1883);
  client.setCallback(callback);
  Ethernet.begin(mac, ip);
  Serial.println(Ethernet.localIP());
}

void loop() {
  
  if (!client.connected()) {
    reconnect();
  }
 
  client.loop();
  automaton.run();

  unsigned long currentMillis = millis();
  // Main Utility Task Loop
  if(currentMillis - previousMillis > polling_interval) {  
    previousMillis = currentMillis;  
    Serial.print("M1 -");
    Serial.println(pot1.state());
    Serial.println("---------");
    Serial.print("M2 -");
    Serial.println(pot2.state());
    // Serial.print("Action = ");
    // Serial.println(action);
    // if (action==1 || action==2)
    // {
    //   char potvalue[2] ;
    //   sprintf(potvalue,"%d",pot1.state());
    //   client.publish("LF1", potvalue);
    //   sprintf(potvalue,"%d",pot2.state());
    //   client.publish("LF2", potvalue);
    // }
    
  }


}


