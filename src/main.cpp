/*
Design Synthesis.net -r.young 7/10/2019 v1.3
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



#define motorPin1   9
#define motorPin2   3
#define dirPin      4
//  RED WIRE COMMON HIGH NO
#define upLimitPin1    6 //BLUE
#define downLimitPin1  5 //YELLO
#define upLimitPin2    7 //BLUE
#define downLimitPin2  8 //YELLOW

#define potPin1   A0    // Setpoint Master
#define potPin2   A1    // Slave output
#define ledPin    A3    // Status All in One LED
#define csPin     A4    // Current Sense Pin
// Debugging Flags
#define debug     1
#define mqtt      1
#define switches  1

// M1 ------------------------------
const int UpperLimit = 412;
const int LowerLimit = 95;
// M2 -----------------------------
const int UpperLimit2 = 392;
const int LowerLimit2 = 60;
//  --------------------------------
int action = 0;
// Automaton Objects ----------------------------------------
Atm_button upSwitch1;
Atm_button downSwitch1;
Atm_button upSwitch2;
Atm_button downSwitch2;
// State Flags ------------------------------------------------------------------------------
bool LF1_UP{false};
bool LF1_DOWN{false};
bool LF2_UP{false};
bool LF2_DOWN{false};

// State Machine functions for Potentiometer--
Atm_analog pot1;  //Set-Point
Atm_analog pot2;  //Slave
Atm_analog cs;
// Controller machine for monitoring the Potenciometers

Atm_led statusLED;

const char* mqtt_mqttServer = "192.198.10.22";
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
unsigned long polling_interval = 500;
int position = 0;

Atm_led motor1; //Master
Atm_led motor2; //Slave
long BREAKVALUE = 160;

uint16_t avgPot1Buffer[16];
uint16_t avgPot2Buffer[16];

long PUSHCORRECTION = 2.8;
long PULLCORRECTION = 0.14;


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  
  Serial.println();

   if ((char)payload[0] == '0') {
   #if debug ==1
    Serial.println("MQTT_STOP");
    #endif
    action=0;
    motor1.trigger(motor1.EVT_OFF);
    motor2.trigger(motor2.EVT_OFF);
 
    statusLED.blink(2000,500).trigger(statusLED.EVT_BLINK);
   
    digitalWrite(dirPin,LOW);
      
  } 

  if ((char)payload[0] == '1') {
   #if debug ==1
    Serial.println("MQTT_CLOSING");
    #endif
    action = 1;
    motor1.trigger(motor1.EVT_ON);
    motor2.trigger(motor2.EVT_ON);
    digitalWrite(dirPin,LOW);
    statusLED.blink(1000,1000).trigger(statusLED.EVT_BLINK); 
  } 

  if ((char)payload[0] == '2') {
    #if debug ==1
    Serial.println("MQTT_OPEN");
   #endif
    action = 2;
    motor1.trigger(motor1.EVT_ON);
    motor2.trigger(motor2.EVT_ON);  
    digitalWrite(dirPin,HIGH);
    statusLED.blink(1000,500).trigger(statusLED.EVT_BLINK);     
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
      statusLED.begin(ledPin).blink(500,500).trigger(statusLED.EVT_BLINK);
      
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
    statusLED.blink(2000,500).trigger(statusLED.EVT_BLINK); 
    if(!LF1_DOWN){
      client.publish("STATUS", "11");
      LF1_DOWN=true;
      LF1_UP=false;
    }

   }else if(v > UpperLimit && action==2){
     motor1.trigger(motor1.EVT_OFF);
    
    statusLED.blink(2000,500).trigger(statusLED.EVT_BLINK); 
    if(!LF1_UP){
       client.publish("STATUS", "12");
       LF1_UP=true;
       LF1_DOWN=false;

    }
     
   }
  
}
void pot2_callback( int idx, int v, int up ) {
 
  if (v < LowerLimit2  && action==1)
  {  
   motor2.trigger(motor2.EVT_OFF); 
   statusLED.blink(2000,500).trigger(statusLED.EVT_BLINK); 
   if(!LF2_DOWN){
      client.publish("STATUS", "21");
      LF2_DOWN=true;
      LF2_UP=false;

   }
    
   }else if(v > UpperLimit2 && action==2){
     motor2.trigger(motor2.EVT_OFF);
     statusLED.blink(2000,500).trigger(statusLED.EVT_BLINK); 
     if(!LF2_UP){
      client.publish("STATUS", "22");  
      LF2_UP=true;
      LF2_DOWN=false;

     }
     
   }
  
}

void button_change( int idx, int v, int up ) {
  
  if (pot1.state()< (LowerLimit -2) || pot1.state()> (UpperLimit + 2))
  {
    motor1.trigger( motor1.EVT_OFF );  
    motor2.trigger( motor2.EVT_OFF);  
  }
  if (pot2.state()< (LowerLimit - 2) || pot2.state() > (UpperLimit + 2))
  {
    motor1.trigger( motor1.EVT_OFF );  
    motor2.trigger( motor2.EVT_OFF);  
  }
  
 
  
}

void setup() {
  // Directional PIN ------------------------------------------
  pinMode(dirPin,OUTPUT);
  pinMode(csPin, INPUT);
  // Limit Switches Init ------------------------------------------
  downSwitch1.begin(downLimitPin1).onRelease(button_change);
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
  statusLED.begin(ledPin);

  client.setServer(mqttServer, 1883);
  client.setCallback(callback);
  Ethernet.begin(mac, ip);
  Serial.println(Ethernet.localIP());
}

void loop() {
  #if mqtt == 1
  if (!client.connected()) {
    reconnect();
  }
 
  client.loop();
  #endif
  automaton.run();

  unsigned long currentMillis = millis();
  // Main Utility Task Loop
  if(currentMillis - previousMillis > polling_interval) {  
    previousMillis = currentMillis;  

    long pos1 = pot1.state() ;
    long pos2 = pot2.state(); 
    long err = pos1 - pos2;

    #if debug ==1
    Serial.print("M1 -");
    Serial.println(pot1.state());
    Serial.print("M2 -");
    Serial.println(pot2.state());
    Serial.println("---------");
    
  
    Serial.print("Error ->");
    Serial.println(err);
    Serial.print("Action");
    Serial.println(action);
    #endif
    
    if (err > 0)   // Error is positive number--
    {
      if (action==2)
      {
        #if debug ==1
        Serial.print("Up > POS : ");
        Serial.println("PUSH 2");
        #endif
        // need to make function that reduces output the larger the error
        motor2.brightness(255 - (abs(err) * PULLCORRECTION) );
        motor1.brightness(255 - (abs(err) * PUSHCORRECTION) );
      }
      if (action==1)
      {
        #if debug ==1
        Serial.print("Down>POS : ");
        Serial.println("PUSH 1");
        #endif
        motor2.brightness(255 - (abs(err) * PUSHCORRECTION));
        motor1.brightness(255 - (abs(err) * PULLCORRECTION));
      }
    }

    if (err < 0)   // Error is negative number--
    {
      if (action==2)
      {
        #if debug ==1
        Serial.print("Up >NEG: ");
        Serial.println("PUSH 2");
        #endif
        motor2.brightness(255 - (abs(err) * PUSHCORRECTION));
        motor1.brightness(255 - (abs(err) * PULLCORRECTION));
      }
      if (action==1)
      {
        #if debug ==1
        Serial.print("Down >NEG : ");
        Serial.println("PUSH 1");
        #endif
        motor2.brightness(255 - (abs(err) * PULLCORRECTION) );
        motor1.brightness(255 - (abs(err) * PUSHCORRECTION));
      }
    }
    
  }


}


