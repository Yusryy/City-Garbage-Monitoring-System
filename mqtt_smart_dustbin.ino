/***************************************************
  Adafruit MQTT Library Ethernet Example

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Alec Moore
  Derived from the code written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>

#include <Servo.h>

Servo servoMain; // Define our Servo

int trigpin = 5;

int echopin = 6;
const int led = 4;

int distance;


float duration;

float cm;



 int DistrigPin = 7; //triggor pin
int DisechoPin = 9; // echo pin
long timeperiod, inches;

/************************* Ethernet Client Setup *****************************/
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

//Uncomment the following, and set to a valid ip if you don't have dhcp available.
//IPAddress iotIP (192, 168, 0, 42);
//Uncomment the following, and set to your preference if you don't have automatic dns.
//IPAddress dnsIP (8, 8, 8, 8);
//If you uncommented either of the above lines, make sure to change "Ethernet.begin(mac)" to "Ethernet.begin(mac, iotIP)" or "Ethernet.begin(mac, iotIP, dnsIP)"


/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "MHMD_AFKAR"
#define AIO_KEY         "aio_ucpG09Xb5D0nxXFoY7xY4pvPksaM"


/************ Global State (you don't need to change this!) ******************/

//Set up the ethernet client
EthernetClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// You don't need to change anything below this line!
#define halt(s) { Serial.println(F( s )); while(1);  }


/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish Disfeed = Adafruit_MQTT_Publish(&mqtt,  AIO_USERNAME "/feeds/Disfeed");

// Setup a feed called 'onoff' for subscribing to changes.
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/onoff");

/*************************** Sketch Code ************************************/

void setup() {
  Serial.begin(115200);

  Serial.println(F("Adafruit MQTT demo"));

  // Initialise the Client
  Serial.print(F("\nInit the Client..."));
  Ethernet.begin(mac);
  delay(1000); //give the ethernet a second to initialize
  

  mqtt.subscribe(&onoffbutton);

   servoMain.attach(2); // servo on digital pin 10

pinMode(trigpin, OUTPUT);

pinMode(echopin, INPUT);
 pinMode(led, OUTPUT);


   Serial.begin(9600); //serial port communication
    pinMode(DistrigPin, OUTPUT); // defining pinmode for trig
    pinMode(DisechoPin, INPUT);  // defining pinmode for echo pin
}



void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &onoffbutton) {
      Serial.print(F("Got: "));
      Serial.println((char *)onoffbutton.lastread);
    }
  }
  digitalWrite(trigpin, LOW);

delay(2);

digitalWrite(trigpin, HIGH);

delayMicroseconds(5);

digitalWrite(trigpin, LOW);

duration = pulseIn(echopin, HIGH);

cm = (duration/34.82);

distance = cm;

if(distance<30)

{
 digitalWrite(led,HIGH);
servoMain.write(180); // Turn Servo back to center position (90 degrees)

delay(500);

}

else{
   digitalWrite(led,LOW);

servoMain.write(0);

delay(5);

}

digitalWrite(DistrigPin, LOW);// sending 10 us pulse
delayMicroseconds(2);
digitalWrite(DistrigPin, HIGH);
delayMicroseconds(10);
digitalWrite(DistrigPin, LOW);
timeperiod = pulseIn(DisechoPin, HIGH);  // integrating pulse
inches = microsecondsToInches(timeperiod);
cm = microsecondsToCentimeters(timeperiod);
Serial.print("distcance in inches=");
Serial.print(inches);
Serial.print("   distance in centimeters=");
Serial.print(cm);
Serial.println();
delay(10);
if(! Disfeed.publish(cm)){
  Serial.println(F("FAILED"));
}
else{
  Serial.println(F("OK"));
}
delay (2000);

  // ping the server to keep the mqtt connection alive
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }
}
 









long microsecondsToInches(long microseconds)
{
return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds)
{
return microseconds / 29 / 2;
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(3000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}
