#include <MicroGear.h>
#include <ESP8266WiFi.h>

// constants won't change. They're used here to 
// set pin numbers:
#define D0 16  // USER LED Wake
#define ledPin  D0        // the number of the LED pin

const char* ssid     = "ZWL";
const char* password = "55555555";

#define APPID   "random0"
#define KEY     "OBWwsgWlYGrU3vB"
#define SECRET  "AlC9DrSyTastaEW6CuqwB90kN"

#define ALIAS   "NodeMCU1"
#define TargetWeb "DigitalOUTPUT_HTML_web"


WiFiClient client;
MicroGear microgear(client);

void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) 
{
    Serial.print("Incoming message --> ");
    msg[msglen] = '\0';
    Serial.println((char *)msg);
    digitalWrite(5, HIGH);
    delay(500);
    digitalWrite(5, LOW);
}


void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) 
{
    Serial.println("Connected to NETPIE...");
    microgear.setAlias(ALIAS);
}

void setup() 
{
    pinMode(5, OUTPUT);
     /* Event listener */
    microgear.on(MESSAGE,onMsghandler);
    microgear.on(CONNECTED,onConnected);

    Serial.begin(115200);
    Serial.println("Starting...");

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) 
    {
       delay(250);
       Serial.print(".");
    }

    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    microgear.init(KEY,SECRET,ALIAS);
    microgear.connect(APPID);
}

void loop() 
{
    
    if (microgear.connected())
    {
       microgear.loop();
       //Serial.println("connected");
      // String data = "/" + String(a) + "/" + String(b);
      microgear.chat(TargetWeb, "300/300/300/300/0");
      if(Serial.available()>0){
        Serial.println(String(Serial.readStringUntil('\n')));
      }
    }
   else 
   {
    Serial.println("connection lost, reconnect...");
    microgear.connect(APPID);
   }
    
}
