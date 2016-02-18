/* test for sending udp over existing acces point
 * as of jan 2nd 2016
 */

#include <ESP8266WiFi.h> 
#include <WiFiUdp.h>
//#include <ESP8266WebServer.h>

/* Set these to your desired credentials. */
const char *ssid = "TripMateSith-0C78";
const char *password = "11111111";

// flight computer
const char* host = "10.10.10.2"; // cellphone was 2, kobo 4,3
const int port = 4353;

WiFiUDP udp;


void send_packet(char *packetBuffer, int j) {
    udp.beginPacket(host, port);
    udp.write(packetBuffer, j);
    udp.endPacket();
    return;
}

char packetBuffer[128];

void setup() {
        delay(1000);
        Serial.begin(115200);
        /* You can remove the password parameter if you want the AP to be open. */
        
        WiFi.begin(ssid, password);
        int i =0;
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.println(i);
            i++;
        }
        Serial.println("");
        
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        
        //udp
        udp.begin(port);
        Serial.print("Local port: ");
        Serial.println(udp.localPort());
        
        for (int i =0; i < 128; i++) {
            packetBuffer[i] = 0;
        }
}

byte j =0 ;
void loop() {
    int incomingByte;
    if (Serial.available() > 0) {
            incomingByte = Serial.read();
//             Serial.println(incomingByte, DEC);
            packetBuffer[j++] = incomingByte;
            if ((j >0 && incomingByte =='\n') || j == 128) {
                if (j < 128) {
                    packetBuffer[j+1] == 0;
                    Serial.println(packetBuffer);
                }
                send_packet(packetBuffer, j);
                j = 0;
            }
    }
}