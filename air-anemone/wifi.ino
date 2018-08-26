/* Create a WiFi access point and provide a web server on it. */



// Causes a compile error, documented here: https://github.com/esp8266/Arduino/issues/398





#ifndef DO_WIFI

void setupWifi() {}
void loopWifi() {}

#else

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

/* Set these to your desired credentials. */
const char *ssid = "air-triffid";
//const char *password = "thereisnospoon";

ESP8266WebServer server(80);

/* Status message.  Go to http://192.168.4.1 in a web browser
   connected to this access point to see it.
*/
void handleRoot() {
  String s = "<h1>Air Triffid</h1>";
  
  server.send(200, "text/html", s);
}

void setupWifi() {
  delay(1000);
  Serial.begin(115200);
  Serial.println();
  Serial.print("Configuring access point...");
  /* You can remove the password parameter if you want the AP to be open. */
  //WiFi.softAP(ssid, password);
  WiFi.softAP(ssid);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started");
}

void loopWifi() {
  server.handleClient();
}

#endif // DO_WIFI
