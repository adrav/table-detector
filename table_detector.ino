/*
   Copyright (c) 2018, Rafal Bednarz
   All rights reserved.

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

 * * Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

 * * Redistributions in binary form must reproduce the above copyright notice, this
     list of conditions and the following disclaimer in the documentation and/or
     other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

const char wifi_ssid[] = "s4a_bgtd";
const char wifi_password[] = "******";
const uint16_t wifi_port = 80;

const uint8_t pG = 0;
const uint8_t pY = 1;
const uint8_t pR = 2;

const uint8_t ledPIN[3] = { D7, D6, D5 };

const uint8_t pinPIR = D8;

uint32_t ledTimers[3] = {0, 0, 0};
uint32_t ledTimeout[3] = {0, 0, 0}; //how long LED should light

uint32_t firstPIRdetectTime = 0;
uint32_t lastPIRdetectTime = 0;

uint32_t curStateTime = 0;


const uint32_t DEFAULT_HARDSTATE_OCCUPIED = 30; // how many secconds should pass while motion detection to assume that table is occupied
const uint32_t DEFAULT_DELAY_OCCUPIED = 120; // delay after last motion detection. If there is no motion during this delay - table is assumed to be free

uint32_t hardstate_occupied_val = DEFAULT_HARDSTATE_OCCUPIED;
uint32_t delay_occupied_val = DEFAULT_DELAY_OCCUPIED;

enum {
  TABLE_FREE = 0,
  TABLE_OCCUPIED = 1,
  TABLE_CHANGING = 2
};

uint8_t tableState;

char mesgQueue[10][172];
int8_t mesgIndx = -1;
bool enableChat = true;


ESP8266WebServer server ( wifi_port );

Adafruit_BMP280 bmp; // I2C

uint32_t readLongFromEEPROM(uint32_t address) {

  uint32_t four = EEPROM.read(address);
  uint32_t three = EEPROM.read(address + 1);
  uint32_t two = EEPROM.read(address + 2);
  uint32_t one = EEPROM.read(address + 3);

  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);

}

void writeLongToEEPROM(uint32_t address, uint32_t val) {

  uint32_t four = (val & 0xFF);
  uint32_t three = ((val >> 8) & 0xFF);
  uint32_t two = ((val >> 16) & 0xFF);
  uint32_t one = ((val >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);


}

void readFromEEPROM() {

  Serial.println(F("Read from EEPROM"));
  uint8_t crc = EEPROM.read(0);
  Serial.print(crc);
  if (crc != 13) return;

  hardstate_occupied_val = readLongFromEEPROM(1);
  delay_occupied_val = readLongFromEEPROM(1 + 4);

}

void saveToEEPROM() {

  Serial.println(F("Save to EEPROM"));
  EEPROM.write(0, 13);

  writeLongToEEPROM(1, hardstate_occupied_val);
  writeLongToEEPROM(1 + 4, delay_occupied_val);

  EEPROM.commit();

}


bool wifiConnect() {

  uint16_t cnt = 0;

  digitalWrite(ledPIN[pR], 1);


  // Wait for connection
  while ( WiFi.status() != WL_CONNECTED ) {

    digitalWrite(ledPIN[pY], 1);
    delay ( 250 );
    Serial.print ( "." );
    digitalWrite(ledPIN[pY], 0);
    delay ( 250 );

  }

  digitalWrite(ledPIN[pR], 0);

  Serial.println ( "" );
  Serial.print ( "Connected to " );
  Serial.println ( wifi_ssid );
  Serial.print ( "IP address: " );
  Serial.println ( WiFi.localIP() );

  return true;

}

void showActivity(uint16_t timeout) {

  uint8_t color;

  switch (tableState) {
    case TABLE_FREE:
    case TABLE_CHANGING:
      color = pG;
      break;
    case TABLE_OCCUPIED:
      color = pR;
      break;

  }

  digitalWrite(ledPIN[color], 0);

  ledTimers[color] = millis();
  ledTimeout[color] = timeout;


}

void LEDhandle() {

  uint32_t curr = millis();

  switch (tableState) {
    case TABLE_FREE:
      digitalWrite(ledPIN[pR], 0);
      digitalWrite(ledPIN[pG], 1);
      break;
    case TABLE_OCCUPIED:
      digitalWrite(ledPIN[pR], 1);
      digitalWrite(ledPIN[pG], 0);
      break;
    case TABLE_CHANGING:
      break;
  }

  for (int i = 0; i < sizeof(ledTimers) / sizeof(uint32_t); i++) {
    if (ledTimeout[i] > 0) {
      if (curr - ledTimers[i] > ledTimeout[i]) {
        ledTimeout[i] = 0;
        digitalWrite(ledPIN[i], 1);
      }

    }
  }

}

void addMsg(const char * str) {

  Serial.println(mesgQueue[mesgIndx]);

  if (mesgIndx >= 9) {
    for (int i = 0; i < 9; i++) {
      strncpy(mesgQueue[i], mesgQueue[i + 1], 171);
      Serial.println("reorder");
    }
  } else {
    mesgIndx++;
  }


  int sec = millis() / 1000;

  sprintf(mesgQueue[mesgIndx], "%02d:%02d:%02d: ",  sec / 3600, (sec / 60) % 60, sec % 60);

  strncat(mesgQueue[mesgIndx], str, 160);

  Serial.print("mesgIndx=");
  Serial.print(mesgIndx);
  Serial.print(" str=");
  Serial.print(str);
  Serial.print(" mesgQueue[mesgIndx]=");
  Serial.println(mesgQueue[mesgIndx]);


}

void serverHandleAddMsg() {

  if (!enableChat) {
    server.send ( 404, "text/plain", "CHAT disabled" );
    return;
  }

  for ( uint8_t i = 0; i < server.args(); i++ ) {
    if (server.argName(i) == "msg") {

      addMsg(server.arg(i).c_str());

      String header = "HTTP/1.1 301 OK\r\nLocation: /\r\n\r\n";
      server.sendContent(header);
      return;

    }
  }

  server.send ( 404, "text/plain", "msg not found" );




}

void serverHandleConfig() {



  for ( uint8_t i = 0; i < server.args(); i++ ) {
    if (server.argName(i) == "******") {
      enableChat = false;
    }

    if (server.argName(i) == "******") {
      enableChat = true;
    }

    if (server.argName(i) == "******") {
      delay_occupied_val = server.arg(i).toInt();
      saveToEEPROM();
    }

    if (server.argName(i) == "******") {
      hardstate_occupied_val = server.arg(i).toInt();
      saveToEEPROM();
    }


  }
  String out;


  out = String("Chat: ") + (enableChat ? "YES" : "NO") + "<br />" 
    + "delay_occupied = " + String(delay_occupied_val) + "<br/>"
    + "hardstate_occupied = " + String(hardstate_occupied_val);

  server.send ( 200, "text/html", out );


}

void serverHandleRoot() {

  Serial.println("serverHandleRoot");

  showActivity(1000);

  String str;
  String title;
  String subtitle1;
  String msgs;
  String temp;

  char strbuffer[40];


  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  uint32_t statemin, statehr;

  statemin = (millis() - curStateTime) / 1000 / 60;
  statehr = statemin / 60;


  switch (tableState) {
    case TABLE_FREE:

    case TABLE_CHANGING:
      sprintf( strbuffer, "[ FREE%s %d hours %d minutes ]", tableState == TABLE_CHANGING ? " (?)" : "", statehr, statemin % 60);

      subtitle1 = String("<h2 class='c1 p2' style='background-color: darkgreen'>TABLE IS FREE</h2>");
      tableState == TABLE_CHANGING ? subtitle1 += "<p>Hold a sec... Something is moving near the table...</p>" : subtitle1 += "<p>It is time to play a game!</p>";
      subtitle1 += "The table has been free for " + String(statehr) + " hours and " + String(statemin % 60) + " minutes";


      break;
    case TABLE_OCCUPIED:
      sprintf( strbuffer, "[ OCCUPIED %d minutes ]", statemin);

      subtitle1 = String("<h2 class='c1 p2' style='background-color: darkred'>TABLE IS OCCUPIED</h2>")
                  + "<p>Be patient!</p>"
                  + "The table was occupied " + String(statemin) + " minutes ago";


      break;
  }

  temp = String(bmp.readTemperature()) + " *C, " + String(bmp.readPressure()) + " Pa";

  title = String(strbuffer);

  Serial.println(mesgIndx);

  for (int8_t i = 0; i <= mesgIndx; i++) {
    msgs = msgs + String(mesgQueue[i]) + "<br/>";
  }

  sprintf(strbuffer, "%02d:%02d:%02d",  hr, min % 60, sec % 60);

  str = "<html>\
<html>\
<head>\
<meta name='viewport' content='width=device-width, initial-scale=1.0'>\
<title>" + title + " Table soccer detector!</title>\
<style> .c1 {color:#fff} .p2 {padding:10px} body {background-color: #eee; font-family: Verdana; color: #333; font-size: 12px; margin: 0 auto}</style>\
</head>\
<body>\
<div style='text-align: center'>"
        + subtitle1 +
        "<p style='margin-top: 20px'>System uptime: " + String(strbuffer) + "</p>\
<p class='c1 p2' style='background-color: #333'>Last messages</p>\
<pre style='margin:40px auto; text-align: left;max-width:500px'>" + msgs + "</pre>\
<form method=GET action='/addMsg'>Msg: <input type=text name=msg maxlength=160/><button type=submit>Send</button></form>\
<p class='c1 p2' style='background-color: #888;'>Author: Rafal Bednarz &lt;shipitday@rbednarz.pl&gt; [" + temp + "]</p>\
</div>\
</body>\
</html>";

  server.send ( 200, "text/html", str );

}

void PIRhandle() {

  uint8_t state = digitalRead(pinPIR);

  digitalWrite(ledPIN[pY], state);

  uint32_t curr = millis();

  // jezeli podczas kolejnych detekcji minelo wiecej czasu niz [delay_occupied] to zaczynamy liczyc od nowa
  // jezeli bedzie nieustanna detekcja (z przerwami nie wiekszymi niz delay) w czasie [hardstate] - to stol zajety
  
  if (curr - lastPIRdetectTime > delay_occupied_val * 1000) { //od ostatniej detekcji minelo wiecej niz DELAY_OCCUPIED sek

    if (tableState == TABLE_OCCUPIED) {
      curStateTime = millis();
    }

    tableState = TABLE_FREE;

    if (state) {
      firstPIRdetectTime = curr; // assume this is new detection

    }



  }


  if (state) {

    lastPIRdetectTime = curr;

    if (lastPIRdetectTime - firstPIRdetectTime > hardstate_occupied_val * 1000) {

      tableState = TABLE_OCCUPIED;
      curStateTime = firstPIRdetectTime;

    } else {

      tableState = TABLE_CHANGING;

    }


  }




}

void setup ( void ) {

  pinMode ( ledPIN[pG], OUTPUT );
  pinMode ( ledPIN[pY], OUTPUT );
  pinMode ( ledPIN[pR], OUTPUT );

  digitalWrite(ledPIN[pG], 0);
  digitalWrite(ledPIN[pY], 0);
  digitalWrite(ledPIN[pR], 0);

  Serial.begin ( 115200 );

  EEPROM.begin(512);

  tableState = TABLE_FREE;
  curStateTime = millis();
  delay(1000);
  
  // uncomment it while compiling:
  //saveToEEPROM(); 

  readFromEEPROM();


  Serial.println("Current config values: ");
  Serial.println("hardstate_occupied_val = " + String(hardstate_occupied_val));
  Serial.println("delay_occupied_val = " + String(delay_occupied_val));

  Serial.println("Trying to connect to WIFI: " + String(wifi_ssid) + " with password: " + String(wifi_password));


  WiFi.begin ( wifi_ssid, wifi_password );

  wifiConnect();

  server.on ( "/", serverHandleRoot );
  server.on ( "/addMsg", serverHandleAddMsg );
  server.on ( "/config", serverHandleConfig );

  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  }

  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");


  server.begin();



}



void loop ( void ) {

  LEDhandle();

  PIRhandle();

  server.handleClient();

  if (WiFi.status() != WL_CONNECTED) {
      Serial.println("reconnecting...");
      wifiConnect();
  }

}
