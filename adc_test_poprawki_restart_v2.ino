#include <WiFi.h>
#include <WiFiUdp.h>
#include "Arduino.h"
#include <analogWrite.h>
#include <EEPROM.h>

//#define zeppelinID 0

#define EEPROM_SIZE 4

#define L_GPIO_pin 27 // podpiąć do AIN2 na mostku H
#define R_GPIO_pin 32 // podpiąć do BIN2 na mostku H

#define L_PWM_pin 33 // podpiąć do AIN1 na mostku H
#define R_PWM_pin 21 // podpiąć do BIN1 na mostku H
#define T_PWM_pin 13

#define H_BRIDGE_MODE_pin 4

#define uS_TO_S_FACTOR 1000000  //Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  15*60        //Time ESP32 will go to sleep (in seconds)

class motor_reverse_dir {
  public:
    bool LEFT = LOW;
    bool RIGHT = LOW;

    void set_reverse_status(int in);
    int get_reverse_status();
};

void motor_reverse_dir::set_reverse_status(int in) {
  if (in == 0) { // #b 00
    LEFT  = LOW;
    RIGHT = LOW;
  }
  else if (in == 1) { // #b 01
    LEFT  = LOW;
    RIGHT = HIGH;
  }
  else if (in == 2) { // #b 10
    LEFT  = HIGH;
    RIGHT = LOW;    
  }
  else if (in == 3) { // #b 11
    LEFT  = HIGH;
    RIGHT = HIGH;    
  }
}

int motor_reverse_dir::get_reverse_status() {
  if ( LEFT == LOW and RIGHT == LOW ) return 0;
  else if ( LEFT == LOW and RIGHT == HIGH ) return 1;
  else if ( LEFT == HIGH and RIGHT == LOW ) return 2;
  else if ( LEFT == HIGH and RIGHT == HIGH ) return 3;
  else return -1;
}

motor_reverse_dir state;

// const char * networkName = "NoGargoyle";
// const char * networkPswd = "polibuda238";
// const char * networkName = "Mechatronix";
// const char * networkPswd = "gumowykrokodyl69";
const char * networkName = "NoGargoyle1";
const char * networkPswd = "polibuda238";
const char * udpAddress = "192.168.4.1";
//const char * udpAddress = "192.168.0.102";
IPAddress current_remote_udp_address;
const int udpPort = 3333;
boolean connected = false;

WiFiUDP udp;

hw_timer_t * timer = NULL;

int VBAT = 0;
int usb = 0;
int usbFlag = 1;

int timeFromLastMsg = 0;
bool VBAT_sended = false;

int zeppelinID = 2;

int time_connection_lost = 0;
int low_bat_timer = 0;

void IRAM_ATTR onTimer(){
    VBAT = analogRead(A13);
    VBAT = map(VBAT, 0, 4096, 0, 666);
    usb = analogRead(A2);
    if ( usb > 2000 ) { // bylo 2000
      usbFlag = 2; // charging
    }
    else {
      usbFlag = 1; // flying
    }
    timeFromLastMsg++;
    time_connection_lost++;
    if (time_connection_lost >= 3) {
      motorsTurnOff(); // wyłączyć silniki
      ESP.restart();
    }
    VBAT_sended = false;
    if ( VBAT < 310 ) {
      low_bat_timer++;
    }
    else {
      low_bat_timer = 0;
    }
    if (low_bat_timer >= 10) {
      motorsTurnOff();
      esp_deep_sleep_start();
    }
}

////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(H_BRIDGE_MODE_pin, OUTPUT);
  pinMode(L_GPIO_pin, OUTPUT);
  pinMode(R_GPIO_pin, OUTPUT);
  pinMode(L_PWM_pin, OUTPUT);
  pinMode(R_PWM_pin, OUTPUT);
  pinMode(T_PWM_pin, OUTPUT);

  digitalWrite(H_BRIDGE_MODE_pin, HIGH); // low - set in/in h bridge mode, HiGH - phasse/enable mode

  EEPROM.begin(EEPROM_SIZE);
  zeppelinID = EEPROM.read(0);
  int rev = EEPROM.read(1);
  Serial.printf("motor reversed = %i \n",rev);
  state.set_reverse_status(rev);
  Serial.print("zeppelinID = ");
  Serial.println(zeppelinID);
  
  connectToWiFi(networkName, networkPswd);

  udp.beginPacket(udpAddress, udpPort);
  udp.printf("heartbit");
  udp.endPacket();

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true); 
  timerAlarmEnable(timer);

}
////////////////////////////////////////////////////////////////
void loop() {
  if(connected) {
    time_connection_lost = 0;
    
    if ( timeFromLastMsg > 1 ) {
      motorsTurnOff();
    }

    if ( VBAT_sended == false ) {    
      statusUdpReport();
    }

    int packetSize = udp.parsePacket();
    if (packetSize) {
      current_remote_udp_address = udp.remoteIP();
      char packetBuffer[255];
      int len = udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;
      }
      handleData_test(len, packetBuffer);

    }
  }
  else {
    motorsTurnOff();
  }
  //delay(1000);
  if (Serial.available()) {
    String str;
    str = Serial.readString();

  if (str.length() == 7 and str[0] == 's' and str[1] == 'e' and str[2] == 't' and str[3] == 'i' and str[4] == 'd' and str[5] == ' ') {
    int newID = str[6] - '0';
    if ( newID >= 0 and newID <= 5) {
      zeppelinID = newID;
      EEPROM.begin(EEPROM_SIZE);
      EEPROM.write(0, zeppelinID);
      if ( EEPROM.commit() ) {
        Serial.printf("new_ID_=_%i \n", newID);
      }
      else {
        Serial.println("error_new_ID_EEPROM_writting_failed");
      }
    }
    else {
      Serial.println("error_wrong_ID");
    }
      
  }
  else if (str.length() == 7 and str[0] == 's' and str[1] == 'e' and str[2] == 't' and str[3] == 'm' and str[4] == 'r' and str[5] == ' ') {
    int mot_rev = str[6] - '0';
    if ( mot_rev >= 0 and mot_rev <= 3) {
      state.set_reverse_status(mot_rev);
      EEPROM.begin(EEPROM_SIZE);
      EEPROM.write(1, mot_rev);
      if ( EEPROM.commit() ) {
        Serial.printf("motors_reverse_set_to %i \n", mot_rev);
      }
      else {
        Serial.println("error_motor_reversed_EEPROM_writting_failed");
      }
    }
    else {
      Serial.printf("error_wrong_motor_reverse_value ( val = %i )\n", mot_rev);
    }
      
  }
  else if (str.length() == 5 and str[0] == 'g' and str[1] == 'e' and str[2] == 't' and str[3] == 'i' and str[4] == 'd') {
    EEPROM.begin(EEPROM_SIZE);
    int id_to_return = EEPROM.read(0);
    Serial.printf("my_ID_is_%i \n", id_to_return);
  }
  else if (str.length() == 5 and str[0] == 'g' and str[1] == 'e' and str[2] == 't' and str[3] == 'm' and str[4] == 'r') {
    EEPROM.begin(EEPROM_SIZE);
    int mr_to_return = EEPROM.read(1);
    Serial.printf("motors_reverse_are_set_to_%i \n", mr_to_return);
  }
  else {
    Serial.printf("error_wrong_command \n");
  }

    
  }

}
////////////////////////////////////////////////////////////////

void handleData_test(int len, char bufer[]){

  if (len == 7 and bufer[0] == 's' and bufer[1] == 'e' and bufer[2] == 't' and bufer[3] == 'i' and bufer[4] == 'd' and bufer[5] == ' ') {
    int newID = bufer[6] - '0';
    if ( newID >= 1 and newID <= 5) {
      zeppelinID = newID;
      EEPROM.write(0, zeppelinID);
      udp.beginPacket(current_remote_udp_address, udpPort);
      udp.printf("new_ID_=_%i \n", newID);
      udp.endPacket();
    }
    else {
      errorUdpReport("error_wrong_ID\n");
    }
      
  }
  else if (len == 7 and bufer[0] == 's' and bufer[1] == 'e' and bufer[2] == 't' and bufer[3] == 'm' and bufer[4] == 'r' and bufer[5] == ' ') {
    int mot_rev = bufer[6] - '0';
    if ( mot_rev >= 0 and mot_rev <= 3) {
      state.set_reverse_status(mot_rev);
      EEPROM.begin(EEPROM_SIZE);
      EEPROM.write(1, mot_rev);
      if ( EEPROM.commit() ) {
        udp.beginPacket(current_remote_udp_address, udpPort);
        udp.printf("motors_reverse_set_to %i \n", mot_rev);
        udp.endPacket();
      }
      else {
        udp.beginPacket(current_remote_udp_address, udpPort);
        udp.printf("error_motor_reversed_EEPROM_writting_failed \n");
        udp.endPacket();
      }
    }
    else {
      udp.beginPacket(current_remote_udp_address, udpPort);
      udp.printf("error_wrong_motor_reverse_value ( val = %i )\n", mot_rev);
      udp.endPacket();
    }
  }
  else if (len == 5 and bufer[0] == 'g' and bufer[1] == 'e' and bufer[2] == 't' and bufer[3] == 'i' and bufer[4] == 'd') {
    EEPROM.begin(EEPROM_SIZE);
    int id_to_return = EEPROM.read(0);
    udp.beginPacket(current_remote_udp_address, udpPort);
    udp.printf("my_ID_is_%i \n", id_to_return);
    udp.endPacket();
  }
  else if (len == 5 and bufer[0] == 'g' and bufer[1] == 'e' and bufer[2] == 't' and bufer[3] == 'm' and bufer[4] == 'r') {
    EEPROM.begin(EEPROM_SIZE);
    int mr_to_return = EEPROM.read(1);
    udp.beginPacket(current_remote_udp_address, udpPort);
    udp.printf("motors_reverse_are_set_to_%i \n", mr_to_return);
    udp.endPacket();
  }
  else {
    udp.beginPacket(current_remote_udp_address, udpPort);
    udp.printf("error_wrong_command \n");
    udp.endPacket();
  }



  

  if ( usbFlag == 2 or VBAT < 325 ) {
    // enter low power consumption mode on DC driver
    motorsTurnOff();
    //Serial.println("usb > 1800 or VBAT < 2000");

      udp.beginPacket(udp.remoteIP(), udpPort);
      udp.println(".........usb > 1800 or VBAT < 330  ...........................");
      udp.endPacket();
    
    return;
  }

  
  if (len == 15 ) {
    if (bufer[0] == 'L' and bufer[5] == 'R' and bufer[10] == 'T') {
      int c = 0;
      for( int i = 0; i < len; i++) {
        if ( isDigit(bufer[i]) ) c++;
      }
      if ( c == 12 ) {
        if ( (bufer[1] == '0' or bufer[1] == '1') and (bufer[6] == '0' or bufer[6] == '1') and (bufer[11] == '0' or bufer[11] == '1') ) {
          char L[3], R[3], T[3];
          L[0] = bufer[2]; L[1] = bufer[3]; L[2] = bufer[4];
          R[0] = bufer[7]; R[1] = bufer[8]; R[2] = bufer[9];
          T[0] = bufer[12]; T[1] = bufer[13]; T[2] = bufer[14];

          //int L_PWM = atoi(&L[0]);
          //int L_PWM = 100 * ((int)L[0] - 48) + 10 * ((int)L[1] - 48) + ((int)L[2] - 48);
          int L_PWM = charToInt(L);
          int R_PWM = charToInt(R);
          int T_PWM = charToInt(T);

          //Serial.printf("decoded PWM -> L=%i  R=%i  T=%i \n", L_PWM, R_PWM, T_PWM);
          //Serial.printf("decoded PWM -> L=%c  R=%c  T=%c \n", L, R, T);
          //Serial.printf("L[0] = %i    L_PWM = %i \n", L[0], L_PWM);

          if ( L_PWM <= 255 and L_PWM >= 0 and R_PWM <= 255 and R_PWM >= 0 and T_PWM <= 255 and T_PWM >= 0 ) {
            // set L direction
            if ( bufer[1] == '0' ) {
              digitalWrite(L_GPIO_pin, state.LEFT);
            }
            else {
              digitalWrite(L_GPIO_pin, !state.LEFT);
              // L_PWM = 255 - L_PWM;

            }
            // set R direction
            if ( bufer[6] == '0' ) {
              digitalWrite(R_GPIO_pin, state.RIGHT);

            }
            else {
              digitalWrite(R_GPIO_pin, !state.RIGHT);
              // R_PWM = 255 - R_PWM;

            }

            if (L_PWM < 45) L_PWM = 0;
            if (R_PWM < 45) R_PWM = 0;
            if (T_PWM < 45) T_PWM = 0;
            
            // set 3 x PWM
            analogWrite(L_PWM_pin, L_PWM);
            analogWrite(R_PWM_pin, R_PWM);
            analogWrite(T_PWM_pin, T_PWM);

//udp.beginPacket(udp.remoteIP(), udpPort);
//udp.printf("analog writed -> Lpwm=%i  Rpwm=%i  Tpwm=%i", L_PWM, R_PWM, T_PWM);
//udp.endPacket();
            
            //digitalWrite(T_PWM_pin, HIGH);
            // reset timer
            timeFromLastMsg = 0;
          }
          else {
            // send error code "error_pwm_value_>_255\n"
            errorUdpReport("error_pwm_value_>_255\n");
          }
        }
        else {
          // send error code "error_not_boolean_values_in_flags\n"
          errorUdpReport("error_not_boolean_values_in_flags\n");
        }
      }
      else {
        // send error code "error_not_a_digit_in_message\n"
        errorUdpReport("error_not_a_digit_in_message\n");
      }
    }
    else {
      // send error code "error_LRT_markers\n"
      errorUdpReport("error_LRT_markers\n");
    }
  }

}

void errorUdpReport(char *txt){
  udp.beginPacket(current_remote_udp_address, udpPort);
  udp.printf(txt);
  udp.endPacket();
}

void statusUdpReportHumanRedable() {  
  udp.beginPacket(current_remote_udp_address, udpPort);
  //udp.print("IP: ");
  //udp.print(WiFi.localIP());
  udp.printf("VBAT: %03i", VBAT);
  udp.printf("  USB: %i", usbFlag);
  udp.printf("  timeout: %05i", timeFromLastMsg);
  udp.endPacket();
  VBAT_sended = true;
}

void statusUdpReport() {  
  udp.beginPacket(udpAddress, udpPort);
  udp.printf(".%i.%03i.%i.", zeppelinID, VBAT, usbFlag);
  // udp.printf(" usb analog read = ", analogRead(A2));
  udp.endPacket();
  VBAT_sended = true;
}


void motorsTurnOff() {
  // set all motor direction GPIO to 0
  digitalWrite(L_GPIO_pin, LOW);
  digitalWrite(R_GPIO_pin, LOW);
  // set all PWM to 0
  analogWrite(L_PWM_pin, 0);
  analogWrite(R_PWM_pin, 0);
  analogWrite(T_PWM_pin, 0);
  //digitalWrite(T_PWM_pin, HIGH);
}

int charToInt(char *c) {
  return 100 * ((int)c[0] - 48) + 10 * ((int)c[1] - 48) + ((int)c[2] - 48);     
}


void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}
