#include <Arduino_LSM6DS3.h>
#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <Servo.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;

#define L_forearm 0.2628// Length of forearm for 1.80m patient (0.146*1.8)
#define L_hand 0.1944 // Length of hand for 1.80m patient (0.108*1.80)
#define bodymass 80
#define m_forearm 1.28// Kg (0.016*bodymass)
#define hand_w 0.48 //Mass of the patient's hand (kg) (0.006 * bodymass)
#define distance_sens_force_from_elbow 0.19 // Distance of the force sensor from the elbow
#define Kp 2 //
#define Ki 0.01
#define Kd 0.5

//Wi-fi connection
char ssid[] = "eEMG3DEMOa";  
char pass[] = "1234567890";    
int keyIndex = 0;             
int status = WL_IDLE_STATUS;
WiFiServer server(80);


// PID Variables
float previousError = 0;
float integral = 0;

int is_trigger=0;
Servo myservo;

String request;

int flex=0; //Flag that identifies the phase of flexion of the excercise
int relax=1; //Flag to state the end of the excercise, another repetition can be done
int estensione=0; //Flag that identifies the phase of extension of the excercise
float motor_drive_torque=0;
float m=0;
float torque;
float angle;
int flag=0;
int flag_est=0;
float force_computed;
float pi=3.14;
unsigned long last_millis;
unsigned long last_millis_est;
int prova=1;
float TorqueMeasured;
int final_position;
int flag_start = 1;
float torque_diff = 0;
int flag_chekMov;
float CoM_forearm_hand=0.3118104; //CoM (0.82*(L_forearm+L_hand))
float total_hf_w=1.76; //Total mass of the patient's hand and forearm (kg)

// FORCE SENSOR
float forceIs, mIs, ForceN, m1 = 0, m2 = -0.267, anaForce1 = -97, anaForce2 = -147, g = 9.81;


// SERVO AND ENCODER
const int servoAnalogInPin = A1; 
const int PWMPin = 9;
float posIs, posIsDeg, posServo1 = 0, posServo2 = 90, posSensor1 = 913, posSensor2 = 521;
float maxAngle = 120;
float minAngle = 0;


float currentMicros=0;
float dt=0;
float previousMicros=0;


void setup() {
     
   Serial.begin(9600);
   while (!Serial);

   // Encoder and Servo
   myservo.attach(PWMPin);

    myservo.write(50);
    delay(1000); //wait 5 seconds
   
   // Force sensor 
    ads.setGain(GAIN_SIXTEEN); //+/- 0.256V 1 bit 0.0078125mV
    ads.begin(0x48);

    Serial.println("Access Point Web Server");
    // check for the WiFi module:
    if (WiFi.status() == WL_NO_MODULE) {
      Serial.println("Communication with WiFi module failed!");
      while (true);
    }

    String fv = WiFi.firmwareVersion();
    if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
      Serial.println("Please upgrade the firmware");
    }

    // by default the local IP address will be 192.168.4.1

    // print the network name (SSID);
    Serial.print("Creating access point named: ");
    Serial.println(ssid);

    status = WiFi.beginAP(ssid); //, pass);

    if (status != WL_AP_LISTENING) {
      Serial.println("Creating access point failed");
      while (true);
    }

    // wait 10 seconds for connection:
    delay(10000);
    // start the web server on port 80
    server.begin();
    // you're connected now, so print out the status
    printWiFiStatus();

}

void loop() {

    // compare the previous status to the current status
    if (status != WiFi.status()) {
      // it has changed update the variable
      status = WiFi.status();

      if (status == WL_AP_CONNECTED) {
        // a device has connected to the AP
        Serial.println("Device connected to AP");
      } else {
        // a device has disconnected from the AP, and we are back in listening mode
        Serial.println("Device disconnected from AP");
      }
    }

    WiFiClient client = server.available();   // search for incoming clients

    if (client) {                             // if you get a client,
      Serial.println("new client");           // print a message out the serial port
      
      // You can send an HTTP header 
      client.println("Hello from Arduino!"); // Send actual data (the message) here

      while (client.connected()) {            // loop while the client's connected
        if (client.available()) {  
          // if there are bytes to read from the client,
          String request = client.readStringUntil('\r');
          Serial.println(request);
          is_trigger = request.toInt();

          if(relax){
            
            if (is_trigger >= 1){
              Serial.print('START');
              is_trigger = 0;
              flex=1;
              relax=0;
          }
          }
          client.flush();
        }

        currentMicros = micros();  // Get time in microseconds
        dt = (currentMicros - previousMicros) / 1000000.0;  // Convert to seconds, used for PI controller
        previousMicros = currentMicros;          

        ForceN = convert_DG2NEWTON();
        posIsDeg = angle_in_deg();
        
        TorqueMeasured = (0.8+6.38*(ForceN * distance_sens_force_from_elbow)); //Measured torque wrt the elbow
        torque = torque_computation(posIsDeg);          

        if (flex){

          if (TorqueMeasured <= torque*0.9 && TorqueMeasured >= torque*1.1) { // If measured torque is close to the calculated gravitational torque, the motor is not activated
            final_position = posIsDeg;
            flag_chekMov=0;
            torque_diff = abs((TorqueMeasured) - (torque));

            if (final_position > maxAngle) { // Respect ROM of the forearm
              final_position = maxAngle;
            } else if (final_position < minAngle) {
              final_position = minAngle;
            }      
          } 
            
          else  {  // If measured torque is lower than the calculated gravitational torque, the Exo moves the forearm in flexion by considering torque difference and the gain
            torque_diff = abs((TorqueMeasured) - (torque));
            flag_chekMov=0;

            float P = Kp * torque_diff;
            integral += torque_diff * dt;
            float I = Ki * integral;
            float derivative = (torque_diff - previousError) / dt;


            final_position = posIsDeg + (P+Kd * derivative +I);

            if (final_position > maxAngle) { // Respect ROM of the forearm
              final_position = maxAngle;
              // Anti-windup correction: stop integral growth
              integral -= (final_position - maxAngle) / Ki;
            } 
            else if (final_position < minAngle) {
              final_position = minAngle;
              // Anti-windup correction: stop integral growth
              integral -= (final_position - minAngle) / Ki;
            }
          }
          previousError = torque_diff;

          if (ForceN<0 && posIsDeg>105 && flag_est==0){ // Force<0 (extention) for at least 0,2 seconds and 105 deg are reached --> start extention phase
            flag_est=1;
            last_millis_est = millis();
            
          }

          if (ForceN>=0 && flag_est==1){
            flag_est=0;
          }

          if (flag_est==1 && millis()-last_millis_est>=200){
            flag_est=0;
            estensione=1;
            flex=0;
          }     
        }            

        if (estensione){

          if (posIsDeg<70){
            estensione=0;
            flex=0;
            relax=1;
            delay(1000);
            integral = 0;
              } // wait 5 seconds--> extension finished
          }

        if (relax==0 && estensione==0 && flag_chekMov==0){ //extension finished go to final position
          myservo.attach(PWMPin);  
          myservo.write(final_position);
          delay(100);
          myservo.detach();
        }

        //monitoring values during code execution
          Serial.print("Torque measured: ");
          Serial.print(TorqueMeasured);
          Serial.print(" , Gravity torque: ");
          Serial.print(torque);
          Serial.print(" , Force: ");
          Serial.print(ForceN);
          Serial.print(" , Angle: ");
          Serial.print(posIsDeg);
          Serial.print(" , Final Position: ");
          Serial.print(final_position);
          Serial.print(" , Error: ");
          Serial.print(torque_diff);
          Serial.print("Stato:");
          Serial.print(flex);
          Serial.print(estensione);
          Serial.println();
          delay(100);      
      }
    }
}


//Function to find the force in Newton
float convert_DG2NEWTON(){
  float forceIs = ads.readADC_Differential_0_1();
  float mIs = ((m2-m1)/(anaForce2-anaForce1))*(forceIs-anaForce1)+m1; 
  float forceN = mIs*g;
  return forceN;
}

//Computation of theoretical torque
float torque_computation(float ang){
  float pi=3.14;
  float M=-CoM_forearm_hand*total_hf_w*g*sin((180-ang)*pi/180);
  return M;
}

//Function to find current position ad degree angle
float angle_in_deg(){
  float posIS_new= analogRead(servoAnalogInPin);
  float posIS_deg_new= ((posServo2-posServo1)/(posSensor2-posSensor1))*(posIS_new-posSensor1)+posServo1;
  return posIS_deg_new;
}


  void printWiFiStatus() {
    // Print the SSID of the network you're attached to:
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    // Print your WiFi shield's IP address:
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);
}

