#include <WiFi.h>

const char *ssid = "ServerESP32";
const char *password = "1234567890";

const int phase_A = 2;
const int phase_B = 3;

const int ENA_M1 = 7;
const int IN1_M1 = 18;
const int IN2_M1 = 19;

volatile int pulsos = 0;
volatile byte anterior = 0;
volatile byte actual = 0;

unsigned long last_time = 0;
unsigned long sample_time = 50;

volatile unsigned long t1 = 0;
volatile unsigned long t2 = 0;
volatile unsigned long t3 = 0;

float rpm = 0;
float pos = 0;
int resolution = 4320;
float w = 0;

const float k_value = (2 * 3.1416 * 1000) / resolution;

/////////////////////////// RECEPCIÓN WIFI //////////////////
String inputString = "";
bool stringComplete = false;
const char separator = ',';
const int dataLength = 3;
float data[dataLength];
float outValue = 0;

float value1, value2, value3, out_value;

WiFiServer server(80);

//////////////////// PWM ////////////////////////
const int pwmPin = ENA_M1;         // Pin GPIO donde se conectará el PWM
const int pwmChannel = 0;      // Canal PWM
const int pwmResolution = 12;   // Resolución en bits
const int pwmFrequency = 1000;  // Frecuencia en Hz

void setup()
{
    Serial.begin(115200);
    pinMode(phase_A, INPUT);
    pinMode(phase_B, INPUT);
    pinMode(IN1_M1, OUTPUT);
    pinMode(IN2_M1, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(phase_A), encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(phase_B), encoder, CHANGE);    
    digitalWrite(IN1_M1, LOW);
    digitalWrite(IN2_M1, LOW);
    ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
    ledcAttachPin(pwmPin, pwmChannel);
    WiFi.softAP(ssid, password);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    server.begin();
}



void encoder()
{
    anterior = actual;

    if (digitalRead(phase_A)) bitSet(actual, 1);
    else bitClear(actual, 1);

    if (digitalRead(phase_B)) bitSet(actual, 0);
    else bitClear(actual, 0);

    if (anterior == 2 && actual == 0) pulsos++;
    if (anterior == 0 && actual == 1) pulsos++;
    if (anterior == 3 && actual == 2) pulsos++;
    if (anterior == 1 && actual == 3) pulsos++;

    if (anterior == 1 && actual == 0) pulsos--;
    if (anterior == 3 && actual == 1) pulsos--;
    if (anterior == 0 && actual == 2) pulsos--;
    if (anterior == 2 && actual == 3) pulsos--;
}

float get_data(String request){
    for(int i= 0; i<dataLength; i++){
        int pos = request.indexOf(separator);       
        data[i] = request.substring(0, pos).toFloat();
        request = request.substring(pos+1);
    }
    value1 = data[0]; 
    value2 = data[1];
    value3 = data[2];
    outValue = value1;
    

    return outValue;
}

void clock_wise(float out_value){
    int duty = out_value;
    digitalWrite(IN1_M1, LOW);
    digitalWrite(IN2_M1, HIGH);
    ledcWrite(pwmChannel, duty);
    Serial.println("DUTY: " + String(duty));
}

void counter_clock_wise(float out_value){
    int duty = abs(out_value);
    digitalWrite(IN1_M1, HIGH);
    digitalWrite(IN2_M1, LOW);
    ledcWrite(pwmChannel, duty);
    Serial.println("DUTY: " + String(duty));
}

void stop_motor(){
    digitalWrite(IN1_M1, LOW);
    digitalWrite(IN2_M1, LOW);
    ledcWrite(pwmChannel, 0);
}

void loop()
{
    WiFiClient client = server.available();
    if (client)
    {
        while (client.connected())
        {

            if (client.available())
            {
                t1 = millis();
                String request = client.readStringUntil('\n');
                t2 = millis();
                Serial.println("TIEMPO DE RECEPCION: " + String(t2 - t1) + " ms");
                Serial.println(request);
                out_value = get_data(request);
                if(out_value > 0){
                    clock_wise(out_value);
                } 
                else if(out_value < 0){
                    counter_clock_wise(out_value);
                }
                else {
                    stop_motor();
                }
                
            }
        }
        client.stop();
        
    }
    if(millis() - last_time > sample_time){        
        rpm = (pulsos * 60 * 1000.0) / (resolution * (millis() - last_time));
        w = (k_value*pulsos)/(millis() - last_time);
        pos = (pulsos * 360) / resolution;
        pulsos = 0;
        last_time = millis();
        Serial.println("w: " + String(w) + "rad/s");
    }
}