n#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>

//Spécifique à l'esp
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h> 
#include <PubSubClient.h> 
#include <ArduinoJson.h>

// Definition des pins
#define DHTPIN 9     // Pin SD2
#define DHTTYPE    DHT11     // DHT 11
#define SELECT_DEBUG 14 // Pin D5
#define MOISTURE_PIN A0
#define WATER_TANK_PIN 12 // Pin D6
#define LED_WIFI D7 //D7 - LED FACADE
#define LED_HA D6 //D8 - LED FACADE
#define LED_ARR D4//D9 - LED FACADE
#define LED_PIN D3
#define LED_COUNT  8

#define SEUIL_HUM_P 750
#define SEUIL_TEMP_E_MAX 25
#define SEUIL_TEMP_E_MIN 20

//WIFI
const char* ssid = "Livebox-90D0";
const char* password = "vZnVfe5b4TQRsR9dZb";

//MQTT
#define MQTT_BROKER       "192.168.1.44"
#define MQTT_BROKER_PORT  1883
#define MQTT_USERNAME     "iblur"
#define MQTT_KEY          "Mjzl9074" 

//Interruption
const long intervale = 2000; 
unsigned long avantMS = 0; 

// Serveur NTP gestion du temps
const long utcOffsetInSeconds = 3600;
char daysOfTheWeek[7][12] = {"Dimanche", "Lundi", "Mardi", "Mercredi", "Jeudi", "Vendredi", "Samedi"};
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);


ESP8266WiFiMulti WiFiMulti;
WiFiClient espClient;
PubSubClient client(espClient);

bool force_grow = true;

DHT dht(DHTPIN, DHTTYPE);
StaticJsonDocument<64> datas;
StaticJsonDocument<64> cmd;
StaticJsonDocument<64> plant;
//Variable permetant de savoir si les services ont été activées
int water_tank_mqtt = 0;
int low_hum = 0;
int control_temp_led = 0;
int etat_led = 0;

//etat service
int etat_service = 1;

bool state = false;

//Strip RGB adressable 
Adafruit_NeoPixel ring(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);


void setup() {
  Serial.begin(9600); //Même bds que le bds de boot de l'ESP
  delay(1000);
  Serial.println("\n ---- CLOUD PLANT IoT PROJECT ----");
  ring.begin();           // INITIALIZE NeoPixel ring object (REQUIRED)
  ring.setBrightness(150); // Set BRIGHTNESS to about 1/5 (max = 255)
  dht.begin();
  led_clear();
  pin_mode(); //Permet de configurer les pins de selection de mode 
  setup_wifi();
  setup_mqtt();
  Serial.println("---- BOOTED ----");
  client.publish("cloudplant/etat_sys/boot", "ESP8266 - CloudPlant - Booted");
  debug_mode(); 
  client.subscribe("cloudplant/etat_sys/cmd");
  client.subscribe("cloudplant/etat_sys/cmd");
  timeClient.begin(); 
  //Informer HomeAssistant de l'allumage du suivi
  client.publish("cloudplant/etat_sys/enable", "1");
  led_clear();

}

void loop() {
  reconnect();
  client.loop();
  timeClient.update(); // Mettre à jour les variable de l'heure 
  unsigned long actualMS = millis(); // process de MàJ de l'intervalle Milli 
  if (actualMS - avantMS >= intervale) { //Permet de ne pas bloquer le proc avec une fonction comme delay(x)
    avantMS = actualMS; 
    normal_process(); 
  }
}


// Process de lecture des données puis l'envoi sur le topic MQTT
void normal_process(){
  datas["temperature"] = get_temp_ext(); 
  datas["humidity"] = get_hum_ext();

  char buffer[256];
  serializeJson(datas, buffer);
  client.publish("cloudplant/sensors/datas", buffer); //Envoi des données de température et d'hum sur le topic MQTT relatif

  plant["humidity"] = get_percent_soil_moisture();

  char buffer2[256];
  serializeJson(plant, buffer2);
  client.publish("cloudplant/sensors/plant", buffer2); //Envoi des données de température et d'hum sur le topic MQTT relatif
  
  if (etat_service == 1){ //Si la surveillance est activée, lancer les script d'automatisation
 
  water_tank();
  control_hum();
  temp_control();
  simu_lux();
  }
  else // Si la surveillance est désactivée, arreter tout les services, sauf celui de la detection d'eau dans le reservoir. 
  {
    water_tank();
    arrosage_off();
  }

}

void simu_lux(){
  //Serial.println(timeClient.getHours());
  if (force_grow)
    led_all_purple();
  else if ((timeClient.getHours()>=18 && timeClient.getHours()<=23)||(timeClient.getHours()>=0 && timeClient.getHours()<=5)){
      led_all_purple();
  }
  else if (etat_led != 0)
    led_clear();

}

void  temp_control(){
  if(get_temp_ext() <  SEUIL_TEMP_E_MIN || get_temp_ext() >  SEUIL_TEMP_E_MAX){
    if(control_temp_led == 0){
      Serial.println("Attention la température actuelle de la pièce n'est pas idéale");
      Serial.print("Température actuelle :");
      Serial.println(get_temp_ext());
      control_temp_led = 1;
    }
    }
  else{
      control_temp_led = 0;
      }
  
  
}

//Calcul de l'humiditée dans la plante
void control_hum(){
  if(get_soil_moisture() >  SEUIL_HUM_P && etat_service == 1 ){
    if(low_hum == 0){
    Serial.println("-- Humidité de la terre faible --");
    arrosage_on();
    low_hum = 1;
    analogWrite(LED_ARR,250);}
    }
  else{
    if(low_hum = 1){
      arrosage_off();
      analogWrite(LED_ARR,250);}
    low_hum = 0;}
}

void arrosage_on(){
  if ((timeClient.getHours()>=7 && timeClient.getHours()<=9)||(timeClient.getHours()>=16 && timeClient.getHours()<=19)){
    Serial.println("Début de l'arrosage");
    // Inserer la commande pour la pompe
  }
  else{ 
    Serial.println("L'arroserage débutera lorsque cela sera possible");
    Serial.println("Horaire d'arrosage : 8h - 10h & 17h - 20h\n\n");
    // Inserer la commande pour la pompe
    }
}

void arrosage_off(){
  //Serial.println("--- Fin de l'arrosage ---");
  // Inserer la commande pour la pompe
  
}

// Verif du niveau de l'eau dans le bac
void water_tank(){
  if (digitalRead(WATER_TANK_PIN) == HIGH){
    if (water_tank_mqtt == 0){
    Serial.println("-- Attention il n'y a plus d'eau dans le reservoir --");
    // Allumer une LED EN ROUGE
    water_tank_mqtt = 1;
    //led_all_red();
    }
      
  }
  else
    water_tank_mqtt = 0;
    //led_clear();
}


//Fonction d'interruption pour la reception d'un topic MQTT 
void callback(char* topic, byte* payload, unsigned int length) {
 
  deserializeJson(cmd, payload, length);
  const char* state = cmd["state"];
  if (cmd["state"] == "1"){
    etat_service = 1;
    client.publish("cloudplant/etat_sys/enable", "1");}
  else if (cmd["state"] == "0"){
    etat_service = 0;
    client.publish("cloudplant/etat_sys/enable", "0");}
  if (etat_service == 1){
      normal_process();
      digitalWrite(LED_BUILTIN, HIGH);}
  else{
      digitalWrite(LED_BUILTIN, LOW);
  }
  Serial.print("État du service = ");
  Serial.println(etat_service);
  
}

void pin_mode(){
  pinMode(SELECT_DEBUG, INPUT_PULLUP);
  //pinMode(SELECT_WIFI, INPUT);
  //pinMode(SELECT_HL, INPUT);
  pinMode(WATER_TANK_PIN, INPUT);
  pinMode(MOISTURE_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_HA, OUTPUT);
  pinMode(LED_WIFI, OUTPUT);
  pinMode(LED_ARR, OUTPUT);
  }

void debug_mode(){
  if (digitalRead(SELECT_DEBUG) == LOW){
    Serial.println("-- DEBUG MODE --");}

  else {
    Serial.println("-- NO DEBUG MODE -- ");
    digitalWrite(LED_BUILTIN, HIGH);  
    }

  while (digitalRead(SELECT_DEBUG) == LOW){
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    Serial.println(get_temp_ext());
    Serial.println(get_hum_ext());
    Serial.println(get_soil_moisture());
  }
  
}

//Connection au wifi
void setup_wifi(){
  WiFiMulti.addAP(ssid, password);
  int i=0;
  while ( WiFiMulti.run() != WL_CONNECTED ) {
    delay (250);
    Serial.print ( "." );
    ring.setPixelColor(i, ring.Color(0,0,255));
    ring.show();
    state = !state;
    digitalWrite(LED_HA,state);
    i++;
    if (i>7){
      i= 0;
      led_clear();
    }
  }
  if (WiFiMulti.run() == WL_CONNECTED)
  Serial.println("");
  Serial.println("WiFi connecté");
  Serial.print("MAC : ");
  Serial.println(WiFi.macAddress());
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.localIP());
  analogWrite(LED_WIFI,50);
}

//Fonction de connection au MQTT
void setup_mqtt(){
  client.setServer(MQTT_BROKER, MQTT_BROKER_PORT);
  client.setCallback(callback); //Activer l'interruption du topic MQTT 
  reconnect();
}

//Fonction pour publier un float sur un topic
void mqtt_publish(String topic, float t){
  char top[topic.length()+1];
  topic.toCharArray(top,topic.length()+1);
  char t_char[50];
  String t_str = String(t);
  t_str.toCharArray(t_char, t_str.length() + 1);
  client.publish(top,t_char);
}

// Fonction de reconnection au topic MQTT
void reconnect(){
  while (!client.connected()) {
    Serial.println("Connection au serveur MQTT ...");
    if (client.connect("ESPClient", MQTT_USERNAME, MQTT_KEY)) {
      Serial.println("MQTT connecté");
      analogWrite(LED_HA,50);
      led_clear();      
      
    }
    else {
      analogWrite(LED_HA,0);
      Serial.print("echec, code erreur= ");
      Serial.println(client.state());
      Serial.println("nouvel essai dans 2s");
      delay(2000);
    }
  }
}

//Focntions qui permet d'acceder au différents capteurs. 
int get_temp_ext(){
  return dht.readTemperature();}
float get_hum_ext(){
  return dht.readHumidity();}
int get_soil_moisture(){
  return analogRead(A0);}
int get_percent_soil_moisture(){
  float p = analogRead(A0)-550;
  p = (p *100)/474;
  p = int(100 - p);
  return p;
}

void led_clear(){
  for (int i=0; i<8; i++){
    ring.setPixelColor(i, ring.Color(0,0,0));
  }
  ring.show();
  etat_led = 0;
}

void led_all_red(){
  for (int i=0; i<LED_COUNT; i++){
    ring.setBrightness(255);
    ring.setPixelColor(i, ring.Color(255,0,0));
  }
  ring.show();
  etat_led = 1;
}

void led_all_purple(){
  for (int i=0; i<LED_COUNT; i++){
    ring.setBrightness(255);
    ring.setPixelColor(i, ring.Color(128,0,128));
  }
  ring.show();
  etat_led = 2;
}
