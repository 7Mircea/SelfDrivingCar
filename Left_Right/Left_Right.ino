#include <Wire.h> //pentru comunicare I2c
#include <TinyGPS++.h> //pentru a salva si lucra cu datele GPS. Nu face partea de preluare a datelor propriu-zisa
#include <SoftwareSerial.h> //pentru comunicare cu GPS
#include "NewPing.h" //pentru HC_SR_04 senzorul de distanță 
const int EnableL = 5;
const int HighL = 6;       // motor partea stanga
const int LowL = 7;

const int EnableR = 10;
const int HighR = 8;       //motor partea dreapta
const int LowR = 9;

/*
const int D0 = 0;     //corespunde la Raspberry pin 21     LSB
const int D1 = 1;     //corespunde la Raspberry pin 22
const int D2 = 2;     //corespunde la Raspberry pin 23
const int D3 = 3;     //corespunde la Raspberry pin 24     MSB
const int D4 = 4;     //corespunde la Raspberry pin 25     MSB
const int D11 = 11;     //corespunde la Raspberry pin 28     MSB
const int D12 = 12;     //corespunde la Raspberry pin 29     MSB*/
const int eroareMax = 30+10;
const int vitezaMax = 180;//as putea scrie maxim 255. Am ales mai putin datorita faptului ca sunt prelucrate prea putine cadre pe secunda si nu are timp 
//sa trimita comanda suficient de repede la 255/

//variabile si constante pentru GPS
static const int RXPin = 0, TXPin = 1;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
int adresaGasita = 0; //variabila are rolul de a nota cand a fost gasita o locatie 
const int nrCautariLocatie = 30;
int nrComenziDeLaUltimaActualizareLocatie = 0;
const int nrComenziIntreActualizariLocatie = 30;

int nrStatieUrmatoare = 0;

char comanda;

int i = 0;
int notificareActualizareLocatie = 0; //folosita pentru a porni sau opri ledul incorporat al Arduino cand se actualizeaza locatia
typedef  struct{
  float latitudine;
  float longitudine;
} locatieGPS;
const int nrStatii=3;
locatieGPS statii[nrStatii];

#define TRIGGER_PIN 11 //pt HC_SR04
#define ECHO_PIN 13 //pt HC_SR04
#define MAX_DISTANCE 400 //pt HC_SR04

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); //pt HC_SR04

void seteazaStatii();

void setup() {
  
  pinMode(EnableL, OUTPUT);
  pinMode(HighL, OUTPUT);
  pinMode(LowL, OUTPUT);

  pinMode(EnableR, OUTPUT);
  pinMode(HighR, OUTPUT);
  pinMode(LowR, OUTPUT);

  /*pinMode(D0, INPUT_PULLUP);
  pinMode(D1, INPUT_PULLUP);
  pinMode(D2, INPUT_PULLUP);
  pinMode(D3, INPUT_PULLUP);
  Serial.begin(9600);*/
  //setare i2c
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Wire.begin(0x8);//se alatura caii i2c cu adresa 8
  Wire.onReceive(receiveEvent);//cheama functia receiveEvent cand datele sunt obtinute

  //setare gps(neo6)
  /*ss.begin(GPSBaud);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);*/
  Serial.begin(9600);

  seteazaStatii();
}

void receiveEvent(int cate) {
  while(Wire.available()) {
    comanda = Wire.read(); //primeste un byte ca un caracter
  }
  ++nrComenziDeLaUltimaActualizareLocatie;
}


void mergiInFata()
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,vitezaMax);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,vitezaMax);
  
}


void mergiCuSpatele()
{
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, LOW);
  analogWrite(EnableL,vitezaMax);

  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, LOW);
  analogWrite(EnableR,vitezaMax);
  
}

void opreste()
{
  digitalWrite(HighL, HIGH);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,0);

  digitalWrite(HighR, HIGH);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,0);
  
}

void mergiInStanga(short comanda)
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,(int)((float)(((float)vitezaMax)/eroareMax)*(float)comanda));

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,vitezaMax); 
}

void mergiInDreapta(short comanda)
{
  digitalWrite(HighL, LOW);
  digitalWrite(LowL, HIGH);
  analogWrite(EnableL,vitezaMax);

  digitalWrite(HighR, LOW);
  digitalWrite(LowR, HIGH);
  analogWrite(EnableR,(int)((float)(((float)vitezaMax)/eroareMax)*(float)comanda));
}


void seteazaStatii(){
  statii[0].latitudine = 44.399676;
  statii[0].longitudine = 26.102720;

  
  statii[1].latitudine = 44.399517;
  statii[1].longitudine = 26.103382;

  statii[2].latitudine = 44.399870;
  statii[2].longitudine = 26.102859;
}

float distanta(locatieGPS actuala,locatieGPS statie) {
  return 10000*sqrtf(((actuala.latitudine-statie.latitudine) * (actuala.latitudine-statie.latitudine)) + ((actuala.longitudine-statie.longitudine) * (actuala.longitudine-statie.longitudine)));
}

float distance_1;
int este_obstacol() {
  distance_1 = sonar.ping_cm();
  Serial.print("distanta ");
  Serial.println(distance_1);
  if (distance_1 <= 20 && distance_1 > 2) { //daca distanta este mai mica de 20cm
    opreste();
    delay(50);
    return 1;
  }
  //pentru ca masuratoarea distantei sa functioneze corect avem nevoie de timp care sa fie suficient intre masuratori
  //t = d/v = (2*4m) / (343m/s) = 23,32ms
  delay(50);
  return 0;
}

void loop() 
{
  //cat timp este obstacol in fata asteapta
  while(este_obstacol()==1);
  //obtinere locatie si pauza
  if (nrComenziDeLaUltimaActualizareLocatie >= nrComenziIntreActualizariLocatie) {
    adresaGasita = 0;
    for(int i = 0; i < nrCautariLocatie && adresaGasita == 0; ++i)
    {
      while (ss.available() > 0)
      {
        gps.encode(ss.read());
        if (gps.location.isUpdated())
        {
          adresaGasita=1;
          locatieGPS pozitieActuala;
          pozitieActuala.latitudine = gps.location.lat();
          pozitieActuala.longitudine = gps.location.lng();
          Serial.println("pozitie actuala urmata de pozitia urmatoarei statii");
          Serial.print(pozitieActuala.latitudine,6);
          Serial.print(' ');
          Serial.print(pozitieActuala.longitudine,6);
          Serial.println(' ');
          Serial.print(statii[nrStatieUrmatoare].latitudine,6);
          Serial.print(' ');
          Serial.print(statii[nrStatieUrmatoare].longitudine,6);
          Serial.println(' ');
          Serial.print("distanta pana la urmatoarea statie: ");
          Serial.println(distanta(pozitieActuala,statii[nrStatieUrmatoare]));
          Serial.print("Latitude: ");
          Serial.print(gps.location.lat(), 6);
          Serial.print("\nLongitude: ");
          Serial.println(gps.location.lng(), 6);
          if (distanta(pozitieActuala,statii[nrStatieUrmatoare]) < 5) {//daca distanta pana la urmatoarea statie este mai mica ca 5 metrii opreste
            opreste();//opreste masina
            delay(5*1000);//asteata 5 minute in statie
            nrStatieUrmatoare = (nrStatieUrmatoare + 1) % nrStatii;
          }
          if (notificareActualizareLocatie == 0) {
            digitalWrite(LED_BUILTIN, HIGH);
            notificareActualizareLocatie = 1;
          } else {
            digitalWrite(LED_BUILTIN, LOW);
            notificareActualizareLocatie = 0;
          }
        } 
        if(adresaGasita)
          break;  
      }
      delay(10);
    }
    nrComenziDeLaUltimaActualizareLocatie = 0;
  } 
  
  //variaza intre 0 si 2*eroareMax+1
  //Serial.println((int)((float)(((float)vitezaMax)/eroareMax)*(float)comanda));
  if (comanda == 30)
    digitalWrite(LED_BUILTIN, HIGH);
  if (comanda == eroareMax) {
    mergiInFata();
  } else if (comanda == 2*eroareMax+1) {
    opreste();
  } else if (comanda > eroareMax) {
    mergiInDreapta((eroareMax<<1) - comanda+10);
  } else if (comanda < eroareMax) {
    mergiInStanga(comanda+10);
    /*
     * cu cat comanda mai aproape de eroareMax cu atat inseamna ca vreau mai putin viraj la stanga, deci motorul stang trebuie sa mearga aproape la fel ca dreptul
     */
  }

  delay(1);
}
