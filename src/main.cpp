/*

Codigo para control y adquisicion de datos para sensores y actuadores en esp32.
Conexion con otro esp32 servidor web para enviar trama de datos de las variables adquiridas y procesadas, a demas de mostrarlas
en pantalla lcd.


Discord: Yeray#6132
..

*/
#include <Arduino.h>
#include "LiquidCrystal_I2C.h"
LiquidCrystal_I2C lcd(0x27, 20, 4);
#include <freertos/task.h>
#include <HardwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>
const int oneWirePin = 5; // PIN PARA SENSOR DE TEMPERATURA DS18B20
OneWire oneWireBus(oneWirePin);
DallasTemperature sensor(&oneWireBus);
DeviceAddress sensorTuberia = {0x28, 0xF8, 0x95, 0x4B, 0xB1, 0x21, 0x09, 0x52}; // direccion del sensor de temperatura

#include <SHT40.h>     // libreria propia para el sensor SHT40
SHT40 sht40;           // objeto para el sensor SHT40
#include <TECO_L510.h> // libreria propia para el variador de frecuencia TECO L510
TECO_L510 teco;
#include <FH400.h> // libreria propia para el sensor FH400
FH400 fh400;

#define EnTxPinn 33 // pin de habilitacion de transmision para rs485
bool flag_clear = true;
bool flag2_clear = true;
bool flag_var = false;

// pin para adc fh400
#define pinBlanco 32 // pin Velocidad fh400
#define pinVerde 35  // pin Temperatura fh400
#define pinCafe 34   // pin Humedad fh400

// BOTON PARA LCD
#define btn_enter 27 // sin usar
#define btn_back 26
void interrupcion_enter();
void interrupcion_back();

ulong prev_time, current_time, prev_time2;
const uint16_t dt1 = 4000;
const uint16_t dt2 = 9000;
const uint16_t dt3 = 400;
int conteo = 0;

float ponderadoTemp = 0;    // variables para ponderar temperatura y humedad
float ponderadoHumedad = 0; // variables para ponderar temperatura y humedad
byte sendData[18];          // buffer para enviar y recibir tramas
byte sendData2[14];         // buffer para enviar y recibir tramas
byte sendData3[50];
byte data[18]; // buffer para enviar y recibir tramas
uint8_t on_off_boton = 0;
uint8_t on_off_boton_next = 0;                                                                                                     // variables de control tramas y webserver
uint8_t ubi_pagina = 0;                                                                                                            // variables de control tramas y webserver
uint8_t ubi_pagina_next = 0;                                                                                                       // variables de control tramas y webserver
float temperatura_estadoMx_setpoint = 0;                                                                                           // variables para recepcion de trama y para sistema de control
float humedad_estadoMx_setpoint = 0;                                                                                               // variables para recepcion de trama y para sistema de control
float termisor1 = 0;                                                                                                               // trama
uint8_t estadoMx = 0;                                                                                                              // trama
const int timeOut = 100;                                                                                                           // tiempo de espera para recepcion de trama
char ACK = 'F';                                                                                                                    // caracter de confirmacion para trama
char NAK = 'E';                                                                                                                    // caracter de confirmacion para trama
void errorRecepcion();                                                                                                             // funciones para recepcion de trama
void okRecepcion();                                                                                                                // funciones para recepcion de trama
int ProccesSerialData(const int timeOut, byte *buffer, const uint8_t bufferLength, void (*okCallBack)(), void (*errorCallBack)()); // funcion de control para recibir o enviar trama
void recibir_trama();
enum SerialState // tipo de dato para recepcion de trama
{
  OKEY,
  ERROR,
  NO_RESPONSE
};

float temp_emulada = 0; // variable para sistema de  control
float hum_emulada = 0;
float temp_emulada_anterior = 0; // variable para sistema de  control
float hum_emulada_anterior = 0;
void promedio_ponderado(float &ptemperatura_general, float &phumedad_general); // funcion para ponderar temperatura y humedad
void sensores_actuadores();                                                    // funcion para mostrar los sensores y actuadores en uart
void tramaSensoresToServer();                                                  // funcion que se encarga de armar la trama de los sensores a servidor
void seleccion_estado();                                                       // muestra el nombre del estado de mexico en el lcd segun la variable de control estadoMx creo
void mostrarClimaEmular(String estadoMx);                                      // funcion para mostrar el clima emulado con base a la trama recibida
void mostrarSensores();
void fromLongToBytes(byte *bytes, long ing);
void fromFloatToBytes(byte *bytes, float f);             // trama para enviar los datos de los sensores al servidor
void fragmentar_trama_recibida(byte buffer[]);           // funcion para fragmentar la trama de clima pronosticada por la base  de datos del servidor
int ProccesACK(const int timeOut, void (*okCallBack)()); // para validar la recepcion de trama
int TryGetACK(int TimeOut);
void i2c_requestFrom();
void _i2cTramaControlToServer(); // para validar la recepcion de trama
void setup()
{

  // i2c Maestro
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // 16 es RX y 17 es TX
  pinMode(btn_back, INPUT_PULLUP);         // no los uso
  pinMode(btn_enter, INPUT_PULLUP);        // no los uso
  attachInterrupt(digitalPinToInterrupt(btn_enter), interrupcion_enter, RISING);
  attachInterrupt(digitalPinToInterrupt(btn_back), interrupcion_back, RISING);
  fh400.setup(pinBlanco, pinVerde, pinCafe); // Config Pines Input Adc
  teco.setup(EnTxPinn);                      // Config Pines Input
  sht40.setup(EnTxPinn);                     // Config Pines Input
  sensor.begin();                            // inicializar sensor de temperatura DS18B20
  lcd.init();
  lcd.backlight();
  // pinMode(EnTx.Pin, OUTPUT); // pin de habilitacion de transmision

  Serial.printf("\n Esp32 Control para Emulacion de Variables Climaticas \n\n");
}

void loop()
{
  // recibir_trama(); //USAR PARA UART
  fh400.datafh400.speed = fh400.filtro_viento();

  seleccion_estado();

  if (millis() - prev_time2 > 1000)
  {
    i2c_requestFrom();
    _i2cTramaControlToServer();
    prev_time2 = millis();
  }
}

void i2c_requestFrom()
{
  Wire.requestFrom(20, 18);
  while (Wire.available())
  {
    for (int i = 0; i < 18; i++)
    {
      data[i] = Wire.read(); // Leer cada byte recibido y almacenarlo en el arreglo
    }
  }
  Serial.printf("\n\t I2C TRAMA recibida \n ");
  fragmentar_trama_recibida(data);
}
void _i2cTramaControlToServer()
{
  byte seleccion_pagina[4];
  byte boton[4];
  byte temp[4];
  byte hum[4];

  // hacer la desfragmentacion  de bytes necesaria para enviar la trama
  fromLongToBytes(seleccion_pagina, ubi_pagina);
  fromLongToBytes(boton, on_off_boton);
  fromFloatToBytes(temp, ponderadoTemp);
  fromFloatToBytes(hum, ponderadoHumedad);

  sendData[0] = 'I';                 // ascii 73
  sendData[1] = seleccion_pagina[0]; // para saber en que pagina se encuentra el usuario
  sendData[2] = seleccion_pagina[1];
  sendData[3] = seleccion_pagina[2];
  sendData[4] = seleccion_pagina[3];

  sendData[5] = boton[0]; // para saber si el boton esta encendido o apagado
  sendData[6] = boton[1];
  sendData[7] = boton[2];
  sendData[8] = boton[3];

  sendData[9] = temp[0];
  sendData[10] = temp[1];
  sendData[11] = temp[2];
  sendData[12] = temp[3];

  sendData[13] = hum[0];
  sendData[14] = hum[1];
  sendData[15] = hum[2];
  sendData[16] = hum[3];
  sendData[17] = ACK;  // ascii 70
  sendData[18] = '\n'; // ascii 10

  // impresion();
  Wire.beginTransmission(20); // Comenzar a comunicarse con esclavo #23
  Wire.write(sendData, 18);
  Wire.endTransmission(); // Finalizar comunicación
}
///  ////////////
void seleccion_estado()
{
  String nombreEstadoMx;
  if (ubi_pagina == 1)
  {
    nombreEstadoMx = "Sinaloa:";
    mostrarClimaEmular(nombreEstadoMx); // muestra el clima en lcd y comienza a emular (sistema de control)
  }
  else if (ubi_pagina == 2)
  {
    nombreEstadoMx = "Yucatan:";
    mostrarClimaEmular(nombreEstadoMx); // muestra el clima en lcd y comienza a emular (sistema de control)
  }
  else if (ubi_pagina == 3)
  {
    nombreEstadoMx = "Otro:";
    mostrarClimaEmular(nombreEstadoMx); // muestra el clima en lcd y comienza a emular (sistema de control)
  }
  else
  {
    if (flag_clear == true)
    { // para solo limpiar una sola vez y que no se explote la lcd
      lcd.clear();
      flag_clear = false;
    }
    lcd.setCursor(7, 1);
    lcd.print("Cimav");
  }
}

void recibir_trama()
{
  digitalWrite(EnTxPinn, LOW); // habilitar recepcion de trama
  if (Serial2.available() > 0)
  {
    byte buffer[18];
    size_t n = Serial2.readBytesUntil('\n', buffer, 18); // devuelve el numero de bytes leidos
    for (int8_t i = 0; i < n; i++)
    {
      Serial.print(buffer[i]);
      Serial.print(" ");
    }
    if (n == 18)
    {

      ProccesSerialData(timeOut, buffer, 18, okRecepcion, errorRecepcion);
    }
  }
}

/////////////////////////////////////////////////////////////////////
int TryGetSerialData(int TimeOut, byte *buffer) // encontrar ACK en el buffer y saber como proceder
{
  // intenta conseguir los delimitadores de la trama para verificar que la trama se recibió correctamente dentro de un tiempo
  // de 100ms, si no se recibe la trama en ese tiempo, se devuelve un error
  // si se recibe la trama, se devuelve un OK.
  // el Ok sirve para proceder a fragmentar la trama
  /* unsigned long StartTime = millis();
  while (!Serial2.available() && (millis() - StartTime) < TimeOut)
  {
  } */

  if (buffer[0] == 'I' /* && buffer[17] == ACK */)
    return OKEY;
  else
    return ERROR;
}

int ProccesSerialData(const int timeOut, byte *buffer, const uint8_t bufferLength, void (*okCallBack)(), void (*errorCallBack)())
{ // Comprueba que en el buffer se encuentre el ACK , recepciono de datos completa

  int rst = TryGetSerialData(timeOut, buffer);
  if (rst == OKEY)
  {
    Serial.print(ACK);
    if (okCallBack != NULL)
    {
      okCallBack(); // al comprobar ACK  envia ACK por UART para avisar correcta recepcion de datos y comienza a frangmentar la trama
      fragmentar_trama_recibida(buffer);
      Serial.printf("  ACK enviado \n");
    }
  }
  else if (rst == ERROR)
  {
    if (okCallBack != NULL)
    {
      errorCallBack();             // al comprobar que no esta ACK  envia NAK por UART para avisar incorrecta recepcion de datos
      digitalWrite(EnTxPinn, LOW); // deshabilitar recepcion de trama
      Serial.printf("NAK enviado \n");
    }
  }
  return rst;
}

void okRecepcion()
{

  digitalWrite(EnTxPinn, HIGH); // deshabilitar recepcion de trama
  Serial2.write(ACK);
  Serial2.write('\n');
  Serial.printf("Recepcion correcta\n");
}

void errorRecepcion()
{
  digitalWrite(EnTxPinn, HIGH); // deshabilitar recepcion de trama
  Serial2.write(NAK);
  Serial2.write('\n');
  Serial.printf("Recepcion incorrecta \n");
}

void mostrarClimaEmular(String nombreEstadoMx)
{ // funcion importante. Aqui comienza para tener la opcion de emular( sistema de control)

  if (flag_clear == true)
  { // para solo limpiar una sola vez y que no se explote la lcd
    lcd.clear();
    flag_clear = false;
  }

  lcd.setCursor(0, 0);
  lcd.print(nombreEstadoMx);

  lcd.setCursor(3, 1);
  lcd.print("Tem.P:");
  lcd.print(temperatura_estadoMx_setpoint);
  lcd.print("C");
  lcd.setCursor(3, 2);
  lcd.print("Hum.P:");
  lcd.print(humedad_estadoMx_setpoint);
  lcd.print("%");
  lcd.setCursor(2, 3);
  lcd.print("Listo Para Emular");

  if (on_off_boton == 0)
  {
    vTaskDelay(pdMS_TO_TICKS(50));
    teco.stopVariador();
  }

  while (on_off_boton == 1) // comenzar sistema de control
  {
    // Mostrando datos de la temperatura y humedad pronosticada
    float tempX = temperatura_estadoMx_setpoint;
    float humX = humedad_estadoMx_setpoint;

    if (flag_clear == false)
    { // para solo limpiar una sola vez y que no se explote la lcd
      lcd.clear();
      flag_clear = true;
    }

    lcd.setCursor(0, 0);
    lcd.print(nombreEstadoMx);

    lcd.setCursor(0, 1);
    lcd.print("T.P:");
    lcd.setCursor(4, 1);
    lcd.print(tempX);
    lcd.setCursor(9, 1);
    lcd.print("C");
    lcd.setCursor(11, 1);
    lcd.print("H.P:");
    lcd.print(humX);
    lcd.setCursor(19, 1);
    lcd.print("%");
    // Mostrando datos de la temperatura y humedad emulada
    lcd.setCursor(0, 3);
    lcd.print("T.E:");
    lcd.setCursor(4, 3);
    lcd.print(ponderadoTemp); // para mostrar la temperatura emulada
    lcd.setCursor(9, 3);
    lcd.print("C");
    lcd.setCursor(11, 3);
    lcd.print("H.E:");
    lcd.print(ponderadoHumedad); // para mostrar la humedad emuladad
    lcd.setCursor(19, 3);
    lcd.print("%");

    ////////////////////////////////////////////////
    // ACCION DE CONTROL AQUI
    sensores_actuadores(); // Ejemplo leer todos los sensores y actuadores
    /////////////////////////////////////////////////

    recibir_trama(); // para usar con uart
    i2c_requestFrom();
    _i2cTramaControlToServer();

    vTaskDelay(pdMS_TO_TICKS(1100));
  }
}
void interrupcion_enter()
{
  // para entrar y salir a while para adquirir y procesar las variables sensadas
  //  auxiliar mientras no se haya corregido la comunicion uart entre esp32
  on_off_boton++;
  Serial.printf(" +1 on_off_boton: %d \n", on_off_boton);
  if (on_off_boton == 3)
  {
    on_off_boton = 1;
  }
}
void interrupcion_back()
{

  ubi_pagina++;
  Serial.printf(" +1 ubi_pagina: %d \n", ubi_pagina);
  if (ubi_pagina == 3)
  {
    ubi_pagina = 1;
  }
}
void fromFloatToBytes(byte *bytes, float f)
{
  int length = sizeof(float);
  for (int i = 0; i < length; i++)
    bytes[i] = ((byte *)&f)[i];
}
void fromLongToBytes(byte *bytes, long ing)
{
  bytes[0] = (byte)((ing & 0xff000000) >> 24);
  bytes[1] = (byte)((ing & 0x00ff0000) >> 16);
  bytes[2] = (byte)((ing & 0x0000ff00) >> 8);
  bytes[3] = (byte)((ing & 0x000000ff));
}

unsigned long getUlong(byte packet[], byte i)
{
  // big endian
  unsigned long value = 0;
  value = (value * 256) + packet[i];
  value = (value * 256) + packet[i + 1];
  value = (value * 256) + packet[i + 2];
  value = (value * 256) + packet[i + 3];
  return value;
}

unsigned int getInt(byte packet[], byte i)
{
  unsigned int value = 0;
  value = (value * 256) + packet[i];
  return value;
}

float getFloat(byte packet[], byte i)
{
  union tag
  {
    byte bin[4];
    float num;
  } u;

  u.bin[0] = packet[i];
  u.bin[1] = packet[i + 1];
  u.bin[2] = packet[i + 2];
  u.bin[3] = packet[i + 3];
  return u.num;
}

void fragmentar_trama_recibida(byte buffer[])
{
  ubi_pagina = getUlong(buffer, 1);
  Serial.printf("\n ubi_pagina: %d \n", ubi_pagina);

  on_off_boton = getUlong(buffer, 5);
  Serial.printf("boton en : %d \n", on_off_boton);

  temperatura_estadoMx_setpoint = getFloat(buffer, 9);
  Serial.printf("temp recibida: %0.2f \n", temperatura_estadoMx_setpoint);

  humedad_estadoMx_setpoint = getFloat(buffer, 13);
  Serial.printf("hum recibida: %0.2f \n", humedad_estadoMx_setpoint);
}

void tramaSensoresToServer()
{

  byte temp1[4];
  byte hum1[4];
  byte temp2[4];
  byte hum2[4];
  byte temp3[4];
  byte hum3[4];
  byte temp4[4];
  byte hum4[4];
  byte temp5[4];
  byte hum5[4];
  byte termi[4];
  byte viento[4];
  /* byte tempfh[4];
  byte humfh[4]; */

  fromFloatToBytes(temp1, 30.20 /* sht40.dataSensor32.FT */);
  fromFloatToBytes(hum1, 30.10 /* sht40.dataSensor32.FRH */);
  fromFloatToBytes(temp2, 20.01 /* sht40.dataSensor33.FT */);
  fromFloatToBytes(hum2, 20.02 /* sht40.dataSensor33.FRH */);
  fromFloatToBytes(temp3, 40.11 /* sht40.dataSensor34.FT */);
  fromFloatToBytes(hum3, 40.12 /* sht40.dataSensor34.FRH */);
  fromFloatToBytes(temp4, 50.00 /* sht40.dataSensor35.FT */);
  fromFloatToBytes(hum4, 50.01 /* sht40.dataSensor35.FRH */);
  fromFloatToBytes(temp5, 09.01 /* sht40.dataSensor36.FT */);
  fromFloatToBytes(hum5, 09.02 /* sht40.dataSensor36.FRH */);
  fromFloatToBytes(termi, 88.7); // CORRECCION PARA DPESUES
  fromFloatToBytes(viento, 11.11 /* fh400.datafh400.speed */);
  /*  fromFloatToBytes(tempfh,   fh400.datafh400.temp ) */
  /* fromFloatToBytes(humfh, fh400.datafh400.hum )  */

  sendData3[0] = 'I'; // ascii 73
  sendData3[1] = temp1[0];
  sendData3[2] = temp1[1];
  sendData3[3] = temp1[2];
  sendData3[4] = temp1[3];

  sendData3[5] = hum1[0];
  sendData3[6] = hum1[1];
  sendData3[7] = hum1[2];
  sendData3[8] = hum1[3];

  sendData3[9] = temp2[0];
  sendData3[10] = temp2[1];
  sendData3[11] = temp2[2];
  sendData3[12] = temp2[3];

  sendData3[13] = hum2[0];
  sendData3[14] = hum2[1];
  sendData3[15] = hum2[2];
  sendData3[16] = hum2[3];

  sendData3[17] = temp3[0];
  sendData3[18] = temp3[1];
  sendData3[19] = temp3[2];
  sendData3[20] = temp3[3];

  sendData3[21] = hum3[0];
  sendData3[22] = hum3[1];
  sendData3[23] = hum3[2];
  sendData3[24] = hum3[3];

  sendData3[25] = temp4[0];
  sendData3[26] = temp4[1];
  sendData3[27] = temp4[2];
  sendData3[28] = temp4[3];

  sendData3[29] = hum4[0];
  sendData3[30] = hum4[1];
  sendData3[31] = hum4[2];
  sendData3[32] = hum4[3];

  sendData3[33] = temp5[0];
  sendData3[34] = temp5[1];
  sendData3[35] = temp5[2];
  sendData3[36] = temp5[3];

  sendData3[37] = hum5[0];
  sendData3[38] = hum5[1];
  sendData3[39] = hum5[2];
  sendData3[40] = hum5[3];

  sendData3[41] = termi[0];
  sendData3[42] = termi[1];
  sendData3[43] = termi[2];
  sendData3[44] = termi[3];

  sendData3[45] = viento[0];
  sendData3[46] = viento[1];
  sendData3[47] = viento[2];
  sendData3[48] = viento[3];
  /*
    sendData3[49] = tempfh[0];
    sendData3[50] = tempfh[1];
    sendData3[51] = tempfh[2];
    sendData3[52] = tempfh[3];

    sendData3[53] = humfh[0];
    sendData3[54] = humfh[1];
    sendData3[55] = humfh[2];
    sendData3[56] = humfh[3]; */

  sendData3[49] = ACK;  // ascii 70
  sendData3[50] = '\n'; // ascii 10

  Serial2.write(sendData3, 50);
  Serial.printf("Trama sensores enviada ");
}

int ProccesACK(const int timeOut, void (*okCallBack)())
{

  int rst = TryGetACK(timeOut);
  if (rst == OKEY)
  {
    if (okCallBack != NULL)
      okCallBack(); // se comprueba si la función de devolución de llamada okCallBack no es un puntero nulo
  }
  else if (rst == ERROR)
  {
    if (okCallBack != NULL)
      // errorCallBack();
      tramaSensoresToServer();
    Serial.printf("Reenviado Trama por error en recepcion\n");
  }
  return rst;
}

int TryGetACK(int TimeOut)
{
  unsigned long StartTime = millis();
  digitalWrite(EnTxPinn, LOW); // habilita el envio de datos
  while (!Serial.available() && (millis() - StartTime) < TimeOut)
  {
  }

  if (Serial2.available())
  {
    if (Serial2.read() == ACK)
      return OKEY;
    if (Serial2.read() == NAK)
      return ERROR;
  }
  return NO_RESPONSE;
}

void okAction()
{
  Serial.printf("\n ACK recibido por el otro esp32 \n");
}
void mostrarSensores()
{

  if (flag_clear == true)
  { // para solo limpiar una sola vez y que no se explote la lcd
    lcd.clear();
    flag_clear = false;
    flag2_clear = false;
  }

  lcd.setCursor(0, 0);
  lcd.print("Sensores Actuadores");

  lcd.setCursor(2, 1);
  lcd.print("Enviando data");

  if (millis() - prev_time > 1200)
  {
    tramaSensoresToServer();
    ProccesACK(timeOut, okAction);
    prev_time = millis();
  }
}

void tramaEmulada_control()
{ // trama para enviar de los valores generados por el sistema de control de temperatura y humedad

  byte temp_control[4];
  byte hum_control[4];
  byte var_extra[4];
  // byte var_extra2[4];
  fromFloatToBytes(temp_control, 30.20);
  fromFloatToBytes(hum_control, 30.10);
  fromFloatToBytes(var_extra, 20.01);
  // fromFloatToBytes(hum2, 20.02 );
  sendData2[0] = 'I'; // ascii 73
  sendData2[1] = temp_control[0];
  sendData2[2] = temp_control[1];
  sendData2[3] = temp_control[2];
  sendData2[4] = temp_control[3];

  sendData2[5] = hum_control[0];
  sendData2[6] = hum_control[1];
  sendData2[7] = hum_control[2];
  sendData2[8] = hum_control[3];

  sendData2[9] = var_extra[0];
  sendData2[10] = var_extra[1];
  sendData2[11] = var_extra[2];
  sendData2[12] = var_extra[3];
  sendData2[13] = ACK;
  sendData2[14] = '\n';
  Serial2.write(sendData2, 14);
}

void sensores_actuadores() // esta funcion es para demostrar el funcionamiento. No es necesaria para el sistema de control
{
  float viento = 0;
  float t33, h33;
  float t34, h34;
  float t35, h35;
  float t36, h36;
  float t37, h37;
  // mandamos peticion a los sensores para que nos envien los datos de temperatura y humedad
  /*  sht40.SendGetSerial(sht40.adrSensor_x32);
   vTaskDelay(pdMS_TO_TICKS(50)); */
  sht40.SendGetSerial(sht40.adrSensor_x33);
  vTaskDelay(pdMS_TO_TICKS(50));
  t33 = sht40.dataSensor33.FT;
  h33 = sht40.dataSensor33.FRH;
  sht40.SendGetSerial(sht40.adrSensor_x34);
  vTaskDelay(pdMS_TO_TICKS(50));
  t34 = sht40.dataSensor34.FT;
  h34 = sht40.dataSensor34.FRH;
  sht40.SendGetSerial(sht40.adrSensor_x35);
  vTaskDelay(pdMS_TO_TICKS(50));
  t35 = sht40.dataSensor35.FT;
  h35 = sht40.dataSensor35.FRH;
  sht40.SendGetSerial(sht40.adrSensor_x36);
  vTaskDelay(pdMS_TO_TICKS(50));
  t36 = sht40.dataSensor36.FT;
  h36 = sht40.dataSensor36.FRH;
  sht40.SendGetSerial(sht40.adrSensor_x37);
  vTaskDelay(pdMS_TO_TICKS(50));
  t37 = sht40.dataSensor37.FT;
  h37 = sht40.dataSensor37.FRH;

  // mostrar en pueto serial los datos de los sensores
  // Serial.printf("Temperatura addr x32: %0.2f , HR: %0.2f\n", sht40.dataSensor32.FT, sht40.dataSensor32.FRH);
  Serial.printf("Temperatura addr x33: %0.2f , HR: %0.2f\n", t33, h33);
  Serial.printf("Temperatura addr x34: %0.2f , HR: %0.2f\n", t34, h35);
  Serial.printf("Temperatura addr x35: %0.2f , HR: %0.2f\n", t35, h35);
  Serial.printf("Temperatura addr x36: %0.2f , HR: %0.2f\n", t36, h36);
  Serial.printf("Temperatura addr x37: %0.2f , HR: %0.2f\n", t37, h37);
  // promedio ponderado de temperatura y humedad
  promedio_ponderado(ponderadoTemp, ponderadoHumedad);

  // Sensar velocidad del viento con sensor FH400
  fh400.datafh400.speed = fh400.filtro_viento();
  viento = fh400.conversion_velocidad();
  Serial.printf("Velocidad del viento: %0.2f\n", fh400.datafh400.speed);
  Serial.printf("Velocidad del viento2: %0.2f\n", viento);

  // sensor temperatura DS18B20 de tuberia a intercambiador de calor
  sensor.requestTemperatures();
  float tuberia_temp = sensor.getTempC(sensorTuberia);
  Serial.printf("Temperatura tuberia: %0.2f\n", tuberia_temp);

  // Variador de frecuencia aumentando cada segundo
  if (millis() - prev_time > 2000)
  {
    conteo = conteo + 100;
    teco.set_freq_variador(conteo);
    vTaskDelay(pdMS_TO_TICKS(50));
    teco.runVariador();
    Serial.printf("Variador  a %d Hz\n", conteo / 100);
    prev_time = millis();
  }
  if (conteo > 5000)
  {
    conteo = 500; // para bajar la velocidad
  }

  vTaskDelay(pdMS_TO_TICKS(1000));
}

void promedio_ponderado(float &ptemperatura_general, float &phumedad_general)
{
  // Promedio ponderado de temperatura y humedad
  //  se usar referencias para retornar los valores de temperatura y humedad
  // toma en cuenta cinco sensores
  float suma_pesos = 0;
  uint8_t num_sensores = 5;

  struct SensorTemperatura
  {
    int sensor_id;
    float temperatura;
    float humedad;
    float peso; // es la importancia relativa que tiene cada sensor con respecto a donde se encuentran ubicados
  };
  SensorTemperatura sensores[num_sensores]; // arreglo de sensores

  sensores[0].sensor_id = 1;
  sensores[0].temperatura = sht40.dataSensor33.FT;
  sensores[0].humedad = sht40.dataSensor33.FRH;
  sensores[0].peso = 0.5;

  sensores[1].sensor_id = 2;
  sensores[1].temperatura = sht40.dataSensor34.FT;
  sensores[1].humedad = sht40.dataSensor34.FRH;
  sensores[1].peso = 0.5;

  sensores[2].sensor_id = 3;
  sensores[2].temperatura = sht40.dataSensor37.FT;
  sensores[2].humedad = sht40.dataSensor37.FRH;
  sensores[2].peso = 0.5;

  sensores[3].sensor_id = 4;
  sensores[3].temperatura = sht40.dataSensor36.FT;
  sensores[3].humedad = sht40.dataSensor36.FRH;
  sensores[3].peso = 0.5;

  sensores[4].sensor_id = 5;
  sensores[4].temperatura = sht40.dataSensor35.FT;
  sensores[4].humedad = sht40.dataSensor35.FRH;
  sensores[4].peso = 0.5;

  // calculo del promedio ponderado
  for (uint8_t i = 0; i < num_sensores; i++)
  {
    ptemperatura_general = ptemperatura_general + (sensores[i].temperatura * sensores[i].peso);
    phumedad_general = phumedad_general + (sensores[i].humedad * sensores[i].peso);
    suma_pesos = suma_pesos + sensores[i].peso;
  }
  ptemperatura_general = ptemperatura_general / suma_pesos;
  phumedad_general = phumedad_general / suma_pesos;
  Serial.printf("Temperatura General: %f \n", ptemperatura_general);
  Serial.printf("Humedad General General: %f \n", phumedad_general);
}

bool al_cambiarTrama_enviar()
{ // al cambiar cualquier variable de control, temperatura o humedad enviar trama

  if (on_off_boton != on_off_boton_next)
  {
    //_tramaServerToControl();
    on_off_boton_next = on_off_boton;
    ubi_pagina_next = ubi_pagina;
    return true;
  }

  if ((temp_emulada != temp_emulada_anterior) || (hum_emulada != hum_emulada_anterior))
  {
    //_tramaServerToControl();
    temp_emulada_anterior = temp_emulada;
    hum_emulada_anterior = hum_emulada;
    return true;
  }

  return false; // ninguno de los casos entonces no se ejecutara la funcion de ProccesACK
}
