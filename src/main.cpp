/* ESP32 MotorScan - Desenvolvido por Daniel Costa*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "WiFi.h"
#include "MPU9250.h"
#include <esp_now.h>

#define SEALEVELPRESSURE_HPA (1013.25)

//Credenciais para aceder ao AP do Gateway
const char* ssid = "ESP32-Access-Point";
const char* password = "123456789";

// Definir nº da placa (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1

// Substituir com o Mac Address do ESP32 do Access Point
uint8_t broadcastAddress[] = {0x3C,0x71, 0xBF, 0x1D, 0xFD, 0x24}; //3C:71:BF:1D:FD:24 (ESP32 Sensor --> Enviar para Gateway)


// Exemplo de tipo de dados da estrutura 
// Must match the receiver structure
typedef struct struct_message {
    int id; // Cada board terá um nº de identificação, não podendo haver duplicados
    float temp; //Temperatura
    float pressure; //Pressão
    float altitude; //Altitude
    float humidity; //Humidade
    float accel_X; //Acelerómetro X (Radial X)
    float accel_Y; //Acelerómetro Y (radial Y)
    float accel_Z; //Acelerómetro Z (Axial)
} struct_message;

//Criação de uma struct_message chamada myData
struct_message myData;

// callback (feedback) quando os dados foram enviados
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nEstado do último pacote enviado:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Entregue" : "Falhou a entrega");
}

MPU9250 IMU(Wire,0x68); // endereço I2C para o MPU 9250
Adafruit_BME280 bme; // endereço I2C para o BME 280

int status;

//Imprime todas os dados lidos dos sensores na consola
void printValues() {
    Serial.print("Temperatura = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressão atmosférica = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Altitude aproximada = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.print("Humidade= ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.print("Acel. Eixo Radial X= ");
    Serial.print(IMU.getAccelX_mss());
    Serial.println();

    Serial.print("Acel. Eixo Radial Y= ");
    Serial.print(IMU.getAccelY_mss());
    Serial.println();

    Serial.print("Acel. Eixo Axial= ");
    Serial.print(IMU.getAccelZ_mss());
    Serial.println();
}

void setup() {
  //Iniciar as comunicações (Baud Rate=115200)
  Serial.begin(115200);
  Serial.println();
  
  // Define o dispositivos como uma estação Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

// Conectando ao Wi-Fi do Gateway
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Falha ao conectar ao Gateway...");
  }

  // Iniciando o protocolo ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar o ESP-NOW");
    return;
  }

  // Uma vez o ESP-NOW inicidado com sucesso, existirá um registo para enviar um callback para obter o estado dos pacotes enviados
  esp_now_register_send_cb(OnDataSent);

  // Registar o dispositivo (peer)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

   // Adicionar dispositivo (peer)        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Falha ao adicionar sensor");
    return;
  }

// Iniciação do Sensor BME 280
 Serial.println("-- BME280 --");
  bool status;
  delay(50);
  status = bme.begin(0x76);  

  if (!status) {
    Serial.println("Não foi possível encontrar o BME280!");
    while (1);
    
  }

// Iniciação do Sensor MPU 9250
 status = IMU.begin();

  if (status < 0) {
    Serial.println("IMU Inicialização falhada");
    Serial.print("Estado: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  //Leitura de dados para a  estrutura a enviar via ESP-NOW para o Gatweway
  IMU.readSensor();
  myData.id = 1; //indica o ID da placa, cada uma deverá ter um ID diferente.
  myData.temp = bme.readTemperature();
  myData.pressure = bme.readPressure() / 100.0F;
  myData.altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
  myData.humidity = bme.readHumidity();
  myData.accel_X = IMU.getAccelX_mss();
  myData.accel_Y = IMU.getAccelY_mss();
  myData.accel_Z = IMU.getAccelZ_mss();
  printValues();

  // Envia mensagem com os dados da estrutura via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  //Estado do envio da mensagem
  if (result == ESP_OK) {
    Serial.println("Enviado com sucesso");
  }
  else {
    Serial.println("Erro ao enviar os dados");
  }
  //Aguarda 10 segundos até o próximo envio
  delay(10000);
  }