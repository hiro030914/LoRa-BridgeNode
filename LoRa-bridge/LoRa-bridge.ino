#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "mbedtls/base64.h"

#define RF_FREQUENCY                925000000 // Hz
#define TX_OUTPUT_POWER             14        // dBm
#define LORA_BANDWIDTH              0         // 125 kHz
#define LORA_SPREADING_FACTOR       7
#define LORA_CODINGRATE             1         // 4/5
#define LORA_PREAMBLE_LENGTH        8
#define LORA_SYMBOL_TIMEOUT         0
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false

#define RX_TIMEOUT_VALUE            1000
#define BUFFER_SIZE                 64
#define PACKET_QUEUE_SIZE           10

constexpr byte HEADER = 0xAA;

# pragma pack(1)
struct SensorPacket {
  uint32_t node_id;
  float temp_data;
  float humi_data;
};

struct Packet {
  uint16_t size;
  SensorPacket payload;
  int16_t rssi;
  int8_t snr;
};
#pragma pack()

Packet packet_queue[PACKET_QUEUE_SIZE];
int packet_queue_head = 0;
int packet_queue_tail = 0;

bool enQueuePacket(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  int next_tail = (packet_queue_tail + 1) % PACKET_QUEUE_SIZE;
  if (next_tail == packet_queue_head) return false; // queue full
  packet_queue[packet_queue_tail].size = size;
  memcpy(&packet_queue[packet_queue_tail].payload, payload, size);
  packet_queue[packet_queue_tail].rssi = rssi;
  packet_queue[packet_queue_tail].snr = snr;
  packet_queue_tail = next_tail;
  return true;
}

bool deQueuePacket(Packet *packet) {
  if (packet_queue_head == packet_queue_tail) return false; // empty
  memcpy(packet, &packet_queue[packet_queue_head], sizeof(Packet));
  packet_queue_head = (packet_queue_head + 1) % PACKET_QUEUE_SIZE;
  return true;
}

// UART送信
void sendPacketUART(Packet *packet) {
  char Header = 'H';
  char Footer = 'F';
  Serial2.write(&Header);
  delay(1000);
  ssize_t n = Serial2.write((uint8_t *)&packet->payload, sizeof(SensorPacket));
  Serial.printf("send : %zd byte", n);
  Serial2.write(&Footer);
  Serial2.flush();

  Serial.printf("[Bridge] UART送信: NodeID=%lu, Temp=%.2f, Humi=%.2f, RSSI=%d, SNR=%d\n",
                packet->payload.node_id,
                packet->payload.temp_data,
                packet->payload.humi_data,
                packet->rssi,
                packet->snr);

  uint8_t *p = (uint8_t *)&packet->payload ;
  Serial.print("payload hex : ");
  for (size_t i = 0; i < sizeof(SensorPacket); i++){
    Serial.printf("%02X ", p[i]);
  }
  Serial.println();

}

static RadioEvents_t RadioEvents;
bool lora_idle = true;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 47, 48); // UART通信
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Serial.println("Bridge Node Ready. Waiting for LoRa packets...");
}

void loop() {
  Packet packet;
  if (lora_idle) {
    lora_idle = false;
    Radio.Rx(0);
  }
  Radio.IrqProcess();

  // デキューしてUART送信
  if (deQueuePacket(&packet)) {
    sendPacketUART(&packet);
  }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  Radio.Sleep();
  if (!enQueuePacket(payload, size, rssi, snr)) {
    Serial.println("⚠️ Queue full, packet dropped.");
  }
  lora_idle = true;
}
