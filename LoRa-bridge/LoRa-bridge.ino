#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "mbedtls/base64.h"

constexpr uint32_t RF_FREQUENCY = 925000000;              // LoRa周波数(Hz)
constexpr int TX_OUTPUT_POWER = 14;                       // 送信出力(dBm)
constexpr int LORA_BANDWIDTH = 0;                         // 125kHz
constexpr int LORA_SPREADING_FACTOR = 7;                  // SF7
constexpr int LORA_CODINGRATE = 1;                        // CR4/5
constexpr int LORA_PREAMBLE_LENGTH = 8;                   // プレアンブル長 // 同期確認
constexpr int LORA_SYMBOL_TIMEOUT = 0;                    // シンボルタイムアウト
constexpr bool LORA_FIX_LENGTH_PAYLOAD_ON = false;        // 可変長ペイロード
constexpr bool LORA_IQ_INVERSION_ON = false;              // IQ反転OFF

//constexpr int RX_TIMEOUT_VALUE = 1000;                  // 受信タイムアウト
constexpr int BUFFER_SIZE = 64;                           // バッファサイズ
constexpr int PACKET_QUEUE_SIZE = 50;                     // キューサイズ

struct Packet {
  uint8_t payload[BUFFER_SIZE];
  uint32_t node_id;
  uint16_t size;
};

Packet packet_queue[PACKET_QUEUE_SIZE];
int packet_queue_head = 0;
int packet_queue_tail = 0;

bool enQueuePacket(uint8_t *payload, uint32_t node_id, uint16_t size) {
  int next_tail = (packet_queue_tail + 1) % PACKET_QUEUE_SIZE;
  if (next_tail == packet_queue_head) return false; // queue full
  memcpy(packet_queue[packet_queue_tail].payload, payload, size);
  packet_queue[packet_queue_tail].node_id = node_id;
  packet_queue[packet_queue_tail].size = size;
  packet_queue_tail = next_tail;
  return true;
}

bool deQueuePacket(Packet *packet) {
  if (packet_queue_head == packet_queue_tail) return false; // empty
  memcpy(packet, &packet_queue[packet_queue_head], sizeof(Packet));
  packet_queue_head = (packet_queue_head + 1) % PACKET_QUEUE_SIZE;
  return true;
}

static RadioEvents_t RadioEvents;
bool lora_idle = true;
uint64_t chipid = ESP.getEfuseMac();
uint32_t node_id = (uint32_t)(chipid & 0xFFFFFFFF);

void setup() {
  Serial.begin(115200);
  //Serial1.begin(115200, SERIAL_8N1, 15, 14); 
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
  if (!enQueuePacket(payload, node_id, size)) {
    Serial.println("⚠️ Queue full, packet dropped.");
  }
  lora_idle = true;
}

// Base64エンコードしてUARTに送信
void sendPacketUART(Packet *packet) {
  unsigned char encoded[128];
  size_t encoded_len = 0;

  int ret = mbedtls_base64_encode(encoded, sizeof(encoded), &encoded_len,
                                  packet->payload, packet->size);
  if (ret != 0) {
    Serial.println("Base64 encode failed!");
    return;
  }

  // Node IDを16進文字列に変換
  char node_id_str[12];
  sprintf(node_id_str, "%08X", packet->node_id);
  // UART送信フォーマット
  Serial2.printf("RX:%s|%u|%s\n", node_id_str, packet->size, encoded);

  Serial.printf("[Bridge] Sent via UART -> NodeID:%s | Size:%u | Encoded:%s\n",
                node_id_str, packet->size, encoded);
}
