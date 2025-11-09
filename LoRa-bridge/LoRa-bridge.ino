#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "mbedtls/base64.h"

// LoRaパラメータ設定
// 固定パラメータのためconstexprによる定義
constexpr uint32_t RF_FREQUENCY = 925000000;             // LoRa周波数(Hz)
constexpr int8_t TX_OUTPUT_POWER = 14;                   // 送信出力(dBm)
constexpr int LORA_BANDWIDTH = 0;                        // 125 kHz
constexpr int LORA_SPREADING_FACTOR = 7;                 // SF7
constexpr int LORA_CODINGRATE = 1;                       // CR4/5
constexpr int LORA_PREAMBLE_LENGTH = 8;                  // プレアンブル長 //同期確認
constexpr int LORA_SYMBOL_TIMEOUT = 0;                   // シンボルタイムアウト
constexpr bool LORA_FIX_LENGTH_PAYLOAD_ON = false;       // 可変長ペイロード
constexpr bool LORA_IQ_INVERSION_ON = false;             // IQ反転オフ

constexpr uint32_t RX_TIMEOUT_VALUE = 1000;              // 受信タイムアウト(ms)
constexpr uint16_t BUFFER_SIZE = 64;                     // バッファサイズ
constexpr uint8_t PACKET_QUEUE_SIZE = 10;                // キューサイズ

// SensorPacket構造体は受信データと同じ型
// ネスト構造でpayload変数をSensorPacket構造体で定義
// プラグマによりパディング削除
# pragma pack(1)
struct SensorPacket {
  uint32_t node_id;          // センサノード識別子
  float temp_data;           // 温度データ
  float humi_data;           // 湿度データ
};
// LoRaのイベントの引数と一致させる
struct Packet {
  uint16_t size;             // 受信パケットサイズ
  SensorPacket payload;      // 受信ペイロード
  int16_t rssi;              // 信号強度
  int8_t snr;                // sn比
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
