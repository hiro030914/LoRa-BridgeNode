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

constexpr char Header = 'H';                             // ヘッダ
constexpr char Footer = 'F';                             // フッタ

// SensorPacket構造体はセンサノードが送信するデータ型
// packet構造体はLoRaイベントの受信引数と一致する形式
// pragma pack(1)により構造体のパディングを削除
# pragma pack(1)
struct SensorPacket {
  uint32_t node_id;          // センサノード識別子
  float temp_data;           // 温度データ
  float humi_data;           // 湿度データ
};
struct Packet {
  uint16_t size;             // 受信パケットサイズ
  SensorPacket payload;      // 受信ペイロード
  int16_t rssi;              // 信号強度
  int8_t snr;                // sn比
};
#pragma pack()

Packet packet_queue[PACKET_QUEUE_SIZE];      // 循環キュー
int packet_queue_head = 0;                   // 先頭キュー
int packet_queue_tail = 0;                   // 末尾キュー

/**
 * @brief 受信した各データをキュー配列に格納する関数
 * @param payload 受信したデータ本体のポインタ
 * @param size データサイズ
 * @param rssi 信号強度
 * @param snr sn比
 * @return true キューへの追加が成功した時
 * @return false キューが満杯になった時
 * @details 末尾の変更は末尾キューのインデントをキューサイズで割った剰余にすることでキューを循環
 */
bool enqueuePacket(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  int next_tail = (packet_queue_tail + 1) % PACKET_QUEUE_SIZE;
  if (next_tail == packet_queue_head) return false; // queue full
  packet_queue[packet_queue_tail].size = size;
  memcpy(&packet_queue[packet_queue_tail].payload, payload, size);
  packet_queue[packet_queue_tail].rssi = rssi;
  packet_queue[packet_queue_tail].snr = snr;
  packet_queue_tail = next_tail;
  return true;
}

/**
 * @brief キューからパケットを取り出す関数
 * @param packet キューから取り出すPacket構造体のアドレス 
 * @return true キューからの取り出しが成功した時
 * @return false キューが空になった時
 * @details 先頭の変更は先頭キューのインデントをキューサイズで割った剰余にすることでキューを循環
 */
bool dequeuePacket(Packet *packet) {
  if (packet_queue_head == packet_queue_tail) return false;          // キューが空
  memcpy(packet, &packet_queue[packet_queue_head], sizeof(Packet));
  packet_queue_head = (packet_queue_head + 1) % PACKET_QUEUE_SIZE;
  return true;
}

/**
 * @brief UART通信によってRaspiにパケットを送信する関数
 * @param packet 送信するPacket構造体のアドレス
 * @details 呼び出されたら初めにヘッダパケットを送信し，Raspiに送信開始を周知し0.2sステイ
 *          バイナリにキャストしたパケットをUARTで送信
 *          フッタを送信し送信終了を周知
 */
void sendPacketUART(Packet *packet) {
  Serial2.write(&Header);
  delay(200);
  ssize_t n = Serial2.write((uint8_t *)&packet->payload, sizeof(SensorPacket));
  Serial.printf("send : %zd byte", n);
  Serial2.write(&Footer);
  Serial2.flush();

  // デバッグ用
  Serial.printf("[Bridge] UART送信 : NodeID=%lu, Temp=%.2f, Humi=%.2f\n"
                packet->payload.node_id,
                packet->payload.temp_data,
                packet->payload.humi_data,
                );

  Serial.printf("[other] : RSSI=%d, SNR=%d\n",
                packet->rssi,
                packet->snr);

  uint8_t *p = (uint8_t *)&packet->payload ;
  Serial.print("payload hex : ");
  for (size_t i = 0; i < sizeof(SensorPacket); i++){
    Serial.printf("%02X ", p[i]);
  }
  Serial.println();

}

static RadioEvents_t RadioEvents;              // LoRa送受信イベントハンドラ登録

bool lora_idle = true;                         // LoRa状態フラグ(true:アイドル，false:受信待機)

void setup() {
  Serial.begin(115200);                         // シリアルモニタ初期化(ボーレート)
  Serial2.begin(115200, SERIAL_8N1, 47, 48);    // UART通信初期化(ボーレート:115200，データフォーマット:SERIAL_8N1, Txピン:47，Rxピン:48)
                                                // ※データフォーマット: ex)8N1 (データビット幅:8bit，パリティビット:no，ストップビット:1bit)
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);       // Heltec固有のボード初期化 ex)LoRaモジュール，OLED etc..

  RadioEvents.RxDone = OnRxDone;                                         // OnRxDone関数を受信時コールバック関数として登録
  Radio.Init(&RadioEvents);                                              // 各コールバック関数の呼び出し
  // 各LoRaパラメータ設定
  Radio.SetChannel(RF_FREQUENCY);                                        // LoRa周波数設定
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
    Radio.Rx(0);           // 受信開始
  }
  Radio.IrqProcess();      // 割り込み処理

  // デキューしてUART送信
  if (dequeuePacket(&packet)) {
    sendPacketUART(&packet);
  }
}

/**
 * @brief LoRaの受信用イベント
 * @param payload 受信したデータ本体
 * @param size データサイズ
 * @param rssi 信号強度
 * @param snr sn比
 * @details if文でエンキュー関数の呼び出しを行い，受信した各データをエンキューする
 *          falseが返ってきた場合はキューが満杯であることを通知
 *          処理を終えたらアイドル状態にする
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  Radio.Sleep();          // LoRaチップスリープ状態
  
  // 受信した各データをエンキュー
  if (!enqueuePacket(payload, size, rssi, snr)) {
    Serial.println("⚠️ Queue full, packet dropped.");
  }
  lora_idle = true;       // アイドル状態
}
