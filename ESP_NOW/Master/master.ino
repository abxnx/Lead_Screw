/*
  Lead-screw MASTER — ESP32 ตัวไหนก็ได้ (ต่อแค่ USB กับคอม)
  ==========================================================
    คอม --(ข้อความคำสั่ง)--> master --ESP-NOW--> slave --> handleCommand() เดิม
    คอม <--(บรรทัด D/A/E/I)-- master <--ESP-NOW-- slave (telemetry ทุก 10 ms)

  คำสั่งพิมพ์เหมือน Serial Monitor ของ slave เลย: G 25 | M -5 | V 10 | A 50 | H | X | Z | S
  master ไม่ตีความคำสั่ง แค่ส่งข้อความต่อ (ยาวไม่เกิน 39 ตัวอักษร)

  บรรทัดที่ master พิมพ์ออก (plot.py อ่าน):
    D,t_ms,seq,target_mm,cmd_mm,enc_mm,flags   telemetry
    A,seq,<คำสั่ง>                             slave ได้รับคำสั่งแล้ว
    E,...                                      ส่งไม่ถึง / link หลุด
    I,...                                      ข้อมูลทั่วไป

  Arduino-ESP32 core 3.x (รองรับ 2.x ด้วย)
*/
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include "protocol.h"

uint8_t SLAVE_MAC[6] = {0x68, 0x67, 0x25, 0x26, 0x3D, 0x28};   // TODO: ดูจาก Serial ของ slave ตอนบูต

const uint32_t SERIAL_BAUD    = 115200;
const uint32_t ACK_TIMEOUT_MS = 30;
const uint8_t  MAX_TRIES      = 4;
const uint32_t HB_PERIOD_MS   = 200;   // ต้องสั้นกว่า LINK_TIMEOUT_MS ของ slave
const uint32_t LINK_WARN_MS   = 300;   // ไม่ได้ telemetry นานเกินนี้ = link หลุด

struct RxItem { uint8_t len; uint8_t data[FRAME_MAX_LEN]; };
QueueHandle_t rxQ;
volatile uint32_t rxDropped = 0;

struct Pending { bool active; CmdFrame f; uint32_t tSent; uint8_t tries; };
Pending  pend = {};
uint32_t txSeq;
uint32_t lastTxMs = 0, lastSlaveMs = 0;
bool     linkUp = false;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t* info, const uint8_t* d, int len) {
  const uint8_t* mac = info->src_addr;
#else
void onRecv(const uint8_t* mac, const uint8_t* d, int len) {
#endif
  if (memcmp(mac, SLAVE_MAC, 6) != 0) return;
  if (len <= 0 || len > FRAME_MAX_LEN) return;
  RxItem it;
  it.len = len;
  memcpy(it.data, d, len);
  if (xQueueSend(rxQ, &it, 0) != pdTRUE) rxDropped++;   // callback: copy แล้วจบ
}

void txFrame(const void* p, size_t n) {
  esp_now_send(SLAVE_MAC, (const uint8_t*)p, n);
  lastTxMs = millis();
}

void sendCmd(const char* text) {
  if (pend.active) Serial.printf("I,seq %lu superseded\n", (unsigned long)pend.f.h.seq);
  memset(&pend.f, 0, sizeof pend.f);
  hdrInit(pend.f.h, MSG_CMD, ++txSeq);
  strncpy(pend.f.text, text, CMD_TEXT_MAX - 1);
  pend.active = true;
  pend.tries  = 1;
  pend.tSent  = millis();
  txFrame(&pend.f, sizeof pend.f);
}

void pollSerial() {
  static char line[CMD_TEXT_MAX];
  static uint8_t n = 0;
  static bool tooLong = false;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') {
      if (n < sizeof(line) - 1) line[n++] = ch; else tooLong = true;
      continue;
    }
    line[n] = 0;
    if (tooLong)  Serial.println("E,TOO_LONG");
    else if (n)   sendCmd(line);
    n = 0; tooLong = false;
  }
}

void handleRx(const RxItem& it) {
  uint8_t type;
  if (!frameValid(it.data, it.len, type)) return;
  lastSlaveMs = millis();
  if (!linkUp) { linkUp = true; Serial.println("I,link up"); }

  if (type == MSG_TELEM) {
    TelemFrame f;
    memcpy(&f, it.data, sizeof f);
    char line[80];
    formatTelemLine(line, sizeof line, f);
    Serial.print(line);
  } else if (type == MSG_ACK) {
    AckFrame a;
    memcpy(&a, it.data, sizeof a);
    if (pend.active && a.h.seq == pend.f.h.seq) {
      pend.active = false;
      Serial.printf("A,%lu,%s\n", (unsigned long)a.h.seq, pend.f.text);
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  rxQ = xQueueCreate(32, sizeof(RxItem));

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("E,ESPNOW_INIT_FAILED");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onRecv);

  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, SLAVE_MAC, 6);
  p.channel = 0;
  p.encrypt = false;
  if (esp_now_add_peer(&p) != ESP_OK) Serial.println("E,ADD_PEER_FAILED");

  txSeq = esp_random();   // กัน slave มองคำสั่งแรกหลัง master รีบูตว่าเป็นของซ้ำ
  Serial.printf("I,master MAC %s\n", WiFi.macAddress().c_str());
}

void loop() {
  pollSerial();

  RxItem it;
  while (xQueueReceive(rxQ, &it, 0) == pdTRUE) handleRx(it);

  uint32_t now = millis();

  if (pend.active && now - pend.tSent >= ACK_TIMEOUT_MS) {      // ไม่ได้ ACK → ส่งซ้ำ seq เดิม
    if (pend.tries < MAX_TRIES) {
      pend.tries++;
      pend.tSent = now;
      txFrame(&pend.f, sizeof pend.f);
    } else {
      Serial.printf("E,NO_ACK,%s\n", pend.f.text);
      pend.active = false;
    }
  }

  if (now - lastTxMs >= HB_PERIOD_MS) {                         // heartbeat
    Hdr h;
    hdrInit(h, MSG_HB, 0);
    txFrame(&h, sizeof h);
  }

  if (linkUp && now - lastSlaveMs > LINK_WARN_MS) {
    linkUp = false;
    Serial.println("E,LINK_DOWN");
  }

  static uint32_t reported = 0;
  if (rxDropped != reported) {
    reported = rxDropped;
    Serial.printf("I,rx queue overflow total=%lu\n", (unsigned long)reported);
  }

  delay(1);   // ให้ IDLE task ได้รัน (กัน WDT)
}
