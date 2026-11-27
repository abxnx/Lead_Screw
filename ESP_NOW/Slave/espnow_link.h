#pragma once
/*
  espnow_link.h — ชั้นสื่อสาร ESP-NOW ของ slave (ไม่ยุ่งกับโค้ดคุม lead screw)

  เพิ่มในไฟล์หลักแค่ 3 บรรทัด:
    1) #include "espnow_link.h"      ใต้ #include <ESP32Encoder.h>
    2) linkBegin();                  บรรทัดสุดท้ายของ setup()
    3) linkPoll();                   บรรทัดแรกของ loop()

  การทำงาน:
    - คำสั่งจาก master เป็นข้อความเดียวกับที่พิมพ์ใน Serial ("G 25", "H", "X", ...)
      → ตอบ ACK แล้วส่งเข้า handleCommand() เดิม ใน loop() (context เดียวกับ Serial)
    - telemetry ทุก 10 ms ส่งจาก FreeRTOS task แยก → ยังส่งต่อเนื่องแม้ loop ถูก block ตอน homing
    - ถ้างานที่สั่งจาก master กำลังวิ่งอยู่แล้วไม่ได้ยิน master เกิน LINK_TIMEOUT_MS → สั่ง "X"
*/
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_arduino_version.h>
#include <FastAccelStepper.h>
#include "protocol.h"

// ----- ของที่มีอยู่แล้วในไฟล์หลัก (ประกาศล่วงหน้า) -----
float positionMM();
float encoderMM();
float stepsToMM(long steps);
void  handleCommand(String line);
extern bool homed;
extern FastAccelStepper* stepper;

// ----- ตั้งค่า -----
const uint32_t LINK_TELEM_MS    = 10;
const uint32_t LINK_TIMEOUT_MS  = 1000;   // 0 = ปิดฟีเจอร์ stop เมื่อ link หลุด

// ----- ภายใน -----
struct LinkRx { uint8_t mac[6]; uint8_t len; uint8_t data[FRAME_MAX_LEN]; };
static QueueHandle_t     linkQ          = nullptr;
static uint8_t           linkMaster[6];
static volatile bool     linkPaired     = false;
static volatile uint32_t linkLastRxMs   = 0;
static uint32_t          linkTelemSeq   = 0;
static uint32_t          linkLastSeq    = 0;
static bool              linkHaveSeq    = false;
static bool              linkRadioJob   = false;   // งานล่าสุดสั่งมาจาก master

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void linkOnRecv(const esp_now_recv_info_t* info, const uint8_t* d, int len) {
  const uint8_t* mac = info->src_addr;
#else
static void linkOnRecv(const uint8_t* mac, const uint8_t* d, int len) {
#endif
  uint8_t type;
  if (!frameValid(d, len, type)) return;
  linkLastRxMs = millis();
  LinkRx it;
  memcpy(it.mac, mac, 6);
  it.len = len;
  memcpy(it.data, d, len);
  xQueueSend(linkQ, &it, 0);          // callback รันใน WiFi task: copy แล้วจบ
}

// จับคู่กับ master ตัวแรกที่ทักมา → ไม่ต้องใส่ MAC ฝั่ง slave
static void linkPair(const uint8_t* mac) {
  if (linkPaired) return;
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, mac, 6);
  p.channel = 0;
  p.encrypt = false;
  if (esp_now_add_peer(&p) == ESP_OK || esp_now_is_peer_exist(mac)) {
    memcpy(linkMaster, mac, 6);
    linkPaired = true;
    Serial.printf("[link] paired with master %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
}

static void linkTelemTask(void*) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(LINK_TELEM_MS));
    if (!linkPaired || stepper == nullptr) continue;
    TelemFrame f;
    memset(&f, 0, sizeof f);
    hdrInit(f.h, MSG_TELEM, linkTelemSeq++);
    f.t_ms      = millis();
    f.target_mm = stepsToMM(stepper->targetPos());
    f.cmd_mm    = positionMM();
    f.enc_mm    = encoderMM();
    f.flags     = (stepper->isRunning() ? FL_RUNNING : 0) | (homed ? FL_HOMED : 0);
    esp_now_send(linkMaster, (const uint8_t*)&f, sizeof f);
  }
}

void linkBegin() {
  linkQ = xQueueCreate(8, sizeof(LinkRx));
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[link] ESP-NOW init FAILED (ยังใช้ Serial ได้ตามปกติ)");
    return;
  }
  esp_now_register_recv_cb(linkOnRecv);
  xTaskCreate(linkTelemTask, "telem", 4096, nullptr, 2, nullptr);
  Serial.printf("[link] slave MAC %s  ← ใส่ใน SLAVE_MAC ของ master\n", WiFi.macAddress().c_str());
}

void linkPoll() {
  LinkRx it;
  while (linkQ && xQueueReceive(linkQ, &it, 0) == pdTRUE) {
    linkPair(it.mac);
    if (!linkPaired || memcmp(it.mac, linkMaster, 6) != 0) continue;
    if (it.data[2] != MSG_CMD) continue;            // HB: แค่ต่อเวลา link

    CmdFrame c;
    memcpy(&c, it.data, sizeof c);
    c.text[CMD_TEXT_MAX - 1] = '\0';
    bool dup = linkHaveSeq && c.h.seq == linkLastSeq;

    // ตอบ ACK ก่อนทำงาน เพราะคำสั่งอย่าง H จะ block นาน
    AckFrame a;
    memset(&a, 0, sizeof a);
    hdrInit(a.h, MSG_ACK, c.h.seq);
    a.status = dup ? ACK_DUP : ACK_OK;
    esp_now_send(linkMaster, (const uint8_t*)&a, sizeof a);
    if (dup) continue;                               // master ส่งซ้ำเพราะ ACK หาย → ไม่ทำซ้ำ

    linkLastSeq = c.h.seq;
    linkHaveSeq = true;
    linkRadioJob = true;
    Serial.printf("[link] cmd: %s\n", c.text);
    handleCommand(String(c.text));                   // ← parser เดิม ไม่แก้อะไร
  }

  // master หายระหว่างที่งานจาก master กำลังวิ่ง → หยุดแบบลดความเร็ว
  if (LINK_TIMEOUT_MS && linkRadioJob && stepper && stepper->isRunning() &&
      millis() - linkLastRxMs > LINK_TIMEOUT_MS) {
    Serial.println("[link] master lost → stop");
    handleCommand("X");
    linkRadioJob = false;
  }
}
