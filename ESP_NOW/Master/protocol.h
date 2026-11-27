#pragma once
/*
  protocol.h — frame ESP-NOW ที่ใช้ร่วมกันระหว่าง master และ slave
  ต้องเป็นไฟล์เดียวกันทั้งสองฝั่ง: copy ไปไว้ในโฟลเดอร์ sketch ทั้งสองตัว

  master -> slave : MSG_CMD  ข้อความคำสั่งดิบ เช่น "G 25" → slave ส่งเข้า handleCommand() เดิมเลย
                    MSG_HB   heartbeat ทุก 200 ms (ให้ slave จับคู่ + รู้ว่า link ยังอยู่)
  slave  -> master: MSG_ACK  ตอบทุก CMD (seq เดียวกัน)
                    MSG_TELEM ทุก 10 ms

  master ส่ง CMD ซ้ำ (seq เดิม) ถ้าไม่ได้ ACK, slave จำ seq ล่าสุดไว้ → ของซ้ำจะไม่ถูกทำซ้ำ
*/
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define PROTO_MAGIC    0xA5
#define PROTO_VER      2
#define ESPNOW_CHANNEL 1          // ทั้งสองตัวต้องตรงกัน
#define CMD_TEXT_MAX   40         // รวม '\0'

enum MsgType : uint8_t { MSG_CMD = 1, MSG_ACK = 2, MSG_TELEM = 3, MSG_HB = 4 };
enum AckStatus : uint8_t { ACK_OK = 0, ACK_DUP = 1 };

// bit ใน TelemFrame.flags
#define FL_RUNNING 0x01
#define FL_HOMED   0x02

struct __attribute__((packed)) Hdr {
  uint8_t  magic, ver, type, rsv;
  uint32_t seq;
};

struct __attribute__((packed)) CmdFrame {
  Hdr  h;
  char text[CMD_TEXT_MAX];
};

struct __attribute__((packed)) AckFrame {      // h.seq = seq ของคำสั่งที่ตอบ
  Hdr     h;
  uint8_t status, rsv[3];
};

struct __attribute__((packed)) TelemFrame {
  Hdr      h;
  uint32_t t_ms;        // millis() ของ slave ตอน sample → ใช้เป็นแกนเวลา
  float    target_mm;   // เป้าหมายของ move ล่าสุด
  float    cmd_mm;      // ตำแหน่งที่สั่ง (step ที่ปล่อยไปแล้ว)
  float    enc_mm;      // ตำแหน่งจริงจาก encoder
  uint8_t  flags, rsv[3];
};

static_assert(sizeof(Hdr)        == 8,  "Hdr");
static_assert(sizeof(CmdFrame)   == 48, "CmdFrame");
static_assert(sizeof(AckFrame)   == 12, "AckFrame");
static_assert(sizeof(TelemFrame) == 28, "TelemFrame");
#define FRAME_MAX_LEN 48

inline void hdrInit(Hdr& h, uint8_t type, uint32_t seq) {
  h.magic = PROTO_MAGIC; h.ver = PROTO_VER; h.type = type; h.rsv = 0; h.seq = seq;
}

inline bool frameValid(const uint8_t* d, int len, uint8_t& type) {
  if (len < (int)sizeof(Hdr) || d[0] != PROTO_MAGIC || d[1] != PROTO_VER) return false;
  type = d[2];
  switch (type) {
    case MSG_CMD:   return len == (int)sizeof(CmdFrame);
    case MSG_ACK:   return len == (int)sizeof(AckFrame);
    case MSG_TELEM: return len == (int)sizeof(TelemFrame);
    case MSG_HB:    return len == (int)sizeof(Hdr);
  }
  return false;
}

// บรรทัดที่ master พิมพ์ให้ Python:  D,t_ms,seq,target_mm,cmd_mm,enc_mm,flags
inline int formatTelemLine(char* buf, size_t n, const TelemFrame& f) {
  return snprintf(buf, n, "D,%lu,%lu,%.3f,%.3f,%.3f,%u\n",
                  (unsigned long)f.t_ms, (unsigned long)f.h.seq,
                  (double)f.target_mm, (double)f.cmd_mm, (double)f.enc_mm, (unsigned)f.flags);
}
