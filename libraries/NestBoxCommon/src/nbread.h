// READ command - one diagnostic request into the car on the server's (or the local API's)
// command, the same code in NestBox_v1F and NestBox_v2O (2026-10-18, FleetNest
// docs/rozhodnutia.md "Príkazy do auta").
//
//   READ=<bus>,<request ID hex>,<response ID hex>,<request bytes hex>
//   e.g. READ=1,710,77A,2202BD  (VW gateway, UDS 22 02BD odometer)
//
// ONLY reading services are sent (nbReadAllowed); anything else is refused and
// never reaches the car: 0x22 ReadDataByIdentifier, 0x21 ReadDataByLocalIdentifier
// (KWP / PSA), 0x01 and 0x09 OBD current data / vehicle info, 0x19 ReadDTCInformation.
// Control actions (lock, ignition ...) come later as a separate command with a
// fixed list of verified actions - never through READ.
//
// The exchange is ISO-TP over 11-bit IDs with flow control sent by us, the way
// v1F canodo.cpp udsRead() does it (verified in the Passat 2026-09-28): single-shot
// frames padded with 0x55, response pending (7F xx 78) waits up to 3 s.
// Result into the next record, PID 0x39D: bytes <seq> <result> <nrc> <data...>
// (data = the whole response, up to NB_READ_RECORD_MAX bytes), and an SD event line
// with the request and the whole response.
#pragma once
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define PID_READ_RESULT 0x39D
#define NB_READ_MAX 128                // response bytes kept
#define NB_READ_RECORD_MAX 56          // of them into the record (the SD line has all)
#define NB_READ_REPEAT_MS 120000UL     // the same command again within this time: not sent again

enum NbReadResult {
  NBR_OK = 0,
  NBR_NOT_READY = 1,     // bus not usable now (e.g. v1F sniff mode)
  NBR_TX_FAILED = 2,
  NBR_NO_REPLY = 3,
  NBR_NRC = 4,           // negative response, code in nrc
  NBR_CF_TIMEOUT = 5,
  NBR_CF_ORDER = 6,
  NBR_BAD_REPLY = 7,
  NBR_FC_FAILED = 9,
  NBR_SYNTAX = 10,       // the command could not be parsed
  NBR_NOT_ALLOWED = 11,  // not a reading service - refused, nothing sent
  NBR_BAD_BUS = 12,      // this box has no such bus
};

struct NbRead {
  uint8_t bus;
  uint32_t txId, rxId;
  uint8_t req[7];
  uint8_t reqLen;
};

static inline bool nbReadAllowed(const uint8_t* req, int len)
{
  if (len < 1) return false;
  switch (req[0]) {
    case 0x22: case 0x21: case 0x01: case 0x09: case 0x19: return true;
    default: return false;
  }
}

static inline int nbHex(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// "1,710,77A,2202BD" (after "READ=") -> r; false = syntax
static inline bool nbReadParse(const char* s, NbRead* r)
{
  memset(r, 0, sizeof(*r));
  char* e;
  r->bus = (uint8_t)strtoul(s, &e, 10);
  if (*e != ',') return false;
  r->txId = strtoul(e + 1, &e, 16);
  if (*e != ',' || r->txId > 0x7FF) return false;
  r->rxId = strtoul(e + 1, &e, 16);
  if (*e != ',' || r->rxId > 0x7FF) return false;
  const char* h = e + 1;
  int n = 0;
  while (h[0] && h[0] != ',' && h[0] != '*' && h[0] != '\r' && h[0] != '\n') {
    int a = nbHex(h[0]), b = nbHex(h[1]);
    if (a < 0 || b < 0 || n >= 7) return false;   // single-frame requests only (<= 7 bytes)
    r->req[n++] = (uint8_t)(a * 16 + b);
    h += 2;
  }
  r->reqLen = (uint8_t)n;
  return n > 0;
}

// the CAN port the exchange runs on (each firmware provides it)
struct NbCanPort {
  bool (*send)(void* ctx, uint32_t id, const uint8_t d[8]);
  // the next frame from `id` (11-bit) within timeoutMs; false = none
  bool (*recv)(void* ctx, uint32_t id, uint8_t d[8], uint8_t* dlc, uint32_t timeoutMs);
  void* ctx;
};

// one request/response; resp = the whole positive response (service + 0x40, ...)
static inline int nbReadExchange(NbCanPort* p, const NbRead* r, uint8_t* resp, int cap, int* respLen, uint8_t* nrc)
{
  const uint8_t PAD = 0x55;
  const uint32_t TMO = 300, PENDING = 3000;
  *respLen = 0;
  *nrc = 0;
  uint8_t f[8];
  memset(f, PAD, 8);
  f[0] = r->reqLen;
  memcpy(f + 1, r->req, r->reqLen);
  if (!p->send(p->ctx, r->txId, f)) return NBR_TX_FAILED;
  uint8_t d[8], dlc = 0;
  uint32_t wait = TMO;
  int want = 0, got = 0;
  for (;;) {
    if (!p->recv(p->ctx, r->rxId, d, &dlc, wait)) return NBR_NO_REPLY;
    if (dlc < 2) continue;
    uint8_t pci = d[0] >> 4;
    if (pci == 0) {                                   // single frame
      int n = d[0] & 0x0F;
      if (n >= 3 && d[1] == 0x7F && d[3] == 0x78) { wait = PENDING; continue; }   // response pending
      if (n > 7) n = 7;
      memcpy(resp, d + 1, n < cap ? n : cap);
      *respLen = n < cap ? n : cap;
      if (n >= 3 && d[1] == 0x7F) { *nrc = d[3]; return NBR_NRC; }
      break;
    }
    if (pci == 1) {                                   // first frame: flow control, the rest
      want = ((d[0] & 0x0F) << 8) | d[1];
      if (want > cap) want = cap;
      got = want < 6 ? want : 6;
      memcpy(resp, d + 2, got);
      uint8_t fc[8] = {0x30, 0x00, 0x00, PAD, PAD, PAD, PAD, PAD};
      if (!p->send(p->ctx, r->txId, fc)) return NBR_FC_FAILED;
      uint8_t seq = 1;
      while (got < want) {
        if (!p->recv(p->ctx, r->rxId, d, &dlc, TMO)) { *respLen = got; return NBR_CF_TIMEOUT; }
        if ((d[0] >> 4) != 2 || (d[0] & 0x0F) != (seq & 0x0F)) { *respLen = got; return NBR_CF_ORDER; }
        int n = want - got < 7 ? want - got : 7;
        memcpy(resp + got, d + 1, n);
        got += n;
        seq++;
      }
      *respLen = got;
      break;
    }
  }
  if (*respLen < 1 || resp[0] != (uint8_t)(r->req[0] + 0x40)) return NBR_BAD_REPLY;
  return NBR_OK;
}

// "READ #<seq> bus=1 710->77A req=2202BD result=0 nrc=00 len=7 resp=6202BD..." (SD event line)
static inline int nbReadLine(char* out, int cap, uint32_t seq, const NbRead* r, int result, uint8_t nrc,
                             const uint8_t* resp, int len)
{
  int n = snprintf(out, cap, "READ #%u bus=%u %X->%X req=", (unsigned)seq, r->bus, (unsigned)r->txId, (unsigned)r->rxId);
  for (int i = 0; i < r->reqLen && n < cap - 3; i++) n += snprintf(out + n, cap - n, "%02X", r->req[i]);
  n += snprintf(out + n, cap - n, " result=%d nrc=%02X len=%d resp=", result, nrc, len);
  for (int i = 0; i < len && n < cap - 3; i++) n += snprintf(out + n, cap - n, "%02X", resp[i]);
  return n;
}

// ---- the command's way through the box (both firmwares) --------------------------------
// the network / web task: nbReadSubmit("1,710,77A,2202BD") - checked at once; the task that
// owns the CAN bus: nbReadTake() and, when done, nbReadDone(); the record task: nbReadForRecord().
#ifdef ARDUINO
#include <Arduino.h>

#define NB_READ_RESULT_Q 8             // results waiting for records (one per record, oldest first)
struct NbReadState {
  portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
  char lastCmd[48] = "";
  uint32_t lastAt = 0;
  uint32_t seq = 0;
  bool pending = false;          // a request waits for the bus owner
  NbRead req;
  uint32_t reqSeq = 0;
  // results for the records: a queue - several commands can come within one record interval
  // (2026-10-18 desk test: with one slot #1 and #2 were overwritten by #3 before a record went)
  uint8_t rec[NB_READ_RESULT_Q][3 + NB_READ_RECORD_MAX];
  int recLen[NB_READ_RESULT_Q];
  uint8_t head = 0, count = 0;
};
extern NbReadState g_nbRead;   // the one instance: nbread.cpp

static inline void nbReadSetResult(uint32_t seq, int result, uint8_t nrc, const uint8_t* resp, int len)
{
  portENTER_CRITICAL(&g_nbRead.mux);
  uint8_t slot = (g_nbRead.head + g_nbRead.count) % NB_READ_RESULT_Q;
  if (g_nbRead.count == NB_READ_RESULT_Q) g_nbRead.head = (g_nbRead.head + 1) % NB_READ_RESULT_Q;  // full: drop the oldest
  else g_nbRead.count++;
  uint8_t* r = g_nbRead.rec[slot];
  r[0] = (uint8_t)seq;
  r[1] = (uint8_t)result;
  r[2] = nrc;
  int n = len < NB_READ_RECORD_MAX ? len : NB_READ_RECORD_MAX;
  if (n > 0) memcpy(r + 3, resp, n);
  g_nbRead.recLen[slot] = 3 + (n > 0 ? n : 0);
  portEXIT_CRITICAL(&g_nbRead.mux);
}

// returns NBR_OK when accepted (or the same command again within NB_READ_REPEAT_MS: already
// done / running, not sent again); else the refusal, which also goes into the next record
static inline int nbReadSubmit(const char* cmd, uint32_t* seqOut)
{
  uint32_t now = millis();
  portENTER_CRITICAL(&g_nbRead.mux);
  bool repeat = !strncmp(g_nbRead.lastCmd, cmd, sizeof(g_nbRead.lastCmd) - 1) && now - g_nbRead.lastAt < NB_READ_REPEAT_MS;
  portEXIT_CRITICAL(&g_nbRead.mux);
  if (repeat) { if (seqOut) *seqOut = g_nbRead.seq; return NBR_OK; }   // the server's resend: not sent to the car again
  NbRead r;
  portENTER_CRITICAL(&g_nbRead.mux);
  uint32_t seq = ++g_nbRead.seq;
  portEXIT_CRITICAL(&g_nbRead.mux);
  if (seqOut) *seqOut = seq;
  int bad = !nbReadParse(cmd, &r) ? NBR_SYNTAX : !nbReadAllowed(r.req, r.reqLen) ? NBR_NOT_ALLOWED : NBR_OK;
  portENTER_CRITICAL(&g_nbRead.mux);
  strncpy(g_nbRead.lastCmd, cmd, sizeof(g_nbRead.lastCmd) - 1);
  g_nbRead.lastAt = now;
  if (!bad) { g_nbRead.req = r; g_nbRead.reqSeq = seq; g_nbRead.pending = true; }
  portEXIT_CRITICAL(&g_nbRead.mux);
  if (bad) nbReadSetResult(seq, bad, 0, nullptr, 0);
  return bad;
}

static inline bool nbReadTake(NbRead* r, uint32_t* seq)
{
  bool got = false;
  portENTER_CRITICAL(&g_nbRead.mux);
  if (g_nbRead.pending) { *r = g_nbRead.req; *seq = g_nbRead.reqSeq; g_nbRead.pending = false; got = true; }
  portEXIT_CRITICAL(&g_nbRead.mux);
  return got;
}

// the record's PID 0x39D bytes, the oldest waiting result; 0 = nothing new
static inline int nbReadForRecord(uint8_t* out, int cap)
{
  int n = 0;
  portENTER_CRITICAL(&g_nbRead.mux);
  if (g_nbRead.count) {
    int l = g_nbRead.recLen[g_nbRead.head];
    n = l < cap ? l : cap;
    memcpy(out, g_nbRead.rec[g_nbRead.head], n);
    g_nbRead.head = (g_nbRead.head + 1) % NB_READ_RESULT_Q;
    g_nbRead.count--;
  }
  portEXIT_CRITICAL(&g_nbRead.mux);
  return n;
}
#endif
