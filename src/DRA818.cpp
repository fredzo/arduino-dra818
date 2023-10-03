/*
  DRA818.cpp

  Copyright (c) 2017, Jerome LOYET

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"
#include "DRA818.h"
#include <stdio.h>
#include <Stream.h>
#include <HardwareSerial.h>
#if !defined (ESP32)
#include <SoftwareSerial.h>
#endif

#ifdef DRA818_SIMU
  #include <DRA818Simu.h>
#endif

#define CTCSS_MAX           38
#define SQUELCH_MAX         8
#define VOLUME_MAX          8
#define VOLUME_MIN          1
#define TIMEOUT             2000
#define HANDSHAKE_REPEAT    3
#define SERIAL_SPEED        9600
#define SERIAL_CONFIG       SERIAL_8N1

// Commands / responses
#define RSP_VERSION         "+VERSION:"
#define RSP_RSSI            "RSSI:"

#define CHECK(a, op, b) if (a op b) a = b

#ifdef DRA818_SIMU
  #ifdef DRA818_DEBUG
    #define LOG(action, msg) if (this->log) this->log->action(msg)
    #define SEND(msg) ({\
      dra818SimuWrite(msg); \
      if (this->log) this->log->write(msg);  \
      })
    #define READ() ({int retval; \
      retval=dra818SimuRead(); \
      /*if (this->log) this->log->write(retval);*/  \
      retval;})
  #else
    #define LOG(action, msg)
    #define SEND(msg) dra818SimuWrite(msg)
    #define READ() dra818SimuRead()
  #endif
  #define AVAILABLE() dra818SimuAvailable()
#else
  #ifdef DRA818_DEBUG
    #define LOG(action, msg) if (this->log) this->log->action(msg)
    #define SEND(msg) ({  \
      this->serial->write(msg); \
      if (this->log) this->log->write(msg);  \
      })
    #define READ() ({int retval; \
      retval=this->serial->read(); \
      if (this->log) this->log->write(retval);  \
      retval;})
  #else
    #define LOG(action, msg)
    #define SEND(msg) this->serial->write(msg)
    #define READ() this->serial->read()
  #endif
  #define AVAILABLE() this->serial->available()
#endif

DRA818::DRA818(HardwareSerial *serial, uint8_t type) {
#ifndef DRA818_SIMU
  serial->begin(SERIAL_SPEED, SERIAL_CONFIG);
#endif
  this->init((Stream *)serial, type);
}

#if !defined (ESP32)
DRA818::DRA818(SoftwareSerial *serial, uint8_t type) {
#ifndef DRA818_SIMU
  serial->begin(SERIAL_SPEED); // We can't configure bits/parity for SoftwareSerial, it's 8N1 as the DRA818
#endif
  this->init((Stream *)serial, type);
}
#endif

void DRA818::init(Stream *serial, uint8_t type) {
  this->serial = serial;
  this->type = type;
}

#ifdef DRA818_DEBUG
void DRA818::set_log(Stream *log) {
  this->log = log;
  LOG(println, F("DRA818: log serial connection active"));
}
#endif

int DRA818::read_response() {
  char ack[3];
  ack[0] = ack[1] = ack[2] = '\0';  // Just to quiet some warnings.

  LOG(print, F("<- "));

  ack[2]=0;
  long start = millis();
  do {
    if (AVAILABLE()) {
      ack[0] = ack[1];
      ack[1] = ack[2];
      ack[2] = READ();
      LOG(write, ack[2]);
    }
  } while (ack[2] != 0xa && (millis() - start) < TIMEOUT);
#ifdef DRA818_DEBUG
  if (ack[2] != 0xa) LOG(write, "\r\n");
#endif
  LOG(print, F("Returned value="));
  LOG(println, ack[0] == '0' );

  return (ack[0] == '0');
}

String DRA818::read_string_response() {
  char buffer[64];
  int bufferIndex = 0;
  char curChar;

  LOG(print, F("<- "));

  long start = millis();
  do {
    if (AVAILABLE()) {
      curChar = READ();
      buffer[bufferIndex] = curChar;
      LOG(write, curChar);
      bufferIndex++;
    }
  } while (curChar != 0xa && (millis() - start) < TIMEOUT);
#ifdef DRA818_DEBUG
  if (curChar != 0xa) LOG(write, "\r\n");
#endif
  buffer[bufferIndex] = 0; // String terminator
  String result = String(buffer);
  LOG(print, F("Returned value="));
  LOG(println, result.c_str());

  return result;
}


int DRA818::group(uint8_t bw, float freq_tx, float freq_rx, uint8_t ctcss_tx, uint8_t squelch, uint8_t ctcss_rx) {
  char buffer[49];
  char buf_rx[9];
  char buf_tx[9];

  CHECK(bw, <, DRA818_12K5);
  CHECK(bw, >, DRA818_25K);

  CHECK(freq_rx, <, ((this->type & DRA818_BAND_FLAG) == DRA818_VHF ? DRA818_VHF_MIN : DRA818_UHF_MIN));
  CHECK(freq_tx, <, ((this->type & DRA818_BAND_FLAG) == DRA818_VHF ? DRA818_VHF_MIN : DRA818_UHF_MIN));

  CHECK(freq_rx, >, ((this->type & DRA818_BAND_FLAG) == DRA818_VHF ? DRA818_VHF_MAX : ((this->type & SA_MODEL_FLAG) == SA_MODEL_FLAG ? SA8X8_UHF_MAX : DRA818_UHF_MAX)));
  CHECK(freq_tx, >, ((this->type & DRA818_BAND_FLAG) == DRA818_VHF ? DRA818_VHF_MAX : ((this->type & SA_MODEL_FLAG) == SA_MODEL_FLAG ? SA8X8_UHF_MAX : DRA818_UHF_MAX)));

  CHECK(ctcss_rx, >, CTCSS_MAX);
  CHECK(ctcss_tx, >, CTCSS_MAX);

  CHECK(squelch, >, SQUELCH_MAX);

  dtostrf(freq_tx, 8, 4, buf_tx);
  dtostrf(freq_rx, 8, 4, buf_rx);

  sprintf(buffer, "AT+DMOSETGROUP=%01d,%s,%s,%04d,%c,%04d\r\n", bw, buf_tx, buf_rx, ctcss_tx, squelch + '0', ctcss_rx);

  LOG(println, F("DRA818::group"));
  LOG(print, F("-> "));
  SEND(buffer);

  return this->read_response();
}

int DRA818::handshake() {
  char i = HANDSHAKE_REPEAT;


  while (i-- > 0) {
    LOG(println, F("DRA818::handshake"));
    LOG(print, F("-> "));

    SEND("AT+DMOCONNECT\r\n");
    if (this->read_response()) {
      return true;
    }
  }

  return false;
}

int DRA818::scan(float freq) {
  char buf[9];

  dtostrf(freq, 8, 4, buf);

  LOG(println, F("DRA818::scan"));
  LOG(print, F("-> "));

  SEND("S+");
  SEND(buf);
  SEND("\r\n");

  return read_response();
}

int DRA818::rssi() {
  if ((this->type & SA_MODEL_FLAG) == 0){
    LOG(println, F("WARNING: DRA818::rssi() only supported by SA818/SA868, not by DRA818."));
    LOG(println, F("Construct your DRA818 object with `type = SA[818]868_[VU]HF` to enable rssi()."));
    return -1;
  }
  LOG(println, F("DRA818::rssi"));
  LOG(print, F("-> "));

  SEND("RSSI?\r\n");

  String rssiString = read_string_response();
  int result = 0;
  bool prefixMatch = rssiString.startsWith(RSP_RSSI);
  if(prefixMatch)
  {
    rssiString = rssiString.substring(strlen(RSP_RSSI));
    result = atoi(rssiString.c_str());
  }
  return result;
}

String DRA818::version() {
  if ((this->type & SA_MODEL_FLAG) == 0){
    LOG(println, F("WARNING: DRA818::version() only supported by SA818/SA868, not by DRA818."));
    LOG(println, F("Construct your DRA818 object with `type = SA[818]868_[VU]HF` to enable rssi()."));
    return String();
  }
  LOG(println, F("DRA818::version"));
  LOG(print, F("-> "));

  SEND("AT+VERSION\r\n");

  String response = read_string_response();
  bool prefixMatch = response.startsWith(RSP_VERSION);
  if(prefixMatch)
  {
    response = response.substring(strlen(RSP_VERSION));
  }
  return response;
}

int DRA818::volume(uint8_t volume) {
  CHECK(volume, >, VOLUME_MAX);
  CHECK(volume, <, VOLUME_MIN);

  LOG(println, F("DRA818::volume"));
  LOG(print, F("-> "));

  SEND("AT+DMOSETVOLUME=");
  SEND(volume + '0');
  SEND("\r\n");

  return read_response();
}

int DRA818::filters(bool pre, bool high, bool low) {
  LOG(println, F("DRA818::filters"));
  LOG(print, F("-> "));

  SEND("AT+SETFILTER=");
  SEND('0' + (char)(pre ? false: true)); // or !pre
  SEND(",");
  SEND('0' + (char)(high ? false: true)); // or !high
  SEND(",");
  SEND('0' + (char)(low ? false: true)); // or !low
  SEND("\r\n");

  return read_response(); // SCAN function return 0 if there is a signal, 1 otherwise
}

#if !defined (ESP32)
DRA818* DRA818::configure(SoftwareSerial *stream, uint8_t type, float freq_rx, float freq_tx, uint8_t squelch, uint8_t volume, uint8_t ctcss_rx, uint8_t ctcss_tx, uint8_t bandwidth, bool pre, bool high, bool low, Stream *log) {
  DRA818 *dra = new DRA818(stream, type);
  return DRA818::configure(dra, freq_rx, freq_tx, squelch, volume, ctcss_rx, ctcss_tx, bandwidth, pre, high, low, log);
}
#endif

DRA818* DRA818::configure(HardwareSerial *stream, uint8_t type, float freq_rx, float freq_tx, uint8_t squelch, uint8_t volume, uint8_t ctcss_rx, uint8_t ctcss_tx, uint8_t bandwidth, bool pre, bool high, bool low, Stream *log) {
  DRA818 *dra = new DRA818(stream, type);
  return DRA818::configure(dra, freq_rx, freq_tx, squelch, volume, ctcss_rx, ctcss_tx, bandwidth, pre, high, low, log);
}

DRA818* DRA818::configure(DRA818 *dra, float freq_rx, float freq_tx, uint8_t squelch, uint8_t volume, uint8_t ctcss_rx, uint8_t ctcss_tx, uint8_t bandwidth, bool pre, bool high, bool low, Stream *log) {
  int ret;
#ifdef DRA818_DEBUG
  dra->set_log(log);
#endif
  delay(TIMEOUT);
  dra->handshake();
  ret = dra->group(bandwidth, freq_tx, freq_rx, ctcss_tx, squelch, ctcss_tx);
  if (ret) ret = dra->volume(volume);
  if (ret) ret = dra->filters(pre, high, low);

  return ret ? dra : NULL;
}
