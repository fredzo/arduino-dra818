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
#define RSP_READ_GROUP      "+DMOREADGROUP:"

#define CHECK(a, op, b) if (a op b) a = b

#define CHAR_TO_INT(a) a - '0'

#ifdef DRA818_SIMU
  #ifdef DRA818_DEBUG
    #define LOG(action, msg) if (this->log) this->log->action(msg)
    #define SEND(msg) ({\
      dra818SimuWrite(msg); \
      if (this->log) this->log->write(msg);  \
      })
    #define READ() ({int retval; \
      retval=dra818SimuRead(); \
      if (this->log) this->log->write(retval);  \
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
      //LOG(write, ack[2]);
    }
  } while (ack[2] != 0xa && (millis() - start) < TIMEOUT);
#ifdef DRA818_DEBUG
  if (ack[2] != 0xa) LOG(write, "\r\n");
#endif
  LOG(print, F("Returned value="));
  LOG(println, ack[0] == '0' );

  return (ack[0] == '0');
}

int DRA818::process_scan_response(int readResponse) {
  //Serial.printf("Process scan with duration = %d.", millis()-scanStartTime);
  if(readResponse == 0)
  {
    return readResponse;
  }
  else
  { // Scan function does not return 0 when frequency is busy but rather stops for more thant 500 ms
    // Let's use a response time based busy detection
    if(millis()-scanStartTime >= DRA818_SCAN_TIME_THRESHOLD)
    {
      return 0;
    }
    else
    {
      return readResponse;
    }
  }
}


String DRA818::read_string_response() {
  char buffer[RSP_BUFFER_SIZE];
  int bufferIndex = 0;
  char curChar;

  LOG(print, F("<- "));

  long start = millis();
  do {
    if (AVAILABLE()) {
      curChar = READ();
      buffer[bufferIndex] = curChar;
      //LOG(write, curChar);
      bufferIndex++;
    }
  } while ((curChar != 0xa) && ((millis() - start) < TIMEOUT) && (bufferIndex<RSP_BUFFER_SIZE));
#ifdef DRA818_DEBUG
  if (curChar != 0xa) LOG(write, "\r\n");
#endif
  buffer[bufferIndex] = 0; // String terminator
  String result = String(buffer);
  LOG(print, F("Returned value="));
  LOG(println, result.c_str());

  return result;
}

int DRA818::group(Parameters parameters) {
  send_group(parameters.bandwidth, parameters.freq_tx, parameters.freq_rx, parameters.ctcss_tx, parameters.squelch, parameters.ctcss_rx);
  return this->read_response();
}

int DRA818::group(uint8_t bw, float freq_tx, float freq_rx, uint8_t ctcss_tx, uint8_t squelch, uint8_t ctcss_rx) {
  send_group(bw, freq_tx, freq_rx, ctcss_tx, squelch, ctcss_rx);
  return this->read_response();
}

int DRA818::handshake() {
  char i = HANDSHAKE_REPEAT;
  while (i-- > 0) {
    send_handshake();
    if (this->read_response()) {
      return true;
    }
  }
  return false;
}

int DRA818::scan(float freq) {
  send_scan(freq);
  return process_scan_response(read_response());
}

int DRA818::tail(bool tail) {
  if(send_tail(tail))
  {
    return read_response();
  }
  else
  {
    return -1;
  }
}


int DRA818::rssi() {
  if(send_rssi())
  {
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
  else
  {
    return -1;
  }
}

String DRA818::version() {
  if(send_version())
  {
    String response = read_string_response();
    bool prefixMatch = response.startsWith(RSP_VERSION);
    if(prefixMatch)
    {
      response = response.substring(strlen(RSP_VERSION));
    }
    return response;
  }
  else
  {
    return String();
  }
}

DRA818::Parameters DRA818::read_group() {
  Parameters result;
  if(send_read_group())
  {
    String response = read_string_response();
    bool prefixMatch = response.startsWith(RSP_READ_GROUP);
    if(prefixMatch)
    {
      response = response.substring(strlen(RSP_READ_GROUP));
      result = parseParameters(response);
    }
  }
  return result;
}

String DRA818::Parameters::toString()
{
  char buffer[128];
  sprintf(buffer, "Parameters[GBW=%d,TFV=%3.4fMHz,RFV=%3.4fMHz,SQ=%d,Tx_CXCSS=%03d,Rx_CXCSS=%03d]",this->bandwidth,this->freq_tx,this->freq_rx,this->squelch,this->ctcss_tx,this->ctcss_rx);
  return String(buffer);
}

int DRA818::volume(uint8_t volume) {
  send_volume(volume);
  return read_response();
}

int DRA818::filters(bool pre, bool high, bool low) {
  send_filters(pre, high, low);
  return read_response(); // SCAN function return 0 if there is a signal, 1 otherwise
}

bool DRA818::send_group(uint8_t bw, float freq_tx, float freq_rx, uint8_t ctcss_tx, uint8_t squelch, uint8_t ctcss_rx) {
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
  return true;
}

bool DRA818::send_handshake() {
    LOG(println, F("DRA818::handshake"));
    LOG(print, F("-> "));

    SEND("AT+DMOCONNECT\r\n");
    return true;
}

bool DRA818::send_scan(float freq) {
  scanStartTime = millis();
  char buf[9];

  dtostrf(freq, 8, 4, buf);

  LOG(println, F("DRA818::scan"));
  LOG(print, F("-> "));

  SEND("S+");
  SEND(buf);
  SEND("\r\n");
  return true;
}

bool DRA818::send_tail(bool tail) {
  if ((this->type & SA_MODEL_FLAG) == 0){
    LOG(println, F("WARNING: DRA818::tail() only supported by SA818/SA868, not by DRA818."));
    LOG(println, F("Construct your DRA818 object with `type = SA[818]868_[VU]HF` to enable tail()."));
    return false;
  }
  LOG(println, F("DRA818::tail"));
  LOG(print, F("-> "));

  SEND("AT+SETTAIL=");
  SEND('0' + (char)(tail ? true: false));
  SEND("\r\n");
  return true;
}


bool DRA818::send_rssi() {
  if ((this->type & SA_MODEL_FLAG) == 0){
    LOG(println, F("WARNING: DRA818::rssi() only supported by SA818/SA868, not by DRA818."));
    LOG(println, F("Construct your DRA818 object with `type = SA[818]868_[VU]HF` to enable rssi()."));
    return false;
  }
  LOG(println, F("DRA818::rssi"));
  LOG(print, F("-> "));

  SEND("RSSI?\r\n");
  return true;
}

bool DRA818::send_version() {
  if ((this->type & SA_MODEL_FLAG) == 0){
    LOG(println, F("WARNING: DRA818::version() only supported by SA818/SA868, not by DRA818."));
    LOG(println, F("Construct your DRA818 object with `type = SA[818]868_[VU]HF` to enable version()."));
    return String();
  }
  LOG(println, F("DRA818::version"));
  LOG(print, F("-> "));

  SEND("AT+VERSION\r\n");
  return true;
}

bool DRA818::send_read_group() {
  if ((this->type & SA_MODEL_FLAG) == 0){
    LOG(println, F("WARNING: DRA818::read_group() only supported by SA818/SA868, not by DRA818."));
    LOG(println, F("Construct your DRA818 object with `type = SA[818]868_[VU]HF` to enable read_group()."));
    return false;
  }
  LOG(println, F("DRA818::read_group"));
  LOG(print, F("-> "));

  SEND("AT+DMOREADGROUP\r\n");
  return true;
}

bool DRA818::send_volume(uint8_t volume) {
  CHECK(volume, >, VOLUME_MAX);
  CHECK(volume, <, VOLUME_MIN);

  LOG(println, F("DRA818::volume"));
  LOG(print, F("-> "));

  SEND("AT+DMOSETVOLUME=");
  SEND(volume + '0');
  SEND("\r\n");
  return true;
}

bool DRA818::send_filters(bool pre, bool high, bool low) {
  LOG(println, F("DRA818::filters"));
  LOG(print, F("-> "));

  SEND("AT+SETFILTER=");
  SEND('0' + (char)(pre ? false: true)); // or !pre
  SEND(",");
  SEND('0' + (char)(high ? false: true)); // or !high
  SEND(",");
  SEND('0' + (char)(low ? false: true)); // or !low
  SEND("\r\n");

  return true;
}

DRA818::Parameters DRA818::parseParameters(String parameterString)
{
  Parameters result;
  char* token = strtok((char*)parameterString.c_str(),",");
  if(token != 0)
  { // GBW
    //Serial.printf("GBW : %s",token);
    result.bandwidth = atoi(token);
    token = strtok(0,",");
    if(token != 0)
    { // TFV
      //Serial.printf("TFV : %s",token);
      result.freq_tx = atof(token);
      token = strtok(0,",");
      if(token != 0)
      { // RFV
        //Serial.printf("RFV : %s",token);
        result.freq_rx = atof(token);
        token = strtok(0,",");
        if(token != 0)
        { // Tx_CXCSS
          //Serial.printf("Tx CXCSS : %s",token);
          result.ctcss_tx = atoi(token);
          token = strtok(0,",");
          if(token != 0)
          { // SQ
            //Serial.printf("SQ : %s",token);
            result.squelch = atoi(token);
            token = strtok(0,",");
            if(token != 0)
            { // Rx_CXCSS
              //Serial.printf("Rx CXCSS : %s",token);
              result.ctcss_rx = atoi(token);
              token = strtok(0,",");
            }
          }
        }
      }
    }
  }
  return result;
}

// Async methods
void DRA818::group_async(Parameters parameters) {
  send_group(parameters.bandwidth, parameters.freq_tx, parameters.freq_rx, parameters.ctcss_tx, parameters.squelch, parameters.ctcss_rx);
}

void DRA818::group_async(uint8_t bandwidth, float freq_tx, float freq_rx, uint8_t ctcss_tx, uint8_t squelch, uint8_t ctcss_rx) {
  send_group(bandwidth, freq_tx, freq_rx, ctcss_tx, squelch, ctcss_rx);
}

void DRA818::group_async_cb(void(*cb)(int)) {
  group_cb = cb;
}

void DRA818::handshake_async() {
  send_handshake();
}

void DRA818::handshake_async_cb(void(*cb)(int)) {
  handshake_cb = cb;
}

void DRA818::scan_async(float freq) {
  send_scan(freq);
}

void DRA818::scan_async_cb(void(*cb)(int)) {
  scan_cb = cb;
}

void DRA818::volume_async(uint8_t volume) {
  send_volume(volume);
}

void DRA818::volume_async_cb(void(*cb)(int)) {
   volume_cb = cb;
}

void DRA818::filters_async(bool pre, bool high, bool low) {
  send_filters(pre,high,low);
}

void DRA818::filters_async_cb(void(*cb)(int)) {
  filters_cb = cb;
}

void DRA818::tail_async(bool tail) {
  send_tail(tail);
}

void DRA818::tail_async_cb(void(*cb)(int)) {
  tail_cb = cb;
}

void DRA818::rssi_async() {
  send_rssi();
}

void DRA818::rssi_async_cb(void(*cb)(int)) {
  rssi_cb = cb;
}

void DRA818::version_async() {
  send_version();
}

void DRA818::version_async_cb(void(*cb)(String)) {
  version_cb = cb;
}

void DRA818::read_group_async() {
  send_read_group();
}
 
void DRA818::read_group_async_cb(void(*cb)(Parameters)) {
  read_group_cb = cb;
}

void DRA818::async_task()
{ // Check for DRA module presponse
  while(AVAILABLE()) {
    char curChar = READ();
    responseBuffer[responseBufferIndex] = curChar;
    //LOG(write, curChar);
    responseBufferIndex++;
    if(responseBufferIndex>=RSP_BUFFER_SIZE) {
      // Buffer full => overwrite last char 
      responseBufferIndex = RSP_BUFFER_SIZE-1;
      LOG(println, "Buffer full !");
    }
    if(curChar == 0x0a)
    { // Process command
      #ifdef DRA818_DEBUG
      LOG(write, "\r\n");
      #endif
      responseBuffer[responseBufferIndex] = 0; // String termination
      String responseString = String(responseBuffer);
      if (responseString.startsWith("S="))
      { // Scan
        if(scan_cb) {
          (*scan_cb)(process_scan_response(CHAR_TO_INT(responseString.charAt(2))));
        }
      }
      else if (responseString.startsWith("RSSI=")||responseString.startsWith("RSSI:"))
      {
        if(rssi_cb) {
          // Parse RSSI value
          responseString = responseString.substring(5);
          (*rssi_cb)(atoi(responseString.c_str()));
        }
      }
      else if(responseString.startsWith("+DMOCONNECT:"))
      { // handshake
        if(handshake_cb) {
          (*handshake_cb)(CHAR_TO_INT(responseString.charAt(12)));
        }
      }
      else if (responseString.startsWith("+DMOSETGROUP:"))
      {
        if(group_cb) {
          (*group_cb)(CHAR_TO_INT(responseString.charAt(13)));
        }
      }
      else if (responseString.startsWith("+DMOSETVOLUME:"))
      {
        if(volume_cb) {
          (*volume_cb)(CHAR_TO_INT(responseString.charAt(14)));
        }
      }
      else if (responseString.startsWith("+DMOSETFILTER:"))
      {
        if(filters_cb) {
          (*filters_cb)(CHAR_TO_INT(responseString.charAt(14)));
        }
      }
      else if (responseString.startsWith("+DMOSETTAIL:"))
      {
        if(tail_cb) {
          (*tail_cb)(CHAR_TO_INT(responseString.charAt(12)));
        }
      }
      else if (responseString.startsWith("+DMOREADGROUP:"))
      {
        if(read_group_cb) {
          // Parse RSSI value
          responseString = responseString.substring(14);
          (*read_group_cb)(parseParameters(responseString));
        }
      }
      else if (responseString.startsWith("+VERSION:"))
      {
        if(version_cb) {
          // Parse RSSI value
          responseString = responseString.substring(9);
          (*version_cb)(responseString);
        }
      }
      else
      {
        LOG(print, F("Unknown response '"));
        responseString.trim();
        LOG(print,responseString.c_str());
        LOG(println, F("'"));
      }
      responseBufferIndex = 0;
    }
  }
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
