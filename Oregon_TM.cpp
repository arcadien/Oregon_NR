#include "Oregon_TM.h"

#include <util/delay.h>

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// This file is part of the Arduino OREGON_NR library.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// The MIT License (MIT)
//
// Copyright (c) 2021 Sergey Zawislak
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Этот файл - часть библиотеки OREGON_NR
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021 Сергей Зависляк
//
// Данная лицензия разрешает лицам, получившим копию данного программного обеспечения и сопутствующей документации
// (в дальнейшем именуемыми «Программное Обеспечение»), безвозмездно использовать Программное Обеспечение без ограничений,
// включая неограниченное право на использование, копирование, изменение, слияние, публикацию, распространение, сублицензирование
// и/или продажу копий Программного Обеспечения, а также лицам, которым предоставляется данное Программное Обеспечение, при соблюдении следующих условий:
//
// Указанное выше уведомление об авторском праве и данные условия должны быть включены во все копии или значимые части данного Программного Обеспечения.
//
// ДАННОЕ ПРОГРАММНОЕ ОБЕСПЕЧЕНИЕ ПРЕДОСТАВЛЯЕТСЯ «КАК ЕСТЬ», БЕЗ КАКИХ-ЛИБО ГАРАНТИЙ, ЯВНО ВЫРАЖЕННЫХ ИЛИ ПОДРАЗУМЕВАЕМЫХ, ВКЛЮЧАЯ ГАРАНТИИ ТОВАРНОЙ
// ПРИГОДНОСТИ, СООТВЕТСТВИЯ ПО ЕГО КОНКРЕТНОМУ НАЗНАЧЕНИЮ И ОТСУТСТВИЯ НАРУШЕНИЙ, НО НЕ ОГРАНИЧИВАЯСЬ ИМИ. НИ В КАКОМ СЛУЧАЕ АВТОРЫ ИЛИ ПРАВООБЛАДАТЕЛИ
// НЕ НЕСУТ ОТВЕТСТВЕННОСТИ ПО КАКИМ-ЛИБО ИСКАМ, ЗА УЩЕРБ ИЛИ ПО ИНЫМ ТРЕБОВАНИЯМ, В ТОМ ЧИСЛЕ, ПРИ ДЕЙСТВИИ КОНТРАКТА, ДЕЛИКТЕ ИЛИ ИНОЙ СИТУАЦИИ,
// ВОЗНИКШИМ ИЗ-ЗА ИСПОЛЬЗОВАНИЯ ПРОГРАММНОГО ОБЕСПЕЧЕНИЯ ИЛИ ИНЫХ ДЕЙСТВИЙ С ПРОГРАММНЫМ ОБЕСПЕЧЕНИЕМ.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Конструктор
#include <Arduino.h>

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

Oregon_TM::Oregon_TM(volatile uint8_t *rfPort, uint8_t rfPin, uint8_t nibbleCount)
{
  this->rfPort = rfPort;
  this->rfPin = rfPin;

  buffer_size = (uint8_t)(nibbleCount / 2) + 2;
  SendBuffer = new uint8_t[buffer_size + 2];

  sens_type = 0x0000;
  timing_corrector2 = 4;
  CCIT_POLY = 0x07;

  rfLow();
}
Oregon_TM::~Oregon_TM()
{
  delete SendBuffer;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Функции передатчика////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void Oregon_TM::sendZero(void)
{
#if not defined(__AVR_ATtiny84__)
  timeMarker.wait(TR_TIME * 4);
  timeMarker.increment(TR_TIME * 4);
#endif
  rfHigh();
  _delay_us(TR_TIME - PULSE_SHORTEN_2);
  rfLow();
  _delay_us(TWOTR_TIME + PULSE_SHORTEN_2);
  rfHigh();

#if defined(__AVR_ATtiny84__)
  _delay_us(TR_TIME);
#endif
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::sendOne(void)
{
#if not defined(__AVR_ATtiny84__)
  timeMarker.wait(TR_TIME * 4 - PULSE_SHORTEN_2);
  timeMarker.increment(TR_TIME * 4);
#endif
  rfLow();
  _delay_us(TR_TIME + PULSE_SHORTEN_2);
  rfHigh();
  _delay_us(TWOTR_TIME - PULSE_SHORTEN_2);
  rfLow();
#if defined(__AVR_ATtiny84__)
  _delay_us(TR_TIME);
#endif
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::sendMSB(uint8_t data)
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();

#if not defined(__AVR_ATtiny84__)
  // Correction for the difference in clock frequencies 1024.07Hz and 1024.60Hz
  timeMarker.increment(timing_corrector2);
#endif
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::sendLSB(uint8_t data)
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();

#if not defined(__AVR_ATtiny84__)
  // Correction for the difference in clock frequencies 1024.07Hz and 1024.60Hz
  timeMarker.increment(timing_corrector2);
#endif
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::sendData()
{
  int q = 0;
  for (uint8_t i = 0; i < buffer_size; i++)
  {
    sendMSB(SendBuffer[i]);
    q++;
    if (q >= buffer_size)
      break;
    sendLSB(SendBuffer[i]);
    q++;
    if (q >= buffer_size)
      break;

// Correction for the difference in clock frequencies 1024.07Hz and 1024.60Hz
#if not defined(__AVR_ATtiny84__)
    timeMarker.increment(4);
#endif
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::sendOregon()
{
#if not defined(__AVR_ATtiny84__)
  timeMarker.reset();
#endif
  sendPreamble();
  sendLSB(0xA);
  sendData();
  sendZero();
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::sendPreamble(void)
{
  sendLSB(0xF);
  sendLSB(0xF);
#if not defined(__AVR_ATtiny84__)
  timeMarker.increment(9);
#endif
  sendLSB(0xF);
  sendLSB(0xF);
#if not defined(__AVR_ATtiny84__)
  timeMarker.increment(9);
#endif
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::calculateAndSetChecksum129(void)
{
  SendBuffer[9] &= 0xF0;
  SendBuffer[10] = 0x00;
  SendBuffer[11] = 0x00;
  uint8_t summ = 0x00;
  uint8_t crc = 0x00;
  uint8_t cur_nible;
  for (int i = 0; i < 10; i++)
  {
    cur_nible = (SendBuffer[i] & 0xF0) >> 4;
    summ += cur_nible;
    if (i != 3)
    {
      crc ^= cur_nible;
      for (int j = 0; j < 4; j++)
        if (crc & 0x80)
          crc = (crc << 1) ^ CCIT_POLY;
        else
          crc <<= 1;
    }
    cur_nible = SendBuffer[i] & 0x0F;
    summ += cur_nible;
    if (i != 2)
    {
      crc ^= cur_nible;
      for (int j = 0; j < 4; j++)
        if (crc & 0x80)
          crc = (crc << 1) ^ CCIT_POLY;
        else
          crc <<= 1;
    }
  }
  SendBuffer[9] += summ & 0x0F;
  SendBuffer[10] += summ & 0xF0;
  SendBuffer[10] += crc & 0x0F;
  SendBuffer[11] += crc & 0xF0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::calculateAndSetChecksum132(void)
{
  SendBuffer[7] &= 0xF0;
  SendBuffer[8] = 0x00;
  SendBuffer[9] = 0x00;
  uint8_t summ = 0x00;
  uint8_t crc = 0x3C;
  uint8_t cur_nible;
  for (int i = 0; i < 8; i++)
  {
    cur_nible = (SendBuffer[i] & 0xF0) >> 4;
    summ += cur_nible;
    if (i != 3)
    {
      crc ^= cur_nible;
      for (int j = 0; j < 4; j++)
        if (crc & 0x80)
          crc = (crc << 1) ^ CCIT_POLY;
        else
          crc <<= 1;
    }
    cur_nible = SendBuffer[i] & 0x0F;
    summ += cur_nible;
    if (i != 2)
    {
      crc ^= cur_nible;
      for (int j = 0; j < 4; j++)
        if (crc & 0x80)
          crc = (crc << 1) ^ CCIT_POLY;
        else
          crc <<= 1;
    }
  }
  SendBuffer[7] += summ & 0x0F;
  SendBuffer[8] += summ & 0xF0;
  SendBuffer[8] += crc & 0x0F;
  SendBuffer[9] += crc & 0xF0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::calculateAndSetChecksum132S(void)
{
  uint8_t summ = 0x00;
  uint8_t crc = 0xD6;
  SendBuffer[6] = SendBuffer[7] = 0x00;
  uint8_t cur_nible;
  for (int i = 0; i < 6; i++)
  {
    cur_nible = (SendBuffer[i] & 0xF0) >> 4;
    summ += cur_nible;
    if (i != 3)
    {
      crc ^= cur_nible;
      for (int j = 0; j < 4; j++)
        if (crc & 0x80)
          crc = (crc << 1) ^ CCIT_POLY;
        else
          crc <<= 1;
    }
    cur_nible = SendBuffer[i] & 0x0F;
    summ += cur_nible;
    if (i != 2)
    {
      crc ^= cur_nible;
      for (int j = 0; j < 4; j++)
        if (crc & 0x80)
          crc = (crc << 1) ^ CCIT_POLY;
        else
          crc <<= 1;
    }
  }
  for (int j = 0; j < 4; j++)
    if (crc & 0x80)
      crc = (crc << 1) ^ CCIT_POLY;
    else
      crc <<= 1;

  SendBuffer[6] += (summ & 0x0F) << 4;
  SendBuffer[6] += (summ & 0xF0) >> 4;
  SendBuffer[7] += (crc & 0x0F) << 4;
  SendBuffer[7] += (crc & 0xF0) >> 4;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::transmit()
{
  if (sens_type == BTHGN129)
    calculateAndSetChecksum129();
  if (sens_type == THGN132)
    calculateAndSetChecksum132();
  if (sens_type == THN132)
    calculateAndSetChecksum132S();

  sendOregon();
  rfLow();

  _delay_us(TWOTR_TIME * 15);
  sendOregon();
  rfLow();
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Функции кодирования данных//////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
void Oregon_TM::setType(uint16_t type)
{
  sens_type = type;
  SendBuffer[0] = (type & 0xFF00) >> 8;
  SendBuffer[1] = type & 0x00FF;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setChannel(uint8_t channel)
{
  SendBuffer[2] &= 0x0F;
  SendBuffer[2] += (channel << 4) & 0xF0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setId(uint8_t ID)
{
  SendBuffer[2] &= 0xF0;
  SendBuffer[2] += (ID & 0xF0) >> 4;
  SendBuffer[3] &= 0x0F;
  SendBuffer[3] += (ID & 0x0F) << 4;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setBatteryFlag(bool level)
{
  SendBuffer[3] &= 0xFB;
  if (level)
    SendBuffer[3] |= 0x04;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setStartCount(uint8_t startcount)
{
  SendBuffer[3] &= 0xF4;
  if (startcount == 8)
    SendBuffer[3] |= 0x08;
  if (startcount == 2)
    SendBuffer[3] |= 0x02;
  if (startcount == 1)
    SendBuffer[3] |= 0x01;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setPressure(float mm_hg_pressure)
{
  uint8_t pressure;

  if (sens_type == BTHGN129)
  {
    pressure = (uint8_t)(mm_hg_pressure - 795);
    SendBuffer[7] &= 0xF0;
    SendBuffer[7] += pressure & 0x0F;
    SendBuffer[8] = (pressure & 0x0F0) + ((pressure & 0xF00) >> 8);
  }

  // prediction on nibble 18
  if (mm_hg_pressure < 1000)
  {
    // rainy
    SendBuffer[9] = 0x30;
  }
  else if (mm_hg_pressure < 1010)
  {
    // cloudy
    SendBuffer[9] = 0x20;
  }
  else if (mm_hg_pressure < 1025)
  {
    // partly cloudy
    SendBuffer[9] = 0x60;
  }
  else
  {
    // Sunny
    SendBuffer[9] = 0xC0;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////
void Oregon_TM::setTemperature(float temp)
{
  if (temp < 0)
  {
    SendBuffer[5] = 0x08;
    temp *= -1;
  }
  else
  {
    SendBuffer[5] = 0x00;
  }
  uint8_t tempInt = (uint8_t)temp;
  uint8_t td = (tempInt / 10);
  uint8_t tf = tempInt - td * 10;
  uint8_t tempFloat = (temp - (float)tempInt) * 10;

  SendBuffer[5] += (td << 4);
  SendBuffer[4] = tf;
  SendBuffer[4] |= (tempFloat << 4);
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setHumidity(uint8_t hum)
{
  if (sens_type != THN132)
  {
    SendBuffer[6] = (hum / 10);
    SendBuffer[6] += (hum - (SendBuffer[6] * 10)) << 4;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setComfort(float temp, uint8_t hum)
{
  if (sens_type != THN132)
  {
    if (hum > 70)
    {
      SendBuffer[7] = 0xC0;
      return;
    }
    if (hum < 40)
    {
      SendBuffer[7] = 0x80;
      return;
    }
    if (temp > 20 && temp < 25)
    {
      SendBuffer[7] = 0x40;
      return;
    }
    else
      SendBuffer[7] = 0x00;
    return;
  }
}