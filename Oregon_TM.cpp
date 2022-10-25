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
  memset(this->sendBuffer, 0, OREGON_BUFFER_MAX_SIZE_IN_BYTES);
  buffer_size = (uint8_t)(nibbleCount / 2) + 2;

  sens_type = 0x0000;
  timing_corrector2 = 4;
  CCIT_POLY = 0x07;

  rfLow();
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
  for (uint8_t i = 0; i < buffer_size; i++)
  {
    sendMSB(sendBuffer[i]);
    sendLSB(sendBuffer[i]);

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
  sendBuffer[9] &= 0xF0;
  sendBuffer[10] = 0x00;
  sendBuffer[11] = 0x00;
  uint8_t summ = 0x00;
  uint8_t crc = 0x00;
  uint8_t cur_nible;
  for (int i = 0; i < 10; i++)
  {
    cur_nible = (sendBuffer[i] & 0xF0) >> 4;
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
    cur_nible = sendBuffer[i] & 0x0F;
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
  sendBuffer[9] += summ & 0x0F;
  sendBuffer[10] += summ & 0xF0;
  sendBuffer[10] += crc & 0x0F;
  sendBuffer[11] += crc & 0xF0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::calculateAndSetChecksum132(void)
{
  sendBuffer[7] &= 0xF0;
  sendBuffer[8] = 0x00;
  sendBuffer[9] = 0x00;
  uint8_t summ = 0x00;
  uint8_t crc = 0x3C;
  uint8_t cur_nible;
  for (int i = 0; i < 8; i++)
  {
    cur_nible = (sendBuffer[i] & 0xF0) >> 4;
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
    cur_nible = sendBuffer[i] & 0x0F;
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
  sendBuffer[7] += summ & 0x0F;
  sendBuffer[8] += summ & 0xF0;
  sendBuffer[8] += crc & 0x0F;
  sendBuffer[9] += crc & 0xF0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::calculateAndSetChecksum132S(void)
{
  uint8_t summ = 0x00;
  uint8_t crc = 0xD6;
  sendBuffer[6] = sendBuffer[7] = 0x00;
  uint8_t cur_nible;
  for (int i = 0; i < 6; i++)
  {
    cur_nible = (sendBuffer[i] & 0xF0) >> 4;
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
    cur_nible = sendBuffer[i] & 0x0F;
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

  sendBuffer[6] += (summ & 0x0F) << 4;
  sendBuffer[6] += (summ & 0xF0) >> 4;
  sendBuffer[7] += (crc & 0x0F) << 4;
  sendBuffer[7] += (crc & 0xF0) >> 4;
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
  sendBuffer[0] = (type & 0xFF00) >> 8;
  sendBuffer[1] = type & 0x00FF;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setChannel(uint8_t channel)
{
  sendBuffer[2] &= 0x0F;
  sendBuffer[2] += (channel << 4) & 0xF0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setId(uint8_t ID)
{
  sendBuffer[2] &= 0xF0;
  sendBuffer[2] += (ID & 0xF0) >> 4;
  sendBuffer[3] &= 0x0F;
  sendBuffer[3] += (ID & 0x0F) << 4;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setBatteryFlag(bool level)
{
  sendBuffer[3] &= 0xFB;
  if (level)
    sendBuffer[3] |= 0x04;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setStartCount(uint8_t startcount)
{
  sendBuffer[3] &= 0xF4;
  if (startcount == 8)
    sendBuffer[3] |= 0x08;
  if (startcount == 2)
    sendBuffer[3] |= 0x02;
  if (startcount == 1)
    sendBuffer[3] |= 0x01;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setPressure(float mm_hg_pressure)
{
  uint8_t pressure;

  if (sens_type == BTHGN129)
  {
    pressure = (uint8_t)(mm_hg_pressure - 795);
    sendBuffer[7] &= 0xF0;
    sendBuffer[7] += pressure & 0x0F;
    sendBuffer[8] = (pressure & 0x0F0) + ((pressure & 0xF00) >> 8);
  }

  // prediction on nibble 18
  if (mm_hg_pressure < 1000)
  {
    // rainy
    sendBuffer[9] = 0x30;
  }
  else if (mm_hg_pressure < 1010)
  {
    // cloudy
    sendBuffer[9] = 0x20;
  }
  else if (mm_hg_pressure < 1025)
  {
    // partly cloudy
    sendBuffer[9] = 0x60;
  }
  else
  {
    // Sunny
    sendBuffer[9] = 0xC0;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////
void Oregon_TM::setTemperature(float temp)
{
  if (temp < 0)
  {
    sendBuffer[5] = 0x08;
    temp *= -1;
  }
  else
  {
    sendBuffer[5] = 0x00;
  }
  uint8_t tempInt = (uint8_t)temp;
  uint8_t td = (tempInt / 10);
  uint8_t tf = tempInt - td * 10;
  uint8_t tempFloat = (temp - (float)tempInt) * 10;

  sendBuffer[5] += (td << 4);
  sendBuffer[4] = tf;
  sendBuffer[4] |= (tempFloat << 4);
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setHumidity(uint8_t hum)
{
  if (sens_type != THN132)
  {
    sendBuffer[6] = (hum / 10);
    sendBuffer[6] += (hum - (sendBuffer[6] * 10)) << 4;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setComfort(float temp, uint8_t hum)
{
  if (sens_type != THN132)
  {
    if (hum > 70)
    {
      sendBuffer[7] = 0xC0;
      return;
    }
    if (hum < 40)
    {
      sendBuffer[7] = 0x80;
      return;
    }
    if (temp > 20 && temp < 25)
    {
      sendBuffer[7] = 0x40;
      return;
    }
    else
      sendBuffer[7] = 0x00;
    return;
  }
}