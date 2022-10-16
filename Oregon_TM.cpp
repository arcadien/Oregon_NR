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

Oregon_TM::Oregon_TM(byte tr_pin, int buf_size)
{
  max_buffer_size = (int)(buf_size / 2) + 2;
  SendBuffer = new byte[max_buffer_size + 2];
  TX_PIN = tr_pin;
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, LOW);
}
Oregon_TM::~Oregon_TM()
{
  delete SendBuffer;
}

Oregon_TM::Oregon_TM(byte tr_pin)
{
  SendBuffer = new byte[max_buffer_size + 2];
  TX_PIN = tr_pin;
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, LOW);
}

Oregon_TM::Oregon_TM(void)
{
  SendBuffer = new byte[max_buffer_size + 2];
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, LOW);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Функции передатчика////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
void Oregon_TM::sendZero(void)
{

  while (time_marker + TR_TIME * 4 >= micros())
    ;
  time_marker += TR_TIME * 4;
  digitalWrite(TX_PIN, HIGH);
  _delay_us(TR_TIME - PULSE_SHORTEN_2);
  digitalWrite(TX_PIN, LOW);
  _delay_us(TWOTR_TIME + PULSE_SHORTEN_2);
  digitalWrite(TX_PIN, HIGH);
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::sendOne(void)
{
  while (time_marker + TR_TIME * 4 - PULSE_SHORTEN_2 >= micros())
    ;
  time_marker += TR_TIME * 4;
  digitalWrite(TX_PIN, LOW);
  _delay_us(TR_TIME + PULSE_SHORTEN_2);
  digitalWrite(TX_PIN, HIGH);
  _delay_us(TWOTR_TIME - PULSE_SHORTEN_2);
  digitalWrite(TX_PIN, LOW);
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::sendMSB(byte data)
{
  (bitRead(data, 4)) ? sendOne() : sendZero();
  (bitRead(data, 5)) ? sendOne() : sendZero();
  (bitRead(data, 6)) ? sendOne() : sendZero();
  (bitRead(data, 7)) ? sendOne() : sendZero();
  time_marker += timing_corrector2; //Поправка на разницу тактовых частот 1024.07Гц и 1024.60Гц
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::sendLSB(byte data)
{
  (bitRead(data, 0)) ? sendOne() : sendZero();
  (bitRead(data, 1)) ? sendOne() : sendZero();
  (bitRead(data, 2)) ? sendOne() : sendZero();
  (bitRead(data, 3)) ? sendOne() : sendZero();
  time_marker += timing_corrector2; //Поправка на разницу тактовых частот 1024.07Гц и 1024.60Гц
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::sendData()
{
  int q = 0;
  for (byte i = 0; i < max_buffer_size; i++)
  {
    sendMSB(SendBuffer[i]);
    q++;
    if (q >= buffer_size)
      break;
    sendLSB(SendBuffer[i]);
    q++;
    if (q >= buffer_size)
      break;
    time_marker += 4; //Поправка на разницу тактовых частот 1024.07Гц и 1024.60Гц
    //Поправка на разницу тактовых частот 1024.07Гц и 1024Гц
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::sendOregon()
{
  time_marker = micros();
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
  time_marker += 9;
  sendLSB(0xF);
  sendLSB(0xF);
  time_marker += 9;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::calculateAndSetChecksum129(void)
{
  byte CCIT_POLY = 0x07;
  SendBuffer[9] &= 0xF0;
  SendBuffer[10] = 0x00;
  SendBuffer[11] = 0x00;
  byte summ = 0x00;
  byte crc = 0x00;
  byte cur_nible;
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
  byte CCIT_POLY = 0x07;
  SendBuffer[7] &= 0xF0;
  SendBuffer[8] = 0x00;
  SendBuffer[9] = 0x00;
  byte summ = 0x00;
  byte crc = 0x3C;
  byte cur_nible;
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
  byte CCIT_POLY = 0x07;
  byte summ = 0x00;
  byte crc = 0xD6;
  SendBuffer[6] = SendBuffer[7] = 0x00;
  byte cur_nible;
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

void Oregon_TM::SendPacket()
{
  if (sens_type == BTHGN129)
    calculateAndSetChecksum129();
  if (sens_type == THGN132)
    calculateAndSetChecksum132();
  if (sens_type == THN132)
    calculateAndSetChecksum132S();

  sendOregon();
  digitalWrite(TX_PIN, LOW);

  _delay_us(TWOTR_TIME * 15);
  sendOregon();
  digitalWrite(TX_PIN, LOW);
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Функции кодирования данных//////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
void Oregon_TM::setType(word type)
{
  sens_type = type;
  SendBuffer[0] = (type & 0xFF00) >> 8;
  SendBuffer[1] = type & 0x00FF;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setChannel(byte channel)
{
  byte channel_code;

  if (sens_type == THGN132)
  {
    if (channel <= 1)
    {
      channel_code = 0x10;
      setId(0xE3);
      send_time = 39000;
    }
    if (channel == 2)
    {
      channel_code = 0x20;
      setId(0xE3);
      send_time = 41000;
    }
    if (channel == 3)
    {
      channel_code = 0x40;
      setId(0xBB);
      send_time = 43000;
    }
  }

  if (sens_type == THN132)
  {
    if (channel <= 1)
    {
      channel_code = 0x10;
      setId(0xE3);
      send_time = 39000;
    }
    if (channel == 2)
    {
      channel_code = 0x20;
      setId(0xE3);
      send_time = 41000;
    }
    if (channel == 3)
    {
      channel_code = 0x40;
      setId(0xBB);
      send_time = 43000;
    }
  }

  if (sens_type == BTHGN129)
  {

    if (channel <= 1)
    {
      channel_code = 0x10;
      setId(0xF1);
      send_time = 53000;
    }
    if (channel == 2)
    {
      channel_code = 0x20;
      setId(0x92);
      send_time = 59000;
    }
    if (channel == 3)
    {
      channel_code = 0x30;
      setId(0xAA);
      send_time = 61000;
    }

    if (channel == 4)
    {
      channel_code = 0x40;
      setId(0x8A);
      send_time = 67000;
    }

    if (channel >= 5)
    {
      channel_code = 0x50;
      setId(0xB1);
      send_time = 71000;
    }
  }

  SendBuffer[2] &= 0x0F;
  SendBuffer[2] += channel_code & 0xF0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setId(byte ID)
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

void Oregon_TM::setStartCount(byte startcount)
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
  byte pressure;

  //Ограничения датчика по даташиту
  // if (mm_hg_pressure < 450) pressure = 600;
  // if (mm_hg_pressure > 790) pressure = 1054;

  if (sens_type == BTHGN129)
  {
    pressure = (byte)(mm_hg_pressure - 795);
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
  byte tempInt = (byte)temp;
  byte td = (tempInt / 10);
  byte tf = tempInt - td * 10;
  byte tempFloat = (temp - (float)tempInt) * 10;

  SendBuffer[5] += (td << 4);
  SendBuffer[4] = tf;
  SendBuffer[4] |= (tempFloat << 4);
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setHumidity(byte hum)
{
  if (sens_type != THN132)
  {
    SendBuffer[6] = (hum / 10);
    SendBuffer[6] += (hum - (SendBuffer[6] * 10)) << 4;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////

void Oregon_TM::setComfort(float temp, byte hum)
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
///////////////////////////////////////////////////////////////////////////////////////////////////

bool Oregon_TM::transmit()
{
  if (millis() >= time_marker_send && send_time)
  {
    SendPacket();
    time_marker_send = millis() + send_time;
    return true;
  }
  else
    return false;
}
