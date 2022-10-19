#ifndef Oregon_TM_h
#define Oregon_TM_h
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

#include <stdint.h>

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <util/delay.h>
#endif

#define TR_TIME 512
#define TWOTR_TIME 1024
#define PULSE_SHORTEN_2 93
#define PULSE_SHORTEN_3 138

#define THGN132 0x1D20
#define THN132 0xEC40
#define BTHGN129 0x5D53

#define OREGON_SEND_BUFFER_SIZE 24

class TimeMarkerBase
{
public:
	TimeMarkerBase()
	{
		_value = 0;
	}

	virtual void reset() = 0;
	void increment(unsigned long value)
	{
		_value += value;
	}
	virtual void wait(unsigned long future) = 0;

protected:
	unsigned long _value;
};

#if defined(ARDUINO)
class ArduinoTimeMarker : public TimeMarkerBase
{
public:
	inline void reset() override
	{
		_value = micros();
	}

	inline void wait(unsigned long future) override
	{
		while (_value + future >= micros())
		{
		}
	}
};
class TimeMarker : public ArduinoTimeMarker
{
};

#else

class OneMhzClockTimeMarker : public TimeMarkerBase
{
public:
	inline void reset() override
	{
		_value = 0;
	}

	// Fixme how many clocks here
	inline void wait(unsigned long future) override
	{
		// unsigned long last = (unsigned long)((_value + future) / 30);
		// for (; last >=x 0; last--)
		// {
		// 	_delay_us(30);
		// }
		// _value = 0;
	}
};
class TimeMarker : public OneMhzClockTimeMarker
{
};
#endif
class Oregon_TM
{
public:
	Oregon_TM(volatile uint8_t *rfPort, uint8_t rfPin, uint8_t nibblesCount);
	~Oregon_TM();
	void setType(uint16_t);
	void setChannel(uint8_t);
	void setId(uint8_t);
	void setBatteryFlag(bool);
	void setStartCount(uint8_t);
	void setTemperature(float);
	void setHumidity(uint8_t);
	void setComfort(float, uint8_t);
	void setPressure(float);
	void transmit();

	inline void rfHigh()
	{
		*rfPort |= (1 << rfPin);
	}

	inline void rfLow()
	{
		*rfPort &= ~(1 << rfPin);
	}

private:
	void sendZero(void);
	void sendOne(void);
	void sendMSB(const uint8_t);
	void sendLSB(const uint8_t);
	void sendData();
	void sendOregon();
	void sendPreamble();
	void calculateAndSetChecksum132();
	void calculateAndSetChecksum129();
	void calculateAndSetChecksum132S();

// ATTiny84 is too slow for advanced
// timing optimisation
#if not defined(__AVR_ATtiny84__)
	TimeMarker timeMarker;
#endif

	uint8_t buffer_size;
	uint8_t *SendBuffer;
	uint16_t sens_type;
	uint8_t timing_corrector2;
	uint8_t CCIT_POLY;
	volatile uint8_t *rfPort;
	uint8_t rfPin;
};

#endif
