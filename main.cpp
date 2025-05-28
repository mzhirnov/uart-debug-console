///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Debug Console via interfacing UART <-> LED&KEY board (TM1638).
//
// Author: Maksim Zhirnov (mzhirnov@gmail.com)
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <util/atomic.h>

#include <stddef.h>
#include <stdint.h>

#define DEBUG_COUNTER 0
#define DEBUG_ECHO    0

///
/// Port pin
///
template <uint8_t Bit>
class Pin {
public:
	static_assert(PB0 <= Bit && Bit <= PB5);
	
	static constexpr uint8_t Mask = 1 << Bit;
	
	void SetOutput() { DDRB |= Mask; }
	void SetInput() { DDRB &= ~Mask; }
		
	bool Read() { return PINB & Mask; }
	void Write(bool value) { value ? High() : Low(); }
	
	void High() { PORTB |= Mask; }
	void Low() { PORTB &= ~Mask; }
	
	void Toggle() {
		// Writing 1 to a bit in the PINx register toggles corresponding bit in the PORTx register
		PINB |= Mask;
	}
};

///
/// TM1638 for keys and leds
///
namespace TM1638 {
	constexpr uint8_t kStrobePin = 2;
	constexpr uint8_t kClockPin = 1;
	constexpr uint8_t kDataPin = 0;
	
	Pin<kStrobePin> _stb;
	Pin<kDataPin> _dio;
	Pin<kClockPin> _clk;
	
	enum Command : uint8_t {
		Data = 0b01000000,
		DisplayControl = 0b10000000,
		Address = 0b11000000,
		
		ReadKeysDataFlag = 0b00000010,
		FixedAddressFlag = 0b00000100,
		DisplayOnFlag = 0b00001000
	};
	
	static void Setup() {
		_stb.SetOutput();
		_clk.SetOutput();
		_dio.SetOutput();
		
		_stb.High();
		_clk.High();
	}
	
	static void SendByte(uint8_t data) {
		// Write on rising edge
		for (int i = 0; i < 8; ++i) {
			_clk.Low();
			_dio.Write(data & 1);
			data >>= 1;
			_clk.High();
		}
	}
	
	static void SendCommand(uint8_t cmd) {
		_stb.Low();
		SendByte(cmd);
		_stb.High();
	}
	
	static void SetBrightness(uint8_t brightness) {
		uint8_t cmd = Command::DisplayControl;
		if (brightness) {
			brightness--;
			cmd |= Command::DisplayOnFlag;
		}
		
		SendCommand(cmd | (brightness & 0b00000111));
	}
	
	static void SetLed(uint8_t pos, bool on) {
		SendCommand(Command::Data | Command::FixedAddressFlag);
		
		_stb.Low();
		SendByte(Command::Address | 1 | ((pos << 1) & 0b00001111));
		SendByte(on ? 1 : 0);
		_stb.High();
	}
	
#if 0
	static void SetLeds(uint8_t leds) {
		SendCommand(Command::Data | Command::FixedAddressFlag);
		
		uint8_t cmd = Command::Address | 1;
		
		for (uint8_t i = 0; i < 8; ++i) {
			_stb.Low();
			SendByte(cmd);
			SendByte(leds & 1);
			_stb.High();
			
			cmd += 2;
			leds >>= 1;
		}
	}
#endif
	
	static void SetDigit(uint8_t pos, uint8_t segments) {
		SendCommand(Command::Data | Command::FixedAddressFlag);
		
		_stb.Low();
		SendByte(Command::Address | ((pos << 1) & 0b00001111));
		SendByte(segments);
		_stb.High();
	}
	
#if 0
	static void SetDigits(uint8_t pos, const uint8_t* segments, uint8_t count) {
		SendCommand(Command::Data | Command::FixedAddressFlag);
		
		pos <<= 1;
		
		for (uint8_t i = 0; i < count; ++i) {
			_stb.Low();
			SendByte(Command::Address | (pos & 0b00001111));
			SendByte(segments[i]);
			_stb.High();
			
			pos += 2;
		}
	}
#endif
	
	static void Clear() {
		SendCommand(Command::Data);
		
		_stb.Low();
		
		SendByte(Command::Address);
		
		for (uint8_t i = 0; i < 16; ++i) {
			SendByte(0);
		}
		
		_stb.High();
	}
	
	static uint8_t ReadByte() {
		uint8_t data = 0;
		
		// Read on rising edge
		for (int i = 0; i < 8; ++i) {
			_clk.High();
			uint8_t value = _dio.Read();
			data = data >> 1 | value << 7;
			_clk.Low();
		}
		
		return data;
	}
	
	static uint8_t ReadKeys() {
		_stb.Low();
		
		SendByte(Command::Data | Command::ReadKeysDataFlag);
		
		_dio.SetInput();
		_dio.High();
		
		// Read on rising edge
		_clk.Low();
				
		uint8_t keys = 0;
		
		for (uint8_t i = 0; i < 4; ++i) {
			keys |= ReadByte() << i;
		}
		
		_clk.High();
		_stb.High();
		
		_dio.SetOutput();
		_dio.Low();
		
		return keys;
	}
} // namespace TM1638

static const uint8_t kLedChars[10] = {
	0b00111111, // 0
	0b00000110, // 1
	0b01011011, // 2
	0b01001111, // 3
	0b01100110, // 4
	0b01101101, // 5
	0b01111101, // 6
	0b00000111, // 7
	0b01111111, // 8
	0b01101111  // 9
};

///
/// Software UART
///
namespace UART {
	constexpr uint8_t kRxPin = 4;
	constexpr uint8_t kTxPin = 3;
	constexpr uint32_t kBaudRate = 9600;
	constexpr uint8_t kRxBufSize = 8;
	
	static_assert(kRxBufSize && (kRxBufSize & (kRxBufSize - 1)) == 0, "RxBufSize must be power of two");
	
	constexpr uint8_t kTxBusy = (1 << EEAR0);
	constexpr uint8_t kRxBusy = (1 << EEAR1);
	constexpr uint8_t kRxWaitingStopBit = (1 << EEAR2);
	
	Pin<kRxPin> _rx;
	Pin<kTxPin> _tx;
	
	uint16_t _txBuffer;
	uint8_t _txCounter;
	
	uint8_t _inbuf[kRxBufSize];
	volatile uint8_t _qin;
	uint8_t _qout;
	
	uint8_t _rxCounter;
	uint8_t _rxBitsLeft;
	
	volatile uint16_t isrCounter;
	
	constexpr uint16_t MillisecondsToCount(uint16_t counter) {
		return (F_CPU / kBaudRate + 1) * 10 * 3 * counter / 1000;
	}
	
	static __ATTR_ALWAYS_INLINE__ void Tick();
	
	ISR(TIM0_COMPA_vect) {
		isrCounter = isrCounter + 1;
		
		Tick();
	}

	static void Setup() {
		_rx.SetInput();
		_tx.SetOutput();
		
		_rx.High();
		_tx.High();
		
		// Enable Timer0 Compare Match Interrupt
		TIMSK0 = (1 << OCIE0A);
		// Clear Counter on Compare Match
		TCCR0A = (1 << WGM01);
		// Prescale Timer0 by 8 and start it
		TCCR0B = (1 << CS01);
		// Compare Match value
		OCR0A = F_CPU / 8 /* prescale */ / kBaudRate / 3 /* ISR must be called 3 times baud rate */;
	}
	
	static bool KbHit() {
		return _qin != _qout;
	}
	
#if 0
	static void FlushInput() {
		_qin = 0;
		_qout = 0;
	}
#endif

	static void PutChar(uint8_t ch) {
		while (EEAR & kTxBusy);
		
		_txBuffer = (ch << 1) | 0x200;
		_txCounter = 3;
		
		EEAR |= kTxBusy;
	}
	
	static uint8_t PeekChar() {
		while (!KbHit());
		return _inbuf[_qout];
	}
	
	static uint8_t GetChar() {
		uint8_t ch = PeekChar();
		_qout = (_qout + 1) & ~kRxBufSize;
		return ch;
	}
	
	void Tick() {
		// EEAR - flags
		// EEDR - input buffer
		
		// Transmit
		if (EEAR & kTxBusy) {
			uint8_t tmp = _txCounter;
			if (--tmp == 0) {
				_tx.Write(_txBuffer & 0x01);
				if (!(_txBuffer >>= 1)) {
					EEAR &= ~kTxBusy;
				}
				tmp = 3;
			}
			_txCounter = tmp;
		}
		
		// Receive
		if (EEAR & kRxWaitingStopBit) {
			uint8_t tmp = _rxCounter;
			if (--tmp == 0) {
				EEAR &= ~(kRxWaitingStopBit | kRxBusy);
				_inbuf[_qin] = EEDR;
				_qin = (_qin + 1) & ~kRxBufSize;
			}
			_rxCounter = tmp;
		}
		else {
			if (!(EEAR & kRxBusy)) {
				bool startBit = _rx.Read();
				if (!startBit) {
					EEAR |= kRxBusy;
					EEDR = 0;
					_rxCounter = 4;
					_rxBitsLeft = 8;
				}
			}
			else {
				uint8_t tmp = _rxCounter;
				if (--tmp == 0) {
					tmp = 3;
					uint8_t bitIn = _rx.Read();
					EEDR = EEDR >> 1 | bitIn << 7;
					if (--_rxBitsLeft == 0) {
						EEAR |= kRxWaitingStopBit;
					}
				}
				_rxCounter = tmp;
			}
		}
	}
} // namespace UART

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main() {
	uint16_t counter;
	uint16_t prevCounter1;
#if DEBUG_COUNTER
	uint8_t digit = 0;
	uint16_t prevCounter2;
#endif // DEBUG_COUNTER
	uint8_t prevKeys = 0;
	
	const uint8_t digits[10] = { kLedChars[0], kLedChars[1], kLedChars[2], kLedChars[3], kLedChars[4],
		kLedChars[5], kLedChars[6], kLedChars[7], kLedChars[8], kLedChars[9] };
	
	// Reduce power consumption by stopping the clock to individual peripherals
	power_adc_disable();
	
	// 1.2 MHz -> 9.6 MHz
	clock_prescale_set(clock_div_1);
	
	UART::Setup();
	
	TM1638::Setup();
	TM1638::SetBrightness(1);	
	TM1638::Clear();
	
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		counter = UART::isrCounter;
	}
	
	prevCounter1 = counter;
#if DEBUG_COUNTER
	prevCounter2 = counter;
#endif // DEBUG_COUNTER
	
	auto fnSendKey = [](uint8_t key, bool pressed) {
		// [0-7][+|-]
		UART::PutChar('0' + key);
		UART::PutChar(pressed ? '+' : '-');
		UART::PutChar('\r');
	};
	
	enum class CommandState {
		Initial,
		Mode,
		Digit,
		Print
	};
	
	// C             - Clear
	// P[digit|.]+   - Print
	// [0-7]D[digit] - Set digit
	// [0-7]L        - Set led on
	// [0-7]l        - Set led off
	auto commandState = CommandState::Initial;
	uint8_t commandDigitIndex;
	uint8_t commandDigitBuffer;
	
	for (;;) {
		while (UART::KbHit()) {
			uint8_t ch = UART::GetChar();
			
#if DEBUG_ECHO
			UART::PutChar(ch);
#endif // DEBUG_ECHO
			
			switch (commandState) {
			case CommandState::Initial:
				if (ch == 'C') {
					TM1638::Clear();
				}
				else if (ch == 'P') {
					commandDigitIndex = 0;
					commandState = CommandState::Print;
				}
				else if ((ch -= '0') < 8) {
					commandDigitIndex = ch;
					commandState = CommandState::Mode;
				}
				break;
			case CommandState::Print:
				if (ch == '.') {
					if (commandDigitIndex > 0) {
						// Turn on previous digit's segment H (dot)
						TM1638::SetDigit(commandDigitIndex - 1, commandDigitBuffer | 0x80);
						
						if (commandDigitIndex > 7) {
							commandState = CommandState::Initial;
						}
					}
				}
				else if (commandDigitIndex > 7) {
					commandState = CommandState::Initial;
				}
				else {
					if (ch == ' ') {
						commandDigitBuffer = 0;
					}
					else if ((ch -= '0') < sizeof(digits)) {
						commandDigitBuffer = digits[ch];
					}
					else {
						commandState = CommandState::Initial;
					}	
					
					if (commandState != CommandState::Initial) {
						TM1638::SetDigit(commandDigitIndex, commandDigitBuffer);
						commandDigitIndex++;
					}
				}
				break;
			case CommandState::Mode:
				commandState = CommandState::Initial;
				if (ch == 'D') {
					commandState = CommandState::Digit;
				}
				else if (ch == 'L') {
					TM1638::SetLed(commandDigitIndex, true);
				}
				else if (ch == 'l') {
					TM1638::SetLed(commandDigitIndex, false);
				}
				break;
			case CommandState::Digit:
				commandState = CommandState::Initial;
				if (ch == ' ') {
					TM1638::SetDigit(commandDigitIndex, 0);
				}
				else if ((ch -= '0') < sizeof(digits)) {
					TM1638::SetDigit(commandDigitIndex, digits[ch]);
				}
				break;
			default:
				commandState = CommandState::Initial;
				break;
			}
		}
		
		ATOMIC_BLOCK(ATOMIC_FORCEON) {
			counter = UART::isrCounter;
		}
		
		// Poll keys
		if (counter - prevCounter1 > UART::MillisecondsToCount(50)) {
			prevCounter1 = counter;
			
			uint8_t keys = TM1638::ReadKeys();
			
			if (uint8_t changedKeys = prevKeys ^ keys) {
				prevKeys = keys;
				
				uint8_t keyMask = 1;
				
				for (uint8_t i = 0; i < 8; ++i) {
					if (changedKeys & keyMask) {
						fnSendKey(i, keys & keyMask);
					}
					
					keyMask <<= 1;
				}
			}
		}

#if DEBUG_COUNTER	
		if (counter - prevCounter2 > UART::MillisecondsToCount(500)) {
			prevCounter2 = counter;
			
			TM1638::SetDigit(0, digits[digit]);
			if (++digit == 10) {
				digit = 0;
			}
		}
#endif // DEBUG_COUNTER
	}
	
	return 0;
}
