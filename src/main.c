/*
 * 스펙트럼 녹음기 (Spectrum Recorder) - ATmega128
 * 마이크로프로세서 실험 Term Project (한성대학교 전자트랙 6조)
 *
 * 원본: 최종보고서의 main.c(test.c)
 * 변경: 보고서 버전에서 is_playing / play_buf_idx / current_play_buf / refill_needed
 *       선언이 ISR(TIMER0_COMP_vect)보다 뒤에 있어 컴파일 오류가 나므로
 *       선언 위치만 ISR 앞으로 옮김. 그 외 로직은 보고서와 동일.
 */
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include "ff.h"
#include "ffconf.h"
#include "diskio.h"
// sd 에 끊어짐 없이 저장하기 위해 버퍼를 2개 사용
#define BUF_SIZE 128
uint8_t buffer_A[BUF_SIZE];
uint8_t buffer_B[BUF_SIZE];
// switch 를 사용할 때 스택 문제로 인하여 atmega128 이 초기화 되어 먼저 전역변수로 선언
FATFS fs_global;
FIL file_global;
// sd 쓰기 모드에서 wav 파일을 만들기 위한 함수
void write_wav_header(FIL* file, uint32_t data_len, uint32_t sample_rate) {
	uint32_t byte_rate = sample_rate * 2;
	uint32_t total_size = data_len + 36;
	uint16_t block_align = 2;
	uint16_t bits_per_sample = 16;
	uint16_t channels = 1;
	UINT bw;
	f_write(file, "RIFF", 4, &bw);
	f_write(file, &total_size, 4, &bw);
	f_write(file, "WAVE", 4, &bw);
	f_write(file, "fmt ", 4, &bw);
	uint32_t subchunk1_size = 16;
	f_write(file, &subchunk1_size, 4, &bw);
	uint16_t audio_fmt = 1;
	f_write(file, &audio_fmt, 2, &bw);
	f_write(file, &channels, 2, &bw);
	f_write(file, &sample_rate, 4, &bw);
	f_write(file, &byte_rate, 4, &bw);
	f_write(file, &block_align, 2, &bw);
	f_write(file, &bits_per_sample, 2, &bw);
	f_write(file, "data", 4, &bw);
	f_write(file, &data_len, 4, &bw);
}
void uart0_init(void)
{
	UCSR0B = 0x00;
	UCSR0A = 0x00;
	UCSR0C = 0x06;
	UBRR0L = 0x67;
	UBRR0H = 0x00;
	UCSR0B = 0x08;
}
int uart0_putchar(char c, FILE* stream)
{
	if (c == '\n') uart0_putchar('\r', stream);
	while (!(UCSR0A & (1 << UDRE0)));
	UDR0 = c;
	return 0;
}
FILE uart0_stdout;
// Vcc
// gnd
// PB0 = CS 3.3V
// PB1 = SCK 3.3V
// PB2 = MOSI 3.3V
// PB3 = MISO 5V
// PB4 = DC 3.3V
// PB5 = RST 3.3V
// PB6 = LED 5V
// PD0 = 외부 인터럽트 sw
// PD4 = up sw
// PD5 = down sw
// PD6 = select sw
// PD7 = 녹음 sw
// PF0 = 테스트용 VR adc
// PF1 = mic adc
// PG0 = atmega128 내부 LED
#define CS_LOW()      (PORTB &= ~(1<<PB0))
#define CS_HIGH()     (PORTB |=  (1<<PB0))
#define SCK_LOW()     (PORTB &= ~(1<<PB1))
#define SCK_HIGH()    (PORTB |=  (1<<PB1))
#define MOSI_LOW()    (PORTB &= ~(1<<PB2))
#define MOSI_HIGH()   (PORTB |=  (1<<PB2))
#define DC_CMD()      (PORTB &= ~(1<<PB4))
#define DC_DATA()     (PORTB |=  (1<<PB4))
#define RST_LOW()     (PORTB &= ~(1<<PB5))
#define RST_HIGH()    (PORTB |=  (1<<PB5))
#define LED_ON()      (PORTB |=  (1<<PB6))
#define LED_OFF()     (PORTB &= ~(1<<PB6))
#define IN_LED_ON()   (PORTG |=  (1<<PG0))
#define IN_LED_OFF()  (PORTG &= ~(1<<PG0))
#define SD_CS_LOW()	  (PORTC &= ~(1<<PC0))
#define SD_CS_HIGH()  (PORTC |=  (1<<PC0))
// spi 를 읽기 위해
uint8_t SPI_txrx(uint8_t d)
{
	SPDR = d;
	while (!(SPSR & (1 << SPIF)));
	return SPDR;
}
// ILI9341 은 Data/Command 핀을 이용해 데이터로 바꾸거나 명령으로 바꾼다
// WriteCommand 로 명령, WriteData 로 데이터
void ILI9341_WriteCommand(uint8_t cmd)
{
	DC_CMD(); CS_LOW();
	SPI_txrx(cmd);
	CS_HIGH();
}
void ILI9341_WriteData(uint8_t data)
{
	DC_DATA(); CS_LOW();
	SPI_txrx(data);
	CS_HIGH();
}
// lcd 가 명령을 받기 위해
void ILI9341_HardReset()
{
	RST_LOW();
	_delay_ms(20);
	RST_HIGH();
	_delay_ms(150);
}
// lcd 에 전원만 넣었을 때 동작을 하지 않기에 필요한 명령과 데이터
void ILI9341_Init(uint8_t madctl)
{
	ILI9341_HardReset();
	ILI9341_WriteCommand(0x01);
	_delay_ms(50);
	ILI9341_WriteCommand(0x11);
	_delay_ms(150);
	ILI9341_WriteCommand(0x36);
	ILI9341_WriteData(madctl);
	ILI9341_WriteCommand(0x3A);
	ILI9341_WriteData(0x55);
	ILI9341_WriteCommand(0x29);
	_delay_ms(20);
}
// 특정 구역에 lcd 를 그리기 위한 명령과 데이터
void ILI9341_SetAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	ILI9341_WriteCommand(0x2A);
	ILI9341_WriteData(x0 >> 8);
	ILI9341_WriteData(x0 & 0xFF);
	ILI9341_WriteData(x1 >> 8);
	ILI9341_WriteData(x1 & 0xFF);
	ILI9341_WriteCommand(0x2B);
	ILI9341_WriteData(y0 >> 8);
	ILI9341_WriteData(y0 & 0xFF);
	ILI9341_WriteData(y1 >> 8);
	ILI9341_WriteData(y1 & 0xFF);
	ILI9341_WriteCommand(0x2C);
}
// 특정 구역 채우기
void ILI9341_FillScreen(uint16_t color)
{
	uint8_t hi = color >> 8, lo = color & 0xFF;
	ILI9341_SetAddrWindow(0, 0, 319, 239);
	DC_DATA(); CS_LOW();
	for (uint32_t i = 0; i < 320UL * 240UL; i++) {
		SPI_txrx(hi); SPI_txrx(lo);
	}
	CS_HIGH();
	IN_LED_OFF();            // atmega128 내부 LED 끄기
}
// lcd 에 특정 색으로 픽셀을 찍기 위한 함수
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	if (x >= 320 || y >= 240) return;
	ILI9341_SetAddrWindow(x, y, x, y);
	DC_DATA();
	CS_LOW();
	SPI_txrx(color >> 8); SPI_txrx(color & 0xFF);
	CS_HIGH();
}
// lcd 에 선 그리기
void ILI9341_DrawLine(int x0, int y0, int x1, int y1, uint16_t color)
{
	int dx = abs(x1 - x0);
	int dy = -abs(y1 - y0);
	int sx = x0 < x1 ? 1 : -1;
	int sy = y0 < y1 ? 1 : -1;
	int err = dx + dy, e2;
	while (1)
	{
		ILI9341_DrawPixel(x0, y0, color);
		if (x0 == x1 && y0 == y1) break;
		e2 = 2 * err;
		if (e2 >= dy) { err += dy; x0 += sx; }
		if (e2 <= dx) { err += dx; y0 += sy; }
	}
}
// lcd 의 사각형 그리기
void ILI9341_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
	if (x >= 320 || y >= 240) return;
	if (x + w > 320) w = 320 - x;
	if (y + h > 240) h = 240 - y;
	uint8_t hi = color >> 8, lo = color & 0xFF;
	ILI9341_SetAddrWindow(x, y, x + w - 1, y + h - 1);
	DC_DATA(); CS_LOW();
	for (uint32_t i = 0; i < (uint32_t)w * h; i++) { SPI_txrx(hi); SPI_txrx(lo); }
	CS_HIGH();
}
// spi 통신을 위한 설정으로 lcd 에 쓰기 속도가 낮으면 버벅거리기에 최대 속도
void SPI0_init()
{
	DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB4) | (1 << PB5) | (1 << PB6);
	DDRB &= ~(1 << PB3);
	CS_HIGH();
	DC_DATA();
	LED_ON();
	SPCR = (1 << SPE) | (1 << MSTR) | (0 << SPR1) | (0 << SPR0) | (0 << CPOL) | (0 << CPHA);
	SPSR = (1 << SPI2X);
}
// lcd 에 바로 글씨를 쓰기 위해
const uint8_t font5x7[96][5] = {
	{0x00,0x00,0x00,0x00,0x00}, // 0x20 ' '
	{0x00,0x00,0x5F,0x00,0x00}, // 0x21 '!'
	{0x00,0x07,0x00,0x07,0x00}, // 0x22 '"'
	{0x14,0x7F,0x14,0x7F,0x14}, // 0x23 '#'
	{0x24,0x2A,0x7F,0x2A,0x12}, // 0x24 '$'
	{0x23,0x13,0x08,0x64,0x62}, // 0x25 '%'
	{0x36,0x49,0x55,0x22,0x50}, // 0x26 '&'
	{0x00,0x05,0x03,0x00,0x00}, // 0x27 '''
	{0x00,0x1C,0x22,0x41,0x00}, // 0x28 '('
	{0x00,0x41,0x22,0x1C,0x00}, // 0x29 ')'
	{0x08,0x2A,0x1C,0x2A,0x08}, // 0x2A '*'
	{0x08,0x08,0x3E,0x08,0x08}, // 0x2B '+'
	{0x00,0x50,0x30,0x00,0x00}, // 0x2C ','
	{0x08,0x08,0x08,0x08,0x08}, // 0x2D '-'
	{0x00,0x60,0x60,0x00,0x00}, // 0x2E '.'
	{0x20,0x10,0x08,0x04,0x02}, // 0x2F '/'
	{0x3E,0x51,0x49,0x45,0x3E}, // 0x30 '0'
	{0x00,0x42,0x7F,0x40,0x00}, // 0x31 '1'
	{0x42,0x61,0x51,0x49,0x46}, // 0x32 '2'
	{0x21,0x41,0x45,0x4B,0x31}, // 0x33 '3'
	{0x18,0x14,0x12,0x7F,0x10}, // 0x34 '4'
	{0x27,0x45,0x45,0x45,0x39}, // 0x35 '5'
	{0x3C,0x4A,0x49,0x49,0x30}, // 0x36 '6'
	{0x01,0x71,0x09,0x05,0x03}, // 0x37 '7'
	{0x36,0x49,0x49,0x49,0x36}, // 0x38 '8'
	{0x06,0x49,0x49,0x29,0x1E}, // 0x39 '9'
	{0x00,0x36,0x36,0x00,0x00}, // 0x3A ':'
	{0x00,0x56,0x36,0x00,0x00}, // 0x3B ';'
	{0x08,0x14,0x22,0x41,0x00}, // 0x3C '<'
	{0x14,0x14,0x14,0x14,0x14}, // 0x3D '='
	{0x00,0x41,0x22,0x14,0x08}, // 0x3E '>'
	{0x02,0x01,0x51,0x09,0x06}, // 0x3F '?'
	{0x32,0x49,0x79,0x41,0x3E}, // 0x40 '@'
	{0x7E,0x11,0x11,0x11,0x7E}, // 0x41 'A'
	{0x7F,0x49,0x49,0x49,0x36}, // 0x42 'B'
	{0x3E,0x41,0x41,0x41,0x22}, // 0x43 'C'
	{0x7F,0x41,0x41,0x22,0x1C}, // 0x44 'D'
	{0x7F,0x49,0x49,0x49,0x41}, // 0x45 'E'
	{0x7F,0x09,0x09,0x09,0x01}, // 0x46 'F'
	{0x3E,0x41,0x49,0x49,0x7A}, // 0x47 'G'
	{0x7F,0x08,0x08,0x08,0x7F}, // 0x48 'H'
	{0x00,0x41,0x7F,0x41,0x00}, // 0x49 'I'
	{0x20,0x40,0x41,0x3F,0x01}, // 0x4A 'J'
	{0x7F,0x08,0x14,0x22,0x41}, // 0x4B 'K'
	{0x7F,0x40,0x40,0x40,0x40}, // 0x4C 'L'
	{0x7F,0x02,0x0C,0x02,0x7F}, // 0x4D 'M'
	{0x7F,0x04,0x08,0x10,0x7F}, // 0x4E 'N'
	{0x3E,0x41,0x41,0x41,0x3E}, // 0x4F 'O'
	{0x7F,0x09,0x09,0x09,0x06}, // 0x50 'P'
	{0x3E,0x41,0x51,0x21,0x5E}, // 0x51 'Q'
	{0x7F,0x09,0x19,0x29,0x46}, // 0x52 'R'
	{0x46,0x49,0x49,0x49,0x31}, // 0x53 'S'
	{0x01,0x01,0x7F,0x01,0x01}, // 0x54 'T'
	{0x3F,0x40,0x40,0x40,0x3F}, // 0x55 'U'
	{0x1F,0x20,0x40,0x20,0x1F}, // 0x56 'V'
	{0x3F,0x40,0x38,0x40,0x3F}, // 0x57 'W'
	{0x63,0x14,0x08,0x14,0x63}, // 0x58 'X'
	{0x07,0x08,0x70,0x08,0x07}, // 0x59 'Y'
	{0x61,0x51,0x49,0x45,0x43}, // 0x5A 'Z'
	{0x00,0x7F,0x41,0x41,0x00}, // 0x5B '['
	{0x02,0x04,0x08,0x10,0x20}, // 0x5C '\'
	{0x00,0x41,0x41,0x7F,0x00}, // 0x5D ']'
	{0x04,0x02,0x01,0x02,0x04}, // 0x5E '^'
	{0x40,0x40,0x40,0x40,0x40}, // 0x5F '_'
	{0x00,0x01,0x02,0x04,0x00}, // 0x60 '`'
	{0x20,0x54,0x54,0x54,0x78}, // 0x61 'a'
	{0x7F,0x48,0x44,0x44,0x38}, // 0x62 'b'
	{0x38,0x44,0x44,0x44,0x20}, // 0x63 'c'
	{0x38,0x44,0x44,0x48,0x7F}, // 0x64 'd'
	{0x38,0x54,0x54,0x54,0x18}, // 0x65 'e'
	{0x08,0x7E,0x09,0x01,0x02}, // 0x66 'f'
	{0x0C,0x52,0x52,0x52,0x3E}, // 0x67 'g'
	{0x7F,0x08,0x04,0x04,0x78}, // 0x68 'h'
	{0x00,0x44,0x7D,0x40,0x00}, // 0x69 'i'
	{0x20,0x40,0x44,0x3D,0x00}, // 0x6A 'j'
	{0x7F,0x10,0x28,0x44,0x00}, // 0x6B 'k'
	{0x00,0x41,0x7F,0x40,0x00}, // 0x6C 'l'
	{0x7C,0x04,0x18,0x04,0x78}, // 0x6D 'm'
	{0x7C,0x08,0x04,0x04,0x78}, // 0x6E 'n'
	{0x38,0x44,0x44,0x44,0x38}, // 0x6F 'o'
	{0x7C,0x14,0x14,0x14,0x08}, // 0x70 'p'
	{0x08,0x14,0x14,0x18,0x7C}, // 0x71 'q'
	{0x7C,0x08,0x04,0x04,0x08}, // 0x72 'r'
	{0x48,0x54,0x54,0x54,0x20}, // 0x73 's'
	{0x04,0x3F,0x44,0x40,0x20}, // 0x74 't'
	{0x3C,0x40,0x40,0x20,0x7C}, // 0x75 'u'
	{0x1C,0x20,0x40,0x20,0x1C}, // 0x76 'v'
	{0x3C,0x40,0x30,0x40,0x3C}, // 0x77 'w'
	{0x44,0x28,0x10,0x28,0x44}, // 0x78 'x'
	{0x0C,0x50,0x50,0x50,0x3C}, // 0x79 'y'
	{0x44,0x64,0x54,0x4C,0x44}, // 0x7A 'z'
	{0x00,0x08,0x36,0x41,0x00}, // 0x7B '{'
	{0x00,0x00,0x7F,0x00,0x00}, // 0x7C '|'
	{0x00,0x41,0x36,0x08,0x00}, // 0x7D '}'
	{0x10,0x08,0x08,0x10,0x08}, // 0x7E '~'
	{0x00,0x00,0x00,0x00,0x00}  // padding to make 96 rows
};
// lcd 에 한 글자 쓰기
void ILI9341_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg)
{
	if (c < 32 || c > 126) return;
	const uint8_t* bitmap = font5x7[c - 32];
	for (uint8_t i = 0; i < 5; i++)
	{
		for (uint8_t j = 0; j < 7; j++)
		{
			if ((bitmap[i] >> j) & 0x01)
				ILI9341_DrawPixel(x + i, y + j, color);
			else
				ILI9341_DrawPixel(x + i, y + j, bg);
		}
	}
}
// lcd 에 한 글자 쓰기를 문자열로
void ILI9341_DrawString(uint16_t x, uint16_t y, const char* str, uint16_t color, uint16_t bg)
{
	while (*str)
	{
		ILI9341_DrawChar(x, y, *str, color, bg);
		x += 6;
		str++;
	}
}
// adc_init 는 테스트용 가변저항, adc2_init 는 실제로 사용할 마이크의 adc 설정
void adc_init(void)
{
	DDRF &= ~((1 << PF0) | (1 << PF1));
	PORTF &= ~((1 << PF0) | (1 << PF1));
	ADMUX = (1 << 6) | (0 << 0);
	ADCSRA = (1 << 7) | (1 << 2) | (1 << 1) | (1 << 0);
}
void adc2_init(void)
{
	DDRF &= ~((1 << PF0) | (1 << PF1));
	PORTF &= ~((1 << PF0) | (1 << PF1));
	ADMUX = (1 << 6) | (1 << 0);
	ADCSRA = (1 << 7) | (1 << 2) | (1 << 1) | (1 << 0);
}
// 공통으로 adc 읽기 사용
uint16_t adc_read(void)
{
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	ADCSRA |= (1 << ADIF);
	return ADC;
}
// fft 의 속도를 올리기 위해 샘플 수를 64 와 입력을 먼저 뒤섞는 시간분할 고속푸리에변환 방식 사용
#define N_WAVE 256
#define LOG2_N_WAVE 8
#define N 64
int8_t Sinewave[N_WAVE - N_WAVE / 4];
int8_t* Cosinewave = Sinewave + N_WAVE / 4;
int8_t real[N];
int8_t imag[N];
void init_wave(void)
{
	for (int i = 0; i < N_WAVE - N_WAVE / 4; i++) Sinewave[i] = (int8_t)(127 * sin(2 * M_PI * i / N_WAVE));
}
// 속도를 좀 더 올리기 위해 부동소수점이 아닌 고정소수점을 사용
int fix_fft(int8_t fr[], int8_t fi[], int m)
{
	int mr, nn, i, j, l, k, istep, n;
	int8_t qr, qi, wr, wi, tr, ti;
	n = 1 << m;
	mr = 0;
	nn = n - 1;
	for (m = 1; m <= nn; ++m) {
		l = n; do { l >>= 1; } while (mr + l > nn);
		mr = (mr & (l - 1)) + l;
		if (mr <= m) continue;
		tr = fr[m]; fr[m] = fr[mr]; fr[mr] = tr;
		ti = fi[m]; fi[m] = fi[mr]; fi[mr] = ti;
	}
	l = 1; k = LOG2_N_WAVE - 1;
	while (l < n) {
		istep = l << 1;
		for (j = 0; j < l; ++j) {
			int idx = j << k;
			wr = Cosinewave[idx];
			wi = -Sinewave[idx];
			for (i = j; i < n; i += istep) {
				int j2 = i + l;
				tr = ((wr * fr[j2]) - (wi * fi[j2])) >> 7;
				ti = ((wr * fi[j2]) + (wi * fr[j2])) >> 7;
				qr = fr[i]; qi = fi[i];
				fr[j2] = qr - tr; fi[j2] = qi - ti;
				fr[i] = qr + tr; fi[i] = qi + ti;
			}
		}
		--k; l = istep;
	}
	return 0;
}
uint8_t calc_magnitude(int16_t real, int16_t imag)
{
	int32_t mag = abs(real) + abs(imag);
	if (mag > 255) mag = 255;
	return (uint8_t)mag;
}
// adc 와 fft 의 그래프를 lcd 에 그리기 위해
#define ADC_BASE_Y 120
#define ADC_HEIGHT 60
#define FFT_BASE_Y 230
#define FFT_HEIGHT 80
#define FFT_START_X 20
#define FFT_BAR_WIDTH 3
#define FFT_GAP 2
// lcd 에서 속도가 버벅일 수 있어서 값이 변한 부분만 다른 색으로 그리도록 만든다
void ILI9341_DrawBarVerticalRealtime(uint16_t x, uint16_t yBottom,
	uint16_t* oldHeight, uint16_t newHeight,
	uint16_t width, uint16_t color)
{
	uint8_t hi = color >> 8, lo = color & 0xFF;
	if (*oldHeight == newHeight) return;
	if (newHeight > *oldHeight) {
		ILI9341_SetAddrWindow(x, yBottom - newHeight, x + width - 1, yBottom - *oldHeight - 1);
		DC_DATA();
		CS_LOW();
		for (uint32_t i = 0; i < (uint32_t)(newHeight - *oldHeight) * width; i++) { SPI_txrx(hi); SPI_txrx(lo); }
		CS_HIGH();
	}
	else {
		ILI9341_SetAddrWindow(x, yBottom - *oldHeight, x + width - 1, yBottom - newHeight - 1);
		DC_DATA();
		CS_LOW();
		for (uint32_t i = 0; i < (uint32_t)(*oldHeight - newHeight) * width; i++) { SPI_txrx(0x00); SPI_txrx(0x00); }
		CS_HIGH();
	}
	*oldHeight = newHeight;
}
// adc 그래프
void draw_adc_bar(uint16_t value)
{
	static uint16_t prev_height = 0;
	uint16_t height = (value * ADC_HEIGHT) / 1023;
	ILI9341_DrawBarVerticalRealtime(20, ADC_BASE_Y, &prev_height, height, 10, 0xF800);
}
// fft 그래프
void draw_fft_spectrum(int8_t* real, int8_t* imag)
{
	static uint16_t prev_heights[N / 2] = { 0 };
	for (uint8_t i = 0; i < N / 2; i++) {
		uint8_t mag = calc_magnitude(real[i], imag[i]);
		if (mag > FFT_HEIGHT) mag = FFT_HEIGHT;
		uint16_t x = FFT_START_X + i * (FFT_BAR_WIDTH + FFT_GAP);
		ILI9341_DrawBarVerticalRealtime(x, FFT_BASE_Y, &prev_heights[i], mag, FFT_BAR_WIDTH, 0x07FF);
	}
}
// 마이크가 실제로 잘 동작하는지 확인하기 위해 만든 파형을 그리는 함수
#define ADC_WAVE_X       20
#define ADC_WAVE_Y       160
#define ADC_WAVE_HEIGHT  80
#define ADC_WAVE_N       32
#define ADC_WAVE_COLOR   0x07E0
#define ADC_WAVE_BG      0x0000
#define ADC_WAVE_X_SCALE 30
uint16_t adc_samples[ADC_WAVE_N] = { 0 };
uint16_t prev_samples[ADC_WAVE_N] = { 0 };
void update_adc_samples(void)
{
	for (uint8_t i = 0; i < ADC_WAVE_N - 1; i++)
		adc_samples[i] = adc_samples[i + 1];
	adc_samples[ADC_WAVE_N - 1] = adc_read();
}
static inline int calc_wave_y(uint16_t value)
{
	return ADC_WAVE_Y - ((value * ADC_WAVE_HEIGHT) / 511 + 20);
}
void draw_adc_wave_line_scaled(void)
{
	for (uint8_t i = 0; i < ADC_WAVE_N - 1; i++) {
		int x1 = ADC_WAVE_X + i * ADC_WAVE_X_SCALE;
		int y1 = calc_wave_y(prev_samples[i]);
		int x2 = ADC_WAVE_X + (i + 1) * ADC_WAVE_X_SCALE;
		int y2 = calc_wave_y(prev_samples[i + 1]);
		ILI9341_DrawLine(x1, y1, x2, y2, ADC_WAVE_BG);
	}
	for (uint8_t i = 0; i < ADC_WAVE_N - 1; i++) {
		int x1 = ADC_WAVE_X + i * ADC_WAVE_X_SCALE;
		int y1 = calc_wave_y(adc_samples[i]);
		int x2 = ADC_WAVE_X + (i + 1) * ADC_WAVE_X_SCALE;
		int y2 = calc_wave_y(adc_samples[i + 1]);
		ILI9341_DrawLine(x1, y1, x2, y2, ADC_WAVE_COLOR);
	}
	for (uint8_t i = 0; i < ADC_WAVE_N; i++)
		prev_samples[i] = adc_samples[i];
}
// 스위치 디바운싱
uint8_t read_btn(uint8_t pin)
{
	if (PIND & (1 << pin))
		return 0;
	for (uint8_t i = 0; i < 3; i++) {
		_delay_ms(50);
		if (PIND & (1 << pin)) return 0;
	}
	return 1;
}
// 현재 모드와 메뉴로 들어갔는지 확인을 위해
volatile uint8_t current_mode = 0;
volatile uint8_t menu_active = 0;
// 스피커로 출력을 하기위해 필요
volatile uint8_t is_playing = 0;
volatile uint8_t play_buf_idx = 0;
volatile uint8_t current_play_buf = 0;
volatile uint8_t refill_needed = 0;
// 외부 인터럽트
ISR(INT0_vect)
{
	_delay_ms(50);
	if (!(PIND & (1 << PD0)))
	{
		IN_LED_ON(); // 인터럽트 동안 atmega128 의 LED 를 켠다
		menu_active = 1;
	}
}
// 스피커 재생을 위한 인터럽트로 버퍼를 확인해 pwm 으로 출력
ISR(TIMER0_COMP_vect)
{
	if (!is_playing) return;
	uint8_t* pBuf = (current_play_buf == 0) ? buffer_A : buffer_B;
	uint8_t low = pBuf[play_buf_idx];
	uint8_t high = pBuf[play_buf_idx + 1];
	uint16_t sample = (uint16_t)low | ((uint16_t)high << 8);
	OCR2 = (uint8_t)(sample >> 8);
	play_buf_idx += 2;

	if (play_buf_idx >= BUF_SIZE) {
		play_buf_idx = 0;
		if (current_play_buf == 0) {
			refill_needed = 1;
			current_play_buf = 1;
		}
		else {
			refill_needed = 2;
			current_play_buf = 0;
		}
	}
}
// 스위치 설정
void switch_init(void)
{
	DDRD &= ~((1 << PD0) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7));
	PORTD |= (1 << PD0) | (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
	EICRA = (1 << ISC01);
	EIMSK = (1 << INT0);
}
// 녹음 스위치를 누르고 있는 지 확인
uint8_t switch_pressed(void)
{
	return (PIND & (1 << PD7)) == 0;
}
// 메뉴 스위치를 누를 시 lcd 에 메뉴 그리기 처음 상태는 Mode1 에 커서를 올려둔다
void draw_menu(uint8_t cursor)
{
	ILI9341_FillRect(20, 20, 100, 8, 0x0000);
	ILI9341_DrawString(40, 30, "---- MENU ----", 0xFFFF, 0x0000);
	ILI9341_DrawString(40, 70, "> Mode1", 0xF800, 0x0000);
	ILI9341_DrawString(40, 90, "  Mode2", 0xFFFF, 0x0000);
	ILI9341_DrawString(40, 110, "  Mode3", 0xFFFF, 0x0000);
	ILI9341_DrawString(40, 130, "  Mode4", 0xFFFF, 0x0000);
}
// 커서의 위치를 업데이트
void update_cursor(uint8_t old_cursor, uint8_t new_cursor)
{
	switch (old_cursor)
	{
	case 1: ILI9341_DrawString(40, 70, "  Mode1", 0xFFFF, 0x0000);
		break;
	case 2: ILI9341_DrawString(40, 90, "  Mode2", 0xFFFF, 0x0000);
		break;
	case 3: ILI9341_DrawString(40, 110, "  Mode3", 0xFFFF, 0x0000);
		break;
	case 4: ILI9341_DrawString(40, 130, "  Mode4", 0xFFFF, 0x0000);
		break;
	}
	switch (new_cursor)
	{
	case 1: ILI9341_DrawString(40, 70, "> Mode1", 0xF800, 0x0000);
		break;
	case 2: ILI9341_DrawString(40, 90, "> Mode2", 0xF800, 0x0000);
		break;
	case 3: ILI9341_DrawString(40, 110, "> Mode3", 0xF800, 0x0000);
		break;
	case 4: ILI9341_DrawString(40, 130, "> Mode4", 0xF800, 0x0000);
		break;
	}
}
// 선택한 모드를 실행
// mode1 은 테스트로 실시간 adc 의 그래프와 fft 의 그래프를 lcd 에 그리기
// mode2 는 테스트로 adc 파형을 lcd 에 그리기
// mode3 는 스위치 누르는 동안 sd 에 adc 값을 wav 파일로 만들어 저장
// mode4 는 저장한 파일을 sd 에서 읽어 fft 그래프로 lcd 에 그리고 이때 sd 모듈과 lcd 는 모두 spi 통신을 하기에 cs 를 계속 바꿔준다
// mode5 는 저장한 파일을 sd 에서 읽어 pwm 을 이용해 스피커 드라이버를 거쳐 스피커로 출력
void run_mode(uint8_t m)
{
	ILI9341_FillRect(20, 20, 100, 8, 0x0000);
	switch (m)
	{
		ILI9341_FillScreen(0x0000);  // 먼저 lcd 에 메뉴를 지우기 위해 검은색으로 채운다
	case 1:
		adc2_init();
		ILI9341_DrawString(20, 10, "MIC FFT", 0xFFFF, 0x0000);
		while (menu_active == 0)
		{
			for (uint8_t i = 0; i < N; i++) {
				uint16_t adc_val = adc_read();
				real[i] = (adc_val >> 2) - 128;
				imag[i] = 0;
			}
			fix_fft(real, imag, 6);
			uint16_t current_adc = adc_read();
			draw_adc_bar(current_adc);
			draw_fft_spectrum(real, imag);
		}
		break;
	case 2:
		adc2_init();
		ILI9341_DrawString(20, 10, "Analog Mic signal", 0xFFFF, 0x0000);
		while (menu_active == 0)
		{
			update_adc_samples();
			draw_adc_wave_line_scaled();
		}
		break;
	case 3:
	{
		FRESULT fr;
		UINT bw;
		uint8_t* current_buf = buffer_A;
		uint8_t* save_buf = NULL;
		uint16_t buf_idx = 0;
		uint8_t active_buf_idx = 0;
		volatile uint8_t data_ready = 0;
		uint32_t total_recorded_bytes = 0;
		adc2_init();
		ILI9341_FillScreen(0x0000);
		ILI9341_DrawString(20, 10, "REC MODE", 0xFFFF, 0x0000);
		ILI9341_DrawString(20, 40, "Press PD7 to REC", 0x07E0, 0x0000);
		DDRD &= ~(1 << PD7);
		PORTD |= (1 << PD7);
		while (PIND & (1 << PD7))
		{
			if (menu_active) break;
		}
		if (!(PIND & (1 << PD7)))
		{
			printf("\r\n[REC] Preparing...\r\n");
			ILI9341_DrawString(20, 60, "Initializing...", 0xFFFF, 0x0000);
			_delay_ms(100);
			SPCR = 0;
			SPSR = 0;
			DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2);
			PORTB |= (1 << PB0);
			DDRC |= (1 << PC0);
			PORTC |= (1 << PC0);
			PORTB |= (1 << PB2);
			for (uint8_t i = 0; i < 80; i++) {
				PORTB |= (1 << PB1); _delay_us(10);
				PORTB &= ~(1 << PB1); _delay_us(10);
			}
			SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
			fr = f_mount(&fs_global, "", 1);
			if (fr == FR_OK) {

				SPCR = (1 << SPE) | (1 << MSTR);
				SPSR |= (1 << SPI2X);
				printf("[REC] Mount OK. Opening File...\r\n");

				fr = f_open(&file_global, "AUDIO.WAV", FA_WRITE | FA_CREATE_ALWAYS);
				if (fr == FR_OK) {
					printf("[REC] Recording Started! Press PD7 to Stop.\r\n");

					uint8_t dummy_header[44] = {0};   // wav 공간 확보
					f_write(&file_global, dummy_header, 44, &bw);
					current_buf = buffer_A;
					buf_idx = 0;
					active_buf_idx = 0;
					data_ready = 0;
					total_recorded_bytes = 0;
					TCCR0 = (1 << WGM01) | (1 << CS01);
					OCR0 = 249;
					TCNT0 = 0;
					TIFR |= (1 << OCF0);

					ADMUX = (1 << REFS0) | (1 << MUX0);
					ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
					while (1)
					{

						if (TIFR & (1 << OCF0)) {
							TIFR |= (1 << OCF0);

							ADCSRA |= (1 << ADSC);
							while (ADCSRA & (1 << ADSC));
							uint16_t adc_val = ADC;
							uint16_t pcm_val = adc_val << 6;
							current_buf[buf_idx++] = pcm_val & 0xFF;
							current_buf[buf_idx++] = (pcm_val >> 8) & 0xFF;
							if (buf_idx >= BUF_SIZE) {
								save_buf = current_buf;
								data_ready = 1;

								if (active_buf_idx == 0) {
									current_buf = buffer_B;
									active_buf_idx = 1;
								}
								else {
									current_buf = buffer_A;
									active_buf_idx = 0;
								}
								buf_idx = 0;
							}
						}
						if (data_ready) {
							PORTB |= (1 << PB0);
							fr = f_write(&file_global, save_buf, BUF_SIZE, &bw);
							if (fr != FR_OK) {
								printf("[REC] Write Error: %d\r\n", fr);
								break;
							}
							total_recorded_bytes += bw;
							data_ready = 0;
						}
						if (!(PIND & (1 << PD7)))
						{
							_delay_ms(50);
							if (!(PIND & (1 << PD7)))
							{
								printf("\r\n[REC] Stop Button Pressed.\r\n");
								break;
							}
						}
					}
					printf("[REC] Finalizing...\r\n");
					// wav 헤더의 업데이트를 위해 파일 처음으로 이동
					fr = f_lseek(&file_global, 0);
					if (fr == FR_OK) {
						write_wav_header(&file_global, total_recorded_bytes, 8000);
						printf("[REC] Header Updated.\r\n");
					}
					f_close(&file_global);
					f_mount(NULL, "", 0);
					printf("[REC] Saved AUDIO.WAV (%lu bytes)\r\n", total_recorded_bytes);
				}
				else {
					printf("[REC] File Open Fail: %d\r\n", fr);
				}
			}
			else {
				printf("[REC] Mount Fail: %d\r\n", fr);
			}
			while (!(PIND & (1 << PD7)));
			_delay_ms(100);
			SPCR = (1 << SPE) | (1 << MSTR);
			SPSR = 0;
			ILI9341_FillScreen(0x0000);
			ILI9341_DrawString(20, 50, "REC COMPLETE", 0x07E0, 0x0000);
			_delay_ms(1000);
		}
	}
	break;
	case 4:
	{
		FRESULT fr;
		UINT br;
		adc2_init();
		ILI9341_FillScreen(0x0000);
		ILI9341_DrawString(20, 10, "SD PLAY FFT", 0xFFFF, 0x0000);
		ILI9341_DrawString(20, 30, "Reading AUDIO.WAV...", 0x07E0, 0x0000);
		SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
		fr = f_mount(&fs_global, "", 1);
		if (fr == FR_OK) {
			fr = f_open(&file_global, "AUDIO.WAV", FA_READ);
			if (fr == FR_OK) {
				f_lseek(&file_global, 44);
				ILI9341_FillScreen(0x0000);
				ILI9341_DrawString(20, 10, "PLAYING...", 0xFFFF, 0x0000);

				while (menu_active == 0)
				{
					PORTB |= (1 << PB0);
					SPCR = (1 << SPE) | (1 << MSTR);
					SPSR |= (1 << SPI2X);
					fr = f_read(&file_global, buffer_A, 128, &br);
					if (fr != FR_OK || br < 128) {
						break;
					}
					for (uint8_t i = 0; i < N; i++) {

						int16_t pcm_val = buffer_A[i * 2] | (buffer_A[i * 2 + 1] << 8);
						real[i] = (int8_t)((pcm_val >> 8) - 128);
						imag[i] = 0;
					}
					// fft 연산
					fix_fft(real, imag, 6);
					SPCR = (1 << SPE) | (1 << MSTR);
					SPSR = (1 << SPI2X);
					// fft 그래프
					draw_fft_spectrum(real, imag);
					// 메뉴 버튼 확인
					if (menu_active) break;
				}
				f_close(&file_global);
			}
			else {
				ILI9341_DrawString(20, 50, "File Open Err", 0xF800, 0x0000);
				_delay_ms(2000);
			}
		}
		else {
			ILI9341_DrawString(20, 50, "SD Mount Err", 0xF800, 0x0000);
			_delay_ms(2000);
		}
		SPI0_init();
		ILI9341_FillScreen(0x0000);
	}
	break;
	case 5:
	{
		FRESULT fr;
		UINT br;
		adc2_init();
		ILI9341_FillScreen(0x0000);
		ILI9341_DrawString(20, 10, "PWM PLAYBACK", 0xFFFF, 0x0000);
		ILI9341_DrawString(20, 30, "Connect PB7 -> Amp", 0x07E0, 0x0000);
		SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
		fr = f_mount(&fs_global, "", 1);
		if (fr == FR_OK) {
			fr = f_open(&file_global, "AUDIO.WAV", FA_READ);
			if (fr == FR_OK) {
				f_lseek(&file_global, 44);
				printf("[PLAY] File Open OK. Buffering...\r\n");
				SPCR = (1 << SPE) | (1 << MSTR);
				SPSR |= (1 << SPI2X);
				f_read(&file_global, buffer_A, BUF_SIZE, &br);
				f_read(&file_global, buffer_B, BUF_SIZE, &br);
				DDRB |= (1 << PB7);

				TCCR2 = (1 << WGM21) | (1 << WGM20) | (1 << COM21) | (1 << CS20);
				OCR2 = 0;

				TCCR0 = (1 << WGM01) | (1 << CS01);
				OCR0 = 249;

				play_buf_idx = 0;
				current_play_buf = 0;
				refill_needed = 0;
				is_playing = 1;
				TIFR |= (1 << OCF0);
				TIMSK |= (1 << OCIE0);
				ILI9341_DrawString(20, 50, "Playing...", 0xFFFF, 0x0000);
				while (menu_active == 0)
				{
					if (refill_needed != 0) {
						PORTB |= (1 << PB0);
						SPCR = (1 << SPE) | (1 << MSTR);
						SPSR |= (1 << SPI2X);
						if (refill_needed == 1) {
							f_read(&file_global, buffer_A, BUF_SIZE, &br);
						}
						else {
							f_read(&file_global, buffer_B, BUF_SIZE, &br);
						}
						if (br < BUF_SIZE) {
							printf("[PLAY] End of File.\r\n");
							break;
						}
						refill_needed = 0;
					}
					if (menu_active) break;
				}

				TIMSK &= ~(1 << OCIE0);
				is_playing = 0;
				OCR2 = 0;
				f_close(&file_global);
			}
			else {
				ILI9341_DrawString(20, 50, "No File", 0xF800, 0x0000);
				_delay_ms(1000);
			}
		}
		else {
			ILI9341_DrawString(20, 50, "Mount Fail", 0xF800, 0x0000);
			_delay_ms(1000);
		}

		SPI0_init();
		ILI9341_FillScreen(0x0000);
	}
	break;
	}
}
// 메뉴 글자 지우기
void menu_remove()
{
	ILI9341_FillRect(20, 20, 100, 8, 0x0000);
	ILI9341_FillRect(40, 30, 85, 8, 0x0000);
	ILI9341_FillRect(40, 70, 80, 8, 0x0000);
	ILI9341_FillRect(40, 90, 80, 8, 0x0000);
	ILI9341_FillRect(40, 110, 80, 8, 0x0000);
	ILI9341_FillRect(40, 130, 80, 8, 0x0000);
}
int main(void)
{
	SPI0_init();
	ILI9341_Init(0xE8);
	ILI9341_FillScreen(0x0000);
	init_wave();
	uart0_init();
	fdev_setup_stream(&uart0_stdout, uart0_putchar, NULL, _FDEV_SETUP_WRITE);
	stdout = &uart0_stdout;
	switch_init();
	sei();   // 글로벌 인터럽트 허용
	uint8_t cursor = 1;
	run_mode(0);
	while (1)
	{
		if (menu_active)
		{
			cursor = 1;
			ILI9341_FillScreen(0x0000);
			SPCR = (1 << SPE) | (1 << MSTR);
			SPSR = 0;
			ILI9341_FillScreen(0x0000);
			draw_menu(cursor);
			while (menu_active)
			{
				uint8_t old_cursor = cursor;  // 이전 커서 위치 저장
				if (read_btn(PD4))   // up
				{
					if (cursor > 1) cursor--;
					if (cursor != old_cursor)
						update_cursor(old_cursor, cursor);  // 메뉴에서 전체가 아니라 커서만 갱신해준다
				}
				if (read_btn(PD5))   // down
				{
					if (cursor < 4) cursor++;
					if (cursor != old_cursor)
						update_cursor(old_cursor, cursor);
				}
				if (read_btn(PD6))   // select
				{
					current_mode = cursor;   // 선택된 모드를 확인
					menu_active = 0;         // 메뉴 종료
					IN_LED_OFF();
					menu_remove();
				}
			}
			run_mode(current_mode);
		}
		else
		{
			run_mode(current_mode);
			_delay_ms(200);
		}
	}
	return 0;
}
