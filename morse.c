/*
Morse code encoder/decoder
by Alex Khrabrov <alex@mroja.net>
June 2010

Licensed under a GPL-compatible free software license - WTFPL.

***

DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
Version 2, December 2004

Copyright (C) 2004 Sam Hocevar
14 rue de Plaisance, 75014 Paris, France
Everyone is permitted to copy and distribute verbatim or modified
copies of this license document, and changing it is allowed as long
as the name is changed.

DO WHAT THE FUCK YOU WANT TO PUBLIC LICENSE
TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION

0. You just DO WHAT THE FUCK YOU WANT TO.

*/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

//---------------------------------------------------------------
// config
//---------------------------------------------------------------

// output and input port configuration
#define OUTPUT_PORT PORTD
#define OUTPUT_PINS (BV(PD6)|BV(PD5)|BV(PB4))
#define OUTPUT_PIN_BUZZER PD6

#define INPUT_PORT PINB
#define INPUT_PIN  PB2

#define BUTTONS_PORT PINB
#define BUTTON_1_PIN PB3
#define BUTTON_2_PIN PB4
#define BUTTON_3_PIN PB5
#define BUTTON_4_PIN PB6

#define MODE_SWITCH_PORT PINB
#define RECEIVE_PIN      PB0
#define TRANSMIT_PIN     PB1

// play durations in ms
#define DURATION_DIT       60
#define DURATION_DAH       (3*DURATION_DIT)
#define DURATION_INNER     15
#define DURATION_NEXT_CHAR 100
#define DURATION_NEXT_WORD 300

// number of samples to calculate averages from
#define AVG_BUF_SIZE 5

// buffer sizes
#define RECEIVE_BUFFER_SIZE 128
#define CHAR_BUFFER_SIZE    128

//---------------------------------------------------------------
// some useful defines
//---------------------------------------------------------------
#ifndef BV
    #define BV(bit) _BV(bit)
#endif

// gbi - get bit
// cbi - clear bit
// sbi - set bit
// tbi - toggle bit

#ifndef gbi
    #define gbi(sfr, bit) (_SFR_BYTE(sfr) & BV(bit))
#endif

#ifndef cbi
    #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~(BV(bit)))
#endif

#ifndef sbi
    #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= BV(bit))
#endif

#ifndef tbi
    #define tbi(sfr, bit) (_SFR_BYTE(sfr) ^= BV(bit))  
#endif

// bit_is_set, bit_is_clear
#ifndef bit_is_set
    #define bit_is_set(sfr, bit) (_SFR_BYTE(sfr) & BV(bit))
#endif

#ifndef bit_is_clear
    #define bit_is_clear(sfr, bit) (!(_SFR_BYTE(sfr) & BV(bit)))
#endif

#define bis(sfr, bit) bit_is_set(sfr, bit)
#define bic(sfr, bit) bit_is_clear(sfr, bit)

#define ever ;;
#define forever for(ever) 

//---------------------------------------
// 16x2 CHAR LCD
//---------------------------------------

// control lines
// PC7 = RS
// PC6 = RW
// PC5 = E

// data lines
// PA0 = D0
// PA1 = D1
// PA2 = D2
// PA3 = D3
// PA4 = D4
// PA5 = D5
// PA6 = D6
// PA7 = D7

static void hd44780_configure_ctrl_lines(void)
{
    DDRC  |=   1 << PC7 | 1 << PC6 | 1 << PC5;
    PORTC &= ~(1 << PC7 | 1 << PC6 | 1 << PC5);
}

inline void hd44780_rs_high(void) { PORTC |=    1 << PC7  ; }
inline void hd44780_rs_low (void) { PORTC &= ~( 1 << PC7 ); }
inline void hd44780_rw_high(void) { PORTC |=    1 << PC6  ; }
inline void hd44780_rw_low (void) { PORTC &= ~( 1 << PC6 ); }
inline void hd44780_e_high (void) { PORTC |=    1 << PC5  ; }
inline void hd44780_e_low  (void) { PORTC &= ~( 1 << PC5 ); }

static void hd44780_configure_data_lines_as_outputs(void)
{
    DDRA = 0xff;
}

static void hd44780_configure_data_lines_as_inputs(void)
{
    DDRA = PORTA = 0;
}

static uint8_t hd44780_read(void)
{
    hd44780_configure_data_lines_as_inputs();
    hd44780_rw_high();
    hd44780_e_high();
    _delay_us(1);
    uint8_t value = PINA;
    hd44780_e_low();
    hd44780_rw_low();
    hd44780_configure_data_lines_as_outputs();
    return value;
}

static void hd44780_write_half(uint8_t value)
{
    value <<= 4;
    value |= PORTA & 0x0f;
    PORTA = value;
    hd44780_e_high();
    hd44780_e_low();
}

static void hd44780_write(uint8_t value)
{
    PORTA = value;
    hd44780_e_high();
    hd44780_e_low();
    _delay_us(50);
}

uint8_t hd44780_read_command(void)
{
    hd44780_rs_low(); 
    return hd44780_read();
}

uint8_t hd44780_read_data(void)
{
    hd44780_rs_high();
    return hd44780_read();
}

static void hd44780_loop_until_ready(void)
{
    //while(hd44780_read_command() & 0b10000000);
}

static void hd44780_loop_until_ready_and_write(uint8_t value)
{
    hd44780_loop_until_ready();
    hd44780_write( value );
}

void hd44780_write_command(uint8_t v)
{
    hd44780_rs_low();
    hd44780_loop_until_ready_and_write(v);
}

void hd44780_write_data(uint8_t v)
{
    hd44780_rs_high();
    hd44780_loop_until_ready_and_write(v);
}

void hd44780_write_text(const uint8_t *p)
{
    while(*p)
        hd44780_write_data(*p++);
}

void hd44780_clear_display(void)
{
    hd44780_write_command(0b00000001);
    _delay_ms(2);
}

void hd44780_return_home(void)
{
    hd44780_write_command(0b00000010);
    _delay_ms(2);
}

static uint8_t cfg1 = 0b00000110;

void hd44780_auto_right(void)
{
    cfg1 |= 0b00000010;
    hd44780_write_command(cfg1);
}

void hd44780_auto_left(void)
{
    cfg1 &= ~0b00000010;
    hd44780_write_command(cfg1);
}

void hd44780_auto_scroll(void)
{
    cfg1 |= 0b00000001;
    hd44780_write_command(cfg1);
}

void hd44780_auto_noscroll(void)
{
    cfg1 &= ~0b00000001;
    hd44780_write_command(cfg1);
}

static uint8_t cfg2 = 0b00001000;

void hd44780_display_on(void)
{
    cfg2 |= 0b00000100;
    hd44780_write_command(cfg2);
}

void hd44780_display_off (void)
{
    cfg2 &= ~0b00000100;
    hd44780_write_command(cfg2);
}

void hd44780_cursor_on(void)
{
    cfg2 |= 0b00000010;
    hd44780_write_command(cfg2);
}

void hd44780_cursor_off(void)
{
    cfg2 &= ~0b00000010;
    hd44780_write_command(cfg2);
}

void hd44780_blinking_on(void)
{
    cfg2 |= 0b00000001;
    hd44780_write_command(cfg2);
}

void hd44780_blinking_off(void)
{
    cfg2 &= ~0b00000001;
    hd44780_write_command(cfg2);
}

void hd44780_shift_right(void)
{
    hd44780_write_command(0b00011100);
}

void hd44780_shift_left(void)
{
    hd44780_write_command(0b00011000);
}

void hd44780_cursor_right(void)
{
    hd44780_write_command(0b00010100);
}

void hd44780_cursor_left(void)
{
    hd44780_write_command(0b00010000);
}

void hd44780_set_caddr(uint8_t x)
{
    hd44780_write_command(0b01000000|(x&0b00111111));
}

void hd44780_set_daddr(uint8_t x)
{
    hd44780_write_command(0b10000000|(x&0b01111111));
}

void hd44780_goto(uint8_t x, uint8_t y)
{
    hd44780_set_daddr(x + 0x40*y);
}

void hd44780_initialize(void)
{
    hd44780_configure_ctrl_lines();
    hd44780_configure_data_lines_as_outputs();
    _delay_ms(15);
    for(uint8_t i=0; i<3; i++)
    {
        hd44780_write_half(0x03);
        _delay_ms(5);
    }
    hd44780_write(0b00111000); // 4 bits, 2 lines, 5x8 fonts
    _delay_us(50);
    hd44780_write(cfg2);
    _delay_us(50);
    hd44780_write(0b00000001); // clear display
    _delay_ms(2);
    hd44780_write(cfg1);
    _delay_us(50);
    hd44780_display_on();
}

//---------------------------------------------------------------
// global buffers
//---------------------------------------------------------------

// in receive mode used to hold decoded text
// in transmit mode holds text to be transmited
static uint8_t chars[CHAR_BUFFER_SIZE];
static uint8_t charsLen = 0;

//---------------------------------------------------------------
// morse code definitions
//---------------------------------------------------------------

#if defined(__GNUC__)
    #define PACKED __attribute__((packed))
#else
    #define PACKED
#endif

#define MAX_CODE_LENGTH 12 // max 12 bits
typedef union
{
    struct
    {
        uint16_t code   : 12; // MAX_CODE_LENGTH bits
        uint16_t length : 4;  // 0...15
    } PACKED;
    uint16_t data;
} PACKED MorseCodeData;

typedef struct
{
    uint8_t ch;
    MorseCodeData u;
} PACKED MorseEntry;

#define _(x)   (x<<1)     // dit - 0
#define ___(x) ((x<<1)|1) // dah - 1
#define MORSE(LEN, CODE) {{ CODE, LEN }}
#define MORSE_ENTRY(CH, LEN, CODE) { CH, MORSE(LEN, CODE) }
static MorseEntry morseCodes[] = 
{
    MORSE_ENTRY('0', 5, ___(___(___(___(___(0)))))),
    MORSE_ENTRY('1', 5, _(___(___(___(___(0)))))),
    MORSE_ENTRY('2', 5, _(_(___(___(___(0)))))),
    MORSE_ENTRY('3', 5, _(_(_(___(___(0)))))),
    MORSE_ENTRY('4', 5, _(_(_(_(___(0)))))),
    MORSE_ENTRY('5', 5, _(_(_(_(_(0)))))),
    MORSE_ENTRY('6', 5, ___(_(_(_(_(0)))))),
    MORSE_ENTRY('7', 5, ___(___(_(_(_(0)))))),
    MORSE_ENTRY('8', 5, ___(___(___(_(_(0)))))),
    MORSE_ENTRY('9', 5, ___(___(___(___(_(0)))))),
    MORSE_ENTRY('A', 2, _(___(0))),
    MORSE_ENTRY('B', 4, ___(_(_(_(0))))),
    MORSE_ENTRY('C', 4, ___(_(___(_(0))))),
    MORSE_ENTRY('D', 3, ___(_(_(0)))),
    MORSE_ENTRY('E', 1, _(0)),
    MORSE_ENTRY('F', 4, _(_(___(_(0))))),
    MORSE_ENTRY('G', 3, ___(___(_(0)))),
    MORSE_ENTRY('H', 4, _(_(_(_(0))))),
    MORSE_ENTRY('I', 2, _(_(0))),
    MORSE_ENTRY('J', 4, _(___(___(___(0))))),
    MORSE_ENTRY('K', 3, ___(_(___(0)))),
    MORSE_ENTRY('L', 4, _(___(_(_(0))))),
    MORSE_ENTRY('M', 2, ___(___(0))),
    MORSE_ENTRY('N', 2, ___(_(0))),
    MORSE_ENTRY('O', 3, ___(___(___(0)))),
    MORSE_ENTRY('P', 4, _(___(___(_(0))))),
    MORSE_ENTRY('Q', 4, ___(___(_(___(0))))),
    MORSE_ENTRY('R', 3, _(___(_(0)))),
    MORSE_ENTRY('S', 3, _(_(_(0)))),
    MORSE_ENTRY('T', 1, ___(0)),
    MORSE_ENTRY('U', 3, _(_(___(0)))),
    MORSE_ENTRY('V', 4, _(_(_(___(0))))),
    MORSE_ENTRY('W', 3, _(___(___(0)))),
    MORSE_ENTRY('X', 4, ___(_(_(___(0))))),
    MORSE_ENTRY('Y', 4, ___(_(___(___(0))))),
    MORSE_ENTRY('Z', 4, ___(___(_(_(0))))),
    MORSE_ENTRY('.', 6, _(___(_(___(_(___(0))))))),
    MORSE_ENTRY(',', 6, ___(___(_(_(___(___(0))))))),
    MORSE_ENTRY('\'',6, _(___(___(___(___(_(0))))))),
    MORSE_ENTRY('_', 6, _(_(___(___(_(___(0))))))),
    MORSE_ENTRY(':', 6, ___(___(___(_(_(_(0))))))),
    MORSE_ENTRY('?', 6, _(_(___(___(_(_(0))))))),
    MORSE_ENTRY('-', 6, ___(_(_(_(_(___(0))))))),
    MORSE_ENTRY('/', 5, ___(_(_(___(_(0)))))),
    MORSE_ENTRY('(', 5, ___(_(___(___(_(0)))))),
    MORSE_ENTRY(')', 6, ___(_(___(___(_(___(0))))))),
    MORSE_ENTRY('=', 5, ___(_(_(_(___(0)))))),
    MORSE_ENTRY('@', 6, _(___(___(_(___(_(0)))))))
};

// number of morse code entries
static const uint8_t morseSize = sizeof(morseCodes)/sizeof(morseCodes[0]);

void reverseCodeBits(void)
{
    uint8_t i = morseSize;
    while(i--)
    {
        uint8_t n = morseCodes[i].u.length; 
        uint8_t in = morseCodes[i].u.code;
        uint8_t tmp = 0;
        for(uint8_t j=0; j<n; j++)
            if(in & (1<<j))
                tmp |= (1<<(n-j-1));
        morseCodes[i].u.code = tmp;
    }
}

//-----------------------------------------------------------------------------------------------
// RECEIVE BUFFER
//-----------------------------------------------------------------------------------------------
// each byte of buffer can contain four 2 bit entries:
#define DIT        0 // 0b00
#define DAH        1 // 0b01
#define CHAR_DELAY 2 // 0b10
#define WORD_DELAY 3 // 0b11
typedef struct
{
    uint8_t x0 : 2;
    uint8_t x1 : 2;
    uint8_t x2 : 2;
    uint8_t x3 : 2;
} PACKED BufEntry; // 1 byte

volatile uint8_t buffElems;  // current number of elements in buffer
volatile BufEntry buffer[RECEIVE_BUFFER_SIZE/4];

uint8_t bufferGet(uint8_t n)
{
    //cli();
    BufEntry x = buffer[n/4];
    switch(n%4)
    {
        case 0:
            return x.x0;
        case 1:
            return x.x1;
        case 2:
            return x.x2;
        case 3:
            return x.x3;
        default:
            return 0;
    }
    //sei();
}

void bufferAppend(uint8_t x)
{
    if(buffElems == (RECEIVE_BUFFER_SIZE-1))
    {
        //shift buffer elements
        for(int i=0; i<buffElems-1; i++)
        {
            uint8_t v = bufferGet(i+1);
            BufEntry tmp = buffer[i/4];
            switch(i%4)
            {
                case 0:
                    tmp.x0 = v; break;
                case 1:
                    tmp.x1 = v; break;
                case 2:
                    tmp.x2 = v; break;
                case 3:
                    tmp.x3 = v;
            }
            buffer[i/4] = tmp;
        }
        buffElems--;
        goto addElem;
    }
    else
    {
        BufEntry tmp;
addElem:
        tmp = buffer[buffElems/4];

        switch(buffElems%4)
        {
            case 0:
                tmp.x0 = x; break;
            case 1:
                tmp.x1 = x; break;
            case 2:
                tmp.x2 = x; break;
            case 3:
                tmp.x3 = x;
        }
        buffer[buffElems/4] = tmp;
        buffElems++;
    }
}

uint8_t morseToAlpha(MorseCodeData x)
{
    for(uint8_t i=0; i<morseSize; i++)
        if(morseCodes[i].u.length == x.length && morseCodes[i].u.code == x.code)
            return morseCodes[i].ch;
    return '*';
}

void decodeBuffer(void)
{
    charsLen = 0;
    MorseCodeData tmp = MORSE(0, 0);

    cli();
    
    for(int i=0; i<buffElems; i++)
    {
        uint8_t v = bufferGet(i);
        
        if((v == DIT || v == DAH) && tmp.length <= MAX_CODE_LENGTH)
        {
            tmp.code = (v << tmp.length) | tmp.code;
            tmp.length++;
        }
        
        if(i == buffElems-1 || bufferGet(i+1) == CHAR_DELAY)
        {
            if(tmp.length)
                chars[charsLen++] = morseToAlpha(tmp);
            tmp.length = 0;
            tmp.code   = 0;
        }
        else if(i == buffElems-1 || bufferGet(i+1) == WORD_DELAY)
        {
            if(tmp.length)
            {
                chars[charsLen++] = morseToAlpha(tmp);
                chars[charsLen++] = ' ';
            }
            tmp.length = 0;
            tmp.code   = 0;
        }
    }
    
    sei();
}

//-----------------------------------------------------------------------------------------------
// INTERRUPT
//------------------------------------------------------------------------------------------------

#define MODE_TRANSMIT 0
#define MODE_RECEIVE  1
volatile uint8_t mode;

// averages
volatile uint16_t ditDurations[AVG_BUF_SIZE];
volatile uint16_t dahDurations[AVG_BUF_SIZE];
volatile uint16_t avgDitDuration;
volatile uint16_t avgDahDuration;

// in 4.096ms quants
volatile uint16_t delayThresholdDit;
volatile uint16_t delayThresholdDah;
volatile uint16_t delayThresholdChar;
volatile uint16_t delayThresholdWord;

volatile uint16_t counter;

#define ON  0
#define OFF 1
volatile uint8_t oldState = OFF;

/* called every 4.096ms */
ISR(TIMER0_OVF_vect)
{
    if(mode == MODE_RECEIVE)
    {
        if(counter == 0xFFF0) // prevent overflow
            counter = ((uint16_t)0xFFF0)-1;
        
        uint8_t state;
        if(bic(INPUT_PORT, INPUT_PIN)) // "on"
            state = ON;
        else
            state = OFF;

        //if(state == ON)   
        //  sbi(OUTPUT_PORT, OUTPUT_PIN_BUZZER);    
        //else
        //  cbi(OUTPUT_PORT, OUTPUT_PIN_BUZZER);    
        
        if(oldState==ON && state == OFF) // on --> off
        {
            // count -> długość trwania impulsu

            if(counter < delayThresholdDit)
            {
                for(uint8_t i=0; i<AVG_BUF_SIZE-1; i++)
                    ditDurations[i] = ditDurations[i+1];
                ditDurations[AVG_BUF_SIZE-1] = counter;
                
                uint16_t sum = 0;
                for(uint8_t i=0; i<AVG_BUF_SIZE; i++)
                    sum += ditDurations[i];
                avgDitDuration = sum/AVG_BUF_SIZE;
            
                bufferAppend(DIT);
            }
            else //if(counter < delayThresholdDah)
            {
                if(counter <= (delayThresholdWord+10)) // prevent counting extra long dahs
                {
                    for(uint8_t i=0; i<AVG_BUF_SIZE-1; i++)
                        dahDurations[i] = dahDurations[i+1];
                    dahDurations[AVG_BUF_SIZE-1] = counter;
                    
                    uint16_t sum = 0;
                    for(uint8_t i=0; i<AVG_BUF_SIZE; i++)
                        sum += dahDurations[i];
                    avgDahDuration = sum/AVG_BUF_SIZE;
                }
                bufferAppend(DAH);
            }
            counter = 0;
        }
        else if(oldState==OFF && state == ON) // off --> on
        {
            // count -> długość trwania przerwy 
            
            if(counter > delayThresholdChar && counter <= delayThresholdWord)
            {
                bufferAppend(CHAR_DELAY);
            }
            else if(counter > delayThresholdWord)
            {
                bufferAppend(WORD_DELAY);
            }
            counter = 0;
        }
        
        oldState = state;
        counter++;
    }
}

//----------------------------------------------------------
// PLAYER
//----------------------------------------------------------
#define PLAY_BUF_SIZE 16
static uint8_t chBuf[PLAY_BUF_SIZE+1]; // PLAY_BUF_SIZE + '\0'
static uint8_t mBuf[PLAY_BUF_SIZE+1];

void shiftBufferLeft(uint8_t *buf, uint8_t val)
{
    for(uint8_t j=0; j<PLAY_BUF_SIZE-1; j++)
        buf[j] = buf[j+1];
    buf[PLAY_BUF_SIZE-1] = val;
}

static const MorseCodeData morseError = MORSE(8,_(_(_(_(_(_(_(_(0))))))))); // error (8 dots)

MorseCodeData alphaToMorse(uint8_t x)
{
    if(x>='a'&&x<='z')
        x -= 32; // convert to upercase
    for(uint8_t i=0; i<morseSize; i++)
        if(morseCodes[i].ch == x)
            return morseCodes[i].u;
    return morseError; 
}

void playChar(uint8_t ch)
{
    MorseCodeData d = alphaToMorse(ch);
    uint8_t code = d.code;
    uint8_t num  = d.length;
    for(uint8_t j=0; j<num; j++, code >>= 1)
    {
        if(j)
            _delay_ms(DURATION_INNER);

        if(code & 1)
            shiftBufferLeft(mBuf, '-');
        else
            shiftBufferLeft(mBuf, '.');     
        hd44780_goto(0,1);
        hd44780_write_text(mBuf);
            
        OUTPUT_PORT |= OUTPUT_PINS; // turn on
        //OUTPUT_PORT &= ~(OUTPUT_PINS); // turn off
        if(code & 1)
            _delay_ms(DURATION_DAH); 
        else
            _delay_ms(DURATION_DIT);
        OUTPUT_PORT &= ~(OUTPUT_PINS); // turn off
        //OUTPUT_PORT |= OUTPUT_PINS; // turn on
    }
}

void morsePlay(uint8_t *buf, uint8_t n)
{
    uint8_t i = PLAY_BUF_SIZE;
    while(i--)
        chBuf[i] = mBuf[i] = ' ';
    chBuf[PLAY_BUF_SIZE] = mBuf[PLAY_BUF_SIZE] = '\0';

    for(i=0; i<n; i++)
    {
        switch(buf[i])
        {
            case ' ':
            case '\n':
            case '\t':
                shiftBufferLeft(chBuf, ' ');
                hd44780_goto(0,0);
                hd44780_write_text(chBuf);
                shiftBufferLeft(mBuf, '|');
                hd44780_goto(0,1);
                hd44780_write_text(mBuf);
                _delay_ms(DURATION_NEXT_WORD);
                continue; // applies to loop
            default:
                shiftBufferLeft(chBuf, buf[i]);
                hd44780_goto(0,0);
                hd44780_write_text(chBuf);
                playChar(buf[i]);
        }
        shiftBufferLeft(mBuf, '/');
        hd44780_goto(0,1);
        hd44780_write_text(mBuf);
        _delay_ms(DURATION_NEXT_CHAR);
    }
}

//----------------------------------------------------
void renderCharBuf(uint8_t maxLen)
{
    uint8_t i, n;
    if(charsLen > maxLen)
        i = 0;
    else
        i = maxLen-charsLen;
    hd44780_goto(0, 0);
    n=i;
    while(n--)
        hd44780_write_data(' ');
    hd44780_goto(i, 0);
    if(charsLen > maxLen)
        i = charsLen-maxLen;
    else
        i = 0;
    for(; i<charsLen; i++)
        hd44780_write_data(chars[i]);
}

//--------------------------------------
// main
//--------------------------------------
int main(void)
{
    // DDRx -> 1 - output, 0 - input
    // PORTx -> 1 - pull-up on, 0 - pull-up off

    // configure input pins
    DDRB  &= ~(BV(INPUT_PIN) | 
               BV(BUTTON_1_PIN) | BV(BUTTON_2_PIN) |
               BV(BUTTON_3_PIN) | BV(BUTTON_4_PIN) | 
               BV(RECEIVE_PIN)  | BV(TRANSMIT_PIN)); // set as inputs
    PORTB |= (BV(INPUT_PIN) | 
              BV(BUTTON_1_PIN) | BV(BUTTON_2_PIN) |
              BV(BUTTON_3_PIN) | BV(BUTTON_4_PIN) | 
              BV(RECEIVE_PIN)  | BV(TRANSMIT_PIN)); // pull-up on
    
    // configure output pins
    DDRD |= OUTPUT_PINS; // set as output
    
    // set outputs to 0
    OUTPUT_PORT &= ~OUTPUT_PINS;
    
    // configure alphanumerical LCD
    hd44780_initialize();
    
    hd44780_goto(0,0);
    hd44780_write_text((uint8_t*)"   MORSE CODE");
    hd44780_goto(0,1);
    hd44780_write_text((uint8_t*)" (c) 2010 Alex");
    _delay_ms(1000);
    
    // set mode
    if(bic(MODE_SWITCH_PORT, RECEIVE_PIN))
        mode = MODE_RECEIVE;
    else if(bic(MODE_SWITCH_PORT, TRANSMIT_PIN))
        mode = MODE_TRANSMIT;
    else
        mode = MODE_TRANSMIT;
    
    forever
    {
        if(mode) // mode == MODE_RECEIVE
        {
            TCNT0 = 0; // reset timer                 
            /* timer0 prescaler = clk/256 -> 16000000Hz / 256 = 62500 Hz per increment */
            /* overflow every 256 ticks = 244.140625Hz = 4.096ms an interrupt */ 
            TCCR0 = 0x04; // set prescaler to 1/256
            TIMSK = BV(TOIE0); // enable overflow interrupt
            
            // clear buffers
            charsLen = 0;
            buffElems = 0;
            
            // used internally in interrupt handler
            counter = 0;
            oldState = OFF;
            
            delayThresholdDit = 15*2;
            delayThresholdDah = 45*2;
            delayThresholdChar = 68*2;
            delayThresholdWord = 160*2;
            
            for(uint8_t i=0; i<AVG_BUF_SIZE; i++)
            {
                ditDurations[i] = delayThresholdDit;
                dahDurations[i] = delayThresholdDah;
            }
            avgDitDuration = delayThresholdDit;
            avgDahDuration = delayThresholdDah;

            hd44780_clear_display();
            hd44780_cursor_off();
            //hd44780_blinking_off();
            hd44780_goto(0,0);
            hd44780_write_text((uint8_t*)"Active mode:");
            hd44780_goto(0,1);
            hd44780_write_text((uint8_t*)"RECEIVER");
            _delay_ms(500);
            hd44780_clear_display();        
        
            sei(); // enable interrupts
                
            forever
            {
            
                //delayThresholdDit = avgDitDuration+4;
                
                delayThresholdDah = avgDahDuration+10;
                delayThresholdDit = delayThresholdDah/2;
                delayThresholdChar = (delayThresholdDah*3)/2;
                delayThresholdWord = (delayThresholdDah*7)/2;
                            
                if(bic(BUTTONS_PORT, BUTTON_1_PIN)) // statistics
                {
                    uint8_t buf[16];
                    hd44780_goto(0,0);
                    hd44780_write_text((uint8_t*)".:");
                    itoa(avgDitDuration*4, (char*)buf, 10);
                    hd44780_write_text(buf);
                    hd44780_write_text((uint8_t*)" ");
                    itoa(delayThresholdDit*4, (char*)buf, 10);
                    hd44780_write_text(buf);
                    hd44780_write_text((uint8_t*)" ");
                    itoa(delayThresholdChar*4, (char*)buf, 10);
                    hd44780_write_text(buf);
                    hd44780_goto(0,1);
                    hd44780_write_text((uint8_t*)"-:");
                    itoa(avgDahDuration*4, (char*)buf, 10);
                    hd44780_write_text(buf);
                    hd44780_write_text((uint8_t*)" ");
                    itoa(delayThresholdDah*4, (char*)buf, 10);
                    hd44780_write_text(buf);
                    hd44780_write_text((uint8_t*)" ");
                    itoa(delayThresholdWord*4, (char*)buf, 10);
                    hd44780_write_text(buf);
                    continue;
                }

                // top line: decoded characters
                decodeBuffer();
                renderCharBuf(16);
                                
                // bottom line: detected morse code
                uint8_t i,n;
                if(buffElems > 16)
                    i = 0;
                else
                    i = 16-buffElems;
                
                hd44780_goto(0, 1);
                n=i;
                while(n--)
                    hd44780_write_data(' ');
                
                hd44780_goto(i, 1);
                
                if(buffElems > 16)
                    i = buffElems-16;
                else
                    i = 0;
                
                for(; i<buffElems; i++)
                {
                    uint8_t data;
                    switch(bufferGet(i))
                    {
                        case DIT:
                            data = '.'; break;
                        case DAH:
                            data = '-'; break;
                        case CHAR_DELAY:
                            data = '/'; break;
                        case WORD_DELAY:
                            data = '|'; break;
                        default:
                            data = '?';
                    }
                    hd44780_write_data(data);
                }
                                
                if(bic(MODE_SWITCH_PORT, TRANSMIT_PIN)) // switch mode
                {
                    mode = MODE_TRANSMIT;
                    _delay_ms(500);
                    break;
                }
            }
            
            cli(); // disable interrupts
        }
        else // mode == MODE_TRANSMIT
        {
            charsLen = 0;

            hd44780_clear_display();
            hd44780_goto(0,0);
            hd44780_write_text((uint8_t*)"Active mode:");
            hd44780_goto(0,1);
            hd44780_write_text((uint8_t*)"TRANSMITTER");
            _delay_ms(500);
            hd44780_clear_display();
            //hd44780_cursor_on();
            //hd44780_blinking_on();
            
            // debug
            //morsePlay((uint8_t*)"Hello World!!!", 14);        

            uint8_t currChar = 10; // index of current character in morseCodes table
            forever
            {
                if(bic(BUTTONS_PORT, BUTTON_1_PIN)) // next character
                {
                    currChar++;
                    if(currChar >= morseSize)
                        currChar = 0;
                }
                else if(bic(BUTTONS_PORT, BUTTON_2_PIN)) // insert into buffer
                {
                    if(charsLen < CHAR_BUFFER_SIZE)
                    {
                        chars[charsLen++] = morseCodes[currChar].ch;
                    }
                    else // error - buffer is full
                    {
                        sbi(OUTPUT_PORT, OUTPUT_PIN_BUZZER); // turn speaker on
                        _delay_ms(200);
                        cbi(OUTPUT_PORT, OUTPUT_PIN_BUZZER); // turn speaker off 
                    }
                }
                else if(bic(BUTTONS_PORT, BUTTON_3_PIN)) // delete char
                {
                    if(charsLen)
                    {
                        charsLen--;
                        _delay_ms(300);
                    }
                }
                else if(bic(BUTTONS_PORT, BUTTON_4_PIN)) // play
                {
                    morsePlay(chars, charsLen);
                    charsLen = 0;
                }
                
                // render editor
                renderCharBuf(15);
                // render current character
                hd44780_write_data(morseCodes[currChar].ch);

                // debouncing
                _delay_ms(150);
                
                if(bic(MODE_SWITCH_PORT, RECEIVE_PIN)) // switch mode
                {
                    mode = MODE_RECEIVE;
                    _delay_ms(500);
                    break;
                }
            }
        }
    }
}
