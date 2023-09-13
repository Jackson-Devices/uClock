/* Uart MIDI out
 *  
 * This example demonstrates how to send MIDI data via Uart 
 * interface on STM32 family. 
 * 
 * This example code is in the public domain.
 *
 * Requires STM32Duino board manager to be installed.
 * 
 * Define HardwareSerial using any available UART/USART. 
 * Nucleo boards have UART/USART pins that are used by the ST-LINK interface and pins D0 and D1 cannot be used (unless using solder bridging).
 * view the stm32duino PeripheralPins.c file for your specific variant (at Arduino_Core_STM32/variants/(MCU family)/(MCU variant)/PeripheralPins.c)
 * and ensure the HAL UART TX and RX pinmaps match the UART/USART used.
 * eg. if USART 4 has an RX pin at PA_1 and a TX pin at PA_0 you would use
 * Serial4(PA1, PA0);
 *
 * Tested on Nucleo-F072RB (PA10=D2 PA9=D8 on the Arduino pins)
 *
 * Note external clock source (HSE) is neccesary for accurate BPM.  external crystal is best, ST-link is ok, internal clock (HSI) will not work well.
 * STM32duino may have defined the clock as HSI for compatibilty reasons.  
 * Check the SystemClock_Config in generic_clock.c file for your variant, found at arduino_core_STM32/variants/(MCU family)/(MCU variant)/generic_clock.c
 * SystemClock_Config can be overridden, use STM32CubeMX clock configuration to generate the code.
 *
 * Code by midilab contact@midilab.co
 * Example modified by Jackson Devices contact@jacksondevices.com
 */
#include <uClock.h>

// MIDI clock, start and stop byte definitions - based on MIDI 1.0 Standards.
#define MIDI_CLOCK 0xF8
#define MIDI_START 0xFA
#define MIDI_STOP  0xFC

HardwareSerial Serial1(PA10, PA9); //USART1 RX=PA10=D2 and TX =PA9=D8 on Nucleo-F072RB

uint8_t bpm_blink_timer = 1;
void handle_bpm_led(uint32_t tick)
{
  // BPM led indicator
  if ( !(tick % (96)) || (tick == 1) ) {  // first compass step will flash longer
    bpm_blink_timer = 8;
    digitalWrite(LED_BUILTIN, HIGH);
  } else if ( !(tick % (24)) ) {   // each quarter led on
    digitalWrite(LED_BUILTIN, HIGH);
  } else if ( !(tick % bpm_blink_timer) ) { // led off
    digitalWrite(LED_BUILTIN, LOW);
    bpm_blink_timer = 1;
  }
}

// Internal clock handlers
void ClockOut96PPQN(uint32_t tick) {
  // Send MIDI_CLOCK to external gear
  Serial1.write(MIDI_CLOCK);
  handle_bpm_led(tick);
}

void onClockStart() {
    // Send MIDI_START to external gear
  Serial1.write(MIDI_START);
}

void onClockStop() {
    // Send MIDI_STOP to external gear
  Serial1.write(MIDI_STOP);
}

void setup() {
  // Initialize Serial1 communication at 31250 bits per second, the default MIDI Serial1 speed communication:
  Serial1.begin(31250);

  // An led to display BPM
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Setup our clock system
  // Inits the clock
  uClock.init();
  // Set the callback function for the clock output to send MIDI Sync message.
  uClock.setClock96PPQNOutput(ClockOut96PPQN);
  // Set the callback function for MIDI Start and Stop messages.
  uClock.setOnClockStartOutput(onClockStart);  
  uClock.setOnClockStopOutput(onClockStop);
  // Set the clock BPM to 126 BPM
  uClock.setTempo(120);
  // Starts the clock, tick-tac-tick-tac...
  uClock.start();
}

// Do it whatever to interface with Clock.stop(), Clock.start(), Clock.setTempo() and integrate your environment...
void loop() {
  
}
