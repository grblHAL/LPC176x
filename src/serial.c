/*

  serial.c - LPC17xx low level functions for transmitting bytes via the serial port

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef __serial_h__
#define __serial_h__

#include <string.h>

#include "driver.h"
#include "serial.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"

#include "chip.h"

static stream_rx_buffer_t rxbuf = {0};
static stream_tx_buffer_t txbuf = {0};
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

/* Pin muxing configuration */
static const PINMUX_GRP_T uart_pinmux[] = {
    {0, 2, IOCON_MODE_INACT | IOCON_FUNC1}, /* TXD0 */
    {0, 3, IOCON_MODE_INACT | IOCON_FUNC1}  /* RXD0 */
};

#ifdef RTS_PORT
static volatile uint8_t rts_state = 0;
#endif

#ifdef ENABLE_XONXOFF
static volatile uint8_t flow_ctrl = XON_SENT; // Flow control state variable
#endif

static io_stream_properties_t serial[] = {
    {
      .type = StreamType_Serial,
      .instance = 0,
      .flags.claimable = On,
      .flags.claimed = Off,
      .flags.connected = On,
      .flags.can_set_baud = On,
      .claim = serialInit
    }
};

void serialRegisterStreams (void)
{
    static io_stream_details_t streams = {
        .n_streams = sizeof(serial) / sizeof(io_stream_properties_t),
        .streams = serial,
    };

    stream_register_streams(&streams);
}

/*
//
// Returns number of characters in serial output buffer
//
static uint16_t serialTxCount (void)
{
  uint16_t tail = txbuf.tail;
  return BUFCOUNT(txbuf.head, tail, TX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
static uint16_t serialRxCount (void)
{
  uint16_t tail = rxbuf.tail, head = rxbuf.head;
  return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}
*/

//
// Returns number of free characters in serial input buffer
//
static uint16_t serialRxFree (void)
{
    unsigned int tail = rxbuf.tail, head = rxbuf.head;

    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
static void serialRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
    SERIAL_MODULE->FCR |= UART_FCR_RX_RS; // Flush FIFO too

#ifdef RTS_PORT
    BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = 0;
#endif
}

//
// Flushes and adds a CAN character to the serial input buffer
//
static void serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
#ifdef RTS_PORT
    BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = 0;
#endif
}

//
// Attempt to send a character bypassing buffering
//
static inline bool serialPutCNonBlocking (const char c)
{
    bool ok = false;

    if((ok = !(SERIAL_MODULE->IER & UART_IER_THREINT) && (Chip_UART_ReadLineStatus(SERIAL_MODULE) & UART_LSR_THRE)))
        SERIAL_MODULE->THR = c;

    return ok;
}

//
// Writes a character to the serial output stream
//
static bool serialPutC (const char c)
{
    if(txbuf.head != txbuf.tail || !serialPutCNonBlocking(c)) { // Try to send character without buffering...

        uint16_t next_head = BUFNEXT(txbuf.head, txbuf);        // .. if not, get pointer to next free slot in buffer

        while(txbuf.tail == next_head) {                        // While TX buffer full
            if(!hal.stream_blocking_callback())                 // check if blocking for space,
                return false;                                   // exit if not (leaves TX buffer in an inconsistent state)
        }

        txbuf.data[txbuf.head] = c;                             // Add data to buffer
        txbuf.head = next_head;                                 // and update head pointer
        Chip_UART_IntEnable(SERIAL_MODULE, UART_IER_THREINT);   // Enable TX interrupts
    }

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
static void serialWriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//
static void serialWrite (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serialPutC(*ptr++);
}

//
// serialGetC - returns -1 if no data available
//
static int16_t serialGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;    // Get buffer pointer

    if(tail == rxbuf.head)
        return -1; // no data available

    char data = rxbuf.data[tail];       // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);  // and update pointer

#ifdef RTS_PORT
    if (rts_state && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM) // Clear RTS if below LWM
        BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = rts_state = 0;
#endif

    return (int16_t)data;
}

static bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool serialSetBaudRate (uint32_t baud_rate)
{
    Chip_UART_SetBaud(SERIAL_MODULE, baud_rate);

    rxbuf.tail = rxbuf.head;
    txbuf.tail = txbuf.head;

    return true;
}

static bool serialEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr serialSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *serialInit (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .connected = true,
        .read = serialGetC,
        .write = serialWriteS,
        .write_char = serialPutC,
        .write_n = serialWrite,
        .write_all = serialWriteS,
        .enqueue_rt_command = serialEnqueueRtCommand,
        .get_rx_buffer_free = serialRxFree,
        .reset_read_buffer = serialRxFlush,
        .cancel_read_buffer = serialRxCancel,
        .suspend_read = serialSuspendInput,
        .set_baud_rate = serialSetBaudRate,
        .set_enqueue_rt_handler = serialSetRtHandler
    };

    if(serial[0].flags.claimed)
        return NULL;

    serial[0].flags.claimed = On;

    Chip_IOCON_SetPinMuxing(LPC_IOCON, uart_pinmux, sizeof(uart_pinmux) / sizeof(PINMUX_GRP_T));

    /* Setup UART for 115.2K8N1 */
    Chip_UART_Init(SERIAL_MODULE);
    Chip_UART_SetBaud(SERIAL_MODULE, baud_rate);
    Chip_UART_ConfigData(SERIAL_MODULE, (UART_LCR_WLEN8|UART_LCR_SBS_1BIT));
    Chip_UART_SetupFIFOS(SERIAL_MODULE, (UART_FCR_FIFO_EN|UART_FCR_TRG_LEV2));
    Chip_UART_TXEnable(SERIAL_MODULE);

    /* Reset and enable FIFOs, FIFO trigger level 3 (14 chars) */
    Chip_UART_SetupFIFOS(SERIAL_MODULE, (UART_FCR_FIFO_EN|UART_FCR_RX_RS|UART_FCR_TX_RS|UART_FCR_TRG_LEV3));

    /* Enable receive data and line status interrupt */
    Chip_UART_IntEnable(SERIAL_MODULE, UART_IER_RBRINT);
//  Chip_UART_IntEnable(SERIAL_MODULE, (UART_IER_RBRINT|UART_IER_RLSINT));

    /* preemption = 3, sub-priority = 1 */
    NVIC_SetPriority(SERIAL_MODULE_INT, 3);
    NVIC_EnableIRQ(SERIAL_MODULE_INT);

    static const periph_pin_t tx = {
        .function = Output_TX,
        .group = PinGroup_UART,
        .port = LPC_GPIO0,
        .pin = 2,
        .mode = { .mask = PINMODE_OUTPUT }
    };

    static const periph_pin_t rx = {
        .function = Input_RX,
        .group = PinGroup_UART,
        .port = LPC_GPIO0,
        .pin = 3,
        .mode = { .mask = PINMODE_NONE }
    };

    hal.periph_port.register_pin(&rx);
    hal.periph_port.register_pin(&tx);

#ifdef RTS_PORT
    RTS_PORT->DIR |= RTS_BIT;
    BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = 0;
#endif

    return &stream;
}

//
void SERIAL_IRQHandler (void)
{
    uint32_t bptr = SERIAL_MODULE->IIR;

    switch(bptr & 0x07) {
/*
        case UART_IIR_INTID_RLS:
            bptr = SERIAL_MODULE->LSR;
            break;
*/
        case UART_IIR_INTID_THRE:;
            uint_fast16_t tail = txbuf.tail;                            // Temp tail position (to avoid volatile overhead)
            SERIAL_MODULE->SCR = UART_TX_FIFO_SIZE;                     // Use UART scratch pad register as
            while((--SERIAL_MODULE->SCR) && tail != txbuf.head) {       // counter variable for filling the transmit FIFO
                SERIAL_MODULE->THR = txbuf.data[tail];                  // Send a byte from the buffer
                tail = BUFNEXT(tail, txbuf);                            // and increment pointer
            }
            if ((txbuf.tail = tail) == txbuf.head)                      // Turn off TX interrupt
                Chip_UART_IntDisable(SERIAL_MODULE, UART_IER_THREINT);  // when buffer empty
            break;

        case UART_IIR_INTID_RDA:
        case UART_IIR_INTID_CTI:;
            while(SERIAL_MODULE->LSR & UART_LSR_RDR) {
                uint32_t data = SERIAL_MODULE->RBR;
                if(!enqueue_realtime_command(data)) { // Read character received

                    uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf);    // Get and increment buffer pointer

                    if(next_head == rxbuf.tail)                         // If buffer full
                        rxbuf.overflow = 1;                             // flag overflow
                    else {
                        rxbuf.data[rxbuf.head] = data;                  // Add data to buffer
                        rxbuf.head = next_head;                         // and update pointer
                      #ifdef RTS_PORT
                        if (!rts_state && BUFCOUNT(rxbuf.head, rxbuf.tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM) // Set RTS if at or above HWM
                            BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = rts_state = 1;
                      #endif
                    }
                }
            }
            break;
    }
}

#endif // __serial_h__
