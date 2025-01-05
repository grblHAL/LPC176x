/*

  usb_serial.c - low level functions for transmitting bytes via the USB virtual serial port

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef __usb_serial_h__
#define __usb_serial_h__

#include "driver.h"

#if USB_SERIAL_CDC

#include <string.h>

#include "chip.h"
#include "app_usbd_cfg.h"
#include "cdc_vcom.h"

#include "usb_serial.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"

extern const  USBD_HW_API_T hw_api;
extern const  USBD_CORE_API_T core_api;
extern const  USBD_CDC_API_T cdc_api;
static const  USBD_API_T g_usbApi = {
    &hw_api,
    &core_api,
    0,
    0,
    0,
    &cdc_api,
    0,
    0x02221101,
};

const USBD_API_T *g_pUsbApi = &g_usbApi;

static USBD_HANDLE_T g_hUsb;
static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

/* Initialize pin and clocks for USB0/USB1 port */
static void usb_pin_clk_init(void)
{
    /* enable USB PLL and clocks */
    Chip_USB_Init();

    /* enable USB 1 port on the board */

    /* set a bias to this pin to enable the host system to detect and begin enumeration */
    Chip_IOCON_PinMux(LPC_IOCON, 2, 9, IOCON_MODE_INACT, IOCON_FUNC1);  /* USB_CONNECT */

    Chip_IOCON_PinMux(LPC_IOCON, 0, 29, IOCON_MODE_INACT, IOCON_FUNC1); /* P0.29 D1+, P0.30 D1- */
    Chip_IOCON_PinMux(LPC_IOCON, 0, 30, IOCON_MODE_INACT, IOCON_FUNC1);

    LPC_USB->USBClkCtrl = 0x12;                /* Dev, AHB clock enable */
    while ((LPC_USB->USBClkSt & 0x12) != 0x12);
}

/**
 * @brief   Handle interrupt from USB0
 * @return  Nothing
 */
void USB_IRQHandler(void)
{
    USBD_API->hw->ISR(g_hUsb);
}

/* Find the address of interface descriptor for given class type. */
USB_INTERFACE_DESCRIPTOR *find_IntfDesc(const uint8_t *pDesc, uint32_t intfClass)
{
    USB_COMMON_DESCRIPTOR *pD;
    USB_INTERFACE_DESCRIPTOR *pIntfDesc = 0;
    uint32_t next_desc_adr;

    pD = (USB_COMMON_DESCRIPTOR *) pDesc;
    next_desc_adr = (uint32_t) pDesc;

    while (pD->bLength) {
        /* is it interface descriptor */
        if (pD->bDescriptorType == USB_INTERFACE_DESCRIPTOR_TYPE) {

            pIntfDesc = (USB_INTERFACE_DESCRIPTOR *) pD;
            /* did we find the right interface descriptor */
            if (pIntfDesc->bInterfaceClass == intfClass) {
                break;
            }
        }
        pIntfDesc = 0;
        next_desc_adr = (uint32_t) pD + pD->bLength;
        pD = (USB_COMMON_DESCRIPTOR *) next_desc_adr;
    }

    return pIntfDesc;
}


#include "grbl/grbl.h"

static stream_rx_buffer_t rxbuf = {0};
static stream_block_tx_buffer_t txbuf = {0};

volatile usb_linestate_t usb_linestate = {0};

static bool is_connected (void)
{
    return usb_linestate.pin.dtr && hal.get_elapsed_ticks() - usb_linestate.timestamp >= 15;
}

//
// Returns number of free characters in the input buffer
//
static uint16_t usbRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;

    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the input buffer
//
static void usbRxFlush (void)
{
    rxbuf.tail = rxbuf.head;
}

//
// Flushes and adds a CAN character to the input buffer
//
static void usbRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = BUFNEXT(rxbuf.head, rxbuf);
}

//
// Writes a single character to the USB output stream, blocks if buffer full
//
static bool usbPutC (const char c)
{
    static uint8_t buf[1];

    *buf = c;

    while(vcom_write(buf, 1) != 1) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    return true;
}

static inline bool _usb_write()
{
    size_t length = txbuf.length;

    txbuf.s = txbuf.data;

    if(vcom_connected()) while(length) {
        txbuf.length = vcom_write((uint8_t *)txbuf.s, length > 64 ? 64 : length);
        txbuf.s += txbuf.length;
        length -= txbuf.length;
        if(!hal.stream_blocking_callback())
            return false;
    }

    txbuf.length = 0;
    txbuf.s = txbuf.data;

    return true;
}

//
// Writes a null terminated string to the USB output stream, blocks if buffer full
// Buffers string up to EOL (LF) before transmitting
//
static void usbWriteS (const char *s)
{
    size_t length = strlen(s);

    if(length == 0)
        return;

    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
        if(!_usb_write())
            return;
    }

    while(length > txbuf.max_length) {
        txbuf.length = txbuf.max_length;
        memcpy(txbuf.s, s, txbuf.length);
        if(!_usb_write())
            return;
        length -= txbuf.max_length;
        s += txbuf.max_length;
    }

    if(length) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        if(s[length - 1] == ASCII_LF)
            _usb_write();
    }
}

//
// Writes a number of characters from string to the USB output stream, blocks if buffer full
//
static void usbWrite (const char *s, uint16_t length)
{
    if(length == 0)
        return;

    if(txbuf.length && (txbuf.length + length) > txbuf.max_length) {
        if(!_usb_write())
            return;
    }

    while(length > txbuf.max_length) {
        txbuf.length = txbuf.max_length;
        memcpy(txbuf.s, s, txbuf.length);
        if(!_usb_write())
            return;
        length -= txbuf.max_length;
        s += txbuf.max_length;
    }

    if(length) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        _usb_write();
    }
}

//
// usbGetC - returns -1 if no data available
//
static int16_t usbGetC (void)
{
    uint_fast16_t tail = rxbuf.tail;    // Get buffer pointer

    if(tail == rxbuf.head)
        return -1; // no data available

    char data = rxbuf.data[tail];       // Get next character
    rxbuf.tail = BUFNEXT(tail, rxbuf);  // and update pointer

    return (int16_t)data;
}

static bool usbSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

static bool usbEnqueueRtCommand (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr usbSetRtHandler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *usbInit (void)
{
    static const io_stream_t stream = {
        .type = StreamType_Serial,
        .state.is_usb = On,
        .state.linestate_event = On,
        .is_connected = is_connected,
        .read = usbGetC,
        .write = usbWriteS,
        .write_char = usbPutC,
        .write_n = usbWrite,
        .enqueue_rt_command = usbEnqueueRtCommand,
        .get_rx_buffer_free = usbRxFree,
        .reset_read_buffer = usbRxFlush,
        .cancel_read_buffer = usbRxCancel,
        .suspend_read = usbSuspendInput,
        .set_enqueue_rt_handler = usbSetRtHandler
    };

    USBD_API_INIT_PARAM_T usb_param;
    USB_CORE_DESCS_T desc;
//  ErrorCode_t ret = LPC_OK;

    /* enable clocks and pinmux */
    usb_pin_clk_init();

    /* initialize call back structures */
    memset((void *) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
    usb_param.usb_reg_base = LPC_USB_BASE + 0x200;
    usb_param.max_num_ep = 3;
    usb_param.mem_base = USB_STACK_MEM_BASE;
    usb_param.mem_size = USB_STACK_MEM_SIZE;

    /* Set the USB descriptors */
    desc.device_desc = (uint8_t *) &USB_DeviceDescriptor[0];
    desc.string_desc = (uint8_t *) &USB_StringDescriptor[0];
    /* Note, to pass USBCV test full-speed only devices should have both
       descriptor arrays point to same location and device_qualifier set to 0.
     */
    desc.high_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
    desc.full_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
    desc.device_qualifier = 0;

    /* USB Initialization */
    if (USBD_API->hw->Init(&g_hUsb, &desc, &usb_param) == LPC_OK) {
        /* Init VCOM interface */
        if (vcom_init(g_hUsb, &desc, &usb_param) == LPC_OK) {
            /*  enable USB interrupts */
            NVIC_SetPriority(USB_IRQn, 1);
            NVIC_EnableIRQ(USB_IRQn);
            /* now connect */
            USBD_API->hw->Connect(g_hUsb, 1);
        }
    }

    txbuf.s = txbuf.data;
    txbuf.max_length = sizeof(txbuf.data);

    return &stream;
}

void usbBufferInput (uint8_t *data, uint32_t length)
{
    while(length--) {
        if(!enqueue_realtime_command(*data)) {                  // Check and strip realtime commands...
            uint16_t next_head = BUFNEXT(rxbuf.head, rxbuf);    // Get and increment buffer pointer
            if(next_head == rxbuf.tail)                         // If buffer full
                rxbuf.overflow = 1;                             // flag overflow
            else {
                rxbuf.data[rxbuf.head] = *data;                 // if not add data to buffer
                rxbuf.head = next_head;                         // and update pointer
            }
        }
        data++;                                                 // next
    }
}

#endif

#endif // __usb_serial_h__
