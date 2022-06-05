/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

// This example runs both host and device concurrently. The USB host receive
// reports from HID device and print it out over USB Device CDC interface.

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// #define UART_DEV
// #define LIB_PICO_STDIO_UART

// #define PICO_DEFAULT_UART_TX_PIN 0
// #define PICO_DEFAULT_UART_RX_PIN 1
// #define PICO_DEFAULT_UART 1

#include "bsp/board.h"
#include "tusb.h"
#include "pico/multicore.h"

#include "usb_descriptors.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF PROTYPES
//--------------------------------------------------------------------+

// uncomment if you are using colemak layout
// #define KEYBOARD_COLEMAK

#ifdef KEYBOARD_COLEMAK
const uint8_t colemak[128] = {
  0  ,  0,  0,  0,  0,  0,  0, 22,
  9  , 23,  7,  0, 24, 17,  8, 12,
  0  , 14, 28, 51,  0, 19, 21, 10,
  15 ,  0,  0,  0, 13,  0,  0,  0,
  0  ,  0,  0,  0,  0,  0,  0,  0,
  0  ,  0,  0,  0,  0,  0,  0,  0,
  0  ,  0,  0, 18,  0,  0,  0,  0,
  0  ,  0,  0,  0,  0,  0,  0,  0,
  0  ,  0,  0,  0,  0,  0,  0,  0,
  0  ,  0,  0,  0,  0,  0,  0,  0
};
#endif

static uint8_t const keycode2ascii[128][2] =  { HID_KEYCODE_TO_ASCII };

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
static uint32_t unconsumed = 0;
static hid_mouse_report_t last_report = {0};

void led_blinking_task(void);
void hid_task(void);

#ifdef USE_MULTICORE
// core1: handle host events
void core1_main() {
  sleep_ms(10);

  // To run USB SOF interrupt in core1, init host stack for pio_usb (roothub port1)
  // on core1
  tuh_init(1);

  while (true) {
    tuh_task(); // tinyusb host task
  }
}
#endif

/*------------- MAIN -------------*/
int main(void)
{
  board_init();
  stdio_uart_init();

  printf("TinyUSB Host HID <-> Device HID Example\r\n");

#ifdef USE_MULTICORE
  tud_init(0);

  multicore_reset_core1();
  multicore_launch_core1(core1_main);
  puts("launch core1");

  while (1)
  {
    tud_task();
    led_blinking_task();
    hid_task();
  }
#else
  tusb_init();

  while (1)
  {
    tud_task(); // tinyusb device task
    tuh_task(); // tinyusb host task
    led_blinking_task();
    hid_task();
  }
#endif

  return 0;
}

//--------------------------------------------------------------------+
// Device Common Callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

//--------------------------------------------------------------------+
// Device CDC
//--------------------------------------------------------------------+

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
  (void) itf;

  char buf[64];
  uint32_t count = tud_cdc_read(buf, sizeof(buf));

  // TODO control LED on keyboard of host stack
  (void) count;
}

//--------------------------------------------------------------------+
// Device HID
//--------------------------------------------------------------------+

// Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc ..)
// tud_hid_report_complete_cb() is used to send the next report after previous one is complete
void hid_task(void)
{
  if ( tud_hid_ready() )
  {
    if ( unconsumed > 0 ) {
       //tud_hid_n_report(0, 0, &last_report, sizeof(last_report));
       // 忽略 pan
       tud_hid_mouse_report(0, last_report.buttons, last_report.x, last_report.y, last_report.wheel, 0);
       unconsumed = 0;
    }
  }

  // Poll every 10ms
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  uint32_t const btn = board_button_read();

  // Remote wakeup
  if ( tud_suspended() && btn )
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }

  /*------------- Mouse -------------*/
  if ( tud_hid_ready() )
  {
    if ( btn )
    {
      int8_t const delta = 5;

      // no button, right + down, no scroll pan
      //tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, delta, delta, 0, 0);
      tud_hid_mouse_report(0, 0x00, delta, delta, 0, 0);
    }
  }
}

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint8_t len)
{
  (void) instance;
  (void) len;
  (void) report;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) bufsize;
}

//--------------------------------------------------------------------+
// Host HID
//--------------------------------------------------------------------+

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
  (void)desc_report;
  (void)desc_len;

  // Interface protocol (hid_interface_protocol_enum_t)
  const char* protocol_str[] = { "None", "Keyboard", "Mouse" };
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  char tempbuf[256];
  int count = sprintf(tempbuf, "[%04x:%04x][%u] HID Interface%u, Protocol = %s\r\n", vid, pid, dev_addr, instance, protocol_str[itf_protocol]);

  tud_cdc_write(tempbuf, count);
  tud_cdc_write_flush();

  // Receive report from boot keyboard & mouse only
  // tuh_hid_report_received_cb() will be invoked when report is available
  if (itf_protocol == HID_ITF_PROTOCOL_KEYBOARD || itf_protocol == HID_ITF_PROTOCOL_MOUSE)
  {
    if ( !tuh_hid_receive_report(dev_addr, instance) )
    {
      tud_cdc_write_str("Error: cannot request report\r\n");
    }
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  char tempbuf[256];
  int count = sprintf(tempbuf, "[%u] HID Interface%u is unmounted\r\n", dev_addr, instance);
  tud_cdc_write(tempbuf, count);
  tud_cdc_write_flush();
}

// look up new key in previous keys
static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  for(uint8_t i=0; i<6; i++)
  {
    if (report->keycode[i] == keycode)  return true;
  }

  return false;
}


// convert hid keycode to ascii and print via usb device CDC (ignore non-printable)
static void process_kbd_report(uint8_t dev_addr, hid_keyboard_report_t const *report)
{
  (void) dev_addr;
  static hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released
  bool flush = false;

  for(uint8_t i=0; i<6; i++)
  {
    uint8_t keycode = report->keycode[i];
    if ( keycode )
    {
      if ( find_key_in_report(&prev_report, keycode) )
      {
        // exist in previous report means the current key is holding
      }else
      {
        // not existed in previous report means the current key is pressed

        // remap the key code for Colemak layout
        #ifdef KEYBOARD_COLEMAK
        uint8_t colemak_key_code = colemak[keycode];
        if (colemak_key_code != 0) keycode = colemak_key_code;
        #endif

        bool const is_shift = report->modifier & (KEYBOARD_MODIFIER_LEFTSHIFT | KEYBOARD_MODIFIER_RIGHTSHIFT);
        uint8_t ch = keycode2ascii[keycode][is_shift ? 1 : 0];

        if (ch)
        {
          if (ch == '\n') tud_cdc_write("\r", 1);
          tud_cdc_write(&ch, 1);
          flush = true;
        }
      }
    }
    // TODO example skips key released
  }

  if (flush) tud_cdc_write_flush();

  prev_report = *report;
}

// send mouse report to usb device CDC
static void process_mouse_report(uint8_t dev_addr, hid_mouse_report_t const * report)
{
  if (unconsumed > 0) {
    char tempbuf[32];
    int count = sprintf(tempbuf, "[%u] unconsumed: %u\r\n", dev_addr, unconsumed);

    tud_cdc_write(tempbuf, count);
    tud_cdc_write_flush();
  }
  
  // 记录 report，注意不是所有设备都按 Boot Protocol 兼容的格式发送 report。比如 Razer 鼠标的 pan 值是取错了的，实际上格式是：
  // Input 5 bit Usage spec.Button.1-5
  // Input 3 bit No Usage (padding)
  // Input 1*2 byte Vendor-FF00.40
  // Input 1 byte spec.GDCs.Wheel
  // Input 2 byte spec.GDCs.X -32768,32767
  // Input 2 byte spec.GDCs.Y -32768,32767
  last_report = *report;
  unconsumed++;

  // //------------- button state  -------------//
  // //uint8_t button_changed_mask = report->buttons ^ prev_report.buttons;
  // char l = report->buttons & MOUSE_BUTTON_LEFT   ? 'L' : '-';
  // char m = report->buttons & MOUSE_BUTTON_MIDDLE ? 'M' : '-';
  // char r = report->buttons & MOUSE_BUTTON_RIGHT  ? 'R' : '-';
  // char b = report->buttons & MOUSE_BUTTON_BACKWARD  ? 'B' : '-';
  // char f = report->buttons & MOUSE_BUTTON_FORWARD  ? 'F' : '-';

  // char tempbuf[32];
  // int count = sprintf(tempbuf, "[%u] %c%c%c%c%c %d %d %d\r\n", dev_addr, l, m, r, b, f, report->x, report->y, report->wheel);

  // tud_cdc_write(tempbuf, count);
  // tud_cdc_write_flush();
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
  (void) len;
  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

  switch(itf_protocol)
  {
    case HID_ITF_PROTOCOL_KEYBOARD:
      process_kbd_report(dev_addr, (hid_keyboard_report_t const*) report );
    break;

    case HID_ITF_PROTOCOL_MOUSE:
      process_mouse_report(dev_addr, (hid_mouse_report_t const*) report );
    break;

    default: break;
  }

  // continue to request to receive report
  if ( !tuh_hid_receive_report(dev_addr, instance) )
  {
    tud_cdc_write_str("Error: cannot request report\r\n");
  }
}

//--------------------------------------------------------------------+
// Blinking Task
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
