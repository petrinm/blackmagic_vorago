/*
 * This file is part of the Black Magic Debug project.
 *
 * MIT License
 *
 * Copyright (c) 2021 Koen De Vleeschauwer
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "general.h"
#include "platform.h"
#include <assert.h>
#include "usb_serial.h"
#include "rtt.h"
#include "rtt_if.h"

/*********************************************************************
*
*       rtt terminal i/o
*
**********************************************************************
*/

/* usb uart receive buffer */
typedef struct {
	char buf[RTT_DOWN_BUF_SIZE];
	uint32_t head;
	uint32_t tail;
} rtt_buffer;

static rtt_buffer rtt_buffers[2];


/* data from host to target: number of free bytes in usb receive buffer */
inline static uint32_t recv_bytes_free(rtt_buffer* buffer)
{
	if (buffer->tail <= buffer->head)
		return RTT_DOWN_BUF_SIZE - buffer->head + buffer->tail - 1U;
	return buffer->tail - buffer->head - 1U;
}

/* data from host to target: true if not enough free buffer space and we need to close flow control */
inline static bool recv_set_nak(rtt_buffer *buffer)
{
	assert(RTT_DOWN_BUF_SIZE > 2U * CDCACM_PACKET_SIZE);
	return recv_bytes_free(buffer) < 2U * CDCACM_PACKET_SIZE;
}

/* debug_serial_receive_callback is called when usb uart has received new data for target.
   this routine has to be fast */

void rtt_serial_receive_callback(usbd_device *dev, uint8_t ep)
{
	(void)dev;
	char usb_buf[CDCACM_PACKET_SIZE];

	/* close flow control while processing packet */
	usbd_ep_nak_set(usbdev, ep, 1);

	const uint16_t len = usbd_ep_read_packet(usbdev, ep, usb_buf, CDCACM_PACKET_SIZE);

	rtt_buffer *buffer = (ep == CDCACM_UART1_ENDPOINT ? &rtt_buffers[0] : &rtt_buffers[1]);

	/* skip flag: drop packet if not enough free buffer space */
	if (rtt_flag_skip && len > recv_bytes_free(buffer)) {
		usbd_ep_nak_set(usbdev, ep, 0);
		return;
	}

	/* copy data to recv_buf */
	for (int i = 0; i < len; i++) {
		uint32_t next_recv_head = (buffer->head + 1U) % RTT_DOWN_BUF_SIZE;
		if (next_recv_head == buffer->tail)
			break; /* overflow */
		buffer->buf[buffer->head] = usb_buf[i];
		buffer->head = next_recv_head;
	}

	/* block flag: flow control closed if not enough free buffer space */
	if (!(rtt_flag_block && recv_set_nak(buffer)))
		usbd_ep_nak_set(usbdev, ep, 0);
}

/* rtt host to target: read one character */
int32_t rtt_getchar(int channel)
{
	int retval;
	rtt_buffer *buffer = (channel ? &rtt_buffers[0] : &rtt_buffers[1]);
	if (buffer->head == buffer->tail)
		return -1;
	retval = (uint8_t)buffer->buf[buffer->tail];
	buffer->tail = (buffer->tail + 1U) % RTT_DOWN_BUF_SIZE;

	/* open flow control if enough free buffer space */
	if (!recv_set_nak(buffer)) {
		uint8_t endpoint = (channel ? CDCACM_UART1_ENDPOINT : CDCACM_UART2_ENDPOINT);
		usbd_ep_nak_set(usbdev, endpoint, 0);
	}
	return retval;
}

/* rtt host to target: true if no characters available for reading */
bool rtt_nodata(int channel)
{
	rtt_buffer *buffer = (channel ? &rtt_buffers[0] : &rtt_buffers[1]);
	return buffer->head == buffer->tail;
}

/* rtt target to host: write string */
uint32_t rtt_write(int channel, const char *buf, uint32_t len)
{
	uint8_t endpoint = (channel ? CDCACM_UART1_ENDPOINT : CDCACM_UART2_ENDPOINT);
	if (len != 0 && usbdev && usb_get_config() && gdb_serial_get_dtr())
	{
		for (uint32_t p = 0; p < len; p += CDCACM_PACKET_SIZE) {
			uint32_t plen = MIN(CDCACM_PACKET_SIZE, len - p);
			while (usbd_ep_write_packet(usbdev, endpoint, buf + p, plen) <= 0)
				continue;
		}
		/* flush 64-byte packet on full-speed */
		if (CDCACM_PACKET_SIZE == 64 && (len % CDCACM_PACKET_SIZE) == 0)
			usbd_ep_write_packet(usbdev, endpoint, NULL, 0);
	}
	return len;
}
