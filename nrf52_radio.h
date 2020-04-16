/*
 *
 * nrf radio
 *
 */
#ifndef nrf52_radio_H_
#define nrf52_radio_H_

#define NRF_CONNECT_RADIO_MODE					RADIO_MODE_MODE_Nrf_1Mbit
#define NRF_CONNECT_RADIO_FREQUENCY				2459000000
#define NRF_BROADCAST_ADDRESS						0x71727374

#define NRF_RADIO_MAXIMUM_PAYLOAD_LENGTH		255UL	// bytes

//
// Packet configuration:
//
// S1 size = 0 bits, S0 size = 0 bytes, payload length size = 8 bits
//
#define NRF_PACKET_CONFIGURATION_PCNF0 \
	( \
			(0) \
		|  (8 << RADIO_PCNF0_LFLEN_Pos) \
		|  (0 << RADIO_PCNF0_S0LEN_Pos) \
		|  (0 << RADIO_PCNF0_S1LEN_Pos) \
		|  (RADIO_PCNF0_S1INCL_Automatic << RADIO_PCNF0_S1INCL_Pos) \
		|  (0 << RADIO_PCNF0_CILEN_Pos) \
		|  (RADIO_PCNF0_PLEN_8bit << RADIO_PCNF0_PLEN_Pos) \
		|  (0 << RADIO_PCNF0_CRCINC_Pos) \
		|  (0 << RADIO_PCNF0_TERMLEN_Pos) \
	)

// Packet configuration:
// Bit 25: 1 Whitening enabled
// Bit 24: 1 Big endian,
// 4 byte base address length (4 address, keep prefix constant),
// 0 byte static length, max 255 byte payload .
//
#define NRF_PACKET_CONFIGURATION_PCNF1 \
	(	\
			(RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos)	\
		|	(RADIO_PCNF1_ENDIAN_Big << RADIO_PCNF1_ENDIAN_Pos)	\
		|	(4UL << RADIO_PCNF1_BALEN_Pos)	\
		|	(0UL << RADIO_PCNF1_STATLEN_Pos)	\
		|	(NRF_RADIO_MAXIMUM_PAYLOAD_LENGTH << RADIO_PCNF1_MAXLEN_Pos)	\
	)

#define NRF_PACKET_CONFIGURATION_CRCCNF \
	(	\
			(3 << RADIO_CRCCNF_LEN_Pos)	\
		|	(1 << RADIO_CRCCNF_SKIPADDR_Pos)	\
	)

#define NRF_PACKET_CONFIGURATION_CRCPOLY	0x5D6DCB
#define NRF_PACKET_CONFIGURATION_CRCINIT	1

//
// the nordic defines are soooo ugly
//
#define RADIO_STATE_DISABLED					RADIO_STATE_STATE_Disabled 
#define RADIO_STATE_RECEIVE_RAMP_UP			RADIO_STATE_STATE_RxRu
#define RADIO_STATE_RECEIVE_IDLE				RADIO_STATE_STATE_RxIdle
#define RADIO_STATE_RECEIVING					RADIO_STATE_STATE_Rx
#define RADIO_STATE_RECEIVE_DISABLE			RADIO_STATE_STATE_RxDisable
#define RADIO_STATE_TRANSMITTING_RAMP_UP	RADIO_STATE_STATE_TxRu
#define RADIO_STATE_TRANSMITTING_IDLE		RADIO_STATE_STATE_TxIdle
#define RADIO_STATE_TRANSMITTING				RADIO_STATE_STATE_Tx
#define RADIO_STATE_TRANSMITTING_DISABLE	RADIO_STATE_STATE_TxDisable


/*
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 *
 * radio states
 *
 *               power_off
 *                 |                             
 *                 v
 *               disabled
 *                 |                             
 *                 v                             
 *               receive_ramp_up <---.
 *                 |                 |
 *                 v                 |
 *         .---> receive_idle        |
 *         |       |                 |
 *         |       v                 |
 *         |     receive_listen  ---------> transmit_ramp_up
 *         |       |                 |        |   
 *         |       v                 |        v
 *         `---  receive_busy         `---- transmit_idle  <--.
 *                                            |               |
 *                                            v               |
 *                                          transmit_busy  ---'
 *
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */

struct nrf52_radio_state;
typedef struct nrf52_radio_state nrf52_radio_state_t;

struct nrf52_radio;
typedef struct nrf52_radio nrf52_radio_t;

struct nrf52_radio_state {
	nrf52_radio_state_t const* (*enter) (nrf52_radio_t*);
	nrf52_radio_state_t const* (*open) (nrf52_radio_t*);
	nrf52_radio_state_t const* (*close) (nrf52_radio_t*);
	nrf52_radio_state_t const* (*send) (nrf52_radio_t*);
	nrf52_radio_state_t const* (*end) (nrf52_radio_t*);
	nrf52_radio_state_t const* (*ready) (nrf52_radio_t*);
	nrf52_radio_state_t const* (*tx_ready) (nrf52_radio_t*);
	nrf52_radio_state_t const* (*busy) (nrf52_radio_t*);
};

struct PACK_STRUCTURE nrf52_radio {
	IO_MEDIA_SOCKET_STRUCT_MEMBERS
	
	NRF_RADIO_Type *registers;
	io_encoding_implementation_t const *encoding;
	
	nrf52_radio_state_t const *state;
	
	io_event_t tx_ready_event;
	io_event_t radio_address_event;
	io_event_t radio_end_event;
	io_event_t radio_ready_event;
	io_event_t send_event;

	io_time_t time_of_last_end_event;

};

#ifdef IMPLEMENT_NRF52_RADIO
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------
//
// inline io state implementation
//
INLINE_FUNCTION nrf52_radio_state_t const*
call_radio_state_enter (nrf52_radio_t *radio) {
	return radio->state->enter (radio);
}

static void
enter_current_state (nrf52_radio_t *this) {
	nrf52_radio_state_t const *current = this->state;
	nrf52_radio_state_t const *next = call_radio_state_enter (this);
	if (next != current) {
		this->state = next;
		enter_current_state (this);
	}
}

static void
call_state (nrf52_radio_t *this,nrf52_radio_state_t const* (*fn) (nrf52_radio_t*)) {
	nrf52_radio_state_t const *current = this->state;
	nrf52_radio_state_t const *next = fn (this);
	if (next != current) {
		this->state = next;
		enter_current_state (this);
	}
}

void
nrf52_radio_address_event (io_event_t *ev) {
	nrf52_radio_t *this = ev->user_value;
	call_state (this,this->state->busy);
}

static void
nrf52_radio_tx_ready_event (io_event_t *ev) {
	nrf52_radio_t *this = ev->user_value;
	call_state (this,this->state->tx_ready);
}

static void
nrf52_radio_send_event (io_event_t *ev) {
	nrf52_radio_t *this = ev->user_value;
	call_state (this,this->state->send);
}

//
// end event happens on completuion of tx or rx
//
static void
nrf52_radio_end_event (io_event_t *ev) {
	nrf52_radio_t *this = ev->user_value;
	call_state (this,this->state->end);
}

static void
nrf52_radio_ready_event (io_event_t *ev) {
	nrf52_radio_t *this = ev->user_value;
	call_state (this,this->state->ready);
}

static void
nrf52_radio_interrupt_handler (void *user_value) {
	nrf52_radio_t *this = user_value;
	
	if (NRF_RADIO->EVENTS_END) {
		this->time_of_last_end_event = io_get_time (this->io);
		NRF_RADIO->EVENTS_END = 0;
		io_enqueue_event (this->io,&this->radio_end_event);
	}

	if (NRF_RADIO->EVENTS_ADDRESS) {
		NRF_RADIO->EVENTS_ADDRESS = 0;
		io_enqueue_event (this->io,&this->radio_address_event);
	}
	
	if (NRF_RADIO->EVENTS_PHYEND) {
		// PHYEND is only generated in Ble_LR125Kbit, Ble_LR500Kbit and
		// Ieee802154_250Kbit modes
		NRF_RADIO->EVENTS_PHYEND = 0;
	}
	
	if (NRF_RADIO->EVENTS_READY) {
		NRF_RADIO->EVENTS_READY = 0;
		io_enqueue_event (this->io,&this->radio_ready_event);
	}

	if (NRF_RADIO->EVENTS_TXREADY) {
		NRF_RADIO->EVENTS_TXREADY = 0;
		io_enqueue_event (this->io,&this->tx_ready_event);
	}

	if (NRF_RADIO->EVENTS_RXREADY) {
		NRF_RADIO->EVENTS_RXREADY = 0;
	}

	if (NRF_RADIO->EVENTS_DISABLED) {
		NRF_RADIO->EVENTS_DISABLED = 0;
	}
}

static io_socket_t*
nrf52_radio_initialise (io_socket_t *socket,io_t *io,io_socket_constructor_t const *C) {
	nrf52_radio_t *this = (nrf52_radio_t*) socket;

	initialise_io_socket (socket,io);
	io_media_socket_initialise (
		(io_media_socket_t*) socket,C->transmit_pipe_length,C->receive_pipe_length
	);
	
	this->registers = NRF_RADIO;

	if (this->registers->STATE != RADIO_STATE_STATE_Disabled) {
		//
		// goto disable ...
		//
	}

	initialise_io_event (
		&this->radio_ready_event,nrf52_radio_ready_event,this
	);

	initialise_io_event (
		&this->radio_end_event,nrf52_radio_end_event,this
	);

	initialise_io_event (
		&this->send_event,nrf52_radio_send_event,this
	);

	initialise_io_event (
		&this->tx_ready_event,nrf52_radio_tx_ready_event,this
	);

	initialise_io_event (
		&this->radio_address_event,nrf52_radio_address_event,this
	);

	register_io_interrupt_handler (
		io,RADIO_IRQn,nrf52_radio_interrupt_handler,this
	);

	return socket;
}

static bool
nrf52_radio_open (io_socket_t *socket) {
	if (
		io_cpu_clock_is_derrived_from (
			io_get_core_clock (io_socket_io(socket)),
			&nrf52_crystal_oscillator_implementation)
	) {
	
		return true;
	} else {
		return false;
	}
}

static void
nrf52_radio_close (io_socket_t *socket) {
}

static bool
nrf52_radio_is_closed (io_socket_t const *socket) {
	nrf52_radio_t *this = (nrf52_radio_t*) socket;
	return this->registers->STATE == RADIO_STATE_STATE_Disabled;
}

static size_t
nrf52_radio_mtu (io_socket_t const *socket) {
	return 128;
}

static bool
nrf52_radio_bind_to_outer_socket (io_socket_t *socket,io_socket_t *outer) {
	return false;
}


static io_encoding_t*
nrf52_radio_new_message (io_socket_t *socket) {
	nrf52_radio_t *this = (nrf52_radio_t*) socket;
	return reference_io_encoding (
		new_io_encoding (this->encoding,io_get_byte_memory(io_socket_io(this)))
	);
}

static bool
nrf52_radio_send_message (io_socket_t *socket,io_encoding_t *encoding) {
	return false;
/*
	if (is_io_twi_encoding (encoding)) {
		nrf52_radio_t *this = (nrf52_radio_t*) socket;
		if (io_encoding_pipe_put_encoding (this->tx_pipe,encoding)) {
			if (io_encoding_pipe_count_occupied_slots (this->tx_pipe) == 1) {
				nrf_twi_output_next_buffer (this);
			}
			return true;
		} else {
			unreference_io_encoding (encoding);
			return false;
		}
	} else {
		return false;
	}
*/
}

EVENT_DATA io_socket_implementation_t nrf52_radio_implementation = {
	.specialisation_of = &io_physical_socket_implementation_base,
	.initialise = nrf52_radio_initialise,
	.free = io_socket_free_panic,
	.open = nrf52_radio_open,
	.close = nrf52_radio_close,
	.is_closed = nrf52_radio_is_closed,
	.bind_inner = io_media_socket_bind_inner,
	.bind_to_outer_socket = nrf52_radio_bind_to_outer_socket,
	.new_message = nrf52_radio_new_message,
	.send_message = nrf52_radio_send_message,
	.iterate_outer_sockets = NULL,
	.mtu = nrf52_radio_mtu,
};


#endif /* IMPLEMENT_NRF52_RADIO */
#endif
/*
------------------------------------------------------------------------------
This software is available under 2 licenses -- choose whichever you prefer.
------------------------------------------------------------------------------
ALTERNATIVE A - MIT License
Copyright (c) 2020 Gregor Bruce
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
------------------------------------------------------------------------------
ALTERNATIVE B - Public Domain (www.unlicense.org)
This is free and unencumbered software released into the public domain.
Anyone is free to copy, modify, publish, use, compile, sell, or distribute this
software, either in source code form or as a compiled binary, for any purpose,
commercial or non-commercial, and by any means.
In jurisdictions that recognize copyright laws, the author or authors of this
software dedicate any and all copyright interest in the software to the public
domain. We make this dedication for the benefit of the public at large and to
the detriment of our heirs and successors. We intend this dedication to be an
overt act of relinquishment in perpetuity of all present and future rights to
this software under copyright law.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
------------------------------------------------------------------------------
*/
