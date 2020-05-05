/*
 *
 * nrf52 radio
 *
 */
#ifndef nrf52_radio_H_
#define nrf52_radio_H_

#define NRF_CONNECT_RADIO_MODE					RADIO_MODE_MODE_Nrf_1Mbit
#define NRF_CONNECT_RADIO_FREQUENCY				2459000000
#define NRF_BROADCAST_ADDRESS						0x71727374
#define NRF_RADIO_MAXIMUM_PAYLOAD_LENGTH		255UL	// bytes

#include <sockets/io_dlc_socket.h>
#include <nrf52_radio_layer.h>

#define DEBUG_RADIO 1

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
	IO_MULTIPLEX_SOCKET_STRUCT_MEMBERS
	
	NRF_RADIO_Type *registers;
	IRQn_Type interrupt_number;
		
	nrf52_radio_state_t const *state;
	
	io_inner_port_binding_t *current_transmit_binding;
	io_encoding_t *current_receive_packet;
	
	io_event_t tx_ready_event;
	io_event_t radio_address_event;
	io_event_t radio_end_event;
	io_event_t radio_ready_event;
	io_event_t send_event;

	io_time_t time_of_last_end_event;

	uint32_t frequency;
	uint32_t mode;
	uint32_t tx_power;
};

io_encoding_t* nrf52_radio_new_transmit_message (io_socket_t*);
size_t nrf52_radio_mtu (io_socket_t const*);

extern EVENT_DATA io_socket_implementation_t nrf52_radio_socket_implementation;
extern EVENT_DATA io_encoding_implementation_t nrf52_radio_encoding_implementation;
 
INLINE_FUNCTION io_encoding_t*
mk_nrf52_radio_encoding (io_byte_memory_t *bm) {
	return nrf52_radio_encoding_implementation.make_encoding(bm);
}

INLINE_FUNCTION bool
is_nrf52_radio_encoding (io_encoding_t const *encoding) {
	return io_is_encoding_of_type (
		encoding,&nrf52_radio_encoding_implementation
	);
}

uint32_t		generate_nrf52_radio_address (io_t*);

#ifdef IMPLEMENT_NRF52_RADIO
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------


uint32_t
generate_nrf52_radio_address (io_t *io) {
	uint32_t addr = 0;
	do {
		addr = io_get_random_u32 (io);
	} while (addr == 0 && addr == NRF_BROADCAST_ADDRESS);
	
	return addr;
}

//
// inline io state implementation
//
static void nrf52_radio_address_event (io_event_t*);
static void nrf52_radio_tx_ready_event (io_event_t*);
static void nrf52_radio_send_event (io_event_t*);
static void nrf52_radio_end_event (io_event_t*);
static void nrf52_radio_ready_event (io_event_t*);
static void nrf52_radio_interrupt_handler (void*);
static void nrf52_radio_enter_current_state (nrf52_radio_t*);

static EVENT_DATA nrf52_radio_state_t nrf52_radio_power_off;

static io_socket_t*
nrf52_radio_initialise (io_socket_t *socket,io_t *io,io_settings_t const *C) {
	nrf52_radio_t *this = (nrf52_radio_t*) socket;

	initialise_io_multiplex_socket (socket,io,C);

	if (is_invalid_io_address (io_socket_address(socket))) {
		io_socket_address(socket) = def_io_u32_address (io_uid (io)->words[0]);
	}
	
	this->mode = NRF_CONNECT_RADIO_MODE;
	this->frequency = NRF_CONNECT_RADIO_FREQUENCY;
	this->tx_power = 8;
	this->current_transmit_binding = NULL;
	this->current_receive_packet = NULL;
	
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
		io,this->interrupt_number,nrf52_radio_interrupt_handler,this
	);

	this->state = &nrf52_radio_power_off;
	nrf52_radio_enter_current_state (this);

	return socket;
}

INLINE_FUNCTION nrf52_radio_state_t const*
call_radio_state_enter (nrf52_radio_t *radio) {
	return radio->state->enter (radio);
}

static void
nrf52_radio_enter_current_state (nrf52_radio_t *this) {
	nrf52_radio_state_t const *current = this->state;
	nrf52_radio_state_t const *next = call_radio_state_enter (this);
	if (next != current) {
		this->state = next;
		 nrf52_radio_enter_current_state (this);
	}
}

static void
call_state (nrf52_radio_t *this,nrf52_radio_state_t const* (*fn) (nrf52_radio_t*)) {
	nrf52_radio_state_t const *current = this->state;
	nrf52_radio_state_t const *next = fn (this);
	if (next != current) {
		this->state = next;
		 nrf52_radio_enter_current_state (this);
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
// end event happens on completion of tx or rx
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
	NRF_RADIO_Type *radio = this->registers;
	
	if (radio->EVENTS_END) {
		this->time_of_last_end_event = io_get_time (this->io);
		radio->EVENTS_END = 0;
		io_enqueue_event (this->io,&this->radio_end_event);
	}

	if (radio->EVENTS_ADDRESS) {
		radio->EVENTS_ADDRESS = 0;
		io_enqueue_event (this->io,&this->radio_address_event);
	}
	
	if (radio->EVENTS_PHYEND) {
		// PHYEND is only generated in Ble_LR125Kbit, Ble_LR500Kbit and
		// Ieee802154_250Kbit modes
		radio->EVENTS_PHYEND = 0;
	}
	
	if (radio->EVENTS_READY) {
		radio->EVENTS_READY = 0;
		io_enqueue_event (this->io,&this->radio_ready_event);
	}

	if (radio->EVENTS_TXREADY) {
		radio->EVENTS_TXREADY = 0;
		io_enqueue_event (this->io,&this->tx_ready_event);
	}

	if (radio->EVENTS_RXREADY) {
		radio->EVENTS_RXREADY = 0;
	}

	if (radio->EVENTS_DISABLED) {
		radio->EVENTS_DISABLED = 0;
	}
}


//
// Packet configuration:
//
// S1 size = 0 bits, S0 size = 0 bytes, frame length bit-size = 8 bits
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
// 0 byte static length, max 255 byte frame .
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
 *
 * nrf52_radio_power_on --
 *
 *-----------------------------------------------------------------------------
 */
static void
nrf52_radio_power_on (nrf52_radio_t *this) {
	NRF_RADIO_Type *radio	= this->registers;
	
	radio->POWER = 1;

	radio->SHORTS          = 0;
	radio->EVENTS_DISABLED = 0;
	radio->TASKS_DISABLE   = 1;
	while (radio->EVENTS_DISABLED == 0);
	radio->EVENTS_DISABLED = 0;
	
	radio->PCNF0 = NRF_PACKET_CONFIGURATION_PCNF0;
	radio->PCNF1 = NRF_PACKET_CONFIGURATION_PCNF1;
	radio->CRCCNF = NRF_PACKET_CONFIGURATION_CRCCNF;
	radio->CRCPOLY = NRF_PACKET_CONFIGURATION_CRCPOLY;
	radio->CRCINIT = NRF_PACKET_CONFIGURATION_CRCINIT;
	
	radio->BASE0 = NRF_BROADCAST_ADDRESS;
	radio->BASE1 = io_u32_address_value(io_socket_address(this));
	radio->PREFIX0 = 0;
	radio->PREFIX1 = 0;

	radio->SHORTS = 0;
	
	radio->INTENSET = (
			(1 << RADIO_INTENSET_END_Pos)
		|	(1 << RADIO_INTENSET_ADDRESS_Pos)
		|	(1 << RADIO_INTENSET_READY_Pos)
		|	(1 << RADIO_INTENSET_TXREADY_Pos)
		|	(1 << RADIO_INTENSET_RXREADY_Pos)
		|	(1 << RADIO_INTENSET_DISABLED_Pos)
	);
	
//	radio->MODECNF0 = (RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos);
	radio->MODECNF0 = (
			(RADIO_MODECNF0_RU_Default << RADIO_MODECNF0_RU_Pos)
		|	(RADIO_MODECNF0_DTX_Center << RADIO_MODECNF0_DTX_Pos)
	);
	
	radio->TXADDRESS = 0;
	radio->RXADDRESSES = (
		RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos
	);
}


static void
nrf52_radio_open_event (io_event_t *ev) {
	nrf52_radio_t *this = ev->user_value;
	call_state (this,this->state->open);
	io_byte_memory_free (io_get_byte_memory (io_socket_io (this)),ev);
}

static bool
nrf52_radio_open (io_socket_t *socket) {
	if (
		io_cpu_clock_is_derrived_from (
			io_get_core_clock (io_socket_io(socket)),
			&nrf52_crystal_oscillator_implementation)
	) {
		io_event_t *ev = io_byte_memory_allocate (
			io_get_byte_memory (io_socket_io (socket)),sizeof(io_event_t)
		);

		initialise_io_event (ev,nrf52_radio_open_event,socket);
		io_enqueue_event (io_socket_io (socket),ev);
	
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

size_t
nrf52_radio_mtu (io_socket_t const *socket) {
	return NRF_RADIO_MAXIMUM_PAYLOAD_LENGTH;
}

static bool
nrf52_radio_bind_to_outer_socket (io_socket_t *socket,io_socket_t *outer) {
	return false;
}

io_encoding_t*
nrf52_radio_new_transmit_message (io_socket_t *socket) {
	io_encoding_t *message = mk_nrf52_radio_encoding (
		io_get_byte_memory(io_socket_io (socket))
	);
	
	if (message != NULL) {
		io_layer_t *layer = push_nrf52_radio_transmit_layer (message);
		if (layer != NULL) {
			io_layer_set_local_address (layer,message,io_socket_address(socket));
			io_layer_set_inner_address (layer,message,IO_NULL_LAYER_ID);
			reference_io_encoding (message);
		} else {
			io_panic (io_socket_io(socket),IO_PANIC_OUT_OF_MEMORY);
		}
	}
	
	return message;
}

static io_encoding_t*
nrf52_radio_new_receive_message (io_socket_t *socket) {
	io_encoding_t *message = mk_nrf52_radio_encoding (
		io_get_byte_memory(io_socket_io (socket))
	);
	
	if (message != NULL) {
		io_layer_t *layer = push_nrf52_radio_receive_layer (message);
		if (layer != NULL) {
			io_layer_set_local_address (
				layer,message,io_socket_address (socket)
			);
			reference_io_encoding (message);
		} else {
		}
	}
	
	return message;
}

static bool
nrf52_radio_send_message (io_socket_t *socket,io_encoding_t *encoding) {
	if (is_nrf52_radio_encoding (encoding)) {
		io_layer_t *base = io_encoding_get_outermost_layer (encoding);
		if (base) {
			io_inner_port_binding_t *inner = io_multiplex_socket_find_inner_binding (
				(io_multiplex_socket_t *) socket,
				io_layer_get_remote_address (base,encoding)
			);
			if (inner) {
				nrf52_radio_frame_t *packet = io_encoding_get_byte_stream (encoding);
				packet->length = io_encoding_length (encoding) - 1;
				if (io_encoding_pipe_put_encoding (	inner->port->transmit_pipe,encoding)) {
					nrf52_radio_t *this = (nrf52_radio_t*) socket;
					io_enqueue_event (io_socket_io (socket),&this->send_event);
					unreference_io_encoding (encoding);
					return true;
				}
			}
		}
	}
	unreference_io_encoding (encoding);
	return false;
}

EVENT_DATA io_socket_implementation_t nrf52_radio_socket_implementation = {
	.specialisation_of = &io_multiplex_socket_implementation,
	.initialise = nrf52_radio_initialise,
	.reference = io_virtual_socket_increment_reference,
	.free = io_virtual_socket_free,
	.open = nrf52_radio_open,
	.close = nrf52_radio_close,
	.is_closed = nrf52_radio_is_closed,
	.bind_inner = io_multiplex_socket_bind_inner,
	.bind_to_outer_socket = nrf52_radio_bind_to_outer_socket,
	.new_message = nrf52_radio_new_transmit_message,
	.send_message = nrf52_radio_send_message,
	.get_receive_pipe = NULL,
	.iterate_inner_sockets = NULL,
	.iterate_outer_sockets = NULL,
	.mtu = nrf52_radio_mtu,
};

static io_encoding_t* 
nrf52_radio_encoding_new (io_byte_memory_t *bm) {
	io_packet_encoding_t *this = io_byte_memory_allocate (
		bm,sizeof(io_packet_encoding_t)
	);

	if (this != NULL) {
		this->implementation = &nrf52_radio_encoding_implementation;
		this->bm = bm;
		this = initialise_io_packet_encoding ((io_packet_encoding_t*) this);
	}

	return (io_encoding_t*) this;
};

static int32_t
nrf52_radio_encoding_limit (void) {
	return NRF_RADIO_MAXIMUM_PAYLOAD_LENGTH;
}

EVENT_DATA io_encoding_implementation_t nrf52_radio_encoding_implementation = {
	.specialisation_of = &io_packet_encoding_implementation,
	.decode_to_io_value = io_binary_encoding_decode_to_io_value,
	.make_encoding = nrf52_radio_encoding_new,
	.free = io_packet_encoding_free,
	.get_io = io_binary_encoding_get_io,
	.grow = io_binary_encoding_grow,
	.grow_increment = default_io_encoding_grow_increment,
	.fill = io_binary_encoding_fill_bytes,
	.append_byte = io_binary_encoding_append_byte,
	.append_bytes = io_binary_encoding_append_bytes,
	.pop_last_byte = io_binary_encoding_pop_last_byte,
	.print = io_binary_encoding_print,
	.reset = io_binary_encoding_reset,
	.layer = &io_packet_layer_api,
	.get_byte_stream = io_binary_encoding_get_byte_stream,
	.get_content = io_binary_encoding_get_content,
	.length = io_binary_encoding_length,
	.limit = nrf52_radio_encoding_limit,
};

static nrf52_radio_state_t const*
nrf52_radio_state_ignore_close (nrf52_radio_t *this) {
	return this->state;
}

static nrf52_radio_state_t const*
nrf52_radio_state_ignore_send (nrf52_radio_t *this) {
	return this->state;
}

static nrf52_radio_state_t const*
nrf52_radio_state_ignore_tx_ready (nrf52_radio_t *this) {
	return this->state;
}

static nrf52_radio_state_t const*
nrf52_radio_state_unexpected_event (nrf52_radio_t *this) {
	io_panic (io_socket_io (this),IO_PANIC_SOMETHING_BAD_HAPPENED);
	return this->state;
}

static void
nrf52_radio_set_mode (nrf52_radio_t *this) {
	this->registers->MODE = (
		(this->mode & RADIO_MODE_MODE_Msk) << RADIO_MODE_MODE_Pos
	);
}

static uint32_t
convert_frequency_in_hz_to_setting (nrf52_radio_t *this) {
	if (this->registers->FREQUENCY & RADIO_FREQUENCY_MAP_Msk) {
		return (this->frequency - 2360000000)/1000000;
	} else {
		return (this->frequency - 2400000000)/1000000;
	}
}

static void
nrf52_radio_set_frequency (nrf52_radio_t *this) {
	uint32_t frequency = convert_frequency_in_hz_to_setting (this);
	this->registers->FREQUENCY = (
			(RADIO_FREQUENCY_MAP_Default << RADIO_FREQUENCY_MAP_Pos)
		|	(frequency << RADIO_FREQUENCY_FREQUENCY_Pos)
	);
}

static void
nrf52_radio_set_tx_power (nrf52_radio_t *this) {
	this->registers->TXPOWER = (
		(this->tx_power << RADIO_TXPOWER_TXPOWER_Pos)
	);
}

/*
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 *
 * radio states
 *
 *               nrf52_radio_power_off
 *                 |                             
 *                 v
 *               nrf52_radio_disabled
 *                 |                             
 *                 v                             
 *               nrf52_radio_receive_ramp_up <----.
 *                 |                              |
 *                 v                              |
 *         .---> nrf52_radio_receive_idle         |
 *         |       |                              |
 *         |       v                              |
 *         |     nrf52_radio_receive_listen -----------> nrf52_radio_transmit_ramp_up
 *         |       |                              |        |   
 *         |       v                              |        v
 *         `---  nrf52_radio_receive_busy          `---- transmit_idle  <--.
 *                                                         |               |
 *                                                         v               |
 *                                                       transmit_busy  ---'
 *
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
static EVENT_DATA nrf52_radio_state_t nrf52_radio_power_off;
static EVENT_DATA nrf52_radio_state_t nrf52_radio_disabled;
static EVENT_DATA nrf52_radio_state_t nrf52_radio_receive_ramp_up;
static EVENT_DATA nrf52_radio_state_t nrf52_radio_receive_idle;
static EVENT_DATA nrf52_radio_state_t nrf52_radio_receive_listen;
static EVENT_DATA nrf52_radio_state_t nrf52_radio_receive_busy;
static EVENT_DATA nrf52_radio_state_t nrf52_radio_transmit_ramp_up;
static EVENT_DATA nrf52_radio_state_t nrf52_radio_transmit_idle;
static EVENT_DATA nrf52_radio_state_t nrf52_radio_transmit_busy;

/*
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 *
 * nrf52_radio_power_off
 *
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
static nrf52_radio_state_t const*
nrf52_radio_power_off_state_enter (nrf52_radio_t *this) {

	NVIC_DisableIRQ(this->interrupt_number);
	
	io_dequeue_event (this->io,&this->radio_end_event);
	io_dequeue_event (this->io,&this->radio_address_event);
	io_dequeue_event (this->io,&this->radio_ready_event);
	
	this->registers->POWER = 0;

	return this->state;
}

static nrf52_radio_state_t const*
nrf52_radio_power_off_state_open (nrf52_radio_t *this) {
	nrf52_radio_power_on (this);
	return &nrf52_radio_disabled;
}

static EVENT_DATA nrf52_radio_state_t nrf52_radio_power_off = {
	.enter = nrf52_radio_power_off_state_enter,
	.open = nrf52_radio_power_off_state_open,
	.close = nrf52_radio_state_ignore_close,
	.send = nrf52_radio_state_ignore_send,
	.end = nrf52_radio_state_unexpected_event,
	.ready = nrf52_radio_state_unexpected_event,
	.tx_ready = nrf52_radio_state_ignore_tx_ready,
	.busy = nrf52_radio_state_unexpected_event,
};

/*
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 *
 * nrf52_radio_disabled
 *
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
static nrf52_radio_state_t const*
nrf52_radio_disabled_state_enter (nrf52_radio_t *this) {

	nrf52_radio_set_mode (this);
	nrf52_radio_set_frequency (this);
	nrf52_radio_set_tx_power (this);

	NVIC_ClearPendingIRQ (this->interrupt_number);
	NVIC_EnableIRQ(this->interrupt_number);
	
	return &nrf52_radio_receive_ramp_up;
}

static EVENT_DATA nrf52_radio_state_t nrf52_radio_disabled = {
	.enter = nrf52_radio_disabled_state_enter,
	.open = nrf52_radio_state_unexpected_event,
	.close = nrf52_radio_state_ignore_close,
	.send = nrf52_radio_state_ignore_send,
	.end = nrf52_radio_state_unexpected_event,
	.ready = nrf52_radio_state_unexpected_event,
	.tx_ready = nrf52_radio_state_ignore_tx_ready,
	.busy = nrf52_radio_state_unexpected_event,
};

/*
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 *
 * nrf52_radio_receive_ramp_up
 *
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
static nrf52_radio_state_t const*
nrf52_radio_receive_ramp_up_state_enter (nrf52_radio_t *this) {
	//
	// listen on my address and broadcast address
	//
	this->registers->BASE0 = NRF_BROADCAST_ADDRESS;
	this->registers->RXADDRESSES = (
			(RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos)
		|	(RADIO_RXADDRESSES_ADDR1_Enabled << RADIO_RXADDRESSES_ADDR1_Pos)
	);

	this->registers->EVENTS_CRCOK = 0;
	this->registers->SHORTS = 0;
	this->registers->TASKS_RXEN = 1;

	return this->state;
}

static nrf52_radio_state_t const*
nrf52_radio_receive_ramp_up_ready_event (nrf52_radio_t *this) {
	return &nrf52_radio_receive_idle;
}

static EVENT_DATA nrf52_radio_state_t nrf52_radio_receive_ramp_up = {
	.enter = nrf52_radio_receive_ramp_up_state_enter,
	.open = nrf52_radio_state_unexpected_event,
	.close = nrf52_radio_state_ignore_close,
	.send = nrf52_radio_state_ignore_send,
	.end = nrf52_radio_state_unexpected_event,
	.ready = nrf52_radio_receive_ramp_up_ready_event,
	.tx_ready = nrf52_radio_state_ignore_tx_ready,
	.busy = nrf52_radio_state_unexpected_event,
};

/*
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 *
 * nrf52_radio_receive_idle
 *
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
static nrf52_radio_state_t const*
nrf52_radio_receive_idle_state_enter (nrf52_radio_t *this) {

	#if defined(DEBUG_RADIO) && DEBUG_RADIO
	io_printf (
		io_socket_io (this),"%-*s%-*sreceive idle\n",
		DBP_FIELD1,"radio",
		DBP_FIELD2,"state"
	);
	#endif

	// do we have any packets to send?
	io_inner_port_binding_t *next = io_multiplex_socket_get_next_transmit_binding (
		(io_multiplex_socket_t*) this
	);
	
	if (next) {
		return &nrf52_radio_transmit_ramp_up;
	} else {
		if (this->current_receive_packet == NULL) {		
			this->current_receive_packet = nrf52_radio_new_receive_message (
				(io_socket_t*) this
			);
		}
		
		if (this->current_receive_packet) {

			this->registers->PACKETPTR = (uint32_t) io_encoding_get_byte_stream (this->current_receive_packet);
			this->registers->TASKS_START = 1;

		} else {
			//io_panic (io_socket_io (this),IO_PANIC_OUT_OF_MEMORY);
		}
		
		return &nrf52_radio_receive_listen;
	}
}

static EVENT_DATA nrf52_radio_state_t nrf52_radio_receive_idle = {
	.enter = nrf52_radio_receive_idle_state_enter,
	.open = nrf52_radio_state_unexpected_event,
	.close = nrf52_radio_state_ignore_close,
	.send = nrf52_radio_state_ignore_send,
	.end = nrf52_radio_state_unexpected_event,
	.ready = nrf52_radio_state_unexpected_event,
	.tx_ready = nrf52_radio_state_ignore_tx_ready,
	.busy = nrf52_radio_state_unexpected_event,
};

/*
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 *
 * nrf52_radio_receive_listen
 *
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
static nrf52_radio_state_t const*
nrf52_radio_receive_listen_state_enter (nrf52_radio_t *this) {

	#if defined(DEBUG_RADIO) && DEBUG_RADIO
	io_printf (
		io_socket_io (this),"%-*s%-*sreceive listen\n",
		DBP_FIELD1,"radio",
		DBP_FIELD2,"state"
	);
	#endif
	
	return this->state;
}

static nrf52_radio_state_t const*
nrf52_radio_receive_listen_state_busy_event (nrf52_radio_t *this) {
	return &nrf52_radio_receive_busy;
}

static EVENT_DATA nrf52_radio_state_t nrf52_radio_receive_listen = {
	.enter = nrf52_radio_receive_listen_state_enter,
	.open = nrf52_radio_state_unexpected_event,
	.close = nrf52_radio_state_ignore_close,
	.send = nrf52_radio_state_ignore_send,
	.end = nrf52_radio_state_unexpected_event,
	.ready = nrf52_radio_state_unexpected_event,
	.tx_ready = nrf52_radio_state_ignore_tx_ready,
	.busy = nrf52_radio_receive_listen_state_busy_event,
};

/*
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 *
 * nrf52_radio_receive_busy
 *
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
static nrf52_radio_state_t const*
nrf52_radio_receive_busy_state_enter (nrf52_radio_t *this) {

	#if defined(DEBUG_RADIO) && DEBUG_RADIO
	io_printf (
		io_socket_io (this),"%-*s%-*sreceive busy\n",
		DBP_FIELD1,"radio",
		DBP_FIELD2,"state"
	);
	#endif
	
	return this->state;
}

static nrf52_radio_state_t const*
nrf52_radio_receive_busy_state_end_event (nrf52_radio_t *this) {
	if (this->registers->CRCSTATUS == 1) {
		nrf52_radio_frame_t *packet = io_encoding_get_byte_stream (
			this->current_receive_packet
		);

		uint8_t const *payload = packet->content;//(uint8_t const*) (packet + 1);
		uint32_t from = read_le_uint32 (payload + 1);
		
		#if 1 && defined(DEBUG_RADIO)
		io_printf (
			io_socket_io (this),"%-*s%-*sreceive packet %u %u from 0x%08x\n",
			DBP_FIELD1,"radio",
			DBP_FIELD2,"state",
			io_encoding_length (this->current_receive_packet),
			this->registers->RXMATCH,
			from
		);
		#endif

		io_inner_port_binding_t *binding = io_multiplex_socket_find_inner_binding (
			(io_multiplex_socket_t*) this,def_io_u32_address (from)
		);

		if (binding == NULL) {
			//
			// new inner socket
			//
		}
		
/*		
		if (io_diplex_stream_receive ((io_diplex_stream_t*) this,packet)) {
			// slot took ownership of encoder
			this->rx_encoder = NULL;
		} else {
			io_encoding_reset (this->current_receive_packet);
		}
*/
		
	} else {
		io_encoding_reset (this->current_receive_packet);
	}
	
	return &nrf52_radio_receive_idle;
}

static EVENT_DATA nrf52_radio_state_t nrf52_radio_receive_busy = {
	.enter = nrf52_radio_receive_busy_state_enter,
	.open = nrf52_radio_state_unexpected_event,
	.close = nrf52_radio_state_ignore_close,
	.send = nrf52_radio_state_ignore_send,
	.end = nrf52_radio_receive_busy_state_end_event,
	.ready = nrf52_radio_state_unexpected_event,
	.tx_ready = nrf52_radio_state_ignore_tx_ready,
	.busy = nrf52_radio_state_unexpected_event,
};

/*
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 *
 * nrf52_radio_transmit_ramp_up
 *
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
static nrf52_radio_state_t const*
nrf52_radio_transmit_ramp_up_state_enter (nrf52_radio_t *this) {

/*
	// may use short to bypass tx idle
	
	NRF_RADIO->SHORTS = (
			RADIO_SHORTS_TXREADY_START_Msk 
		|	RADIO_SHORTS_READY_START_Msk 
   );

*/
	this->registers->TASKS_TXEN = 1;

	return this->state;
}

static nrf52_radio_state_t const*
ready_event_in_transmit_ramp_up (nrf52_radio_t *this) {
	return this->state;
}

static nrf52_radio_state_t const*
nrf52_radio_state_tx_ready_in_transmit_ramp_up (nrf52_radio_t *this) {
	return &nrf52_radio_transmit_idle;
}

static EVENT_DATA nrf52_radio_state_t nrf52_radio_transmit_ramp_up = {
	.enter = nrf52_radio_transmit_ramp_up_state_enter,
	.open = nrf52_radio_state_unexpected_event,
	.close = nrf52_radio_state_ignore_close,
	.send = nrf52_radio_state_ignore_send,
	.end = nrf52_radio_state_unexpected_event,
	.ready = ready_event_in_transmit_ramp_up,
	.tx_ready = nrf52_radio_state_tx_ready_in_transmit_ramp_up,
	.busy = nrf52_radio_state_unexpected_event,
};

/*
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 *
 * nrf52_radio_transmit_idle
 *
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
static nrf52_radio_state_t const*
nrf52_radio_transmit_idle_state_enter (nrf52_radio_t *this) {

	io_inner_port_binding_t *next = io_multiplex_socket_get_next_transmit_binding (
		(io_multiplex_socket_t*) this
	);
	
	if (next != NULL) {
		io_encoding_t *message;

		if (io_encoding_pipe_peek (this->current_transmit_binding->port->transmit_pipe,&message)) {
			io_layer_t *layer = io_encoding_get_layer (
				message,&nrf52_radio_layer_transmit_implementation
			);
			
			if (layer) {
				nrf52_radio_frame_t *packet = io_encoding_get_byte_stream(message);
				packet->length = io_encoding_length (message) - 1;
				
				//this->registers->BASE0 = layer->remote_address;
				this->registers->BASE0 = io_u32_address_value (
					io_layer_get_remote_address(layer,message)
				);
				this->registers->TXADDRESS = 0; // logical 0
				this->registers->PACKETPTR = (uint32_t)  io_encoding_get_byte_stream(message);
				this->registers->TASKS_START = 1;
			}
		
		} else {
			// panic
		}
		
		return this->state;
	} else {
		io_multiplex_socket_round_robin_signal_transmit_available (
			(io_multiplex_socket_t*) this
		);
		return &nrf52_radio_receive_ramp_up;
	}
}

static nrf52_radio_state_t const*
nrf52_radio_state_transmit_idle_busy_event (nrf52_radio_t *this) {
	return &nrf52_radio_transmit_busy;
}

static EVENT_DATA nrf52_radio_state_t nrf52_radio_transmit_idle = {
	.enter = nrf52_radio_transmit_idle_state_enter,
	.open = nrf52_radio_state_unexpected_event,
	.close = nrf52_radio_state_ignore_close,
	.send = nrf52_radio_state_ignore_send,
	.end = nrf52_radio_state_unexpected_event,
	.ready = nrf52_radio_state_unexpected_event,
	.tx_ready = nrf52_radio_state_unexpected_event,
	.busy = nrf52_radio_state_transmit_idle_busy_event,
};

/*
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 *
 * nrf52_radio_transmit_busy
 *
 *-----------------------------------------------------------------------------
 *-----------------------------------------------------------------------------
 */
static nrf52_radio_state_t const*
nrf52_radio_transmit_busy_state_enter (nrf52_radio_t *this) {

	return this->state;
}

static nrf52_radio_state_t const*
nrf52_radio_transmit_busy_state_end_event (nrf52_radio_t *this) {

	io_encoding_pipe_pop_encoding (this->current_transmit_binding->port->transmit_pipe);
	this->current_transmit_binding = NULL;
/*
	io_encoding_t *next;
	
	if (io_encoding_pipe_peek (this->tx_pipe,&next)) {
		#ifdef DEBUG_RADIO
		io_printf (
			this->io,
			"radio sent %u bytes, to=%u\n",
			io_encoder_get_length(next),io_encoder_get_destination(next)
		);
		#endif

		free_io_encoder (next);
	} else {
		io_panic (io_socket_io (this),IO_PANIC_SOMETHING_BAD_HAPPENED);
	}
*/

	return &nrf52_radio_transmit_idle;
}


static EVENT_DATA nrf52_radio_state_t nrf52_radio_transmit_busy = {
	.enter = nrf52_radio_transmit_busy_state_enter,
	.open = nrf52_radio_state_unexpected_event,
	.close = nrf52_radio_state_ignore_close,
	.send = nrf52_radio_state_ignore_send,
	.end = nrf52_radio_state_unexpected_event,
	.ready = nrf52_radio_state_unexpected_event,
	.tx_ready = nrf52_radio_state_unexpected_event,
	.busy = nrf52_radio_transmit_busy_state_end_event,
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
