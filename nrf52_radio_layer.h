/*
 *
 * nrf radio layer
 *
 */
#ifndef io_nrf52_radio_layer_H_
#define io_nrf52_radio_layer_H_


typedef struct PACK_STRUCTURE nrf52_radio_frame {
	uint8_t length;	// not including length byte
	uint8_t source_address[4];
	uint8_t inner_layer_id[4];
	uint8_t content[];
} nrf52_radio_frame_t;

typedef struct PACK_STRUCTURE nrf52_radio_layer {
	IO_LAYER_STRUCT_PROPERTIES
	uint32_t remote_address;
} nrf52_radio_layer_t;

void*			get_nrf52_radio_transmit_layer (io_encoding_t*);
io_layer_t*	push_nrf52_radio_transmit_layer (io_encoding_t*);
io_layer_t*	push_nrf52_radio_receive_layer (io_encoding_t*);

#ifdef IMPLEMENT_NRF52_RADIO
//-----------------------------------------------------------------------------
//
// implementtaion
//
//-----------------------------------------------------------------------------
EVENT_DATA io_layer_implementation_t nrf52_radio_layer_implementation = {
	SPECIALISE_VIRTUAL_IO_LAYER_IMPLEMENTATION(NULL)
};

extern EVENT_DATA io_layer_implementation_t nrf52_radio_layer_transmit_implementation;
extern EVENT_DATA io_layer_implementation_t nrf52_radio_layer_receive_implementation;

void*
get_nrf52_radio_transmit_layer (io_encoding_t *encoding) {
	return io_encoding_get_layer (
		encoding,&nrf52_radio_layer_transmit_implementation
	);
}

io_layer_t*
push_nrf52_radio_receive_layer (io_encoding_t *encoding) {
	return io_encoding_push_layer (
		encoding,&nrf52_radio_layer_receive_implementation
	);
}

io_layer_t*
push_nrf52_radio_transmit_layer (io_encoding_t *encoding) {
	return io_encoding_push_layer (
		encoding,&nrf52_radio_layer_transmit_implementation
	);
}

static bool
nrf52_radio_layer_match_address (io_layer_t *layer,io_address_t address) {
	nrf52_radio_layer_t *this = (nrf52_radio_layer_t*) layer;
	io_address_t dest = def_io_u32_address(this->remote_address);
	return (
			(compare_io_addresses (dest,def_io_u32_address(NRF_BROADCAST_ADDRESS)) == 0)
		||	(compare_io_addresses (dest,address) == 0)
	);
}	

/*
static io_address_t
nrf52_radio_layer_get_layer_address (io_layer_t *layer,io_encoding_t *encoding) {
	nrf52_radio_layer_t *this = (nrf52_radio_layer_t*) layer;
	return def_io_u32_address(this->remote_address);
}

static bool
nrf52_radio_layer_set_layer_address (
	io_layer_t *layer,io_encoding_t *message,io_address_t local
) {
	nrf52_radio_layer_t *this = (nrf52_radio_layer_t*) layer;
	this->remote_address = io_u32_address_value (local);
	return true;
}
*/

static io_address_t
nrf52_radio_layer_get_remote_address (io_layer_t *layer,io_encoding_t *encoding) {
	nrf52_radio_layer_t *this = (nrf52_radio_layer_t*) layer;
	return def_io_u32_address(this->remote_address);
}

static bool
nrf52_radio_layer_set_remote_address (
	io_layer_t *layer,io_encoding_t *message,io_address_t local
) {
	nrf52_radio_layer_t *this = (nrf52_radio_layer_t*) layer;
	this->remote_address = io_u32_address_value (local);
	return true;
}

static io_address_t
nrf52_radio_layer_get_source_address (io_layer_t *layer,io_encoding_t *encoding) {
	nrf52_radio_frame_t *packet = io_encoding_get_byte_stream (encoding);
	uint32_t addr = read_le_uint32 (packet->source_address);
	return def_io_u32_address(addr);
}

static bool
nrf52_radio_layer_set_source_address (
	io_layer_t *layer,io_encoding_t *message,io_address_t local
) {
	if (message) {
		io_layer_t *layer = io_encoding_get_layer (
			message,&nrf52_radio_layer_transmit_implementation
		);
		if (layer) {
			nrf52_radio_frame_t *packet  = io_layer_get_byte_stream (layer,message);
			write_le_uint32 (packet->source_address,io_u32_address_value (local));
			return true;
		}
	}
	return false;
}

static io_address_t
nrf52_radio_layer_get_inner_address (io_layer_t *layer,io_encoding_t *encoding) {
	nrf52_radio_frame_t *packet = io_encoding_get_byte_stream (encoding);
	uint32_t addr = read_le_uint32 (packet->inner_layer_id);
	return def_io_u32_address(addr);
}

static bool
nrf52_radio_layer_set_inner_address (
	io_layer_t *layer,io_encoding_t *message,io_address_t id
) {
	if (message && io_address_size(id) <= 4) {
		io_layer_t *layer = io_encoding_get_layer (
			message,&nrf52_radio_layer_transmit_implementation
		);
		if (layer) {
			nrf52_radio_frame_t *packet  = io_layer_get_byte_stream (layer,message);
			write_le_uint32 (packet->inner_layer_id,io_u32_address_value (id));
			return true;
		}
	}
	return false;
}

static io_layer_t*
mk_nrf52_radio_layer_transmit (io_packet_encoding_t *packet) {
	nrf52_radio_layer_t *this = io_byte_memory_allocate (
		packet->bm,sizeof(nrf52_radio_layer_t)
	);
	if (this) {
		this->implementation = &nrf52_radio_layer_transmit_implementation;
		this->layer_offset_in_byte_stream = io_encoding_length ((io_encoding_t*) packet);
		this->remote_address = 0;
		// allocate layer's header space in byte stream
		io_encoding_fill ((io_encoding_t*) packet,0,sizeof(nrf52_radio_frame_t));
	}
	
	return (io_layer_t*) this;
}

static void
free_nrf52_radio_layer (io_layer_t *layer,io_byte_memory_t *bm) {
	io_byte_memory_free (bm,layer);
}

static io_layer_t*
nrf52_radio_layer_swap_tx (io_layer_t *layer,io_encoding_t *encoding) {
	return io_encoding_push_layer (
		encoding,&nrf52_radio_layer_receive_implementation
	);
}

static io_address_t
nrf52_radio_layer_any_address (void) {
	return def_io_u32_address(NRF_BROADCAST_ADDRESS);
}

EVENT_DATA io_layer_implementation_t nrf52_radio_layer_transmit_implementation = {
	.specialisation_of = &nrf52_radio_layer_implementation,
	.any = nrf52_radio_layer_any_address,
	.make = mk_nrf52_radio_layer_transmit,
	.free = free_nrf52_radio_layer,
	.swap = nrf52_radio_layer_swap_tx,
	.decode = NULL,
	.match_address = nrf52_radio_layer_match_address,
	.get_remote_address = nrf52_radio_layer_get_remote_address,
	.set_remote_address = nrf52_radio_layer_set_remote_address,
	.get_local_address = nrf52_radio_layer_get_source_address,
	.set_local_address = nrf52_radio_layer_set_source_address,
	.get_inner_address = nrf52_radio_layer_get_inner_address,
	.set_inner_address = nrf52_radio_layer_set_inner_address,
};

static io_layer_t*
mk_nrf52_radio_layer_receive (io_packet_encoding_t *packet) {
	nrf52_radio_layer_t *this = io_byte_memory_allocate (
		packet->bm,sizeof(nrf52_radio_layer_t)
	);
	if (this) {
		this->implementation = &nrf52_radio_layer_receive_implementation;
		this->layer_offset_in_byte_stream = io_encoding_length ((io_encoding_t*) packet);
		this->remote_address = 0;
		// allow space for max length packet
		io_encoding_fill ((io_encoding_t*) packet,0,NRF_RADIO_MAXIMUM_PAYLOAD_LENGTH);
	}
	
	return (io_layer_t*) this;
}

static io_inner_port_t*
nrf52_radio_layer_decode (
	io_layer_t *layer,io_encoding_t *encoding,io_multiplex_socket_t* socket
) {
	nrf52_radio_frame_t *packet = io_layer_get_byte_stream (layer,encoding);

	if (packet->length >= 8) {
		io_address_t addr = io_layer_get_remote_address (layer,encoding);
		io_inner_port_binding_t *inner = io_multiplex_socket_find_inner_binding (socket,addr);

		if (inner) {
			return inner->port;
		} else {
			addr = io_layer_get_inner_address (layer,encoding);
			
		}
	}
	
	return NULL;
}

EVENT_DATA io_layer_implementation_t nrf52_radio_layer_receive_implementation = {
	.specialisation_of = &nrf52_radio_layer_implementation,
	.any = nrf52_radio_layer_any_address,
	.make = mk_nrf52_radio_layer_receive,
	.free = free_nrf52_radio_layer,
	.swap = NULL,
	.decode = nrf52_radio_layer_decode,
	.match_address = nrf52_radio_layer_match_address,
	.get_remote_address = nrf52_radio_layer_get_source_address,
	.set_remote_address = nrf52_radio_layer_set_source_address,
	.get_local_address = nrf52_radio_layer_get_remote_address,
	.set_local_address = nrf52_radio_layer_set_remote_address,
	.get_inner_address = nrf52_radio_layer_get_inner_address,
	.set_inner_address = nrf52_radio_layer_set_inner_address,
};

#endif /* IMPLEMENT_NRF52_RADIO */
#endif
