/*
 *
 * nrf radio layer
 *
 */
#ifndef io_nrf52_radio_layer_H_
#define io_nrf52_radio_layer_H_

void*			get_nrf52_radio_layer (io_encoding_t*);
io_layer_t*	push_nrf52_radio_transmit_layer (io_encoding_t*);
io_layer_t*	push_nrf52_radio_receive_layer (io_encoding_t*);

#ifdef IMPLEMENT_NRF52_RADIO
//-----------------------------------------------------------------------------
//
// implementation
//
//-----------------------------------------------------------------------------
typedef struct PACK_STRUCTURE nrf52_radio_frame {
	uint8_t length;	// not including length byte
	uint8_t source_address[4];
	uint8_t inner_layer_id[4];
	uint8_t content[];
} nrf52_radio_frame_t;

typedef struct PACK_STRUCTURE nrf52_radio_layer {
	IO_LAYER_STRUCT_PROPERTIES
	uint32_t destination_address;
} nrf52_radio_layer_t;

extern EVENT_DATA io_layer_implementation_t nrf52_radio_layer_implementation;

static nrf52_radio_layer_t*
mk_nrf52_radio_layer (io_byte_memory_t *bm,io_encoding_t *packet) {
	nrf52_radio_layer_t *this = io_byte_memory_allocate (
		bm,sizeof(nrf52_radio_layer_t)
	);
	
	if (this) {
		this->implementation = &nrf52_radio_layer_implementation;
		this->destination_address = 0;
	}
	
	return this;
}

static io_layer_t*
mk_nrf52_radio_transmit_layer (io_byte_memory_t *bm,io_encoding_t *packet) {
	nrf52_radio_layer_t *this = mk_nrf52_radio_layer (bm,packet);

	if (this) {
		this->layer_offset_in_byte_stream = io_encoding_length (packet);
		io_encoding_fill (packet,0,sizeof(nrf52_radio_frame_t));
	}
	
	return (io_layer_t*) this;
}

static io_layer_t*
mk_nrf52_radio_receive_layer (io_byte_memory_t *bm,io_encoding_t *packet) {
	nrf52_radio_layer_t *this = mk_nrf52_radio_layer (bm,packet);

	if (this) {
		this->layer_offset_in_byte_stream = io_encoding_increment_decode_offest (
			packet,sizeof(nrf52_radio_frame_t)
		);
		//
		// this layer gets properties from encoding
		//
	}
	
	return (io_layer_t*) this;
}

static bool
nrf52_radio_layer_match_address (io_layer_t *layer,io_address_t address) {
	nrf52_radio_layer_t *this = (nrf52_radio_layer_t*) layer;
	io_address_t dest = def_io_u32_address(this->destination_address);
	return (
			(compare_io_addresses (dest,def_io_u32_address(NRF_BROADCAST_ADDRESS)) == 0)
		||	(compare_io_addresses (dest,address) == 0)
	);
}	

static io_address_t
nrf52_radio_layer_get_destination_address (io_layer_t *layer,io_encoding_t *encoding) {
	nrf52_radio_layer_t *this = (nrf52_radio_layer_t*) layer;
	return def_io_u32_address(this->destination_address);
}

static bool
nrf52_radio_layer_set_destination_address (
	io_layer_t *layer,io_encoding_t *message,io_address_t local
) {
	nrf52_radio_layer_t *this = (nrf52_radio_layer_t*) layer;
	this->destination_address = io_u32_address_value (local);
	return true;
}

static io_address_t
nrf52_radio_layer_get_source_address (io_layer_t *layer,io_encoding_t *encoding) {
	nrf52_radio_frame_t *packet = io_layer_get_byte_stream (layer,encoding);
	uint32_t addr = read_le_uint32 (packet->source_address);
	return def_io_u32_address(addr);
}

static bool
nrf52_radio_layer_set_source_address (
	io_layer_t *layer,io_encoding_t *message,io_address_t local
) {
	if (message) {
		io_layer_t *layer = io_encoding_get_layer (
			message,&nrf52_radio_layer_implementation
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
	nrf52_radio_frame_t *packet = io_layer_get_byte_stream (layer,encoding);
	uint32_t addr = read_le_uint32 (packet->inner_layer_id);
	return def_io_u32_address(addr);
}

static bool
nrf52_radio_layer_set_inner_address (
	io_layer_t *layer,io_encoding_t *message,io_address_t id
) {
	if (message && io_address_size(id) <= 4) {
		io_layer_t *layer = io_encoding_get_layer (
			message,&nrf52_radio_layer_implementation
		);
		if (layer) {
			nrf52_radio_frame_t *packet  = io_layer_get_byte_stream (layer,message);
			write_le_uint32 (packet->inner_layer_id,io_u32_address_value (id));
			return true;
		}
	}
	return false;
}

static void
free_nrf52_radio_layer (io_layer_t *layer,io_byte_memory_t *bm) {
	io_byte_memory_free (bm,layer);
}

static io_layer_t*
nrf52_radio_layer_push_receive_layer (io_layer_t *layer,io_encoding_t *encoding) {
	return push_nrf52_radio_receive_layer (encoding);
}

static io_address_t
nrf52_radio_layer_any_address (void) {
	return def_io_u32_address(NRF_BROADCAST_ADDRESS);
}

static io_inner_binding_t*
nrf52_radio_layer_select_inner_binding (
	io_layer_t *layer,io_encoding_t *encoding,io_socket_t* socket
) {
	nrf52_radio_frame_t *packet = io_layer_get_byte_stream (layer,encoding);

	if (packet->length >= 8) {
		io_address_t addr = io_layer_get_destination_address (layer,encoding);
		io_multiplex_socket_t* mux = (io_multiplex_socket_t*) socket;
		io_inner_binding_t *inner = io_multiplex_socket_find_inner_binding (mux,addr);

		if (inner) {
			return inner;
		} else {
			addr = io_layer_get_inner_address (layer,encoding);
			
		}
	}
	
	return NULL;
}

static bool
io_nrf52_radio_layer_load_header (io_layer_t *layer,io_encoding_t *encoding) {
	nrf52_radio_frame_t *packet = io_encoding_get_byte_stream (encoding);
	packet->length = io_encoding_length (encoding) - 1;
	return true;
}

EVENT_DATA io_layer_implementation_t nrf52_radio_layer_implementation = {
	.specialisation_of = NULL,
	.any = nrf52_radio_layer_any_address,
	.free = free_nrf52_radio_layer,
	.push_receive_layer = nrf52_radio_layer_push_receive_layer,
	.select_inner_binding = nrf52_radio_layer_select_inner_binding,
	.match_address = nrf52_radio_layer_match_address,
	.load_header = io_nrf52_radio_layer_load_header,
	.get_content = NULL,
	.get_destination_address = nrf52_radio_layer_get_destination_address,
	.set_destination_address = nrf52_radio_layer_set_destination_address,
	.get_source_address = nrf52_radio_layer_get_source_address,
	.set_source_address = nrf52_radio_layer_set_source_address,
	.get_inner_address = nrf52_radio_layer_get_inner_address,
	.set_inner_address = nrf52_radio_layer_set_inner_address,
};


void*
get_nrf52_radio_layer (io_encoding_t *encoding) {
	return io_encoding_get_layer (
		encoding,&nrf52_radio_layer_implementation
	);
}

io_layer_t*
push_nrf52_radio_receive_layer (io_encoding_t *encoding) {
	return io_encoding_push_layer (encoding,mk_nrf52_radio_receive_layer);
}

io_layer_t*
push_nrf52_radio_transmit_layer (io_encoding_t *encoding) {
	return io_encoding_push_layer (encoding,mk_nrf52_radio_transmit_layer);
}


#endif /* IMPLEMENT_NRF52_RADIO */
#endif
