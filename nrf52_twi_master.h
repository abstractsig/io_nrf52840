/*
 *
 * nrf twi
 *
 */
#ifndef nrf52_twi_bus_master_H_
#define nrf52_twi_bus_master_H_

typedef struct io_port {
	io_encoding_pipe_t *transmit_pipe;
	io_encoding_pipe_t *rx_pipe;
	io_event_t *tx_available;
	io_event_t *rx_available;
} io_port_t;

typedef struct PACK_STRUCTURE {
	io_address_t address;
	io_port_t *port;
} io_binding_t;

typedef struct PACK_STRUCTURE nrf52_twi_slave {
	IO_SOCKET_STRUCT_MEMBERS

	io_event_t *transmit_available;
	io_event_t *receive_data_available;
	
	io_socket_t *bus_master;
	
} nrf52_twi_slave_t;

extern EVENT_DATA io_socket_implementation_t nrf52_twi_slave_implementation;

typedef struct PACK_STRUCTURE nrf52_twi_bus_master {
	IO_SOCKET_STRUCT_MEMBERS

	io_encoding_implementation_t const *encoding;

	io_twi_transfer_t current_transfer;
	io_event_t transfer_complete;

	io_binding_t *slaves;
	uint32_t number_of_slaves;
	io_binding_t *round_robin_cursor;
	
	uint16_t transmit_pipe_length;
	uint16_t receive_pipe_length;
	
	io_encoding_pipe_t *tx_pipe;
	const uint8_t *next_byte,*end;
	io_encoding_pipe_t *rx_pipe;
	
	NRF_TWI_Type *registers;
	IRQn_Type interrupt_number;
	nrf_io_pin_t sda_pin;
	nrf_io_pin_t scl_pin;

	uint32_t maximum_speed;

} nrf52_twi_master_t;

extern EVENT_DATA io_socket_implementation_t nrf52_twi_master_implementation;


#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------

io_port_t*
mk_io_port (
	io_byte_memory_t *bm,
	uint16_t tx_length,
	uint16_t rx_length,
	io_event_t *tx_available,
	io_event_t *rx_available
) {
	io_port_t *this = io_byte_memory_allocate (bm,sizeof(io_port_t));

	if (this) {
		this->tx_available = tx_available;
		this->rx_available = rx_available;
		this->transmit_pipe = mk_io_encoding_pipe (bm,tx_length);
		if (this->transmit_pipe == NULL) {
			goto nope;
		}
		this->rx_pipe = mk_io_encoding_pipe (bm,rx_length);
		if (this->rx_pipe == NULL) {
			goto nope;
		}
	}
	
	return this;
	
nope:
	io_byte_memory_free(bm,this);
	return NULL;
}

void
free_io_port (io_byte_memory_t *bm,io_port_t *this) {
	free_io_encoding_pipe (this->transmit_pipe,bm);
	free_io_encoding_pipe (this->rx_pipe,bm);
	io_byte_memory_free (bm,this);
}

static void	nrf52_twi_master_interrupt (void*);
static void	nrf52_twi_master_transfer_complete (io_event_t*);

static io_socket_t*
nrf52_twi_master_initialise (io_socket_t *socket,io_t *io,io_socket_constructor_t const *C) {
	nrf52_twi_master_t *this = (nrf52_twi_master_t*) socket;

	initialise_io_socket (socket,io);

	this->encoding = C->encoding;
	this->slaves = NULL;
	this->number_of_slaves = 0;
	this->round_robin_cursor = this->slaves;
	
	register_io_interrupt_handler (
		io,this->interrupt_number,nrf52_twi_master_interrupt,this
	);

	initialise_io_event (
		&this->transfer_complete,nrf52_twi_master_transfer_complete,this
	);

	this->transmit_pipe_length = C->transmit_pipe_length;
	this->receive_pipe_length = C->receive_pipe_length;

	this->tx_pipe = mk_io_encoding_pipe (
		io_get_byte_memory(io),io_socket_constructor_transmit_pipe_length(C)
	);
	
	return socket;
}

static bool
nrf52_twi_master_open (io_socket_t *socket) {
	nrf52_twi_master_t *this = (nrf52_twi_master_t*) socket;

	if ((this->registers->ENABLE & TWI_ENABLE_ENABLE_Msk) == 0) {
	
		// set as inputs with drive strength GPIO_PIN_CNF_DRIVE_S0D1

		nrf_gpio_cfg_input (
			nrf_gpio_pin_map(this->sda_pin),nrf_gpio_pin_pull_mode(this->sda_pin)
		);
		nrf_gpio_cfg_input (
			nrf_gpio_pin_map(this->scl_pin),nrf_gpio_pin_pull_mode(this->scl_pin)
		);
		
		nrf_gpio_pin_set_drive_level (this->sda_pin);
		nrf_gpio_pin_set_drive_level (this->scl_pin);

		this->registers->PSEL.SDA = nrf_gpio_pin_map(this->sda_pin);
		this->registers->PSEL.SCL = nrf_gpio_pin_map(this->scl_pin);
		this->registers->FREQUENCY = TWI_FREQUENCY_FREQUENCY_K400;
		this->registers->SHORTS = 0;
		
		this->registers->INTENCLR = 0xffffffff;
		this->registers->INTENSET = (
				(TWI_INTENSET_TXDSENT_Enabled << TWI_INTENSET_TXDSENT_Pos)
			|	(TWI_INTENSET_RXDREADY_Enabled << TWI_INTENSET_RXDREADY_Pos)
			|	(TWI_INTENSET_ERROR_Enabled << TWI_INTENSET_ERROR_Pos)
			|	(TWI_INTENSET_STOPPED_Enabled << TWI_INTENSET_STOPPED_Pos)
		);
		
		this->next_byte = NULL;

		NVIC_SetPriority (this->interrupt_number,NORMAL_INTERRUPT_PRIORITY);
		NVIC_ClearPendingIRQ (this->interrupt_number);
		NVIC_EnableIRQ (this->interrupt_number);
		
		this->registers->ENABLE = 5;
	}
	
	return true;
}

static void
nrf52_twi_master_close (io_socket_t *socket) {
	nrf52_twi_master_t *this = (nrf52_twi_master_t*) socket;

	if (this->registers->ENABLE != 0) {
	
		this->registers->ENABLE = 0;
	}
}

static bool
nrf52_twi_master_is_closed (io_socket_t const *socket) {
	nrf52_twi_master_t *this = (nrf52_twi_master_t*) socket;
	return (this->registers->ENABLE & SPI_ENABLE_ENABLE_Msk) == 0;
}

static void
nrf52_twi_master_interrupt (void *user_value) {
	nrf52_twi_master_t *this = user_value;
	
	if (this->registers->EVENTS_TXDSENT) {
		this->registers->EVENTS_TXDSENT = 0;
		if (io_twi_transfer_tx_length(&this->current_transfer) > 0) {
			this->registers->TXD = *(this->next_byte)++;
			io_twi_transfer_tx_length(&this->current_transfer)--;
		} else if (io_twi_transfer_rx_length(&this->current_transfer) > 0) {
		
			if (io_twi_transfer_rx_length(&this->current_transfer) == 1) {
				this->registers->SHORTS = (
						(TWI_SHORTS_BB_SUSPEND_Disabled << TWI_SHORTS_BB_SUSPEND_Pos)
					||	(TWI_SHORTS_BB_STOP_Enabled << TWI_SHORTS_BB_STOP_Pos)
				);
			} else {
				this->registers->SHORTS = (
						(TWI_SHORTS_BB_SUSPEND_Enabled << TWI_SHORTS_BB_SUSPEND_Pos)
					||	(TWI_SHORTS_BB_STOP_Disabled << TWI_SHORTS_BB_STOP_Pos)
				);
			}
			
			this->registers->TASKS_STARTRX = 1;
		} else {
			this->registers->TASKS_STOP = 1;
		}
	}

	if (this->registers->EVENTS_RXDREADY) {
		this->registers->EVENTS_RXDREADY = 0;
		io_twi_transfer_tx_length(&this->current_transfer)--;
		
		// needs an rx encoding
	}

	if (this->registers->EVENTS_ERROR) {
		this->registers->EVENTS_ERROR = 0;
		
		//
		// this is interpteted as failure to communicate with slave,
		// e.g. slave is powerwered down or address is invalid
		//
	
		io_panic (io_socket_io(this),IO_PANIC_DEVICE_ERROR);
	}
	
	if (this->registers->EVENTS_STOPPED) {
		this->registers->EVENTS_STOPPED = 0;
		this->registers->SHORTS = 0;
		io_enqueue_event (io_socket_io(this),&this->transfer_complete);
	}
}

static io_encoding_t*
nrf52_twi_master_new_message (io_socket_t *socket) {
	nrf52_twi_master_t *this = (nrf52_twi_master_t*) socket;
	return reference_io_encoding (
		new_io_encoding (this->encoding,io_get_byte_memory(io_socket_io(this)))
	);
}

static bool
nrf_twi_output_next_buffer (nrf52_twi_master_t *this) {
	io_encoding_t *next;
	if (io_encoding_pipe_peek (this->tx_pipe,&next)) {
		io_twi_transfer_t const *cmd = io_encoding_get_get_ro_header (next);
		this->current_transfer = *cmd;
		this->registers->ADDRESS = io_twi_transfer_bus_address(cmd);

		io_encoding_get_content (next,&this->next_byte,&this->end);
		if (
				this->end - this->next_byte
			&& io_twi_transfer_tx_length(&this->current_transfer) > 0
		) {
			this->registers->TASKS_STARTTX = 1;
			this->registers->TXD = *(this->next_byte)++;
			io_twi_transfer_tx_length(&this->current_transfer)--;
		} else {
			// get and free ...
		}
		return true;
	} else {
		return false;
	}
}

static void
signal_transmit_available_to_next_slave (nrf52_twi_master_t *this) {
	io_binding_t *at = this->round_robin_cursor;
	
	do {
		this->round_robin_cursor ++;
		if ( (this->round_robin_cursor - this->slaves) == this->number_of_slaves) {
			this->round_robin_cursor = this->slaves;
		}
		
		io_event_t *ev = this->round_robin_cursor->port->tx_available;
		if (ev) {
			io_enqueue_event (io_socket_io (this),ev);
			break;
		}
		
	} while (this->round_robin_cursor != at);
}

static void
nrf52_twi_master_transfer_complete (io_event_t *ev) {
	nrf52_twi_master_t *this = ev->user_value;
	
	if (io_encoding_pipe_pop_encoding (this->tx_pipe)) {
		// done
	} else {
		io_panic (io_socket_io(this),IO_PANIC_SOMETHING_BAD_HAPPENED);
	}

	if (!nrf_twi_output_next_buffer (this)) {
		signal_transmit_available_to_next_slave (this);
	}
}

static bool
nrf52_twi_master_send_message (io_socket_t *socket,io_encoding_t *encoding) {
	if (is_io_twi_encoding (encoding)) {
		nrf52_twi_master_t *this = (nrf52_twi_master_t*) socket;
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
}

static size_t
nrf52_twi_master_mtu (io_socket_t const *socket) {
	return 128;
}

io_binding_t*
nrf52_twi_master_find_inner_port (nrf52_twi_master_t *this,uint8_t slave_address) {
	io_binding_t *slave = NULL;
	io_binding_t *b = this->slaves;
	io_binding_t *end = b + this->number_of_slaves;
	io_address_t sa = def_io_u8_address(slave_address);
	
	while (b < end) {
		if (compare_io_addresses (b->address,sa) == 0) {
			slave = b;
			break;
		}
		b++;
	}
	
	return slave;
}

static bool
nrf52_twi_master_bind_inner (
	io_socket_t *socket,io_address_t a,io_event_t *tx,io_event_t *rx
) {
	if (io_address_size (a) == 1) {
		nrf52_twi_master_t *this = (nrf52_twi_master_t*) socket;
		io_binding_t *slave = nrf52_twi_master_find_inner_port (this,io_u8_address_value(a));
		
		if (slave == NULL) {
			io_byte_memory_t *bm = io_get_byte_memory (io_socket_io(socket));
			io_port_t *p = mk_io_port (
				bm,
				this->transmit_pipe_length,
				this->receive_pipe_length,
				tx,
				rx
			);
			if (p != NULL) {
				io_binding_t *more = io_byte_memory_reallocate (
					bm,this->slaves,sizeof(io_binding_t) * (this->number_of_slaves + 1)
				);
				
				if (more != NULL) {
					this->round_robin_cursor = (more + (this->round_robin_cursor - this->slaves));
					this->slaves = more;
					this->slaves[this->number_of_slaves] = (io_binding_t) {a,p};
					this->number_of_slaves++;
				}
			} else {
				// out of memory
			}
		} else {
			io_port_t *p = slave->port;
			io_dequeue_event (io_socket_io (socket),p->tx_available);
			io_dequeue_event (io_socket_io (socket),p->rx_available);
			p->tx_available = tx;
			p->rx_available = rx;
			reset_io_encoding_pipe (p->transmit_pipe);
			reset_io_encoding_pipe (p->rx_pipe);
		}

		return true;
	} else {
		return false;
	}
}

EVENT_DATA io_socket_implementation_t nrf52_twi_master_implementation = {
	.specialisation_of = &io_physical_socket_implementation_base,
	.initialise = nrf52_twi_master_initialise,
	.free = io_socket_free_panic,
	.open = nrf52_twi_master_open,
	.close = nrf52_twi_master_close,
	.is_closed = nrf52_twi_master_is_closed,
	.bind_inner = nrf52_twi_master_bind_inner,
	.bind_to_outer_socket = NULL,
	.new_message = nrf52_twi_master_new_message,
	.send_message = nrf52_twi_master_send_message,
	.iterate_outer_sockets = NULL,
	.mtu = nrf52_twi_master_mtu,
};

static io_socket_t*
nrf52_twi_slave_initialise (io_socket_t *socket,io_t *io,io_socket_constructor_t const *C) {
	nrf52_twi_slave_t *this = (nrf52_twi_slave_t*) socket;

	initialise_io_socket (socket,io);
	this->bus_master = NULL;
	
	this->transmit_available = NULL;
	this->receive_data_available = NULL;
	
	return socket;
}

static bool
nrf52_twi_slave_open (io_socket_t *socket) {
	nrf52_twi_slave_t *this = (nrf52_twi_slave_t*) socket;
	if (this->bus_master != NULL) {
		return io_socket_open (this->bus_master);
	} else {
		return false;
	}
}

static void
nrf52_twi_slave_close (io_socket_t *socket) {

	// need unbind inner
	
	// io_socket_unbind_inner (this->bus_master,io_socket_address(socket),this->transmit_available,this->receive_data_available)
}

static bool
nrf52_twi_slave_is_closed (io_socket_t const *socket) {
	return false;
}

static bool
nrf52_twi_slave_bind (io_socket_t *socket,io_address_t a,io_event_t *tx,io_event_t *rx) {
	nrf52_twi_slave_t *this = (nrf52_twi_slave_t*) socket;

	this->transmit_available = tx;
	this->receive_data_available = rx;

	return io_socket_bind_to_outer_socket (socket,this->bus_master);
}

static bool
nrf52_twi_slave_bind_to_master (io_socket_t *socket,io_socket_t *outer) {
	nrf52_twi_slave_t *this = (nrf52_twi_slave_t*) socket;

	io_socket_bind_inner (
		outer,
		io_socket_address(socket),
		this->transmit_available,
		this->receive_data_available
	);
	
	this->bus_master = outer;
	
	return true;
}

static io_encoding_t*
nrf52_twi_slave_new_message (io_socket_t *socket) {
	nrf52_twi_slave_t *this = (nrf52_twi_slave_t*) socket;
	io_encoding_t *message = io_socket_new_message (this->bus_master);
	io_twi_transfer_t *cmd  = io_encoding_get_get_rw_header (message);

	io_twi_transfer_bus_address (cmd) = io_u8_address_value (this->address);
	
	return message;
}

static bool
nrf52_twi_slave_send_message (io_socket_t *socket,io_encoding_t *encoding) {
	nrf52_twi_slave_t *this = (nrf52_twi_slave_t*) socket;
	return io_socket_send_message (this->bus_master,encoding);
}

EVENT_DATA io_socket_implementation_t nrf52_twi_slave_implementation = {
	.specialisation_of = &io_physical_socket_implementation_base,
	.initialise = nrf52_twi_slave_initialise,
	.free = io_socket_free_panic,
	.open = nrf52_twi_slave_open,
	.close = nrf52_twi_slave_close,
	.is_closed = nrf52_twi_slave_is_closed,
	.bind_to_outer_socket = nrf52_twi_slave_bind_to_master,
	.bind_inner = nrf52_twi_slave_bind,
	.new_message = nrf52_twi_slave_new_message,
	.send_message = nrf52_twi_slave_send_message,
	.iterate_outer_sockets = NULL,
	.mtu = nrf52_twi_master_mtu,
};


#endif /* IMPLEMENT_IO_CPU */
#endif

