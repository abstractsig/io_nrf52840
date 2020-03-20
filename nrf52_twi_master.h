/*
 *
 * nrf twi
 *
 */
#ifndef nrf52_twi_master_H_
#define nrf52_twi_master_H_


typedef struct PACK_STRUCTURE nrf52_twi_master {
	IO_SOCKET_STRUCT_MEMBERS

	io_encoding_implementation_t const *encoding;
	io_t *io;

	io_encoding_pipe_t *tx_pipe;
	io_twi_transfer_t current_transfer;
	const uint8_t *next_byte,*end;
	io_event_t transmit_complete;
	io_event_t *signal_transmit_available;

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

static void	nrf52_twi_master_interrupt (void*);
static void	nrf52_twi_tx_complete (io_event_t*);

static io_socket_t*
nrf52_twi_master_initialise (io_socket_t *socket,io_t *io,io_socket_constructor_t const *C) {
	nrf52_twi_master_t *this = (nrf52_twi_master_t*) socket;

	this->io = io;
	this->encoding = C->encoding;
	this->signal_transmit_available = NULL;
	
	register_io_interrupt_handler (
		io,this->interrupt_number,nrf52_twi_master_interrupt,this
	);

	this->tx_pipe = mk_io_encoding_pipe (
		io_get_byte_memory(io),io_socket_constructor_transmit_pipe_length(C)
	);

	initialise_io_event (
		&this->transmit_complete,nrf52_twi_tx_complete,this
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

static io_event_t*
nrf52_twi_master_bindr (io_socket_t *socket,io_event_t *rx) {
	return NULL;
}

static io_pipe_t*
nrf52_twi_master_bindt (io_socket_t *socket,io_event_t *ev) {
	nrf52_twi_master_t *this = (nrf52_twi_master_t*) socket;
	
	if (this->signal_transmit_available) {
		io_dequeue_event (this->io,this->signal_transmit_available);
	}
	this->signal_transmit_available = ev;
	
	return NULL;
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
		io_panic (this->io,IO_PANIC_DEVICE_ERROR);
	}
	
	if (this->registers->EVENTS_STOPPED) {
		this->registers->EVENTS_STOPPED = 0;
		this->registers->SHORTS = 0;
		io_enqueue_event (this->io,&this->transmit_complete);
	}
}

static io_encoding_t*
nrf52_twi_master_new_message (io_socket_t *socket) {
	nrf52_twi_master_t *this = (nrf52_twi_master_t*) socket;
	return reference_io_encoding (
		new_io_encoding (this->encoding,io_get_byte_memory(this->io))
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
nrf52_twi_tx_complete (io_event_t *ev) {
	nrf52_twi_master_t *this = ev->user_value;
	io_encoding_t *next;
	
	if (io_encoding_pipe_get_encoding (this->tx_pipe,&next)) {
		unreference_io_encoding (next);
	} else {
		io_panic (this->io,IO_PANIC_SOMETHING_BAD_HAPPENED);
	}

	if (!nrf_twi_output_next_buffer (this)) {
		if (this->signal_transmit_available) {
			io_enqueue_event (this->io,this->signal_transmit_available);
		}
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

static io_t*
nrf52_twi_master_get_io (io_socket_t *socket) {
	nrf52_twi_master_t *this = (nrf52_twi_master_t*) socket;
	return this->io;
}

static size_t
nrf52_twi_master_mtu (io_socket_t const *socket) {
	return 128;
}

EVENT_DATA io_socket_implementation_t nrf52_twi_master_implementation = {
	.specialisation_of = &io_physical_socket_implementation_base,
	.initialise = nrf52_twi_master_initialise,
	.free = io_socket_free_panic,
	.get_io = nrf52_twi_master_get_io,
	.open = nrf52_twi_master_open,
	.close = nrf52_twi_master_close,
	.is_closed = nrf52_twi_master_is_closed,
	.bindr = nrf52_twi_master_bindr,
	.bindt = nrf52_twi_master_bindt,
	.new_message = nrf52_twi_master_new_message,
	.send_message = nrf52_twi_master_send_message,
	.iterate_inner_sockets = NULL,
	.iterate_outer_sockets = NULL,
	.mtu = nrf52_twi_master_mtu,
};


#endif /* IMPLEMENT_IO_CPU */
#endif

