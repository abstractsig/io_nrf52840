/*
 *
 * nrf usart with dma
 *
 */
#ifndef nrf52_usarte_H_
#define nrf52_usarte_H_

typedef struct PACK_STRUCTURE nrf52_uart {
	IO_SOCKET_STRUCT_MEMBERS
	
	io_encoding_implementation_t const *encoding;

	io_encoding_pipe_t *tx_pipe;
	io_event_t transmit_complete;

	// for single bind
	io_event_t *signal_transmit_available;
	io_event_t *signal_receive_data_available;
	
	io_byte_pipe_t *rx_pipe;
	uint8_t* rx_buffer[2];
	uint8_t* active_rx_buffer;
	uint8_t* next_rx_buffer;
	
	nrf_io_pin_t tx_pin;
	nrf_io_pin_t rx_pin;
	nrf_io_pin_t rts_pin;
	nrf_io_pin_t cts_pin;

	NRF_UARTE_Type *uart_registers;
	IRQn_Type interrupt_number;
	uint32_t baud_rate;

} nrf52_uart_t;

extern EVENT_DATA io_socket_implementation_t nrf52_uart_implementation;


#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// mplementtaion
//
//-----------------------------------------------------------------------------
static void	nrf52_uart_interrupt (void*);
static void	nrf52_uart_tx_complete (io_event_t*);

#define NRF52_UART_RX_DMA_BUFFER_LENGTH 1

static io_socket_t*
nrf52_uart_initialise (io_socket_t *socket,io_t *io,io_settings_t const *C) {
	nrf52_uart_t *this = (nrf52_uart_t*) socket;
	this->io = io;
	
	this->tx_pipe = mk_io_encoding_pipe (
		io_get_byte_memory(io),io_settings_transmit_pipe_length(C)
	);

	this->rx_pipe = mk_io_byte_pipe (
		io_get_byte_memory(io),io_settings_receive_pipe_length(C)
	);

	this->rx_buffer[0] = io_byte_memory_allocate (
		io_get_byte_memory (io),NRF52_UART_RX_DMA_BUFFER_LENGTH
	);
	this->rx_buffer[1] = io_byte_memory_allocate (
		io_get_byte_memory (io),NRF52_UART_RX_DMA_BUFFER_LENGTH
	);
	this->active_rx_buffer = this->rx_buffer[0];
	this->next_rx_buffer = this->rx_buffer[1];
	this->signal_transmit_available = NULL;
	this->signal_receive_data_available = NULL;
	
	initialise_io_event (
		&this->transmit_complete,nrf52_uart_tx_complete,this
	);

	register_io_interrupt_handler (
		io,this->interrupt_number,nrf52_uart_interrupt,this
	);
	
	return socket;
}

static void
nrf52_uart_free (io_socket_t *socket) {
	// no action required, uarts are static
}

static void
nrf_uart_start_rx (nrf52_uart_t *this) {

	if (nrf_io_pin_is_valid(this->cts_pin)) {
		io_set_pin_to_input(io_socket_io (this),this->cts_pin.io);
		while (io_read_pin(io_socket_io(this),this->cts_pin.io));
	}

	this->uart_registers->SHORTS = (
		(UARTE_SHORTS_ENDRX_STARTRX_Enabled << UARTE_SHORTS_ENDRX_STARTRX_Pos)
	);
	this->uart_registers->RXD.PTR = (uint32_t) this->active_rx_buffer;
	this->uart_registers->RXD.MAXCNT = NRF52_UART_RX_DMA_BUFFER_LENGTH;
	this->uart_registers->TASKS_STARTRX = 1;
}

static bool
nrf52_uart_open (io_socket_t *socket,io_socket_open_flag_t flag) {
	nrf52_uart_t *this = (nrf52_uart_t*) socket;

	if (this->uart_registers->ENABLE == 0) {

		if (nrf_io_pin_is_valid (this->rts_pin)) {
			//
			// and not hardware flow control
			//
			io_set_pin_to_output (io_socket_io(this),this->rts_pin.io);
			write_to_io_pin (io_socket_io(this),this->rts_pin.io,0);
		}		

		if (nrf_io_pin_is_valid (this->cts_pin)) {
			io_set_pin_to_input (io_socket_io(this),this->cts_pin.io);
		}

		this->uart_registers->BAUDRATE = this->baud_rate;
		this->uart_registers->PSEL.RXD = nrf_gpio_pin_map(this->rx_pin);
		this->uart_registers->PSEL.TXD = nrf_gpio_pin_map(this->tx_pin);
		
		// no parity, no hardware flow control
		this->uart_registers->CONFIG = 0;

		this->uart_registers->TXD.MAXCNT = 0;		
		this->uart_registers->ENABLE = 8;
		
		this->uart_registers->INTENSET = (
				(1 << UARTE_INTENSET_RXTO_Pos)
			|	(1 << UARTE_INTENSET_RXSTARTED_Pos)
		//	|	(1 << UARTE_INTENSET_RXDRDY_Pos)
			|	(1 << UARTE_INTENSET_ENDRX_Pos)
			|	(1 << UARTE_INTENSET_ENDTX_Pos)
		);
		
		this->uart_registers->EVENTS_CTS = 0;
		
		NVIC_SetPriority (this->interrupt_number,HIGH_INTERRUPT_PRIORITY);
		NVIC_ClearPendingIRQ (this->interrupt_number);
		NVIC_EnableIRQ (this->interrupt_number);

		nrf_uart_start_rx (this);
		return true;
	} else {
		// already open
		return false;
	}
}

static void
nrf52_uart_close (io_socket_t *socket) {
	nrf52_uart_t *this = (nrf52_uart_t*) socket;
	
	NVIC_DisableIRQ (this->interrupt_number);
	this->uart_registers->ENABLE = 0;

	if (this->signal_transmit_available) {
		io_dequeue_event (io_socket_io (this),this->signal_transmit_available);
	}
	if (this->signal_receive_data_available) {
		io_dequeue_event (io_socket_io (this),this->signal_receive_data_available);
	}
}

static io_encoding_t*
nrf52_uart_new_message (io_socket_t *socket) {
	nrf52_uart_t *this = (nrf52_uart_t*) socket;
	return reference_io_encoding (
		new_io_encoding (this->encoding,io_get_byte_memory(io_socket_io(this)))
	);
}

static bool
nrf_uart_output_next_buffer (nrf52_uart_t *this) {
	io_encoding_t *next;
	if (
			this->uart_registers->ENABLE 
		&& io_encoding_pipe_peek (this->tx_pipe,&next)
	) {
		const uint8_t *begin,*end;
		io_encoding_get_content (next,&begin,&end);
		this->uart_registers->TXD.MAXCNT = end - begin;
		this->uart_registers->TXD.PTR = (uint32_t) begin;
		this->uart_registers->TASKS_STARTTX = 1;
		return true;
	} else {
		return false;
	}
}

static bool
nrf52_uart_send_message_blocking (io_socket_t *socket,io_encoding_t *encoding) {
	if (is_io_binary_encoding (encoding)) {
		nrf52_uart_t *this = (nrf52_uart_t*) socket;
		if (io_encoding_pipe_put_encoding (this->tx_pipe,encoding)) {
			if (io_encoding_pipe_count_occupied_slots (this->tx_pipe) == 1) {
				nrf_uart_output_next_buffer (this);
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

void
nrf52_uart_tx_complete (io_event_t *ev) {
	nrf52_uart_t *this = ev->user_value;
	this->uart_registers->EVENTS_TXSTARTED = 0;
	
	if (io_encoding_pipe_pop_encoding (this->tx_pipe)) {
		// done
	} else {
		io_panic (io_socket_io(this),IO_PANIC_SOMETHING_BAD_HAPPENED);
	}
	
	if (
			!nrf_uart_output_next_buffer (this)
		&&	this->uart_registers->ENABLE
		&& this->signal_transmit_available
	) {
		io_enqueue_event (io_socket_io (this),this->signal_transmit_available);
	}
}

void
nrf52_uart_interrupt (void *user_value) {
	nrf52_uart_t *this = user_value;

	if (this->uart_registers->EVENTS_ERROR) {
		//io_panic(io_socket_io(this),PANIC_DEVICE_ERROR);
		this->uart_registers->EVENTS_ERROR = 0;
		this->uart_registers->EVENTS_RXTO = 0;
		this->uart_registers->EVENTS_ENDRX = 0;
		this->uart_registers->EVENTS_RXSTARTED = 0;
	}

	if (this->uart_registers->EVENTS_RXTO) {
		//
		// this is a stopped event, need a flush ...
		//
		if (this->uart_registers->RXD.AMOUNT > 0) {
			if (this->signal_receive_data_available) {
				io_enqueue_event (
					io_socket_io (this),this->signal_receive_data_available
				);
			}
		}
		this->uart_registers->EVENTS_RXTO = 0;
	}

	if (this->uart_registers->EVENTS_ENDTX) {
		io_enqueue_event (io_socket_io(this),&this->transmit_complete);
		this->uart_registers->EVENTS_ENDTX = 0;
	}

	if (this->uart_registers->EVENTS_RXDRDY) {
		// occurs once for each received byte
		this->uart_registers->EVENTS_RXDRDY = 0;
	}
	
	//
	// rx drops bytes in console if compiled with -O0
	//
	if (this->uart_registers->EVENTS_ENDRX) {
		if (this->uart_registers->RXD.AMOUNT > 0) {
			io_byte_pipe_put_bytes (
				this->rx_pipe,
				this->active_rx_buffer,
				this->uart_registers->RXD.AMOUNT
			);
			{
				// swap buffers
				uint8_t *temp = this->next_rx_buffer;
				this->next_rx_buffer = this->active_rx_buffer;
				this->active_rx_buffer = temp;
			}
			if (this->signal_receive_data_available) {
				io_enqueue_event (
					io_socket_io (this),this->signal_receive_data_available
				);
			}
		}
		this->uart_registers->EVENTS_ENDRX = 0;
	}

	if (this->uart_registers->EVENTS_RXSTARTED) {
		this->uart_registers->RXD.PTR = (uint32_t) this->next_rx_buffer;
		this->uart_registers->EVENTS_RXSTARTED = 0;
	}
}

static size_t
nrf52_uart_mtu (io_socket_t const *socket) {
	return 1024;
}

EVENT_DATA io_socket_implementation_t nrf52_uart_implementation = {
	SPECIALISE_IO_SOCKET_IMPLEMENTATION (
		&io_physical_socket_implementation
	)
	.initialise = nrf52_uart_initialise,
	.free = nrf52_uart_free,
	.open = nrf52_uart_open,
	.close = nrf52_uart_close,
	.new_message = nrf52_uart_new_message,
	.send_message = nrf52_uart_send_message_blocking,
	.mtu = nrf52_uart_mtu,
};

#endif /* IMPLEMENT_IO_CPU */
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
