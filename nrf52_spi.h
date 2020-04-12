/*
 *
 * nrf spi
 *
 */
#ifndef nrf52_spi_H_
#define nrf52_spi_H_

typedef struct PACK_STRUCTURE nrf52_spi {
	IO_SOCKET_STRUCT_MEMBERS

	io_encoding_implementation_t const *encoding;

	NRF_SPI_Type *registers;
	nrf_io_pin_t mosi_pin;
	nrf_io_pin_t miso_pin;
	nrf_io_pin_t sclk_pin;
	nrf_io_pin_t cs_pin;
	uint32_t maximum_speed;

} nrf52_spi_t;

extern EVENT_DATA io_socket_implementation_t nrf52_spi_implementation;

#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// mplementtaion
//
//-----------------------------------------------------------------------------

/*
	eg?

//
// binary pins
//
#define SPI_ENABLE			def_nrf_io_output_pin(1,10,NRF_GPIO_ACTIVE_LEVEL_HIGH,GPIO_PIN_INACTIVE)
#define SPI_RESET				def_nrf_io_output_pin(1,11,NRF_GPIO_ACTIVE_LEVEL_HIGH,GPIO_PIN_INACTIVE)
#define SPI_SPI_ENABLE		def_nrf_io_output_pin(0,30,NRF_GPIO_ACTIVE_LEVEL_HIGH,GPIO_PIN_ACTIVE)
#define SPI_INTERRUPT		def_nrf_io_interrupt_pin (\
											1,8,\
											NRF_IO_PIN_ACTIVE_LOW,\
											NRF_GPIO_PIN_PULLUP,\
											SPI_INTERRUPT_GPIOTE_CHANNEL,\
											GPIOTE_CONFIG_POLARITY_LoToHi\
										)

*/
#define SPI_PSEL_SCK_MSK (SPI_PSEL_SCK_CONNECT_Msk | SPI_PSEL_SCK_PORT_Msk | SPI_PSEL_SCK_PIN_Msk)
#define SPI_PSEL_MOSI_MSK (SPI_PSEL_MOSI_CONNECT_Msk | SPI_PSEL_MOSI_PORT_Msk | SPI_PSEL_MOSI_PIN_Msk)
#define SPI_PSEL_MISO_MSK (SPI_PSEL_MISO_CONNECT_Msk | SPI_PSEL_MISO_PORT_Msk | SPI_PSEL_MISO_PIN_Msk)

static io_socket_t*
nrf52_spi_initialise (io_socket_t *socket,io_t *io,io_socket_constructor_t const *C) {
	nrf52_spi_t *this = (nrf52_spi_t*) socket;
	this->io = io;
	
	this->registers->ENABLE &= ~SPI_ENABLE_ENABLE_Msk;
	
	io_set_pin_to_output (this->io,this->cs_pin.io);
	
	this->registers->PSEL.MISO &= ~SPI_PSEL_MISO_MSK;
	this->registers->PSEL.MISO |= (
			(SPI_PSEL_MISO_CONNECT_Connected << SPI_PSEL_MISO_CONNECT_Pos)
		|	nrf_gpio_pin_map(this->miso_pin)
	);
	
	this->registers->PSEL.MOSI &= ~SPI_PSEL_MOSI_MSK;
	this->registers->PSEL.MOSI |= (
			(SPI_PSEL_MOSI_CONNECT_Connected << SPI_PSEL_MOSI_CONNECT_Pos)
		|	nrf_gpio_pin_map(this->mosi_pin)
	);
	
	this->registers->PSEL.SCK &= ~SPI_PSEL_SCK_MSK;
	this->registers->PSEL.SCK |= (
			(SPI_PSEL_SCK_CONNECT_Connected << SPI_PSEL_SCK_CONNECT_Pos)
		|	nrf_gpio_pin_map(this->sclk_pin)
	);
	
	this->registers->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M8;
	
	this->registers->CONFIG = (
			(SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos)
		|	(SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos)
		|	(SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos)
	);
	
	return socket;
}

static size_t
nrf52_spi_mtu (io_socket_t const *socket) {
	return 1024;
}
static bool
nrf52_spi_open (io_socket_t *socket) {
	nrf52_spi_t *this = (nrf52_spi_t*) socket;

	if ((this->registers->ENABLE & SPI_ENABLE_ENABLE_Msk) == 0) {
		this->registers->ENABLE |= (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
	}
	
	return true;
}

static void
nrf52_spi_close (io_socket_t *socket) {
}

static bool
nrf52_spi_is_closed (io_socket_t const *socket) {
	nrf52_spi_t *this = (nrf52_spi_t*) socket;
	return (this->registers->ENABLE & SPI_ENABLE_ENABLE_Msk) == 0;
}

static io_event_t*
nrf52_spi_bindr (io_socket_t *socket,io_event_t *rx) {
	return NULL;
}

static io_pipe_t*
nrf52_spi_bindt (io_socket_t *socket,io_event_t *ev) {
	return NULL;
}

static io_encoding_t*
nrf52_spi_new_message (io_socket_t *socket) {
	return NULL;
}

static bool
nrf52_spi_send_message (io_socket_t *socket,io_encoding_t *encoding) {
	return false;
}

EVENT_DATA io_socket_implementation_t nrf52_spi_implementation = {
	.specialisation_of = &io_physical_socket_implementation_base,
	.initialise = nrf52_spi_initialise,
	.free = io_socket_free_panic,
	.open = nrf52_spi_open,
	.close = nrf52_spi_close,
	.is_closed = nrf52_spi_is_closed,
	.bindr = nrf52_spi_bindr,
	.bindt = nrf52_spi_bindt,
	.new_message = nrf52_spi_new_message,
	.send_message = nrf52_spi_send_message,
	.mtu = nrf52_spi_mtu,
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
