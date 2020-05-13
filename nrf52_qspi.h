/*
 *
 * nrf spi
 *
 */
#ifndef nrf52_qspi_H_
#define nrf52_qspi_H_

#define SPI_FLASH_RDSR				0x05
#define SPI_FLASH_WREN				0x06
#define SPI_FLASH_WRDI				0x04
#define SPI_FLASH_CE					0x60
#define SPI_FLASH_RDID				0x9f

#define SPI_FLASH_SR_WIP			0x01
#define SPI_FLASH_SR_WEL			0x02

#define SPI_FLASH_MACRONIX			0xc2
#define SPI_FLASH_MACRONIX_MX25	0x20

typedef union {
	uint32_t reg;
	struct {
		uint32_t manufacturer:8;
		uint32_t type:8;
		uint32_t capacity:8;
		uint32_t :8;
	} device;
} jedec_chip_identification_t;

typedef struct PACK_STRUCTURE nrf52_qspi {
	IO_SOCKET_STRUCT_MEMBERS

	io_encoding_implementation_t const *encoding;
	NRF_QSPI_Type *qspi_registers;
	IRQn_Type interrupt_number;
	uint32_t maximum_speed;

	jedec_chip_identification_t expected_chip;
	uint32_t number_of_physical_sectors;
	uint32_t sector_size;
	uint32_t page_size;

	nrf_io_pin_t io0_pin;
	nrf_io_pin_t io1_pin;
	nrf_io_pin_t io2_pin;
	nrf_io_pin_t io3_pin;
	nrf_io_pin_t sck_pin;
	nrf_io_pin_t cs_pin;

} nrf52_qspi_t;

#define nrf52_qspi_number_of_sectors(sm)	(sm)->number_of_physical_sectors
#define nrf52_qspi_sector_size(sm)			(sm)->sector_size
#define nrf52_qspi_page_size(sm)				(sm)->page_size
#define nrf52_qspi_io(sm)						(sm)->io

//#define nrf52_qspi_block_size(sm)			sizeof(io_storage_block_t)

/*

typedef struct {

	bool	(*enable) (io_sector_memory_t const *);
	bool	(*disable) (io_sector_memory_t const *);
		
	bool	(*read) (io_sector_memory_t const *,uint32_t,uint8_t*,size_t);
	size_t (*write) (io_sector_memory_t const *,uint32_t,uint8_t const*,size_t);
	
	bool	(*erase) (io_sector_memory_t const*,uint32_t);

	bool (*reference) (io_sector_memory_t const*);
	bool (*unreference) (io_sector_memory_t const*);
	
} sector_memory_implementation_t;


*/


extern EVENT_DATA io_socket_implementation_t nrf52_qspi_implementation;

#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// mplementtaion
//
//-----------------------------------------------------------------------------

#define QSPI_ERASE_LEN_4KB (0UL)
#define QSPI_ERASE_LEN_64KB (1UL)
#define QSPI_ERASE_LEN_All (2UL)


#define GET_CINSTRCONF (NRF_QSPI->CINSTRCONF & ~(\
				QSPI_CINSTRCONF_OPCODE_Msk \
			|	QSPI_CINSTRCONF_LENGTH_Msk\
		))

#define QSPI_CHIP_STATUS(s)	((s) << QSPI_STATUS_SREG_Pos)



static io_socket_t*
nrf52_qspi_initialise (io_socket_t *socket,io_t *io,io_settings_t const *C) {
	nrf52_qspi_t *this = (nrf52_qspi_t*) socket;
	this->io = io;
	return socket;
}

static size_t
nrf52_qspi_mtu (io_socket_t const *socket) {
	return 1024;
}

static bool
nrf52_identify_flash_chip (nrf52_qspi_t const *this) {
	NRF_QSPI_Type *qspi = this->qspi_registers;

	qspi->CINSTRDAT0 = 0;
	qspi->CINSTRDAT1 = 0;
	qspi->CINSTRCONF = GET_CINSTRCONF | (
			(SPI_FLASH_RDID << QSPI_CINSTRCONF_OPCODE_Pos)
		|	(4 << QSPI_CINSTRCONF_LENGTH_Pos)
		|	(1 << QSPI_CINSTRCONF_LIO3_Pos)
	);
	while ((qspi->STATUS & QSPI_STATUS_READY_Msk) == 0);
	
	jedec_chip_identification_t id = {.reg = qspi->CINSTRDAT0};

	if (
			id.device.manufacturer == this->expected_chip.device.manufacturer
		&&	id.device.type == this->expected_chip.device.type
		&&	id.device.capacity == this->expected_chip.device.capacity
	) {
		return true;
	} else {
		io_panic (io_socket_io(this),IO_PANIC_DEVICE_ERROR);
		return false;
	}
}

static bool
nrf52_qspi_open (io_socket_t *socket,io_socket_open_flag_t flag) {
	nrf52_qspi_t *this = (nrf52_qspi_t*) socket;
	NRF_QSPI_Type *qspi = this->qspi_registers;
	
	if (qspi->ENABLE == 0) {
		uint32_t reg;
		//
		// select high drive on all pins ...
		//
		nrf_gpio_pin_set_drive_level (this->cs_pin);
		nrf_gpio_pin_set_drive_level (this->sck_pin);
		nrf_gpio_pin_set_drive_level (this->io0_pin);
		nrf_gpio_pin_set_drive_level (this->io1_pin);
		nrf_gpio_pin_set_drive_level (this->io2_pin);
		nrf_gpio_pin_set_drive_level (this->io3_pin);
		
		NRF_QSPI->PSEL.CSN = nrf_gpio_pin_map(this->cs_pin);
		NRF_QSPI->PSEL.SCK = nrf_gpio_pin_map(this->sck_pin);
		NRF_QSPI->PSEL.IO0 = nrf_gpio_pin_map(this->io0_pin);
		NRF_QSPI->PSEL.IO1 = nrf_gpio_pin_map(this->io1_pin);
		NRF_QSPI->PSEL.IO2 = nrf_gpio_pin_map(this->io2_pin);
		NRF_QSPI->PSEL.IO3 = nrf_gpio_pin_map(this->io3_pin);

		NRF_QSPI->IFCONFIG0 &= ~QSPI_IFCONFIG0_PPSIZE_Msk;
		NRF_QSPI->IFCONFIG0 |= (
			QSPI_IFCONFIG0_PPSIZE_256Bytes << QSPI_IFCONFIG0_PPSIZE_Pos
		);
		// 32 MHz
		reg = NRF_QSPI->IFCONFIG1 & ~QSPI_IFCONFIG1_SCKFREQ_Msk;
		reg |= (2 << QSPI_IFCONFIG1_SCKFREQ_Pos);
		NRF_QSPI->IFCONFIG1 = reg;
		
		NRF_QSPI->ENABLE = 1;
		
		NRF_QSPI->EVENTS_READY = 0;
		NRF_QSPI->TASKS_ACTIVATE = 1;
		while (NRF_QSPI->EVENTS_READY == 0);
		
		// get device ID
		if (!nrf52_identify_flash_chip (this)) {
			return false;
		}
	}
	return true;
	
	return true;
}

static void
nrf52_qspi_close (io_socket_t *socket) {
	nrf52_qspi_t *this = (nrf52_qspi_t*) socket;
	NRF_QSPI_Type *qspi = this->qspi_registers;

	if (qspi->ENABLE) {
		//
		// wait for completion of all actions
		//
		qspi->TASKS_DEACTIVATE = 1;
		while (qspi->EVENTS_READY == 0);
		
		qspi->ENABLE = 0;

	}
}

static bool
nrf52_qspi_is_closed (io_socket_t const *socket) {
	nrf52_qspi_t *this = (nrf52_qspi_t*) socket;
	return this->qspi_registers->ENABLE == 0;
}

static io_encoding_t*
nrf52_qspi_new_message (io_socket_t *socket) {
	return NULL;
}

static bool
nrf52_qspi_send_message (io_socket_t *socket,io_encoding_t *encoding) {
	return false;
}

EVENT_DATA io_socket_implementation_t nrf52_qspi_implementation = {
	SPECIALISE_IO_SOCKET_IMPLEMENTATION (
		&io_physical_socket_implementation
	)
	.initialise = nrf52_qspi_initialise,
	.open = nrf52_qspi_open,
	.close = nrf52_qspi_close,
	.is_closed = nrf52_qspi_is_closed,
	.new_message = nrf52_qspi_new_message,
	.send_message = nrf52_qspi_send_message,
	.mtu = nrf52_qspi_mtu,
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

