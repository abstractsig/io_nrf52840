/*
 *
 * nrf pins
 *
 */
#ifndef nrf52_pins_H_
#define nrf52_pins_H_

//
// pins
//
#define NRF_GPIO_PIN_MAP_PORT(map)	((map) >> 5)
#define NRF_GPIO_PIN_MAP_PIN(map)	((map) & 0x1F)
#define NRF_GPIO_PIN_MAP(port,pin)	(((port) << 5) | ((pin) & 0x1F))

#define NRF_GPIO_ACTIVE_LEVEL_LOW		0
#define NRF_GPIO_ACTIVE_LEVEL_HIGH		1

#define NRF_IO_PIN_ACTIVE_LOW				0
#define NRF_IO_PIN_ACTIVE_HIGH			1

#define GPIO_PIN_INACTIVE					0
#define GPIO_PIN_ACTIVE						1

typedef union PACK_STRUCTURE {
	io_pin_t io;
	uint32_t u32;
	struct PACK_STRUCTURE {
		uint32_t	pin_map:7;
		uint32_t active_level:1;
		uint32_t initial_state:1;
		uint32_t pull_mode:3;
		uint32_t gpiote_channel:3;
		uint32_t event_sense:2;				// GPIOTE_CONFIG_POLARITY_xxx
		uint32_t drive_level:3;
		uint32_t :12;
	} nrf;
} nrf_io_pin_t;

#define nrf_gpio_pin_map(pin)								(pin).nrf.pin_map
#define nrf_gpio_pin_active_level(pin)					(pin).nrf.active_level
#define nrf_gpio_pin_drive_level(pin)					(pin).nrf.drive_level
#define nrf_gpio_pin_pull_mode(pin)						(pin).nrf.pull_mode
#define nrf_gpio_pin_initial_state(pin)				(pin).nrf.initial_state
#define nrf_gpio_pin_event_channel_number(pin)		(pin).nrf.gpiote_channel
#define nrf_gpio_pin_event_sense(pin)					(pin).nrf.event_sense

#define nrf_io_pin_is_null(pin)	(nrf_gpio_pin_map(pin) == NRF_GPIO_PIN_MAP(2,0))
#define nrf_io_pin_is_valid(pin)	(!nrf_io_pin_is_null(pin))

#define def_nrf_gpio_null_pin() (nrf_io_pin_t) {\
		.nrf.pin_map = NRF_GPIO_PIN_MAP(2,0),\
	}

#define def_nrf_io_alternate_pin(port,pin_number,D) (nrf_io_pin_t) {\
		.nrf.pin_map = NRF_GPIO_PIN_MAP(port,pin_number),\
		.nrf.active_level = NRF_IO_PIN_ACTIVE_HIGH,\
		.nrf.drive_level = D,\
		.nrf.initial_state = GPIO_PIN_INACTIVE,\
		.nrf.pull_mode = NRF_GPIO_PIN_NOPULL,\
		.nrf.gpiote_channel = 0,\
		.nrf.event_sense = GPIOTE_CONFIG_POLARITY_None,\
	}
	
#define def_nrf_io_pin_alternate_high_drive(port,pin_number) (nrf_io_pin_t) {\
		.nrf.pin_map = NRF_GPIO_PIN_MAP(port,pin_number),\
		.nrf.active_level = 0,\
		.nrf.drive_level = 3,\
		.nrf.initial_state = GPIO_PIN_INACTIVE,\
		.nrf.pull_mode = NRF_GPIO_PIN_NOPULL,\
		.nrf.gpiote_channel = 0,\
		.nrf.event_sense = GPIOTE_CONFIG_POLARITY_None,\
	}

#define def_nrf_io_input_pin(port,pin_number,active,pull) (nrf_io_pin_t) {\
		.nrf.pin_map = NRF_GPIO_PIN_MAP(port,pin_number),\
		.nrf.active_level = active,\
		.nrf.initial_state = GPIO_PIN_INACTIVE,\
		.nrf.pull_mode = pull,\
		.nrf.gpiote_channel = 0,\
		.nrf.event_sense = GPIOTE_CONFIG_POLARITY_None,\
	}

#define def_nrf_io_output_pin(port,pin_number,active,initial) (nrf_io_pin_t) {\
		.nrf.pin_map = NRF_GPIO_PIN_MAP(port,pin_number),\
		.nrf.active_level = active,\
		.nrf.initial_state = initial,\
		.nrf.pull_mode = NRF_GPIO_PIN_NOPULL,\
		.nrf.gpiote_channel = 0,\
		.nrf.event_sense = GPIOTE_CONFIG_POLARITY_None,\
	}

#define def_nrf_io_interrupt_pin(port,pin_number,active,pull,channel,sense) (nrf_io_pin_t) {\
		.nrf.pin_map = NRF_GPIO_PIN_MAP(port,pin_number),\
		.nrf.active_level = active,\
		.nrf.initial_state = 0,\
		.nrf.pull_mode = pull,\
		.nrf.gpiote_channel = channel,\
		.nrf.event_sense = sense,\
	}

void	nrf52_configure_reset_pin (nrf_io_pin_t);
void	nrf_gpio_pin_set_drive_level (nrf_io_pin_t);

#ifdef IMPLEMENT_IO_CPU
//-----------------------------------------------------------------------------
//
// Implementtaion
//
//-----------------------------------------------------------------------------
#include <nrf/nrf52_sdk.h>

#define NRF_GPIOTE_PERIPHERAL		NRF_GPIOTE


static void
nrf52_write_to_io_pin (io_t *io,io_pin_t rpin,int32_t state) {
	nrf_io_pin_t pin = {rpin};
	if (state ^ nrf_gpio_pin_active_level (pin)) {
		nrf_gpio_pin_write(nrf_gpio_pin_map(pin),0);
	} else {
		nrf_gpio_pin_write(nrf_gpio_pin_map(pin),1);
	}
}

static void
nrf52_set_io_pin_to_input (io_t *io,io_pin_t rpin) {
	nrf_io_pin_t pin = {rpin};
	nrf_gpio_cfg_input (
		nrf_gpio_pin_map(pin),nrf_gpio_pin_pull_mode(pin)
	);
}

static void
nrf52_set_io_pin_to_output (io_t *io,io_pin_t rpin) {
	nrf_io_pin_t pin = {rpin};
	nrf52_write_to_io_pin (io,rpin,nrf_gpio_pin_initial_state(pin));
	nrf_gpio_cfg_output(nrf_gpio_pin_map(pin));
}

static void
nrf52_toggle_io_pin (io_t *io,io_pin_t rpin) {
	nrf_io_pin_t pin = {rpin};
	nrf_gpio_pin_toggle(nrf_gpio_pin_map(pin));
}

static int32_t
nrf52_read_io_input_pin (io_t *io,io_pin_t rpin) {
	nrf_io_pin_t pin = {rpin};
	return (
			nrf_gpio_pin_read (nrf_gpio_pin_map(pin))
		==	nrf_gpio_pin_active_level(pin)
	);
}

void
nrf_gpio_pin_set_drive_level (nrf_io_pin_t pin) {
	uint32_t pin_number = nrf_gpio_pin_map(pin);
	NRF_GPIO_Type *reg = nrf_gpio_pin_port_decode(&pin_number);
	uint32_t cfg = reg->PIN_CNF[pin_number] & ~GPIO_PIN_CNF_DRIVE_Msk;
	cfg |= (nrf_gpio_pin_drive_level(pin) << GPIO_PIN_CNF_DRIVE_Pos);
	reg->PIN_CNF[pin_number] = cfg;
}

static void
nrf52_set_io_pin_interrupt (io_t *io,io_pin_t rpin,io_interrupt_handler_t *h) {
	nrf_io_pin_t pin = {rpin};
	
	nrf_gpio_cfg_input(nrf_gpio_pin_map(pin),nrf_gpio_pin_pull_mode(pin));

	NRF_GPIOTE_PERIPHERAL->CONFIG[nrf_gpio_pin_event_channel_number(pin)] = (
			(GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos)
		|	(NRF_GPIO_PIN_MAP_PIN(nrf_gpio_pin_map(pin)) << GPIOTE_CONFIG_PSEL_Pos)
		#ifdef GPIOTE_CONFIG_PORT_Pos
		|	(NRF_GPIO_PIN_MAP_PORT(nrf_gpio_pin_map(pin)) << GPIOTE_CONFIG_PORT_Pos)
		#endif
		|	(nrf_gpio_pin_event_sense(pin) << GPIOTE_CONFIG_POLARITY_Pos)
	);
}

static bool
nrf52_io_pin_is_valid (io_t *io,io_pin_t p) {
	nrf_io_pin_t pin = {p};
	return nrf_io_pin_is_valid (pin);
}

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
