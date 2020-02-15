/*
 *
 * a physical driver for WINC15x0 WiFi modules
 *
 */
#ifndef io_nrf52_winc15x0_H_
#define io_nrf52_winc15x0_H_
#include <io_cpu.h>

typedef struct PACK_STRUCTURE nrf52_winc15x0 {
	IO_SOCKET_STRUCT_MEMBERS
	
	io_encoding_implementation_t const *encoding;
	io_t *io;

	nrf_io_pin_t reset_pin;
	nrf_io_pin_t winc_enable_pin;
	nrf_io_pin_t interrupt_pin;

	nrf_io_pin_t spi_enable_pin;

} nrf52_winc15x0_t;

extern EVENT_DATA io_socket_implementation_t nrf52_winc15x0_implementation;

#ifdef IMPLEMENT_WINC15X0_SOCKET
//-----------------------------------------------------------------------------
//
// winc15x0 socket implementation
//
//-----------------------------------------------------------------------------

static io_socket_t*
nrf52_winc15x0_initialise (
	io_socket_t *socket,io_t *io,io_socket_constructor_t const *C
) {	
	nrf52_winc15x0_t *this = (nrf52_winc15x0_t*) socket;
	
	this->io = io;
	this->encoding = C->encoding;
	
	io_set_pin_to_output (io,this->spi_enable_pin.io);
	io_set_pin_to_output (io,this->winc_enable_pin.io);
	io_set_pin_to_output (io,this->reset_pin.io);
	io_set_pin_to_interrupt (io,this->interrupt_pin.io);
	
	return socket;
}

static void
nrf52_winc15x0_free (io_socket_t *socket) {
	io_byte_memory_t *bm = io_get_byte_memory (io_socket_get_io(socket));

	// ....
	
	io_byte_memory_free (bm,socket);
}

static io_t*
nrf52_winc15x0_get_io (io_socket_t *socket) {
	nrf52_winc15x0_t *this = (nrf52_winc15x0_t*) socket;
	return this->io;
}

static bool
nrf52_winc15x0_open (io_socket_t *socket) {
	return false;
}

EVENT_DATA io_socket_implementation_t nrf52_winc15x0_implementation = {
	.specialisation_of = &io_physical_socket_implementation_base,
	.new = io_socket_new_null,
	.initialise = nrf52_winc15x0_initialise,
	.free = nrf52_winc15x0_free,
	.get_io = nrf52_winc15x0_get_io,
	.open = nrf52_winc15x0_open,
	.close = NULL,
	.bindr = NULL,
	.bindt = NULL,
	.new_message = NULL,
	.send_message = NULL,
	.iterate_inner_sockets = NULL,
	.iterate_outer_sockets = NULL,
	.mtu = NULL,
};
#endif /* IMPLEMENT_WINC15X0_SOCKET */
#ifdef IMPLEMENT_VERIFY_WINC15X0_SOCKET
#include <verify_io.h>

UNIT_SETUP(setup_nrf52_winc15x0_socket_unit_test) {
	return VERIFY_UNIT_CONTINUE;
}

UNIT_TEARDOWN(teardown_nrf52_winc15x0_socket_unit_test) {
}

void
nrf52_winc15x0_socket_unit_test (V_unit_test_t *unit) {
	static V_test_t const tests[] = {
		0
	};
	unit->name = "winc15x0 socket";
	unit->description = "nrf winc150x socket unit test";
	unit->tests = tests;
	unit->setup = setup_nrf52_winc15x0_socket_unit_test;
	unit->teardown = teardown_nrf52_winc15x0_socket_unit_test;
}
#endif /* IMPLEMENT_VERIFY_WINC15X0_SOCKET */
#endif
