/*
 *
 *
 */
#ifndef WITH_QSPI_VERIFY
#define WITH_QSPI_VERIFY
#include <verify_io.h>

TEST_BEGIN(test_nrf_qspi_1) {
	io_socket_t *qspi = io_get_socket (TEST_IO,QSPI_SOCKET);
	if (VERIFY (qspi != NULL,NULL)) {
		if (VERIFY (io_socket_open (qspi,IO_SOCKET_OPEN_CONNECT),NULL)) {
			io_socket_close(qspi);
			VERIFY (io_socket_is_closed(qspi),NULL);
		}
	}
}
TEST_END


UNIT_SETUP(setup_nrf_qspi_unit_test) {
	if (
			io_get_socket (TEST_IO,QSPI_SOCKET) != NULL
		&&	io_socket_is_closed (io_get_socket (TEST_IO,QSPI_SOCKET))
	) {
		return VERIFY_UNIT_CONTINUE;
	} else {
		return VERIFY_UNIT_SKIP;
	}
}

UNIT_TEARDOWN(teardown_nrf_qspi_unit_test) {
}

void
nrf_qspi_unit_test (V_unit_test_t *unit) {
	static V_test_t const tests[] = {
		test_nrf_qspi_1,
		0
	};
	unit->name = "qspi";
	unit->description = "nrf52 qspi unit test";
	unit->tests = tests;
	unit->setup = setup_nrf_qspi_unit_test;
	unit->teardown = teardown_nrf_qspi_unit_test;
}

#define IO_QSPI_UNIT_TEST nrf_qspi_unit_test,

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


