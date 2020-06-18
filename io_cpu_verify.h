/*
 *
 * tests specific to the nrf52 CPU
 *
 */
#ifndef nrf52_verify_H_
#define nrf52_verify_H_
#ifdef IMPLEMENT_VERIFY_IO_CPU

static void
test_io_events_1_ev (io_event_t *ev) {
	*((uint32_t*) ev->user_value) = 1;
}

TEST_BEGIN(test_io_events_1) {
	volatile uint32_t a = 0;
	io_event_t ev;
		
	initialise_io_event (&ev,test_io_events_1_ev,(void*) &a);

	io_enqueue_event (TEST_IO,&ev);
	while (a == 0);
	VERIFY (a == 1,NULL);
}
TEST_END

static uint32_t test_io_events_2_ev_result = 0;

static void
test_io_events_2_ev (io_event_t *ev) {
	test_io_events_2_ev_result = io_is_in_event_thread (ev->user_value);
}

TEST_BEGIN(test_io_events_2) {
	io_event_t ev;
		
	initialise_io_event (&ev,test_io_events_2_ev,TEST_IO);

	test_io_events_2_ev_result = 0;
	
	io_enqueue_event (TEST_IO,&ev);
	io_wait_for_all_events (TEST_IO);
	
	VERIFY (test_io_events_2_ev_result == 1,NULL);
}
TEST_END

TEST_BEGIN(test_time_clock_alarms_1) {
	volatile uint32_t a = 0;
	io_alarm_t alarm;
	io_event_t ev;
	io_time_t t;
	initialise_io_event (&ev,test_io_events_1_ev,(void*) &a);
	
	t = io_get_time (TEST_IO);
	initialise_io_alarm (
		&alarm,&ev,&ev,
		(io_time_t) {t.nanoseconds + millisecond_time(200).nanoseconds}
	);

	io_enqueue_alarm (TEST_IO,&alarm);

	//io_printf (TEST_IO,"from %lld to %lld\n",t.nanoseconds,alarm.when.nanoseconds);
	
	while (a == 0);
	VERIFY (a == 1,NULL);	
}
TEST_END

TEST_BEGIN(test_io_random_1) {
	uint32_t rand[3];

	rand[0] = io_get_random_u32(TEST_IO);
	rand[1] = io_get_random_u32(TEST_IO);
	rand[2] = io_get_random_u32(TEST_IO);

	if (rand[0] == rand[1])	rand[1] = io_get_random_u32(TEST_IO);
	if (rand[0] == rand[1]) rand[1] = io_get_random_u32(TEST_IO);
	if (rand[0] == rand[1]) rand[1] = io_get_random_u32(TEST_IO);

	if (rand[1] == rand[2]) rand[2] = io_get_random_u32(TEST_IO);
	if (rand[1] == rand[2]) rand[2] = io_get_random_u32(TEST_IO);
	if (rand[1] == rand[2]) rand[2] = io_get_random_u32(TEST_IO);

	VERIFY(rand[0] != rand[1],NULL);
	VERIFY(rand[1] != rand[2],NULL);

	rand[0] = io_get_next_prbs_u32(TEST_IO);
	rand[1] = io_get_next_prbs_u32(TEST_IO);
	rand[2] = io_get_next_prbs_u32(TEST_IO);

	if (rand[0] == rand[1])	rand[1] = io_get_next_prbs_u32(TEST_IO);
	if (rand[0] == rand[1]) rand[1] = io_get_next_prbs_u32(TEST_IO);
	if (rand[0] == rand[1]) rand[1] = io_get_next_prbs_u32(TEST_IO);

	if (rand[1] == rand[2]) rand[2] = io_get_next_prbs_u32(TEST_IO);
	if (rand[1] == rand[2]) rand[2] = io_get_next_prbs_u32(TEST_IO);
	if (rand[1] == rand[2]) rand[2] = io_get_next_prbs_u32(TEST_IO);

	VERIFY(rand[0] != rand[1],NULL);
	VERIFY(rand[1] != rand[2],NULL);
}
TEST_END

static bool
radio_emulator_send_message (io_socket_t *socket,io_encoding_t *encoding) {
	io_layer_t *layer = get_nrf52_radio_layer (encoding);
	if (layer) {
//	if (is_nrf52_radio_encoding (encoding)) {
		io_layer_load_header (layer,encoding);
//		nrf52_radio_frame_t *packet = io_encoding_get_byte_stream (encoding);
//		packet->length = io_encoding_length (encoding) - 1;
		return io_multiplexer_socket_send_message (socket,encoding);
	} else {
		// not generated by me so should not decrement ?
		unreference_io_encoding (encoding);
		return false;
	}
}

EVENT_DATA io_socket_implementation_t nrf52_radio_emulator = {
	SPECIALISE_IO_SOCKET_EMULATOR_IMPLEMENTATION (
		&io_socket_emulator_implementation
	)
	.new_message = nrf52_radio_socket_new_message,
	.send_message = radio_emulator_send_message,
};


bool
dlc_link (io_t *io,io_address_t a,io_socket_t **outer,io_socket_t **inner) {
	return false;
}

static io_socket_t*
allocate_test_radio_socket (io_t *io,io_address_t address) {
	io_socket_emulator_t *socket = io_byte_memory_allocate (
		io_get_byte_memory (io),sizeof(io_socket_emulator_t)
	);
	socket->implementation = &nrf52_radio_emulator;
	socket->address = duplicate_io_address (io_get_byte_memory (io),address);
	return (io_socket_t*) socket;
}

TEST_BEGIN(test_io_radio_packet_encoding_1) {
	io_byte_memory_t *bm = io_get_byte_memory (TEST_IO);
	memory_info_t begin,end;

	io_byte_memory_get_info (bm,&begin);

	const io_settings_t radio_settings = {
		.transmit_pipe_length = 5,
		.receive_pipe_length = 5,
	};

	const io_settings_t bus = {
		.transmit_pipe_length = 3,
		.receive_pipe_length = 3,
	};
		
	socket_builder_t network[] = {
		{0,allocate_io_dlc_socket,IO_DLC_LAYER_ID,NULL,false,BINDINGS({0,1},END_OF_BINDINGS)},
		{1,allocate_test_radio_socket,generate_nrf52_radio_address (TEST_IO),&radio_settings,false,BINDINGS({1,4},END_OF_BINDINGS)},
		{2,allocate_io_dlc_socket,IO_DLC_LAYER_ID,NULL,false,BINDINGS({2,3},END_OF_BINDINGS)},
		{3,allocate_test_radio_socket,generate_nrf52_radio_address (TEST_IO),&radio_settings,false,BINDINGS({3,4},END_OF_BINDINGS)},
		{4,allocate_io_shared_media,io_invalid_address(),&bus,false,NULL},
	};

	io_socket_t * socket[SIZEOF(network)] = {0};
	
	build_io_sockets (TEST_IO,socket,network,SIZEOF(network));
	
	io_encoding_t *encoding = io_socket_new_message(socket[0]);

	if (VERIFY(encoding != NULL,NULL)) {

		if (VERIFY (is_nrf52_radio_encoding (encoding),NULL)) {
			
			io_layer_t *layer = get_nrf52_radio_layer (encoding);
			if (VERIFY (layer != NULL,NULL)) {
				// because there is only one layer
				VERIFY (
					io_layer_get_byte_stream (layer,encoding) == io_encoding_get_byte_stream (encoding),
					NULL
				);
				
				io_address_t dest = io_layer_get_destination_address (layer,encoding);
				VERIFY (
					compare_io_addresses (
						dest,def_io_u32_address(NRF_BROADCAST_ADDRESS)
					) == 0,
					NULL
				);

				io_address_t local = io_layer_get_source_address (layer,encoding);
				VERIFY (
					compare_io_addresses (
						local,io_socket_address(socket[1])
					) == 0,
					NULL
				);
			}
		}
		
		io_socket_open (socket[0],IO_SOCKET_OPEN_CONNECT);
		VERIFY (io_socket_send_message (socket[0],encoding),NULL);
		
		io_wait_for_all_events (TEST_IO);
	}
	
	free_io_sockets (socket,socket + SIZEOF(network));

	io_byte_memory_get_info (bm,&end);
	VERIFY (end.used_bytes == begin.used_bytes,NULL);
}
TEST_END

UNIT_SETUP(setup_io_cpu_unit_test) {
	return VERIFY_UNIT_CONTINUE;
}

UNIT_TEARDOWN(teardown_io_cpu_unit_test) {
}

static void
io_cpu_unit_test (V_unit_test_t *unit) {
	static V_test_t const tests[] = {
		test_io_events_1,
		test_io_events_2,
		test_time_clock_alarms_1,
		test_io_random_1,
		test_io_radio_packet_encoding_1,
		0
	};
	unit->name = "io cpu";
	unit->description = "io cpu unit test";
	unit->tests = tests;
	unit->setup = setup_io_cpu_unit_test;
	unit->teardown = teardown_io_cpu_unit_test;
}


void
run_ut_io_cpu (V_runner_t *runner) {
	static const unit_test_t test_set[] = {
		io_cpu_unit_test,
		0
	};
	V_run_unit_tests(runner,test_set);
}

#endif /* IMPLEMENT_VERIFY_IO_CPU */
#endif
