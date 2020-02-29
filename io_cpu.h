/*
 *
 * io nrf52840 cpu
 *
 */
#ifndef io_cpu_H_
#define io_cpu_H_
#include <io_core.h>
#include <nrf52840.h>
#include <nrf52840_bitfields.h>

typedef struct PACK_STRUCTURE nrf_time_clock {

	NRF_TIMER_Type *high_timer;
	NRF_TIMER_Type *low_timer;	
	IRQn_Type interrupt_number;
	uint32_t ppi_channel;

	io_event_t alarm;
	io_t *io;
	
} nrf_64bit_time_clock_t;

void	start_nrf_time_clock (io_t*,nrf_64bit_time_clock_t*);

#define NRF52840_IO_CPU_STRUCT_MEMBERS \
	IO_STRUCT_MEMBERS				\
	io_value_memory_t *vm;\
	io_byte_memory_t *bm;\
	uint32_t in_event_thread;\
	io_value_pipe_t *tasks;\
	nrf_64bit_time_clock_t tc;\
	uint32_t prbs_state[4]; \
	/**/

typedef struct PACK_STRUCTURE nrf52840_io {
	NRF52840_IO_CPU_STRUCT_MEMBERS
} nrf52840_io_t;

void	initialise_cpu_io (io_t*);

#define NUMBER_OF_ARM_INTERRUPT_VECTORS	16L
#define NUMBER_OF_NRF_INTERRUPT_VECTORS	47L
#define NUMBER_OF_INTERRUPT_VECTORS	(NUMBER_OF_ARM_INTERRUPT_VECTORS + NUMBER_OF_NRF_INTERRUPT_VECTORS)

#define ENABLE_INTERRUPTS	\
	do {	\
		__DMB();	\
		__enable_irq();	\
	} while (0)

#define DISABLE_INTERRUPTS	\
	do {	\
		__disable_irq();	\
		__DMB();	\
	} while (0)

#define HIGHEST_INTERRUPT_PRIORITY			0
#define HIGH_INTERRUPT_PRIORITY				1
#define NORMAL_INTERRUPT_PRIORITY			2
#define LOW_INTERRUPT_PRIORITY				3
#define LOWEST_INTERRUPT_PRIORITY			7
#define EVENT_LOOP_INTERRUPT_PRIORITY		LOWEST_INTERRUPT_PRIORITY

#define EVENT_THREAD_INTERRUPT		SWI0_EGU0_IRQn
#define SET_EVENT_PENDING				NVIC_SetPendingIRQ (EVENT_THREAD_INTERRUPT)

typedef struct PACK_STRUCTURE nrf52_oscillator {
	IO_CPU_CLOCK_SOURCE_STRUCT_MEMBERS
} nrf52_oscillator_t;

typedef struct nrf52_core_clock {
	IO_CPU_DEPENDANT_CLOCK_STRUCT_MEMBERS
} nrf52_core_clock_t;

extern EVENT_DATA io_cpu_clock_implementation_t nrf52_on_chip_oscillator_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t nrf52_crystal_oscillator_implementation;
extern EVENT_DATA io_cpu_clock_implementation_t nrf52_core_clock_implementation;

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

#define def_nrf_gpio_alternate_pin(port,pin_number) (nrf_io_pin_t) {\
		.nrf.pin_map = NRF_GPIO_PIN_MAP(port,pin_number),\
		.nrf.active_level = 0,\
		.nrf.drive_level = 0,\
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

//
// sockets
//

typedef struct PACK_STRUCTURE nrf52_uart {
	IO_SOCKET_STRUCT_MEMBERS
	
	io_t *io;
	io_encoding_implementation_t const *encoding;

	io_encoding_pipe_t *tx_pipe;
	io_event_t signal_transmit_available;
	io_event_t transmit_complete;
	
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

typedef struct PACK_STRUCTURE nrf52_spi {
	IO_SOCKET_STRUCT_MEMBERS

	io_encoding_implementation_t const *encoding;
	io_t *io;

	NRF_SPI_Type *spi_registers;
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
// nrf52840 Implementtaion
//
//-----------------------------------------------------------------------------
//
// to allow for secure peripherals
//
#define NRF_PERIPHERAL(P)			P
#define NRF_GPIOTE_PERIPHERAL		NRF_GPIOTE
#include <nrf52_sdk.h>

#define NFR_PERSISTANT_MEMORY_SECTION __attribute__ ((section(".io_config")))

/*
 *-----------------------------------------------------------------------------
 *
 * the initial io state settings
 *
 *-----------------------------------------------------------------------------
 */
static NFR_PERSISTANT_MEMORY_SECTION io_persistant_state_t ioc = {
	.first_run_flag = IO_FIRST_RUN_SET,
	.power_cycles = 0,
	.uid = {{0}},
	.secret = {{0}},
	.shared = {{0}},
};

static bool
nrf52840_io_config_is_first_run (io_t *io) {
	return ioc.first_run_flag == IO_FIRST_RUN_SET;
}

INLINE_FUNCTION void
wait_for_flash_ready (void) {
	while (NRF_NVMC->READYNEXT == 0);
}

static bool
io_erase_nvm_page (io_t *io,uint32_t *page_address) {
	if (NRF_NVMC->CONFIG == (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos)) {

		ENTER_CRITICAL_SECTION(io);
		
		// Enable erase.
		NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
		__ISB();
		__DSB();

		NRF_NVMC->ERASEPAGE = (uint32_t) page_address;
		wait_for_flash_ready ();
		
		// back to read only
		NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
		__ISB();
		__DSB();

		EXIT_CRITICAL_SECTION(io);
		
		return true;
	} else {
		return false;
	}
}

static bool
io_write_nvm_page (io_t *io,uint32_t const *data,uint32_t num_words,uint32_t *write_address) {
	if (NRF_NVMC->CONFIG == (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos)) {
		uint32_t i;

		ENTER_CRITICAL_SECTION (io);
		
		// Enable erase.
		NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
		__ISB();
		__DSB();

		NRF_NVMC->ERASEPAGE = (uint32_t) write_address;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy);
		
		// Enable write.
		NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
		__ISB();
		__DSB();

		for (i = 0; i < num_words; i++) {
			write_address[i] = data[i];
			wait_for_flash_ready();
		}
		
		// back to read only
		NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);
		__ISB();
		__DSB();

		EXIT_CRITICAL_SECTION (io);
		
		return true;
	} else {
		return false;
	}
}

static bool
nrf52840_io_config_clear_first_run (io_t *io) {
	if (ioc.first_run_flag == IO_FIRST_RUN_SET) {
		io_persistant_state_t new_ioc = ioc;
		new_ioc.first_run_flag = IO_FIRST_RUN_CLEAR;

//		io_gererate_uid (io,&new_ioc.uid);
//		io_gererate_authentication_key_pair (io,&new_ioc.secret,&new_ioc.shared);
		
		io_erase_nvm_page (io,(uint32_t*) &ioc);
		io_write_nvm_page (
			io,io_config_u32_ptr(new_ioc),io_config_u32_size(),(uint32_t*) &ioc
		);
		return true;
	} else {
		return false;
	}
}

io_uid_t const*
nrf52840_io_config_uid (io_t *io) {
	return &ioc.uid;
}

//
// cpu clocks
//

static float64_t
nrf52_crystal_oscillator_get_frequency (io_cpu_clock_pointer_t this) {
	return 64000000.0;
}

static bool
nrf52_crystal_oscillator_start (io_t *io,io_cpu_clock_pointer_t this) {
	if (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
		//
		// this also connects the clock to the CPU 
		// (i.e replaces the on-chip clock)
		//
		NRF_CLOCK->TASKS_HFCLKSTART    = 1;
		while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
	}
	return true;
}

EVENT_DATA io_cpu_clock_implementation_t nrf52_crystal_oscillator_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_frequency = nrf52_crystal_oscillator_get_frequency,
	.link_input_to_output = NULL,
	.link_output_to_input = NULL,
	.start = nrf52_crystal_oscillator_start,
	.stop = NULL,
};

static float64_t
nrf52_on_chip_oscillator_get_frequency (io_cpu_clock_pointer_t this) {
	return 64000000.0;
}

static bool
nrf52_on_chip_oscillator_start (io_t *io,io_cpu_clock_pointer_t this) {
	return true;
}

EVENT_DATA io_cpu_clock_implementation_t nrf52_on_chip_oscillator_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_frequency = nrf52_on_chip_oscillator_get_frequency,
	.link_input_to_output = NULL,
	.link_output_to_input = NULL,
	.start = nrf52_on_chip_oscillator_start,
	.stop = NULL,
};

static float64_t
nrf52_core_clock_get_frequency (io_cpu_clock_pointer_t clock) {
	nrf52_core_clock_t const *this = (nrf52_core_clock_t const*) (
		io_cpu_clock_ro_pointer (clock)
	);
	return io_cpu_clock_get_frequency (this->input);
}

static bool
nrf52_core_clock_start (io_t *io,io_cpu_clock_pointer_t clock) {
	if (io_cpu_dependant_clock_start_input (io,clock)) {
		return true;
	} else {
		return false;
	}
}

EVENT_DATA io_cpu_clock_implementation_t nrf52_core_clock_implementation = {
	.specialisation_of = &io_cpu_clock_implementation,
	.get_frequency = nrf52_core_clock_get_frequency,
	.link_input_to_output = NULL,
	.link_output_to_input = NULL,
	.start = nrf52_core_clock_start,
	.stop = NULL,
};

//
// 64bit time clock
//
#define TIME_CLOCK_NS_PER_TICK		125LL

static void
nrf52_time_clock_interrupt (void *user_value) {
	nrf_64bit_time_clock_t *this = user_value;
	this->low_timer->EVENTS_COMPARE[0] = 0;
	io_enqueue_event (this->io,&this->alarm);
}

static uint64_t
read_64bit_count (nrf_64bit_time_clock_t *this) {
	io_time_t time = {0};

	do {
		this->low_timer->TASKS_CAPTURE[1] = 1;
		this->high_timer->TASKS_CAPTURE[1] = 1;
		
		time.part.low = this->low_timer->CC[1];
		time.part.high = this->high_timer->CC[1];
		this->high_timer->TASKS_CAPTURE[1] = 1;
	} while (time.part.high != this->high_timer->CC[1]);
	
	return time.u64;
}

static bool
set_64bit_time_clock_alarm_time (nrf_64bit_time_clock_t *this) {
	if (this->io->alarms != &s_null_io_alarm) {
		int64_t next_alarm_count = (
			this->io->alarms->when.nanoseconds / TIME_CLOCK_NS_PER_TICK
		);
		int64_t current_count = read_64bit_count (this) + 10000;

		if (next_alarm_count > current_count) {
			this->low_timer->CC[0] = next_alarm_count;
			return true;
		} else {
			// alarm is not in future
			return false;
		}
	} else {
		return false;
	}
}

static io_time_t
nrf_64bit_time_clock_get_time (nrf_64bit_time_clock_t *this) {
	return (io_time_t) {
		.nanoseconds = read_64bit_count(this) * TIME_CLOCK_NS_PER_TICK,
	};
}

static bool
process_next_alarm (nrf_64bit_time_clock_t *this) {
	if (this->io->alarms != &s_null_io_alarm) {
		volatile io_time_t t = nrf_64bit_time_clock_get_time (this);
		
		if (t.nanoseconds >= this->io->alarms->when.nanoseconds) {
			io_alarm_t *alarm = this->io->alarms;
			this->io->alarms = this->io->alarms->next_alarm;
			alarm->next_alarm = NULL;
			alarm->at->event_handler (alarm->at);
			
			//
			// tollerance check ...
			//
			return true;
		} else {
			//while(1) {}
		}
	}
	return false;
}

static void
process_alarm_queue (io_event_t *ev) {
	nrf_64bit_time_clock_t *this = ev->user_value;
	uint32_t count = 0;
	
	while (process_next_alarm(this)) {
		count++;
	}
	
	if (count) {
		set_64bit_time_clock_alarm_time (this);
	}
}

void
start_nrf_time_clock (io_t *io,nrf_64bit_time_clock_t *this) {

	this->io = io;
	initialise_io_event (&this->alarm,process_alarm_queue,this);

	register_io_interrupt_handler (
		this->io,this->interrupt_number,nrf52_time_clock_interrupt,this
	);

	this->high_timer->TASKS_STOP = 1;
	this->low_timer->TASKS_STOP = 1;

	this->high_timer->TASKS_CLEAR = 1;
	this->low_timer->TASKS_CLEAR = 1;

	this->high_timer->MODE = TIMER_MODE_MODE_Counter;
	this->high_timer->BITMODE = (
		TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos
	);

	this->low_timer->MODE = TIMER_MODE_MODE_Timer;
	this->low_timer->BITMODE = (
		TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos
	);
	this->low_timer->PRESCALER = 1;  // 125ns resolution

	// get high timer to count low timer roll-overs
	this->low_timer->CC[5] = 0xffffffff; // reduce count to test
	this->low_timer->SHORTS = (
		TIMER_SHORTS_COMPARE5_CLEAR_Enabled << TIMER_SHORTS_COMPARE5_CLEAR_Pos
	);

	NRF_PPI->CH[this->ppi_channel].EEP = (
		(uint32_t) &this->low_timer->EVENTS_COMPARE[5]
	);

	NRF_PPI->CH[this->ppi_channel].TEP = (
		(uint32_t) &this->high_timer->TASKS_COUNT
	);
	NRF_PPI->CHENSET = (1 << this->ppi_channel);

	this->low_timer->CC[0] = 0;	// a starter for 10

	this->low_timer->INTENSET = (
		TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos
	);

	NVIC_SetPriority (this->interrupt_number,NORMAL_INTERRUPT_PRIORITY);
	NVIC_ClearPendingIRQ (this->interrupt_number);
	NVIC_EnableIRQ (this->interrupt_number);
	
	this->high_timer->TASKS_START = 1;
	this->low_timer->TASKS_START = 1;
}

static void
nrf_time_clock_enqueue_alarm (io_t *io,io_alarm_t *alarm) {
	nrf_64bit_time_clock_t *this = &((nrf52840_io_t*) io)->tc;
	
	ENTER_CRITICAL_SECTION(io);

	if (alarm->when.nanoseconds < this->io->alarms->when.nanoseconds) {
		alarm->next_alarm = this->io->alarms;
		this->io->alarms = alarm;
		if (!set_64bit_time_clock_alarm_time (this)) {
			io_panic (io,0);
		}
	} else {
		io_alarm_t *pre = this->io->alarms;
		while (alarm->when.nanoseconds > pre->when.nanoseconds) {
			if (pre->next_alarm == &s_null_io_alarm) {
				break;
			}
			pre = pre->next_alarm;
		}
		alarm->next_alarm = pre->next_alarm;
		pre->next_alarm = alarm;
	}

	EXIT_CRITICAL_SECTION(io);
}

static void
nrf_time_clock_dequeue_alarm (io_t *io,io_alarm_t *alarm) {
	if (alarm->next_alarm != NULL) {
		ENTER_CRITICAL_SECTION (io);
		if (alarm == io->alarms) {
			nrf_64bit_time_clock_t *this = &((nrf52840_io_t*) io)->tc;
			io->alarms = io->alarms->next_alarm;
			set_64bit_time_clock_alarm_time (this);
		} else {
			io_alarm_t *pre = io->alarms;
			while (pre) {
				if (alarm == pre->next_alarm) {
					pre->next_alarm = alarm->next_alarm;
					break;
				}
				pre = pre->next_alarm;
			}
		}
		alarm->next_alarm = NULL;
		EXIT_CRITICAL_SECTION (io);
	}
}

//
// needed for boards that use the 5V power setup
//
void
nrf_io_pin_voltage_setup (void) {
	if (
			(NRF_UICR->REGOUT0 & UICR_REGOUT0_VOUT_Msk)
		==	(UICR_REGOUT0_VOUT_DEFAULT << UICR_REGOUT0_VOUT_Pos)
	) {
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

		NRF_UICR->REGOUT0 = (
				(NRF_UICR->REGOUT0 & ~((uint32_t)UICR_REGOUT0_VOUT_Msk))
			|	(UICR_REGOUT0_VOUT_3V0 << UICR_REGOUT0_VOUT_Pos)
		);

		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

		// System reset is needed to update UICR registers.
		NVIC_SystemReset();
	}
}

void
nrf52_configure_reset_pin (nrf_io_pin_t reset_pin) {
	if ((NRF_UICR->PSELRESET[0] & 0x3f) == 0x3f) {
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

		NRF_UICR->PSELRESET[0] = (
				(UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)
			|	nrf_gpio_pin_map (reset_pin)
		);
		NRF_UICR->PSELRESET[1] = (
				(UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)
			|	nrf_gpio_pin_map (reset_pin)
		);
		
		NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
		while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
	}
}

//
// Sockets
//
static void	nrf52_uart_interrupt (void*);
static void	nrf52_uart_tx_complete (io_event_t*);

#define NRF52_UART_RX_DMA_BUFFER_LENGTH 1

static io_socket_t*
nrf52_uart_initialise (io_socket_t *socket,io_t *io,io_socket_constructor_t const *C) {
	nrf52_uart_t *this = (nrf52_uart_t*) socket;
	this->io = io;
	
	this->tx_pipe = mk_io_encoding_pipe (
		io_get_byte_memory(io),io_socket_constructor_transmit_pipe_length(C)
	);
	initialise_io_event (
		&this->signal_transmit_available,NULL,this
	);

	this->rx_pipe = mk_io_byte_pipe (
		io_get_byte_memory(io),io_socket_constructor_receive_pipe_length(C)
	);

	this->rx_buffer[0] = io_byte_memory_allocate (
		io_get_byte_memory (io),NRF52_UART_RX_DMA_BUFFER_LENGTH
	);
	this->rx_buffer[1] = io_byte_memory_allocate (
		io_get_byte_memory (io),NRF52_UART_RX_DMA_BUFFER_LENGTH
	);
	this->active_rx_buffer = this->rx_buffer[0];
	this->next_rx_buffer = this->rx_buffer[1];

	initialise_io_event (
		&this->transmit_complete,nrf52_uart_tx_complete,this
	);

	register_io_interrupt_handler (
		io,this->interrupt_number,nrf52_uart_interrupt,this
	);
	
	return socket;
}

static void
nrf_uart_start_rx (nrf52_uart_t *this) {

	if (nrf_io_pin_is_valid(this->cts_pin)) {
		io_set_pin_to_input(this->io,this->cts_pin.io);
		while (io_read_pin(this->io,this->cts_pin.io));
	}

	this->uart_registers->SHORTS = (
		(UARTE_SHORTS_ENDRX_STARTRX_Enabled << UARTE_SHORTS_ENDRX_STARTRX_Pos)
	);
	this->uart_registers->RXD.PTR = (uint32_t) this->active_rx_buffer;
	this->uart_registers->RXD.MAXCNT = NRF52_UART_RX_DMA_BUFFER_LENGTH;
	this->uart_registers->TASKS_STARTRX = 1;
}

static bool
nrf52_uart_open (io_socket_t *socket) {
	nrf52_uart_t *this = (nrf52_uart_t*) socket;

	if (this->uart_registers->ENABLE == 0) {

		if (nrf_io_pin_is_valid (this->rts_pin)) {
			//
			// and not hardware flow control
			//
			io_set_pin_to_output (this->io,this->rts_pin.io);
			write_to_io_pin (this->io,this->rts_pin.io,0);
		}		

		if (nrf_io_pin_is_valid (this->cts_pin)) {
			io_set_pin_to_input (this->io,this->cts_pin.io);
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

	io_dequeue_event (this->io,io_pipe_event (this->tx_pipe));
	io_dequeue_event (this->io,io_pipe_event (this->rx_pipe));
}

static io_t*
nrf52_uart_get_io (io_socket_t *socket) {
	nrf52_uart_t *this = (nrf52_uart_t*) socket;
	return this->io;
}

static io_event_t*
nrf52_uart_bindr (io_socket_t *socket,io_event_t *rx) {
	nrf52_uart_t *this = (nrf52_uart_t*) socket;
	if (io_event_is_active (io_pipe_event(this->rx_pipe))) {
		merge_into_io_event(rx,io_pipe_event(this->rx_pipe));
		return io_pipe_event(this->rx_pipe);
	} else {
		return NULL;
	}
}

static void*
get_new_encoding (void *socket) {
	return io_socket_new_message (socket);
}

static io_pipe_t*
nrf52_uart_bindt (io_socket_t *socket,io_event_t *ev) {
	nrf52_uart_t *this = (nrf52_uart_t*) socket;
	if (!io_event_is_active (io_pipe_event(this->tx_pipe))) {
		this->signal_transmit_available = *ev;
		this->tx_pipe->user_action = get_new_encoding;
		this->tx_pipe->user_value = this;
		return (io_pipe_t*) (this->tx_pipe);
	} else {
		return NULL;
	}
}

static io_encoding_t*
nrf52_uart_new_message (io_socket_t *socket) {
	nrf52_uart_t *this = (nrf52_uart_t*) socket;
	return reference_io_encoding (
		new_io_encoding (this->encoding,io_get_byte_memory(this->io))
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
		io_encoding_get_ro_bytes (next,&begin,&end);
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
	io_encoding_t *next;
	
	if (io_encoding_pipe_get_encoding (this->tx_pipe,&next)) {
		unreference_io_encoding (next);
	} else {
		io_panic (this->io,IO_PANIC_SOMETHING_BAD_HAPPENED);
	}
	
	if (
			!nrf_uart_output_next_buffer (this)
		&&	this->uart_registers->ENABLE
		&& io_event_is_valid (io_pipe_event (this->tx_pipe))
	) {
		io_enqueue_event (this->io,io_pipe_event (this->tx_pipe));
	}
}

void
nrf52_uart_interrupt (void *user_value) {
	nrf52_uart_t *this = user_value;

	if (this->uart_registers->EVENTS_ERROR) {
		//io_panic(this->io,PANIC_DEVICE_ERROR);
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
			io_enqueue_event (this->io,io_pipe_event (this->rx_pipe));
		}
		this->uart_registers->EVENTS_RXTO = 0;
	}

	if (this->uart_registers->EVENTS_ENDTX) {
		io_enqueue_event (this->io,&this->transmit_complete);
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
			io_enqueue_event (this->io,io_pipe_event (this->rx_pipe));
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
	.specialisation_of = &io_physical_socket_implementation_base,
	.initialise = nrf52_uart_initialise,
	.free = NULL,
	.get_io = nrf52_uart_get_io,
	.open = nrf52_uart_open,
	.close = nrf52_uart_close,
	.bindr = nrf52_uart_bindr,
	.bindt = nrf52_uart_bindt,
	.new_message = nrf52_uart_new_message,
	.send_message = nrf52_uart_send_message_blocking,
	.iterate_inner_sockets = NULL,
	.iterate_outer_sockets = NULL,
	.mtu = nrf52_uart_mtu,
};

#define SPI_PSEL_SCK_MSK (SPI_PSEL_SCK_CONNECT_Msk | SPI_PSEL_SCK_PORT_Msk | SPI_PSEL_SCK_PIN_Msk)
#define SPI_PSEL_MOSI_MSK (SPI_PSEL_MOSI_CONNECT_Msk | SPI_PSEL_MOSI_PORT_Msk | SPI_PSEL_MOSI_PIN_Msk)
#define SPI_PSEL_MISO_MSK (SPI_PSEL_MISO_CONNECT_Msk | SPI_PSEL_MISO_PORT_Msk | SPI_PSEL_MISO_PIN_Msk)

static io_socket_t*
nrf52_spi_initialise (io_socket_t *socket,io_t *io,io_socket_constructor_t const *C) {
	nrf52_spi_t *this = (nrf52_spi_t*) socket;
	this->io = io;
	
	this->spi_registers->ENABLE &= ~SPI_ENABLE_ENABLE_Msk;
	
	io_set_pin_to_output (this->io,this->cs_pin.io);
	
	this->spi_registers->PSEL.MISO &= ~SPI_PSEL_MISO_MSK;
	this->spi_registers->PSEL.MISO |= (
			(SPI_PSEL_MISO_CONNECT_Connected << SPI_PSEL_MISO_CONNECT_Pos)
		|	nrf_gpio_pin_map(this->miso_pin)
	);
	
	this->spi_registers->PSEL.MOSI &= ~SPI_PSEL_MOSI_MSK;
	this->spi_registers->PSEL.MOSI |= (
			(SPI_PSEL_MOSI_CONNECT_Connected << SPI_PSEL_MOSI_CONNECT_Pos)
		|	nrf_gpio_pin_map(this->mosi_pin)
	);
	
	this->spi_registers->PSEL.SCK &= ~SPI_PSEL_SCK_MSK;
	this->spi_registers->PSEL.SCK |= (
			(SPI_PSEL_SCK_CONNECT_Connected << SPI_PSEL_SCK_CONNECT_Pos)
		|	nrf_gpio_pin_map(this->sclk_pin)
	);
	
	this->spi_registers->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M8;
	
	this->spi_registers->CONFIG = (
			(SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos)
		|	(SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos)
		|	(SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos)
	);
	
	return socket;
}

static io_t*
nrf52_spi_get_io (io_socket_t *socket) {
	nrf52_spi_t *this = (nrf52_spi_t*) socket;
	return this->io;
}

static size_t
nrf52_spi_mtu (io_socket_t const *socket) {
	return 1024;
}
static bool
nrf52_spi_open (io_socket_t *socket) {
	nrf52_spi_t *this = (nrf52_spi_t*) socket;

	if ((this->spi_registers->ENABLE & SPI_ENABLE_ENABLE_Msk) == 0) {
		this->spi_registers->ENABLE |= (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
	}
	
	return true;
}
static void
nrf52_spi_close (io_socket_t *socket) {
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
	.get_io = nrf52_spi_get_io,
	.open = nrf52_spi_open,
	.close = nrf52_spi_close,
	.bindr = nrf52_spi_bindr,
	.bindt = nrf52_spi_bindt,
	.new_message = nrf52_spi_new_message,
	.send_message = nrf52_spi_send_message,
	.iterate_inner_sockets = NULL,
	.iterate_outer_sockets = NULL,
	.mtu = nrf52_spi_mtu,
};

//
// Io
//
static io_byte_memory_t*
nrf52_io_get_byte_memory (io_t *io) {
	nrf52840_io_t *this = (nrf52840_io_t*) io;
	return this->bm;
}

static io_value_memory_t*
nrf52_io_get_stvm (io_t *io) {
	nrf52840_io_t *this = (nrf52840_io_t*) io;
	return this->vm;
}

static void
nrf52_do_gc (io_t *io,int32_t count) {
	io_value_memory_do_gc (io_get_short_term_value_memory (io),count);
}

static void
nrf52_signal_event_pending (io_t *io) {
	SET_EVENT_PENDING;
}

static void
nrf52_panic (io_t *io,int code) {
	DISABLE_INTERRUPTS;
	while (1);
}

static bool
nrf52_enter_critical_section (io_t *env) {
	uint32_t interrupts_are_enabled = !(__get_PRIMASK() & 0x1);
	DISABLE_INTERRUPTS;
	return interrupts_are_enabled;
}

void
nrf52_exit_critical_section (io_t *env,bool were_enabled) {
	if (were_enabled) {
		ENABLE_INTERRUPTS;
	}
}

static uint32_t
nrf52840_get_random_u8 (void) {	
	NRF_RNG->EVENTS_VALRDY = 0;
	while (NRF_RNG->EVENTS_VALRDY == 0);	
	return NRF_RNG->VALUE;
}

uint32_t
nrf52840_get_random_u32 (void) {
	uint32_t r = nrf52840_get_random_u8();
	r <<= 8;
	r += nrf52840_get_random_u8();
	r <<= 8;
	r += nrf52840_get_random_u8();
	r <<= 8;
	r += nrf52840_get_random_u8();
	return r;
}

static uint32_t
nrf52_get_random_u32 (io_t *io) {
	uint32_t r;
	
	bool h = enter_io_critical_section (io);
	
	r = nrf52840_get_random_u32 ();

	exit_io_critical_section (io,h);

	return r;
}

INLINE_FUNCTION uint32_t prbs_rotl(const uint32_t x, int k) {
	return (x << k) | (x >> (32 - k));
}

static uint32_t
nrf52_get_prbs_random_u32 (io_t *io) {
	nrf52840_io_t *this = (nrf52840_io_t*) io;
	uint32_t *s = this->prbs_state;
	bool h = enter_io_critical_section (io);
	const uint32_t result = prbs_rotl (s[0] + s[3], 7) + s[0];

	const uint32_t t = s[1] << 9;

	s[2] ^= s[0];
	s[3] ^= s[1];
	s[1] ^= s[2];
	s[0] ^= s[3];

	s[2] ^= t;

	s[3] = prbs_rotl (s[3], 11);

	exit_io_critical_section (io,h);
	return result;
}

static void
wait_for_all_events (io_t *io) {
	io_event_t *event;
	io_alarm_t *alarm;
	do {
		ENTER_CRITICAL_SECTION(io);
		event = io->events;
		alarm = io->alarms;
		EXIT_CRITICAL_SECTION(io);
	} while (
			event != &s_null_io_event
		&&	alarm != &s_null_io_alarm
	);
}

static void
nrf52_log (io_t *io,char const *fmt,va_list va) {

}

static bool
nrf52_is_in_event_thread (io_t *io) {
	return ((nrf52840_io_t*) io)->in_event_thread;
}

static void
nrf52_wait_for_event (io_t *io) {
	__WFI();
}

static io_interrupt_handler_t cpu_interrupts[NUMBER_OF_INTERRUPT_VECTORS];

static void
null_interrupt_handler (void *w) {
	while(1);
}

static void	
nrf52_register_interrupt_handler (
	io_t *io,int32_t number,io_interrupt_action_t handler,void *user_value
) {
	io_interrupt_handler_t *i = (
		cpu_interrupts + number + NUMBER_OF_ARM_INTERRUPT_VECTORS
	);
	i->action = handler;
	i->user_value = user_value;
}

static bool	
nrf52_unregister_interrupt_handler (
	io_t *io,int32_t number,io_interrupt_action_t handler
) {
	io_interrupt_handler_t *i = (
		cpu_interrupts + number + NUMBER_OF_ARM_INTERRUPT_VECTORS
	);
	if (i->action == handler) {
		i->action = null_interrupt_handler;
		i->user_value = io;
		return true;
	} else {
		return false;
	}
}

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
nrf52_set_io_pin_interrupt (io_t *io,io_pin_t rpin) {
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

static void
nrf52_signal_task_pending (io_t *io) {
	// no action required
}

static bool
nrf52_enqueue_task (io_t *io,vref_t r_task) {
	nrf52840_io_t *this = (nrf52840_io_t*) io;
	return io_value_pipe_put_value (this->tasks,r_task);
}

static bool
nrf52_do_next_task (io_t *io) {
	nrf52840_io_t *this = (nrf52840_io_t*) io;
	vref_t r_task;
	if (io_value_pipe_get_value (this->tasks,&r_task)) {
		vref_t const *argv;
		uint32_t argc;
		
		if (io_vector_value_get_values (r_task,&argc,&argv)) {
			
			// ...
		
			return true;
		}
	}
	return false;
}

static io_time_t
nrf52_get_time (io_t *io) {
	nrf52840_io_t *this = (nrf52840_io_t*) io;
	return nrf_64bit_time_clock_get_time (&this->tc);
//	io_time_t t = {
//		.nanoseconds = read_64bit_count(&this->tc) * TIME_CLOCK_NS_PER_TICK
//	};
//	return t;
}

void
add_io_implementation_cpu_methods (io_implementation_t *io_i) {
	add_io_implementation_core_methods (io_i);

	io_i->is_first_run = nrf52840_io_config_is_first_run;
	io_i->clear_first_run = nrf52840_io_config_clear_first_run;
	io_i->get_byte_memory = nrf52_io_get_byte_memory;
	io_i->get_short_term_value_memory = nrf52_io_get_stvm;
	io_i->do_gc = nrf52_do_gc;
	io_i->get_random_u32 = nrf52_get_random_u32;
	io_i->get_next_prbs_u32 = nrf52_get_prbs_random_u32;
	io_i->signal_task_pending = nrf52_signal_task_pending;
	io_i->enqueue_task = nrf52_enqueue_task;
	io_i->do_next_task = nrf52_do_next_task;
	io_i->signal_event_pending = nrf52_signal_event_pending;
	io_i->enter_critical_section = nrf52_enter_critical_section;
	io_i->exit_critical_section = nrf52_exit_critical_section;
	io_i->in_event_thread = nrf52_is_in_event_thread;
	io_i->wait_for_event = nrf52_wait_for_event;
	io_i->get_time = nrf52_get_time,
	io_i->enqueue_alarm = nrf_time_clock_enqueue_alarm;
	io_i->dequeue_alarm = nrf_time_clock_dequeue_alarm;
	io_i->register_interrupt_handler = nrf52_register_interrupt_handler;
	io_i->unregister_interrupt_handler = nrf52_unregister_interrupt_handler;
	io_i->wait_for_all_events = wait_for_all_events;

	io_i->set_io_pin_output = nrf52_set_io_pin_to_output,
	io_i->set_io_pin_input = nrf52_set_io_pin_to_input,
	io_i->set_io_pin_interrupt = nrf52_set_io_pin_interrupt,
	io_i->set_io_pin_alternate = io_pin_nop,
	io_i->read_from_io_pin = nrf52_read_io_input_pin,
	io_i->write_to_io_pin = nrf52_write_to_io_pin,
	io_i->toggle_io_pin = nrf52_toggle_io_pin,
	io_i->valid_pin = nrf52_io_pin_is_valid,

	io_i->log = nrf52_log;
	io_i->panic = nrf52_panic;
}

static void
event_thread (void *io) {
	nrf52840_io_t *this = io;
	this->in_event_thread = true;
	while (next_io_event (io));
	this->in_event_thread = false;
}

void
initialise_cpu_io (io_t *io) {
	nrf52840_io_t *this = (nrf52840_io_t*) io;
	this->in_event_thread = false;
	register_io_interrupt_handler (
		io,EVENT_THREAD_INTERRUPT,event_thread,io
	);
	NVIC_EnableIRQ (EVENT_THREAD_INTERRUPT);
	
	start_nrf_time_clock (io,&this->tc);
}

static void apply_nrf_cpu_errata (void);

static void
initialise_ram_interrupt_vectors (void) {
	io_interrupt_handler_t *i = cpu_interrupts;
	io_interrupt_handler_t *e = i + NUMBER_OF_INTERRUPT_VECTORS;
	while (i < e) {
		i->action = null_interrupt_handler;
		i->user_value = NULL;
		i++;
	}
}

static void
handle_io_cpu_interrupt (void) {
	io_interrupt_handler_t const *interrupt = &cpu_interrupts[
		SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk
	];
	interrupt->action(interrupt->user_value);
}

static void
initialise_c_runtime (void) {
	extern uint32_t ld_start_of_sdata_in_flash;
	extern uint32_t ld_start_of_sdata_in_ram,ld_end_of_sdata_in_ram;
	extern uint32_t ld_start_of_bss,ld_end_of_bss;

	uint32_t *src = &ld_start_of_sdata_in_flash;
	uint32_t *dest = &ld_start_of_sdata_in_ram;

	while(dest < &ld_end_of_sdata_in_ram) *dest++ = *src++;
	dest = &ld_start_of_bss;
	while(dest < &ld_end_of_bss) *dest++ = 0;

	// fill stack/heap region of RAM with a pattern
	extern uint32_t ld_end_of_static_ram_allocations;
	uint32_t *end = (uint32_t*) __get_MSP();
	dest = &ld_end_of_static_ram_allocations;
	while (dest < end) {
		*dest++ = 0xdeadc0de;
	}
	
	initialise_ram_interrupt_vectors ();
}

#include <io_board.h>

int main(void);
void
nrf52_core_reset (void) {

	initialise_c_runtime ();
	apply_nrf_cpu_errata ();

	SCB->CPACR |= (
			(3UL << 10*2)	// set CP10 Full Access
		|	(3UL << 11*2)	// set CP11 Full Access 
	);
	
	NRF_NVMC->ICACHECNF = NVMC_ICACHECNF_CACHEEN_Msk;
	NRF_RNG->TASKS_START = 1;
	NVIC_SetPriorityGrouping(0);

	main ();
	while (1);
}

extern uint32_t ld_top_of_c_stack;
__attribute__ ((section(".isr_vector")))
const void* s_flash_vector_table[NUMBER_OF_INTERRUPT_VECTORS] = {
	&ld_top_of_c_stack,
	nrf52_core_reset,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,

	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
	handle_io_cpu_interrupt,
};


static bool errata_36(void);
static bool errata_66(void);
static bool errata_98(void);
static bool errata_103(void);
static bool errata_115(void);
static bool errata_120(void);
static bool errata_136(void);

void
apply_nrf_cpu_errata (void) {
	/* Enable SWO trace functionality. If ENABLE_SWO is not defined, SWO pin will be used as GPIO (see Product
	Specification to see which one). */
	#if defined (ENABLE_SWO)
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	NRF_CLOCK->TRACECONFIG |= CLOCK_TRACECONFIG_TRACEMUX_Serial << CLOCK_TRACECONFIG_TRACEMUX_Pos;
	NRF_P1->PIN_CNF[0] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
	#endif

    /* Enable Trace functionality. If ENABLE_TRACE is not defined, TRACE pins will be used as GPIOs (see Product
       Specification to see which ones). */
    #if defined (ENABLE_TRACE)
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        NRF_CLOCK->TRACECONFIG |= CLOCK_TRACECONFIG_TRACEMUX_Parallel << CLOCK_TRACECONFIG_TRACEMUX_Pos;
        NRF_P0->PIN_CNF[7]  = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        NRF_P1->PIN_CNF[0]  = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        NRF_P0->PIN_CNF[12] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        NRF_P0->PIN_CNF[11] = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
        NRF_P1->PIN_CNF[9]  = (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos);
    #endif
    
    /* Workaround for Errata 36 "CLOCK: Some registers are not reset when expected" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_36()){
        NRF_CLOCK->EVENTS_DONE = 0;
        NRF_CLOCK->EVENTS_CTTO = 0;
        NRF_CLOCK->CTIV = 0;
    }
    
    /* Workaround for Errata 66 "TEMP: Linearity specification not met with default settings" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_66()){
        NRF_TEMP->A0 = NRF_FICR->TEMP.A0;
        NRF_TEMP->A1 = NRF_FICR->TEMP.A1;
        NRF_TEMP->A2 = NRF_FICR->TEMP.A2;
        NRF_TEMP->A3 = NRF_FICR->TEMP.A3;
        NRF_TEMP->A4 = NRF_FICR->TEMP.A4;
        NRF_TEMP->A5 = NRF_FICR->TEMP.A5;
        NRF_TEMP->B0 = NRF_FICR->TEMP.B0;
        NRF_TEMP->B1 = NRF_FICR->TEMP.B1;
        NRF_TEMP->B2 = NRF_FICR->TEMP.B2;
        NRF_TEMP->B3 = NRF_FICR->TEMP.B3;
        NRF_TEMP->B4 = NRF_FICR->TEMP.B4;
        NRF_TEMP->B5 = NRF_FICR->TEMP.B5;
        NRF_TEMP->T0 = NRF_FICR->TEMP.T0;
        NRF_TEMP->T1 = NRF_FICR->TEMP.T1;
        NRF_TEMP->T2 = NRF_FICR->TEMP.T2;
        NRF_TEMP->T3 = NRF_FICR->TEMP.T3;
        NRF_TEMP->T4 = NRF_FICR->TEMP.T4;
    }
    
    /* Workaround for Errata 98 "NFCT: Not able to communicate with the peer" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_98()){
        *(volatile uint32_t *)0x4000568Cul = 0x00038148ul;
    }
    
    /* Workaround for Errata 103 "CCM: Wrong reset value of CCM MAXPACKETSIZE" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_103()){
        NRF_CCM->MAXPACKETSIZE = 0xFBul;
    }
    
    /* Workaround for Errata 115 "RAM: RAM content cannot be trusted upon waking up from System ON Idle or System OFF mode" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_115()){
        *(volatile uint32_t *)0x40000EE4 = (*(volatile uint32_t *)0x40000EE4 & 0xFFFFFFF0) | (*(uint32_t *)0x10000258 & 0x0000000F);
    }
    
    /* Workaround for Errata 120 "QSPI: Data read or written is corrupted" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_120()){
        *(volatile uint32_t *)0x40029640ul = 0x200ul;
    }
    
    /* Workaround for Errata 136 "System: Bits in RESETREAS are set when they should not be" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_136()){
        if (NRF_POWER->RESETREAS & POWER_RESETREAS_RESETPIN_Msk){
            NRF_POWER->RESETREAS =  ~POWER_RESETREAS_RESETPIN_Msk;
        }
    }
    
    /* Enable the FPU if the compiler used floating point unit instructions. __FPU_USED is a MACRO defined by the
     * compiler. Since the FPU consumes energy, remember to disable FPU use in the compiler if floating point unit
     * operations are not used in your code. */
    #if (__FPU_USED == 1)
        SCB->CPACR |= (3UL << 20) | (3UL << 22);
        __DSB();
        __ISB();
    #endif

    /* Configure NFCT pins as GPIOs if NFCT is not to be used in your code. If CONFIG_NFCT_PINS_AS_GPIOS is not defined,
       two GPIOs (see Product Specification to see which ones) will be reserved for NFC and will not be available as
       normal GPIOs. */
    #if defined (CONFIG_NFCT_PINS_AS_GPIOS)
        if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == (UICR_NFCPINS_PROTECT_NFC << UICR_NFCPINS_PROTECT_Pos)){
            NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            NRF_UICR->NFCPINS &= ~UICR_NFCPINS_PROTECT_Msk;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            NVIC_SystemReset();
        }
    #endif

    /* Configure GPIO pads as pPin Reset pin if Pin Reset capabilities desired. If CONFIG_GPIO_AS_PINRESET is not
      defined, pin reset will not be available. One GPIO (see Product Specification to see which one) will then be
      reserved for PinReset and not available as normal GPIO. */
    #if defined (CONFIG_GPIO_AS_PINRESET)
        if (((NRF_UICR->PSELRESET[0] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)) ||
            ((NRF_UICR->PSELRESET[1] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos))){
            NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            NRF_UICR->PSELRESET[0] = 18;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            NRF_UICR->PSELRESET[1] = 18;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            NVIC_SystemReset();
        }
    #endif

}


static bool errata_36(void)
{
    if (*(uint32_t *)0x10000130ul == 0x8ul){
        if (*(uint32_t *)0x10000134ul == 0x0ul){
            return true;
        }
        if (*(uint32_t *)0x10000134ul == 0x1ul){
            return true;
        }
        if (*(uint32_t *)0x10000134ul == 0x2ul){
            return true;
        }
    }
    
    return true;
}


static bool errata_66(void)
{
    if (*(uint32_t *)0x10000130ul == 0x8ul){
        if (*(uint32_t *)0x10000134ul == 0x0ul){
            return true;
        }
        if (*(uint32_t *)0x10000134ul == 0x1ul){
            return true;
        }
        if (*(uint32_t *)0x10000134ul == 0x2ul){
            return true;
        }
    }
    
    return true;
}


static bool errata_98(void)
{
    if (*(uint32_t *)0x10000130ul == 0x8ul){
        if (*(uint32_t *)0x10000134ul == 0x0ul){
            return true;
        }
    }
    
    return false;
}


static bool errata_103(void)
{
    if (*(uint32_t *)0x10000130ul == 0x8ul){
        if (*(uint32_t *)0x10000134ul == 0x0ul){
            return true;
        }
    }
    
    return false;
}


static bool errata_115(void)
{
    if (*(uint32_t *)0x10000130ul == 0x8ul){
        if (*(uint32_t *)0x10000134ul == 0x0ul){
            return true;
        }
    }
    
    return false;
}


static bool errata_120(void)
{
    if (*(uint32_t *)0x10000130ul == 0x8ul){
        if (*(uint32_t *)0x10000134ul == 0x0ul){
            return true;
        }
    }
    
    return false;
}


static bool errata_136(void)
{
    if (*(uint32_t *)0x10000130ul == 0x8ul){
        if (*(uint32_t *)0x10000134ul == 0x0ul){
            return true;
        }
        if (*(uint32_t *)0x10000134ul == 0x1ul){
            return true;
        }
        if (*(uint32_t *)0x10000134ul == 0x2ul){
            return true;
        }
    }
    
    return true;
}
#endif /* IMPLEMENT_IO_CPU */
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


UNIT_SETUP(setup_io_cpu_unit_test) {
	return VERIFY_UNIT_CONTINUE;
}

UNIT_TEARDOWN(teardown_io_cpu_unit_test) {
}

void
io_cpu_unit_test (V_unit_test_t *unit) {
	static V_test_t const tests[] = {
		test_io_events_1,
		test_io_events_2,
		test_time_clock_alarms_1,
		test_io_random_1,
		0
	};
	unit->name = "io cpu";
	unit->description = "io cpu unit test";
	unit->tests = tests;
	unit->setup = setup_io_cpu_unit_test;
	unit->teardown = teardown_io_cpu_unit_test;
}
#define IO_CPU_UNIT_TESTS \
	io_cpu_unit_test,\
	/**/
#else
#define IO_CPU_UNIT_TESTS
#endif /* IMPLEMENT_VERIFY_IO_CPU */
#endif /* io_cpu_H_ */
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

