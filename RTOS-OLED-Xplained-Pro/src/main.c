#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "doom.h"

#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

#define BUZZER          PIOD
#define BUZZER_ID       ID_PIOD
#define BUZZER_IDX     30
#define BUZZER_IDX_MASK (1u << BUZZER_IDX) // esse já está pronto.

/************************************************************************/
/* pins  config                                                          */
/************************************************************************/

/************************************************************************/
/* globals                                                              */
/************************************************************************/

/************************************************************************/
/* prototypes and types                                                 */
/************************************************************************/

typedef struct {
	uint tone;
	uint duration;
} Note;

void pin_toggle(Pio *pio, uint32_t mask);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq) ;
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) { }
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* recursos RTOS                                                        */
/************************************************************************/

void but1_callback(void);
void io_init(void);

QueueHandle_t xQueueNote;
SemaphoreHandle_t xSemaphoreRTT;
SemaphoreHandle_t xSemaphoreStart;

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/
void but1_callback(void) {
	xSemaphoreGiveFromISR(xSemaphoreStart, 0);
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
static void task_music(void *pvParameters) {
	
	// Definições
	Note notas;
	int tempo = 225;
	int notes = sizeof(melody) / sizeof(melody[0]) / 2;
	int wholenote = (60000 * 4) / tempo;
	
	// Rotina principal
	for(;;){
		int divider;
		int noteDuration;
		int thisNote;
		if (xSemaphoreTake(xSemaphoreStart, 500 / portTICK_PERIOD_MS) == pdTRUE){
			for (thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
				divider = melody[thisNote + 1];
				if (divider > 0) {
					noteDuration = (wholenote) / divider;
					} else if (divider < 0) {
					noteDuration = (wholenote) / abs(divider);
					noteDuration *= 1.5;
				}
				notas.tone = melody[thisNote];
				notas.duration = noteDuration;
				xQueueSend(xQueueNote, &notas,portMAX_DELAY);
			}
		}
	}
}

static void task_note(void *pvParameters) {

	Note notas;
	// Rotina principal
	for(;;){
		int tempo;
		int freq;
		if (xQueueReceive(xQueueNote, &notas, (TickType_t)500)) {
			freq = 4*notas.tone;
			tempo = notas.duration;
			TC_init(TC0, ID_TC1, 1, freq);
			tc_start(TC0, 1);
			RTT_init(1000,tempo,RTT_MR_ALMIEN);      //tempo/1000 = 

			if (xSemaphoreTake(xSemaphoreRTT,portMAX_DELAY) == pdTRUE){
			}
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/


static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);

	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);

}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		tc_stop(TC0,1);
		xSemaphoreGiveFromISR(xSemaphoreRTT, 0);
	}
}

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq) {
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);

	/** ATIVA clock canal 0 TC */
	if(ul_tcclks == 0 )
		pmc_enable_pck(PMC_PCK_6);

	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void TC1_Handler(void) {
	volatile uint32_t status = tc_get_status(TC0, 1);
	pin_toggle(BUZZER, BUZZER_IDX_MASK);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

void io_init(void){
	pmc_enable_periph_clk(BUT_1_PIO_ID);
	pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
	pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE,
	but1_callback);
	pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
	pio_get_interrupt_status(BUT_1_PIO);
	NVIC_EnableIRQ(BUT_1_PIO_ID);
	NVIC_SetPriority(BUT_1_PIO_ID, 4);
	
	pmc_enable_periph_clk(BUZZER_ID);
	pio_set_output(BUZZER,BUZZER_IDX_MASK,0,0,0);
	pio_configure(BUZZER, PIO_OUTPUT_0, BUZZER_IDX_MASK, PIO_DEFAULT);
}
/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	sysclk_init();
	board_init();
	configure_console();
	io_init();
	
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	xQueueNote= xQueueCreate(32, sizeof(Note));
	xSemaphoreRTT = xSemaphoreCreateBinary();
	xSemaphoreStart = xSemaphoreCreateBinary();
	
	if (xSemaphoreRTT == NULL) {
		printf("Falha em criar xSemaphore \n");
	}
	
	if (xSemaphoreStart == NULL) {
		printf("Falha em criar xSemaphore \n");
	}
	
	if (xQueueNote == NULL) {
		printf("Falha em criar xQueue Note \n");
	}

	/* Create task to control oled */
	if (xTaskCreate(task_music, "music", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create game task\r\n");
	}
	
	if (xTaskCreate(task_note, "note", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create game task\r\n");
	}

	vTaskStartScheduler();

	while(1){}

	return 0;
}
