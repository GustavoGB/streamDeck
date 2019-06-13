#include "asf.h"
#include "main.h"
#include <string.h>
#include "bsp/include/nm_bsp.h"
#include "socket/include/socket.h"
#include <stdio.h>
#include <assert.h>


#define TASK_RECEBE_STREAM_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_RECEBE_STREAM_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_POTENCIOMETRO_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_POTENCIOMETRO_STACK_PRIORITY        (tskIDLE_PRIORITY)

#define TASK_BOTOES_STACK_SIZE            (2*1024/sizeof(portSTACK_TYPE))
#define TASK_BOTOES_STACK_PRIORITY        (tskIDLE_PRIORITY)

// defines do Botao de start
#define BUTTSTART_PIO           PIOC
#define BUTTSTART_PIO_ID        ID_PIOC
#define BUTTSTART_PIO_IDX       17u
#define BUTTSTART_PIO_IDX_MASK  (1u << BUTTSTART_PIO_IDX)

#define BUTTSTART2_PIO           PIOC
#define BUTTSTART2_PIO_ID        ID_PIOC
#define BUTTSTART2_PIO_IDX       30u
#define BUTTSTART2_PIO_IDX_MASK  (1u << BUTTSTART2_PIO_IDX)

#define BUTTSTART3_PIO           PIOA
#define BUTTSTART3_PIO_ID        ID_PIOA
#define BUTTSTART3_PIO_IDX       4u
#define BUTTSTART3_PIO_IDX_MASK  (1u << BUTTSTART3_PIO_IDX)

#define LED_ACIONA_PIO           PIOD               // periferico que controla o LED
#define LED_ACIONA_PIO_ID        ID_PIOD               // ID do periférico PIOC (controla LED)
#define LED_ACIONA_PIO_IDX       21u                   // ID do LED no PIO
#define LED_ACIONA_PIO_IDX_MASK  (1u << LED_ACIONA_PIO_IDX)  // Mascara para CONTROLARMOS o LED

//FITA LED
#define FITA_AZUL_PIO           PIOA
#define FITA_AZUL_PIO_ID        ID_PIOA
#define FITA_AZUL_PIO_IDX       0u
#define FITA_AZUL_PIO_IDX_MASK  (1u << FITA_AZUL_PIO_IDX)

#define FITA_VERDE_PIO           PIOD
#define FITA_VERDE_PIO_ID        ID_PIOD
#define FITA_VERDE_PIO_IDX       28u
#define FITA_VERDE_PIO_IDX_MASK  (1u << FITA_VERDE_PIO_IDX)

#define FITA_VERMELHA_PIO           PIOA
#define FITA_VERMELHA_PIO_ID        ID_PIOA
#define FITA_VERMELHA_PIO_IDX       3u
#define FITA_VERMELHA_PIO_IDX_MASK  (1u << FITA_VERMELHA_PIO_IDX)

//BARRINHA LED VIEWS
//PINO 5
#define LED_PIO2           PIOA
#define LED_PIO_ID2        ID_PIOA
#define LED_PIO_IDX2       13u
#define LED_PIO_IDX_MASK2  (1u << LED_PIO_IDX2)
////PINO 6
#define LED_PIO3           PIOC
#define LED_PIO_ID3        ID_PIOC
#define LED_PIO_IDX3       19u
#define LED_PIO_IDX_MASK3  (1u << LED_PIO_IDX3)
//PINO 7
#define LED_PIO4           PIOA
#define LED_PIO_ID4        ID_PIOA
#define LED_PIO_IDX4       4u
#define LED_PIO_IDX_MASK4  (1u << LED_PIO_IDX4)
//PINO 8
#define LED_PIO5           PIOA
#define LED_PIO_ID5        ID_PIOA
#define LED_PIO_IDX5       3u
#define LED_PIO_IDX_MASK5  (1u << LED_PIO_IDX5)
//PINO 10
#define LED_PIO6           PIOA
#define LED_PIO_ID6        ID_PIOA
#define LED_PIO_IDX6       6u
#define LED_PIO_IDX_MASK6  (1u << LED_PIO_IDX6)
//PINO 12
#define LED_PIO7           PIOD
#define LED_PIO_ID7        ID_PIOD
#define LED_PIO_IDX7       25u
#define LED_PIO_IDX_MASK7  (1u << LED_PIO_IDX7)
// PINO 14
#define LED_PIO8           PIOD
#define LED_PIO_ID8        ID_PIOD
#define LED_PIO_IDX8       24u
#define LED_PIO_IDX_MASK8  (1u << LED_PIO_IDX8)
// PINO 16
#define LED_PIO9           PIOA
#define LED_PIO_ID9        ID_PIOA
#define LED_PIO_IDX9       24u
#define LED_PIO_IDX_MASK9  (1u << LED_PIO_IDX9)

#define AFEC_CHANNEL_POT_SENSOR 8 //PA19

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)


/** The conversion data value */
volatile uint32_t pot_ul_value = 0;



void usart_log(char* name, char* log);
void usart_put_string(Usart *usart, char str[]);

#define UART_COMM USART1

volatile long g_systimer = 0;

volatile Bool flag_b1 = false;
volatile Bool flag_b2 = false;
volatile Bool flag_b3 = false;

volatile Bool g1_is_conversion_done = false;

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);


/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
	
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate =		CONF_UART_BAUDRATE,
		.charlength =	CONF_UART_CHAR_LENGTH,
		.paritytype =	CONF_UART_PARITY,
		.stopbits =		CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

void butt1Callback(void){
	flag_b1 = true;
}

void butt2Callback(void){
	flag_b2 = true;
}

void butt3Callback(void){
	flag_b3 = true;
}

void pressed(char button, char header, char eop){
	
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, button);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, header);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM,eop);
	flag_b1 = false;
	flag_b2 = false;
	flag_b3 = false;
	
}

void mandaAnalogico(int valor ,char eop){
	char s[8];
	sprintf(s, "%04d", valor);
	while(!usart_is_tx_ready(UART_COMM));
    usart_write(UART_COMM, 'A');
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, s[0]);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, s[1]);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, s[2]);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM, s[3]);
	while(!usart_is_tx_ready(UART_COMM));
	usart_write(UART_COMM,eop);

}

void recebe_status_stream(char *s){

  uint i = 0;
  while(1){
    if(usart_serial_is_rx_ready(CONF_UART)){
      usart_serial_getchar(CONF_UART, &s[i]);
      if(s[i]=='X'){
        s[i] = NULL;
        i=0;
        break;
      }else{
        i++;
      }          
    }    
  } 

}



static void AFEC_pot_callback(void)
{
	pot_ul_value = afec_channel_get_value(AFEC0, AFEC_CHANNEL_POT_SENSOR);
	g1_is_conversion_done = true;
}

static void config_POT(void){
/*************************************
   * Ativa e configura AFEC
   *************************************/
  /* Ativa AFEC - 0 */
	afec_enable(AFEC0);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(AFEC0, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(AFEC0, AFEC_TRIG_SW);

	/* configura call back */
	afec_set_callback(AFEC0, AFEC_INTERRUPT_EOC_8,	AFEC_pot_callback, 1);

	/*** Configuracao específica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(AFEC0, AFEC_CHANNEL_POT_SENSOR, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	 down to 0.
	 */
	afec_channel_set_analog_offset(AFEC0, AFEC_CHANNEL_POT_SENSOR, 0x200);

	/***  Configura sensor de temperatura ***/
	//struct afec_temp_sensor_config afec_pot_sensor_cfg;

	//afec_temp_sensor_get_config_defaults(&afec_pot_sensor_cfg);
	//afec_temp_sensor_set_config(AFEC0, &afec_pot_sensor_cfg);

	/* Selecina canal e inicializa conversão */
	afec_channel_enable(AFEC0, AFEC_CHANNEL_POT_SENSOR);
}

static int32_t convert_adc_to_pot(int32_t ADC_value){

  int32_t ul_vol;
  int32_t ul_temp;

  /*
   * converte bits -> tensão (Volts)
   */
	ul_vol = ADC_value * VOLT_REF / (float) MAX_DIGITAL;

  /*
   * According to datasheet, The output voltage VT = 0.72V at 27C
   * and the temperature slope dVT/dT = 2.33 mV/C
   */
  
  return(ul_vol);
}

void config_console(void) {
	usart_serial_options_t config;
	config.baudrate = 115200;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
}
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	uint32_t channel = 1;

	/* Configura o PMC */
	/* O TimerCounter ? meio confuso
	o uC possui 3 TCs, cada TC possui 3 canais
	TC0 : ID_TC0, ID_TC1, ID_TC2
	TC1 : ID_TC3, ID_TC4, ID_TC5
	TC2 : ID_TC6, ID_TC7, ID_TC8
	*/
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  4Mhz e interrup?c?o no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura e ativa interrup?c?o no TC canal 0 */
	/* Interrup??o no C */
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);

	/* Inicializa o canal 0 do TC */
	tc_start(TC, TC_CHANNEL);
}
void TC1_Handler(void){
	volatile uint32_t ul_dummy;

	/****************************************************************
	* Devemos indicar ao TC que a interrup??o foi satisfeita.
	******************************************************************/
	ul_dummy = tc_get_status(TC0, 1);

	/* Avoid compiler warning */
	UNUSED(ul_dummy);
	afec_channel_enable(AFEC0, AFEC_CHANNEL_POT_SENSOR);
	afec_start_software_conversion(AFEC0);
}

void usart_put_string(Usart *usart, char str[]) {
	usart_serial_write_packet(usart, str, strlen(str));
}

int usart_get_string(Usart *usart, char buffer[], int bufferlen, int timeout_ms) {
	long timestart = g_systimer;
	uint32_t rx;
	uint32_t counter = 0;
	
	while(g_systimer - timestart < timeout_ms && counter < bufferlen - 1) {
		if(usart_read(usart, &rx) == 0) {
			//timestart = g_systimer; // reset timeout
			buffer[counter++] = rx;
		}
	}
	buffer[counter] = 0x00;
	return counter;
}

void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout) {
	usart_put_string(usart, buffer_tx);
	usart_get_string(usart, buffer_rx, bufferlen, timeout);
}

void usart_log(char* name, char* log) {
	usart_put_string(USART1, "[");
	usart_put_string(USART1, name);
	usart_put_string(USART1, "] ");
	usart_put_string(USART1, log);
	usart_put_string(USART1, "\r\n");
}



/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		printf("--- task ## %u", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(1000);
	}
}



void recebe_stream(void){
	
	//BARRINHA LED
	pmc_enable_periph_clk(LED_PIO_ID2);
	pmc_enable_periph_clk(LED_PIO_ID3);
	pmc_enable_periph_clk(LED_PIO_ID4);
	pmc_enable_periph_clk(LED_PIO_ID5);
	pmc_enable_periph_clk(LED_PIO_ID6);
	pmc_enable_periph_clk(LED_PIO_ID7);
	pmc_enable_periph_clk(LED_PIO_ID8);
	pmc_enable_periph_clk(LED_PIO_ID9);
	
	pio_set_output(LED_PIO2, LED_PIO_IDX_MASK2, 0, 0, 0);
	pio_set_output(LED_PIO3,LED_PIO_IDX_MASK3,0 ,0, 0);
	pio_set_output(LED_PIO4,LED_PIO_IDX_MASK4,0 ,0, 0);
	pio_set_output(LED_PIO5,LED_PIO_IDX_MASK5,0 ,0, 0);
	pio_set_output(LED_PIO6, LED_PIO_IDX_MASK6, 0, 0, 0);
	pio_set_output(LED_PIO7,LED_PIO_IDX_MASK7,0 ,0, 0);
	pio_set_output(LED_PIO8,LED_PIO_IDX_MASK8,0 ,0, 0);
	pio_set_output(LED_PIO9,LED_PIO_IDX_MASK9,0 ,0, 0);
	
	//FITA LED
	pio_set(FITA_VERDE_PIO, FITA_VERDE_PIO_IDX_MASK);
	pio_clear(FITA_AZUL_PIO, FITA_AZUL_PIO_IDX_MASK);
	pio_set(FITA_VERMELHA_PIO, FITA_VERMELHA_PIO_IDX_MASK);
	pio_clear(LED_ACIONA_PIO, LED_ACIONA_PIO_IDX_MASK); // Coloca 1 no pino LED verde
	
	//BARRINHA LED
	pio_set(LED_PIO2, LED_PIO_IDX_MASK2);
	pio_set(LED_PIO3, LED_PIO_IDX_MASK3);
	pio_set(LED_PIO4, LED_PIO_IDX_MASK4);
	pio_set(LED_PIO5, LED_PIO_IDX_MASK5);
	pio_set(LED_PIO6, LED_PIO_IDX_MASK6);
	pio_set(LED_PIO7, LED_PIO_IDX_MASK7);
	pio_set(LED_PIO8, LED_PIO_IDX_MASK8);
	pio_set(LED_PIO9, LED_PIO_IDX_MASK9);

	char status_stream[32];
	int vizu;
	
	while(1){
		recebe_status_stream(status_stream);
		vizu = atoi(&status_stream[1]);
		//IMPORTANTE -----------------------------------------------------------
		//1 led barrinha = LED_PIO2
		//2 led barrinha = LED_PIO3
		//3 led barrinha = LED_PIO8
		//4 led barrinha = LED_PIO7
		//5 led barrinha = LED_PIO6
		//----------------------------------------------------------------------
		
		if(status_stream[0] == 'L'){
			pio_set(LED_ACIONA_PIO, LED_ACIONA_PIO_IDX_MASK); // Coloca 1 no pino LED verde
			pio_clear(FITA_VERDE_PIO, FITA_VERDE_PIO_IDX_MASK);
			pio_set(FITA_AZUL_PIO, FITA_AZUL_PIO_IDX_MASK);
			pio_set(FITA_VERMELHA_PIO, FITA_VERMELHA_PIO_IDX_MASK);
			if(vizu==1){
				pio_set(LED_PIO2, LED_PIO_IDX_MASK2);
				pio_clear(LED_PIO3, LED_PIO_IDX_MASK3);
				pio_clear(LED_PIO4, LED_PIO_IDX_MASK4);
				pio_clear(LED_PIO5, LED_PIO_IDX_MASK5);
				pio_clear(LED_PIO6, LED_PIO_IDX_MASK6);
				pio_clear(LED_PIO7, LED_PIO_IDX_MASK7);
				pio_clear(LED_PIO8, LED_PIO_IDX_MASK8);
				pio_clear(LED_PIO9, LED_PIO_IDX_MASK9);
			}
			else if(vizu==2){
				pio_set(LED_PIO2, LED_PIO_IDX_MASK2);
				pio_set(LED_PIO3, LED_PIO_IDX_MASK3);
				pio_clear(LED_PIO4, LED_PIO_IDX_MASK4);
				pio_clear(LED_PIO5, LED_PIO_IDX_MASK5);
				pio_clear(LED_PIO6, LED_PIO_IDX_MASK6);
				pio_clear(LED_PIO7, LED_PIO_IDX_MASK7);
				pio_clear(LED_PIO8, LED_PIO_IDX_MASK8);
				pio_clear(LED_PIO9, LED_PIO_IDX_MASK9);
			}
			else if(vizu==3){
				pio_set(LED_PIO2, LED_PIO_IDX_MASK2);
				pio_set(LED_PIO3, LED_PIO_IDX_MASK3);
				pio_clear(LED_PIO4, LED_PIO_IDX_MASK4);
				pio_clear(LED_PIO5, LED_PIO_IDX_MASK5);
				pio_clear(LED_PIO6, LED_PIO_IDX_MASK6);
				pio_clear(LED_PIO7, LED_PIO_IDX_MASK7);
				pio_set(LED_PIO8, LED_PIO_IDX_MASK8);
				pio_clear(LED_PIO9, LED_PIO_IDX_MASK9);
			}
			else if(vizu==4){
				pio_set(LED_PIO2, LED_PIO_IDX_MASK2);
				pio_set(LED_PIO3, LED_PIO_IDX_MASK3);
				pio_set(LED_PIO4, LED_PIO_IDX_MASK4);
				pio_clear(LED_PIO5, LED_PIO_IDX_MASK5);
				pio_clear(LED_PIO6, LED_PIO_IDX_MASK6);
				pio_set(LED_PIO7, LED_PIO_IDX_MASK7);
				pio_set(LED_PIO8, LED_PIO_IDX_MASK8);
				pio_clear(LED_PIO9, LED_PIO_IDX_MASK9);
			}
			else if(vizu>=5){
				pio_set(LED_PIO2, LED_PIO_IDX_MASK2);
				pio_set(LED_PIO3, LED_PIO_IDX_MASK3);
				pio_set(LED_PIO4, LED_PIO_IDX_MASK4);
				pio_set(LED_PIO5, LED_PIO_IDX_MASK5);
				pio_set(LED_PIO6, LED_PIO_IDX_MASK6);
				pio_set(LED_PIO7, LED_PIO_IDX_MASK7);
				pio_set(LED_PIO8, LED_PIO_IDX_MASK8);
				pio_set(LED_PIO9, LED_PIO_IDX_MASK9);
			}
		}
		else if (status_stream[0] == 'N'){
			pio_clear(LED_PIO2, LED_PIO_IDX_MASK2);
			pio_clear(LED_PIO3, LED_PIO_IDX_MASK3);
			pio_clear(LED_PIO4, LED_PIO_IDX_MASK4);
			pio_clear(LED_PIO5, LED_PIO_IDX_MASK5);
			pio_clear(LED_PIO6, LED_PIO_IDX_MASK6);
			pio_clear(LED_PIO7, LED_PIO_IDX_MASK7);
			pio_clear(LED_PIO8, LED_PIO_IDX_MASK8);
			pio_clear(LED_PIO9, LED_PIO_IDX_MASK9);
			
			pio_set(LED_ACIONA_PIO, LED_ACIONA_PIO_IDX_MASK); // Coloca 1 no pino LED verde
			pio_set(FITA_VERDE_PIO, FITA_VERDE_PIO_IDX_MASK);
			pio_set(FITA_AZUL_PIO, FITA_AZUL_PIO_IDX_MASK);
			pio_clear(FITA_VERMELHA_PIO, FITA_VERMELHA_PIO_IDX_MASK);
		}
	}
}

void potenciometro_task(void){
	config_POT();
	afec_start_software_conversion(AFEC0);
	TC_init(TC0, ID_TC1, 1, 4);

	int ad_old;
	while(1){
		if (g1_is_conversion_done == true){
			g1_is_conversion_done = false;
			
			if(ad_old < pot_ul_value*0.95 || ad_old > pot_ul_value*1.05) {
				mandaAnalogico(pot_ul_value, 'X');
			}
			ad_old = pot_ul_value;
		}
		vTaskDelay(100);
	}
}

void botoes_task(void){
	/////FAZER POR CALLBACK SO EDGE
	pmc_enable_periph_clk(BUTTSTART_PIO_ID);
	pmc_enable_periph_clk(LED_ACIONA_PIO_ID);
	
	pmc_enable_periph_clk(FITA_AZUL_PIO_ID);
	pmc_enable_periph_clk(FITA_VERDE_PIO_ID);
	pmc_enable_periph_clk(FITA_VERMELHA_PIO_ID);

	pio_set_output(LED_ACIONA_PIO, LED_ACIONA_PIO_IDX_MASK, 0, 0, 0);
	
	//FITA LED
	pio_set_output(FITA_AZUL_PIO, FITA_AZUL_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(FITA_VERDE_PIO, FITA_VERDE_PIO_IDX_MASK, 0, 0, 0);
	pio_set_output(FITA_VERMELHA_PIO, FITA_VERMELHA_PIO_IDX_MASK, 0, 0, 0);

	pio_set_input(BUTTSTART_PIO,BUTTSTART_PIO_IDX_MASK,PIO_DEFAULT);
	pio_set_input(BUTTSTART2_PIO,BUTTSTART2_PIO_IDX_MASK,PIO_DEFAULT);
	pio_set_input(BUTTSTART3_PIO,BUTTSTART3_PIO_IDX_MASK,PIO_DEFAULT);

	// configura pino ligado ao bot?o como entrada com um pull-up.
	pio_pull_up(BUTTSTART_PIO, BUTTSTART_PIO_IDX_MASK, 1);
	pio_pull_up(BUTTSTART2_PIO, BUTTSTART2_PIO_IDX_MASK, 1);
	pio_pull_up(BUTTSTART3_PIO, BUTTSTART3_PIO_IDX_MASK, 1);

	pio_handler_set(BUTTSTART_PIO, BUTTSTART_PIO_ID, BUTTSTART_PIO_IDX_MASK, PIO_IT_RISE_EDGE, butt1Callback);
	pio_handler_set(BUTTSTART2_PIO, BUTTSTART2_PIO_ID, BUTTSTART2_PIO_IDX_MASK, PIO_IT_RISE_EDGE, butt2Callback);
	pio_handler_set(BUTTSTART3_PIO, BUTTSTART3_PIO_ID, BUTTSTART3_PIO_IDX_MASK, PIO_IT_RISE_EDGE, butt3Callback);
	
	pio_enable_interrupt(BUTTSTART_PIO, BUTTSTART_PIO_IDX_MASK);
	NVIC_EnableIRQ(BUTTSTART_PIO_ID);
	NVIC_SetPriority(BUTTSTART_PIO_ID, 4);
	
	pio_enable_interrupt(BUTTSTART2_PIO, BUTTSTART2_PIO_IDX_MASK);
	NVIC_EnableIRQ(BUTTSTART2_PIO_ID);
	NVIC_SetPriority(BUTTSTART2_PIO_ID, 4);
	
	pio_enable_interrupt(BUTTSTART3_PIO, BUTTSTART3_PIO_IDX_MASK);
	NVIC_EnableIRQ(BUTTSTART3_PIO_ID);
	NVIC_SetPriority(BUTTSTART3_PIO_ID, 4);
	
	char button1 = '0';
	char button2 = '0';
	char button3 = '0';

	char eop = 'X';
	char header1 = 'L';
	char header2 = 'M';
	char header3 = 'J';
	char buffer[1024];
	
	while(1) {
		if(flag_b1){
			pressed('1', header1, eop);
		}
		else if(flag_b2){
			pressed('1', header2, eop);
		}
		else if(flag_b3){
			pressed('1', header3, eop);
		}
		else{
			button1 = '0';
			button2 = '0';
		}
		vTaskDelay(100);
	}
}

int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	config_console();
	
			
	/* Create tasks */
	if (xTaskCreate(recebe_stream, "recebe_stream", 2*TASK_RECEBE_STREAM_STACK_SIZE, NULL, TASK_RECEBE_STREAM_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test recebe_stream task\r\n");
	}
	
	if (xTaskCreate(potenciometro_task, "potenciometro_task", TASK_POTENCIOMETRO_STACK_SIZE, NULL, TASK_POTENCIOMETRO_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test potenciometro_task task\r\n");
	}
	
	if (xTaskCreate(botoes_task, "botoes_task", TASK_RECEBE_STREAM_STACK_SIZE, NULL, TASK_RECEBE_STREAM_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test botoes_task task\r\n");
	}
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){

	}


	return 0;
	

}
