/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>



// defines do Botao de start
#define BUTTSTART_PIO           PIOC
#define BUTTSTART_PIO_ID        ID_PIOC
#define BUTTSTART_PIO_IDX       17u
#define BUTTSTART_PIO_IDX_MASK  (1u << BUTTSTART_PIO_IDX)

#define BUTTSTART2_PIO           PIOC
#define BUTTSTART2_PIO_ID        ID_PIOC
#define BUTTSTART2_PIO_IDX       30u
#define BUTTSTART2_PIO_IDX_MASK  (1u << BUTTSTART2_PIO_IDX)

#define LED_VERDE_PIO           PIOD               // periferico que controla o LED
#define LED_VERDE_PIO_ID        ID_PIOD               // ID do periférico PIOC (controla LED)
#define LED_VERDE_PIO_IDX       21u                   // ID do LED no PIO
#define LED_VERDE_PIO_IDX_MASK  (1u << LED_VERDE_PIO_IDX)  // Mascara para CONTROLARMOS o LED

#define AFEC_CHANNEL_POT_SENSOR 8 //PA19

/** Reference voltage for AFEC,in mv. */
#define VOLT_REF        (3300)

/** The maximal digital value */
/** 2^12 - 1                  */
#define MAX_DIGITAL     (4095)


/** The conversion data value */
volatile uint32_t pot_ul_value = 0;


// Descomente o define abaixo, para desabilitar o Bluetooth e utilizar modo Serial via Cabo
//#define DEBUG_SERIAL

void usart_log(char* name, char* log);
void usart_put_string(Usart *usart, char str[]);

#ifdef DEBUG_SERIAL
#define UART_COMM USART1
#else
#define UART_COMM USART0
#endif

volatile long g_systimer = 0;

volatile Bool flag_b1 = false;
volatile Bool flag_b2 = false;

volatile Bool g1_is_conversion_done = false;


void butt1Callback(void){
	flag_b1 = true;
}

void butt2Callback(void){
	flag_b2 = true;
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

char recebe_status_stream(){
	char buffer[32];
	char status;
	if(!usart_read(UART_COMM, &status)){
		//sprintf(buffer, "dado: %d", status);
		//usart_put_string(USART1, buffer);
	}
	return status;
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
void SysTick_Handler() {
	g_systimer++;
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
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART1, &config);
	usart_enable_tx(USART1);
	usart_enable_rx(USART1);
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

void hc05_config_server(void) {
	usart_serial_options_t config;
	config.baudrate = 9600;
	config.charlength = US_MR_CHRL_8_BIT;
	config.paritytype = US_MR_PAR_NO;
	config.stopbits = false;
	usart_serial_init(USART0, &config);
	usart_enable_tx(USART0);
	usart_enable_rx(USART0);
	
	 // RX - PB0  TX - PB1 
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 0), PIO_DEFAULT);
	 pio_configure(PIOB, PIO_PERIPH_C, (1 << 1), PIO_DEFAULT);
}

int hc05_server_init(void) {
	char buffer_rx[128];
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);	
	usart_send_command(USART0, buffer_rx, 1000, "AT+NAMEBOLHA1", 1000);
	usart_log("hc05_server_init", buffer_rx);
	usart_send_command(USART0, buffer_rx, 1000, "AT", 1000);
	usart_send_command(USART0, buffer_rx, 1000, "AT+PIN0000", 1000);
	usart_log("hc05_server_init", buffer_rx);
}


int main (void)
{
	board_init();
	sysclk_init();
	delay_init();
	SysTick_Config(sysclk_get_cpu_hz() / 1000); // 1 ms
	config_console();
	
	#ifndef DEBUG_SERIAL
	usart_put_string(USART1, "Inicializando...\r\n");
	usart_put_string(USART1, "Config HC05 Server...\r\n");
	hc05_config_server();
	hc05_server_init();
	#endif
	
	/////FAZER POR CALLBACK SO EDGE
	pmc_enable_periph_clk(BUTTSTART_PIO_ID);
	pmc_enable_periph_clk(LED_VERDE_PIO_ID);

	pio_set_output(LED_VERDE_PIO, LED_VERDE_PIO_IDX_MASK, 0, 0, 0);

	pio_set_input(BUTTSTART_PIO,BUTTSTART_PIO_IDX_MASK,PIO_DEFAULT);
	pio_set_input(BUTTSTART2_PIO,BUTTSTART2_PIO_IDX_MASK,PIO_DEFAULT);

	// configura pino ligado ao bot?o como entrada com um pull-up.
	pio_pull_up(BUTTSTART_PIO, BUTTSTART_PIO_IDX_MASK, 1);
	pio_pull_up(BUTTSTART2_PIO, BUTTSTART2_PIO_IDX_MASK, 1);
		
	pio_handler_set(BUTTSTART_PIO, BUTTSTART_PIO_ID, BUTTSTART_PIO_IDX_MASK, PIO_IT_RISE_EDGE, butt1Callback);
	pio_handler_set(BUTTSTART2_PIO, BUTTSTART2_PIO_ID, BUTTSTART2_PIO_IDX_MASK, PIO_IT_RISE_EDGE, butt2Callback);

	
	pio_enable_interrupt(BUTTSTART_PIO, BUTTSTART_PIO_IDX_MASK);
	NVIC_EnableIRQ(BUTTSTART_PIO_ID);
	NVIC_SetPriority(BUTTSTART_PIO_ID, 4);
	
	pio_enable_interrupt(BUTTSTART2_PIO, BUTTSTART2_PIO_IDX_MASK);
	NVIC_EnableIRQ(BUTTSTART2_PIO_ID);
	NVIC_SetPriority(BUTTSTART2_PIO_ID, 4);
	
	
	char button1 = '0';
	char button2 = '0'; 

	char eop = 'X';
	char header1 = 'L';
	char header2 = 'M';
	char buffer[1024];
	int ad_old;
	char status_stream;
	
	config_POT();
	afec_start_software_conversion(AFEC0);
	TC_init(TC0, ID_TC1, 1, 4);
		
	while(1) {
		if(flag_b1){
			pressed('1', header1, eop);

		}
		else if(flag_b2){
			pressed('1', header2, eop);

		}	
		else{
			button1 = '0';
			button2 = '0';
		}
		if (g1_is_conversion_done == true){
			g1_is_conversion_done = false;
			
 			if(ad_old < pot_ul_value*0.95 || ad_old > pot_ul_value*1.05)
				mandaAnalogico(pot_ul_value, 'X');
			ad_old = pot_ul_value;
		}
		
		status_stream = recebe_status_stream();
		
		if(status_stream == 'L'){
			pio_set(LED_VERDE_PIO, LED_VERDE_PIO_IDX_MASK); // Coloca 1 no pino LED verde

		}
		else if (status_stream == 'N'){
			pio_clear(LED_VERDE_PIO, LED_VERDE_PIO_IDX_MASK); // Coloca 0 no pino LED verde

		}	
	}
}