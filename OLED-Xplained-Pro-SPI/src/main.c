#include <asf.h>

#include "oled/gfx_mono_ug_2832hsweg04.h"
#include "oled/gfx_mono_text.h"
#include "oled/sysfont.h"

#define LED_1_PIO PIOA
#define LED_1_PIO_ID ID_PIOA
#define LED_1_IDX 0
#define LED_1_IDX_MASK (1 << LED_1_IDX)

#define LED_2_PIO PIOC
#define LED_2_PIO_ID ID_PIOC
#define LED_2_IDX 30
#define LED_2_IDX_MASK (1 << LED_2_IDX)

#define LED_3_PIO PIOB
#define LED_3_PIO_ID ID_PIOB
#define LED_3_IDX 2
#define LED_3_IDX_MASK (1 << LED_3_IDX)

#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

#define BUT_2_PIO  PIOC
#define BUT_2_PIO_ID   ID_PIOC
#define BUT_2_IDX  31
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

#define BUT_3_PIO  PIOA
#define BUT_3_PIO_ID   ID_PIOA
#define BUT_3_IDX  19
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)

//#################################################
//#	              GLOBAL VARS                     #
//#################################################

char global_string [128];
char timer_string [8];
int buffer_senha [6] = {0,0,0,0,0,0};
int gabarito_senha [6] = {1,1,2,2,3,1};
int senha_counter = 0;
volatile bloqueado_flag = 0;
volatile aberto_flag = 0;
volatile checa_senha_flag = 0;
int error_counter = 0;
int timer = 0;
int tc_counter = 0;


void draw_senha(void);
void clean_display(void);
void update_display(int x, int y);
void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq);


//#################################################
//#	                FUNCTIONS                     #
//#################################################

void but1_callback(void) {
	if (!pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK)){
		if(!bloqueado_flag && senha_counter<6){
			buffer_senha[senha_counter] = 1;
			gfx_mono_draw_string("*", (32+(senha_counter-1)*16), 16, &sysfont);
			senha_counter += 1;
			if (senha_counter>=6){
				checa_senha_flag = 1;
			}
		}
		if (aberto_flag){
			TC_init(TC1, ID_TC3, 0, 5);
			tc_start(TC1, 0);
			tc_counter = 0;
		}
	}
	if (pio_get(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK)){
		if (aberto_flag && tc_counter>3){
			tc_stop(TC1, 0);
			tc_counter = 0;
			clean_display();
			senha_counter = 0;
			sprintf(global_string, "Cofre Fechado");
			update_display(0,0);
			aberto_flag = 0;
			pio_clear(LED_1_PIO, LED_1_IDX_MASK);
			pio_clear(LED_2_PIO, LED_2_IDX_MASK);
			pio_clear(LED_3_PIO, LED_3_IDX_MASK);
		}
	}
}

void but2_callback(void) {
	if(!bloqueado_flag && senha_counter<6){
		buffer_senha[senha_counter] = 2;
		gfx_mono_draw_string("*", (32+(senha_counter-1)*16), 16, &sysfont);
		senha_counter += 1;
		if (senha_counter>=6){
			checa_senha_flag = 1;
		}
	}
}

void but3_callback(void) {
	if(!bloqueado_flag && senha_counter<6){
		buffer_senha[senha_counter] = 3;
		gfx_mono_draw_string("*", (32+(senha_counter-1)*16), 16, &sysfont);
		senha_counter += 1;
		if (senha_counter>=6){
			checa_senha_flag = 1;
		}
	}
}

void pin_toggle(Pio *pio, uint32_t mask) {
	if(pio_get_output_data_status(pio, mask))
	pio_clear(pio, mask);
	else
	pio_set(pio,mask);
}

int checa_senha(void){
	for (int i=0; i<6; i++){
		if (buffer_senha[i]!=gabarito_senha[i]){
			return 0;
		}
	}	
	return 1;
}
	
void clean_display(void){
	gfx_mono_generic_draw_filled_rect(0, 0, 128, 32, 0);
}
void update_display(int x, int y){
	gfx_mono_draw_string(global_string, x, y, &sysfont);
}


void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void TC1_Handler(void) {
	volatile uint32_t status = tc_get_status(TC0, 1);
	pin_toggle(LED_1_PIO, LED_1_IDX_MASK);
	pin_toggle(LED_2_PIO, LED_2_IDX_MASK);
	pin_toggle(LED_3_PIO, LED_3_IDX_MASK);
}

void TC3_Handler(void) {
	volatile uint32_t status = tc_get_status(TC1, 0);
	tc_counter += 1;
}

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

	/* Get RTT status - ACK */
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		//modifica toda vez que o alarme for acionado
		bloqueado_flag = 0;
		senha_counter = 0;
		tc_stop(TC0, 1);
		sprintf(global_string, "Cofre Fechado");
		update_display(0,0);
		
	}
	
	/* IRQ due to Time has changed */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		// modifica a cada iteração
	}
}


void io_init(void) {
  pmc_enable_periph_clk(LED_1_PIO_ID);
  pmc_enable_periph_clk(LED_2_PIO_ID);
  pmc_enable_periph_clk(LED_3_PIO_ID);
  pmc_enable_periph_clk(BUT_1_PIO_ID);
  pmc_enable_periph_clk(BUT_2_PIO_ID);
  pmc_enable_periph_clk(BUT_3_PIO_ID);

  pio_configure(LED_1_PIO, PIO_OUTPUT_0, LED_1_IDX_MASK, PIO_DEFAULT);
  pio_configure(LED_2_PIO, PIO_OUTPUT_0, LED_2_IDX_MASK, PIO_DEFAULT);
  pio_configure(LED_3_PIO, PIO_OUTPUT_0, LED_3_IDX_MASK, PIO_DEFAULT);

  pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);

  pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_EDGE,
  but1_callback);
  pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_FALL_EDGE,
  but2_callback);
  pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE,
  but3_callback);

  pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
  pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
  pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);

  pio_get_interrupt_status(BUT_1_PIO);
  pio_get_interrupt_status(BUT_2_PIO);
  pio_get_interrupt_status(BUT_3_PIO);

  NVIC_EnableIRQ(BUT_1_PIO_ID);
  NVIC_SetPriority(BUT_1_PIO_ID, 4);

  NVIC_EnableIRQ(BUT_2_PIO_ID);
  NVIC_SetPriority(BUT_2_PIO_ID, 4);

  NVIC_EnableIRQ(BUT_3_PIO_ID);
  NVIC_SetPriority(BUT_3_PIO_ID, 4);
}

int main(void) {
  board_init();
  sysclk_init();
  delay_init();
  io_init();
  gfx_mono_ssd1306_init();
  
  sprintf(global_string, "Cofre Fechado");
  update_display(0,0);
  
  while (1) {
	if (checa_senha_flag){
		if (checa_senha()){
			aberto_flag = 1;
			error_counter = 0;
		}else{
			error_counter += 1;
			if (error_counter>=2){
				bloqueado_flag = 1;
				TC_init(TC0, ID_TC1, 1, 5);
				tc_start(TC0, 1);
				clean_display();
				sprintf(global_string, "BLOQUEADO");
				update_display(20, 0);
				RTT_init(1, 10, RTT_MR_ALMIEN);
				error_counter=0;
			}else{
				clean_display();
				sprintf(global_string, "Senha Errada");
				update_display(0,0);
				senha_counter = 0;
			}
		}
		checa_senha_flag = 0;
	}
	if (aberto_flag){
		clean_display();
		sprintf(global_string, "Aberto");
		update_display(40, 0);
		pio_set(LED_1_PIO, LED_1_IDX_MASK);
		pio_set(LED_2_PIO, LED_2_IDX_MASK);
		pio_set(LED_3_PIO, LED_3_IDX_MASK);
	}
	pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
  }
}
