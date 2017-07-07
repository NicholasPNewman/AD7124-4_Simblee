#include "adspi.h"

adspiClass adspi;

void adspiClass::start(void){
  pinMode(ADSPI_CS, OUTPUT);
  start_exclk(ADSPI_EXCLK);
  start_timer();
}

int adspiClass::comm(int command, int value) 
{
  uint8_t dat_in;
  
  // take the SS pin low to select the chip:
  digitalWrite(ADSPI_CS, LOW);
  //  send in the address and value via SPI:
  SPI.transfer(command);
  dat_in = (SPI.transfer(value));
  // take the SS pin high to de-select the chip:
  digitalWrite(ADSPI_CS, HIGH); 

  return dat_in;
}

// @brief: reads ID register and returns 1 for correct response and 0 for incorrect
int adspiClass::verify() 
{
  uint8_t dat_in;
  uint8_t dat_valid = 4;
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_R_ID);
  if (SPI.transfer(0) == dat_valid) {
    digitalWrite(ADSPI_CS, HIGH);
    return 1;
  }
  else {
    digitalWrite(ADSPI_CS, HIGH);
    return 0;
  }
}

// @brief: returns the contents of the status register
uint8_t adspiClass::status() 
{
  uint8_t dat_in;
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_R_STAT);
  dat_in = SPI.transfer(0);
  digitalWrite(ADSPI_CS, HIGH);
  return dat_in;
}

// @brief: returns the contents of the error register
uint8_t adspiClass::err() 
{
  uint8_t dat_in;
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_R_ERR);
  dat_in = SPI.transfer(0);
  digitalWrite(ADSPI_CS, HIGH);
  return dat_in;  
}

// @breif: returns the contents of the config register (16 bit)
uint16_t adspiClass::getconfig(int setup_n)
{
  if ((setup_n < 0) || (setup_n > 7)) {
    return 0;
  }
  uint8_t dat_in1;
  uint8_t dat_in2;
  uint16_t dat_16;
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_R_CONFIG + setup_n);
  dat_in1 = SPI.transfer(0);
  dat_in2 = SPI.transfer(0);
  digitalWrite(ADSPI_CS, HIGH);
  dat_16 = (dat_in1 << 8) + dat_in2;
  return dat_16;  
}

// @breif: returns the contents of the filter register (24 bit)
int adspiClass::getfilter(int setup_n) 
{
  if ((setup_n < 0) || (setup_n > 7)) {
    return 0;
  }
  uint8_t dat_in1;
  uint8_t dat_in2;
  uint8_t dat_in3;
  int dat_24;
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_R_CONFIG + setup_n);
  dat_in1 = SPI.transfer(0);
  dat_in2 = SPI.transfer(0);
  dat_in3 = SPI.transfer(0);
  digitalWrite(ADSPI_CS, HIGH);
  dat_24 = (dat_in1 << 16) + (dat_in2 << 8) + dat_in3;
  return dat_24;  
}

// @breif: request data and read 24-bit converted value
int adspiClass::data() 
{
  uint8_t dat_in1;
  uint8_t dat_in2;
  uint8_t dat_in3;
  int dat_24;
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_R_DATA);
  dat_in1 = SPI.transfer(0);
  dat_in2 = SPI.transfer(0);
  dat_in3 = SPI.transfer(0);
  digitalWrite(ADSPI_CS, HIGH);
  dat_24 = (dat_in1 << 16) + (dat_in2 << 8) + dat_in3;
  return dat_24;  
}
// @brief: configure an individual channel
void adspiClass::channel_cfg(int channel_n, int c_en, int c_setup, int c_ainp, int c_ainm) 
{
  uint16_t ch_config;
  uint8_t  config_1;
  uint8_t  config_2;

  ch_config     = ((c_en     << CHN_ENABLE) ||    // 1 bit (enable the channel (max 4))
                   (c_setup  << CHN_SETUP)  ||    // 3 bit (choose the associated setup (0))
                   (c_ainp   << CHN_AINP)   ||    // 5 bit (choose AINP input)
                   (c_ainm   << CHN_AINM));       // 5 bit (choose AINM input)

  // split new config into two parts for sending
  config_1 = (ch_config >> 8);
  config_2 = (ch_config & (0xFF));

  // command and send
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_W_CHANNEL + channel_n);   // write to channel register base address plus offset
  SPI.transfer(config_1);
  SPI.transfer(config_2);
  digitalWrite(ADSPI_CS, HIGH);

  Serial.println("Channels configured.");
  
}

// @brief: configure an individual setup
void adspiClass::setup_cfg(int setup_n) 
{
  Serial.println("Setups configured.");
}

// @brief: configure system diagnostics
void adspiClass::diag_cfg(int config_n) 
{
  Serial.println("Diagnostics configured.");
}

// @brief: configure  ADC control
void adspiClass::control_cfg(int clk_sel, int mode, int power, int ref_en, int cs_en, int data_status,
                       int cont_read, int dout_rdy) 
{
  uint16_t ctrl_config;
  uint8_t  config_1;
  uint8_t  config_2;

  ctrl_config     = ((clk_sel     << CTRL_CLK_SEL)      ||    // 2 bit (11 external clk / 4) (other -> internal clk)
                     (mode        << CTRL_MODE)         ||    // 4 bit (see data sheet for mode options)
                     (power       << CTRL_POWER)        ||    // 2 bit (00 low) (01 mid) (10,11 full)
                     (ref_en      << CTRL_REF_EN)       ||    // 1 bit (allow interal reference access through REFOUT pin)
                     (cs_en       << CTRL_CS_EN)        ||    // 1 bit (keep 0 for normal op, see datasheet for 1)
                     (data_status << CTRL_DATA_STATUS)  ||    // 1 bit (enabled sets transmission of status register after data)
                     (cont_read   << CTRL_CONT_READ)    ||
                     (dout_rdy    << CTRL_DOUT));

  // split config into two parts for sending   
  config_1 = (ctrl_config >> 8);
  config_2 = (ctrl_config & (0xFF));

  // command and send
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_W_CTRL);
  SPI.transfer(config_1);
  SPI.transfer(config_2);
  digitalWrite(ADSPI_CS, HIGH);

  Serial.println("Controls configured.");
}

void adspiClass::start_exclk(int pin)
{
  pinMode(pin, OUTPUT);
  NRF_TIMER2->MODE        = TIMER_MODE_MODE_Timer;       // Set the timer in Timer Mode.
  NRF_TIMER2->PRESCALER   = 0;                          
  NRF_TIMER2->BITMODE     = TIMER_BITMODE_BITMODE_16Bit; // 16 bit mode.
  NRF_TIMER2->TASKS_CLEAR = 1;
  NRF_TIMER2->CC[0]       = 2;
  NRF_TIMER2->EVENTS_COMPARE[0] = 0;
  NRF_TIMER2->SHORTS      = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
  NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
  NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];
  NRF_PPI->CHEN = (PPI_CHEN_CH0_Enabled << PPI_CHEN_CH0_Pos);
  
  nrf_gpiote_task_config(0, pin, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);

  NRF_TIMER2->TASKS_START = 1;
}

void adspiClass::start_timer(void)
{
  NRF_TIMER0->TASKS_STOP = 1;               // stop timer
  NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer; // set to Timer mode
  NRF_TIMER0->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);
  NRF_TIMER0->PRESCALER = 4;                // 16MHz / (2 ^ prescaler), adj clk freq
  NRF_TIMER0->TASKS_CLEAR = 1;              // clear timer
  NRF_TIMER0->CC[0] = 95;
  NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
  NRF_TIMER0->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
  
  // current unresolved issue line, TIMER_IRQHandler doesn't point properly to l#224 below
  // dynamic_attachInterrupt(TIMER0_IRQn, TIMER_IRQHandler);
  NRF_TIMER0->TASKS_START = 1;
}

void TIMER_IRQHandler(void){
  if (NRF_TIMER0->EVENTS_COMPARE[0])
  {
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NRF_TIMER0->TASKS_CLEAR = 1;
    data();
  }
}