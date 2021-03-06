#include "adspi.h"

adspiClass adspi;

struct ad7124_device my_ad7124;                    /* A new driver instance */
struct ad7124_device *ad7124_handler = &my_ad7124; /* A driver handle to pass around */

int regInt;                                        /* Variable to iterate through registers */
long timeout = 1000;                               /* Number of tries before a function times out */
long ret = 0;                                      /* Return value */
long sample;                                       /* Stores raw value read from the ADC */

void adspiClass::begin(void){
  pinMode(ADSPI_CS, OUTPUT);
  start_exclk(ADSPI_EXCLK);
  //start_timer();

  ret = AD7124_Setup(ad7124_handler, AD7124_SLAVE_ID, (ad7124_st_reg *)&ad7124_regs);
  if (ret < 0)
  {
    /* AD7124 initialization failed, check the value of ret! */
    Serial.print("Initialization failed, error code: ");
    Serial.println(ret);
  }
  else
  {
    //Serial.println("Initialization successful.");
  } 
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

void adspiClass::reset(void)
{
  int i = 0;
  
  digitalWrite(ADSPI_CS, LOW);
  for (i = 0; i < 8; i++){
    SPI.transfer(0xF);
  }
  digitalWrite(ADSPI_CS, HIGH);
}

// @brief: reads ID register and returns 1 for correct response and 0 for incorrect
int adspiClass::verify() 
{
  uint8_t dat_in;
  uint8_t dat_valid = 4;
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_R_ID);
  dat_in = SPI.transfer(0);
  return (dat_in == dat_valid);
  digitalWrite(ADSPI_CS, HIGH);
}

// @brief: returns the contents of the status register
uint8_t adspiClass::status(void) 
{
  uint8_t dat_in;
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_R_STAT);
  dat_in = SPI.transfer(0);
  digitalWrite(ADSPI_CS, HIGH);
  return dat_in;
}

// @breif: returns the contents of the control register (16 bit)
uint16_t adspiClass::control(void)
{
  uint8_t dat_in1;
  uint8_t dat_in2;
  uint16_t dat_16;
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_R_CTRL);
  dat_in1 = SPI.transfer(0);
  dat_in2 = SPI.transfer(0);
  digitalWrite(ADSPI_CS, HIGH);
  dat_16 = (dat_in1 << 8) + dat_in2;
  return dat_16;  
}

// @brief: returns the contents of a channel register
uint8_t adspiClass::channels(int channel_n) 
{
  uint8_t dat_in;
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_R_CHANNEL + channel_n);
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

// @breif: request data and read 24-bit converted value DEPRECATED
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

// @breif: writes to the control register to set up continuous read mode
// TODO: OR 0x800 into current control val to accomodate new setups
// ALWAYS follow with desired number of data_cont_read fxs
void adspiClass::setup_cont_read(void)
{
  int16_t command = 0x800;
  digitalWrite(ADSPI_CS, LOW);
  SPI.transfer(COMM_W_CTRL);
  SPI.transfer16(command);
}


// @brief: read data during CS low part of continuous read
// ONLY USE during continuous read operation, special behavior of CS pin
int adspiClass::data_cont_read() 
{
  uint8_t dat_in1;
  uint8_t dat_in2;
  uint8_t dat_in3;
  int dat_24;
  dat_in1 = SPI.transfer(0);
  dat_in2 = SPI.transfer(0);
  dat_in3 = SPI.transfer(0);
  dat_24 = (dat_in1 << 16) | (dat_in2 << 8) | dat_in3;
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
  
}

// @brief: configure an individual setup
void adspiClass::setup_cfg(int setup_n) 
{
  // Deprecated. Use RegEdit
}

// @brief: configure system diagnostics
void adspiClass::diag_cfg(int config_n) 
{
  // Deprecated. Use RegEdit
}

// @brief: configure  ADC control
// Used to bypass RegEdit for faster control register writes. Use datasheet for inputs.
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
}

// @brief: digital code to voltage: differential.
// translates the conversion code into a voltage. Accurate to .001V at gain = 1. Full scale configuration improves this number.
double adspiClass::D2V_Diff(int32_t val)
{
  double bits = pow(2,23);
  double V_ref = 3.3;
  double gain = 1;
 
  return ((val/bits) - 1)*(V_ref / gain);
}

// @brief: digital code to voltage: single-ended.
// translates the conversion code into a voltage. Accurate to .001V at gain = 1. Full scale configuration improves this number.
double adspiClass::D2V_Sing(int32_t val)
{
  double bits = pow(2,24);
  double V_ref = 3.3;
  double gain = 1;

  return ((val * V_ref) / (bits * gain));
}

// @breif: reads a single register and updates its value in the AD7124_regs struct
// returns error code (>0 success)
int32_t adspiClass::update_reg_val(ad7124_st_reg* pReg)
{
  ret = AD7124_ReadRegister(ad7124_handler, pReg);
  return ret;
}

// @brief: prints the contents of the AD7124_regs struct. Does not update registers,
// user after read_regs() to ensure accurate reporting
void adspiClass::print_regs(void)
{
  for (regInt = AD7124_Status; (regInt < AD7124_REG_NO) && !(ret < 0); regInt++)
  {   
    ad7124_registers regNr = static_cast<ad7124_registers>(regInt);
    int dummy = ad7124_regs[regNr].value;
    Serial.print(regInt, HEX);
    Serial.print(" : 0x");
    Serial.println(dummy, HEX);
  }
}

// @breif: reads all the register contents of the AD7124. Updates the
// AD7124_regs struct, best used in concert with print_regs().
void adspiClass::read_regs(void)
{
  for (regInt = AD7124_Status; (regInt < AD7124_REG_NO) && !(ret < 0); regInt++)
  {
    ad7124_registers regNr = static_cast<ad7124_registers>(regInt);
    ret = AD7124_ReadRegister(ad7124_handler, &ad7124_regs[regNr]);
    if (ret < 0) {
      Serial.print(regInt);
      Serial.print(", error code: ");
      Serial.println(ret);
    }
  }
}

// @brief: in continuous-conversion mode, monitors the status register for data_ready pin active
// then reads the data register. Prints via serial monitor.
void adspiClass::print_data_wStatus(void)
{
    ret = AD7124_WaitForConvReady(ad7124_handler, timeout);
    if (0)//ret < 0)
    {
        Serial.print("ConvReady error :");
        Serial.println(ret);
    }
    ret = AD7124_ReadData(ad7124_handler, &sample);
    if (ret < 0)
    {
      Serial.print("ReadData error :");
      Serial.println(ret);
    }
    else
    {
      //Serial.println(ad7124_regs[AD7124_Data].value, HEX);
      Serial.println(D2V_Sing(ad7124_regs[AD7124_Data].value), 7);
    } 
}

// @brief: in continuous-conversion mode, reads the data register, no check. 
// Best used within DRDY pin pin interrupt handler. Prints via serial monitor.
void adspiClass::print_data_nStatus(void)
{
    ret = AD7124_NoCheckReadRegister(ad7124_handler, &ad7124_regs[AD7124_Data]);
    if (ret < 0)
    {
      Serial.print("ReadData error :");
      Serial.println(ret);
    }
    else
    {
      Serial.println(D2V_Sing(ad7124_regs[AD7124_Data].value), 7);
    } 
}

// @breif: edits the AD7124_regs struct to configure PGA to preset values.
// accepts gain arguments of (1, 2, 4... ...64, 128). Use outside of continuous-reads.
int adspiClass::SetPGA(int gain, int setup)
{
  int code = 0;
  uint32_t command = 0;
  
  if ((setup < 0) || (setup > 8)){
    return (-3);
  }

  ad7124_registers regNr = static_cast<ad7124_registers>(AD7124_CFG0_REG);

  switch (gain) {
    case (1):
    {
      code = 0;
      break;
    }
    case (2):
    {
      code = 1;
      break;
    }
    case (4):
    {
      code = 2;
      break;
    }
    case (8):
    {
      code = 3;
      break;
    }
    case (16):
    {
      code = 4;
      break;
    }
    case (32):
    {
      code = 5;
      break;
    }
    case (64):
    {
      code = 6;
      break;
    }
    case (128):
    {
      code = 7;
      break;
    }
    default:
      return (-4);
  }
  
  command = ((ad7124_regs[AD7124_CFG0_REG + setup].value & 0xFFF8) | code);
  ad7124_regs[regNr].value = command;
  return AD7124_WriteRegister(ad7124_handler, ad7124_regs[regNr]);

}

// @brief: starts the 4MHz external clock (1MHz IC internally divided). Default output via pin 14.
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

// @brief: start timer and interrupt handlers. Default 10kHz interrupt.
// odd error when compiled outside of Arduino IDE, to use insert function without adspiClass:: header into sketch
 
// void adspiClass::start_timer(void){
//   NRF_TIMER0->TASKS_STOP = 1;               // stop timer
//   NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer; // set to Timer mode
//   NRF_TIMER0->BITMODE = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);
//   NRF_TIMER0->PRESCALER = 4;                // 16MHz / (2 ^ prescaler), adj clk freq
//   NRF_TIMER0->TASKS_CLEAR = 1;              // clear timer
//   NRF_TIMER0->CC[0] = 95;
//   NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;
//   NRF_TIMER0->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
//   dynamic_attachInterrupt(TIMER0_IRQn, TIMER_IRQHandler);
//   NRF_TIMER0->TASKS_START = 1;
// }

// @brief: timer interrupt handler. 2.6us startup time, ~1us latency.

// static void adspiClass::TIMER_IRQHandler(void){
//   if (NRF_TIMER0->EVENTS_COMPARE[0])
//   {
//     NRF_TIMER0->EVENTS_COMPARE[0] = 0;
//     NRF_TIMER0->TASKS_CLEAR = 1;
//   }
// }
