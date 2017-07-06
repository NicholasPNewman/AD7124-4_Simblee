#ifndef adspi_h
#define adspi_h

#include <Arduino.h>
#include "SPI.h"

#define ADSPI_CS              (21)
#define ADSPI_EXCLK           (14)

// read commands
#define COMM_R_STAT         (0x40)
#define COMM_R_CTRL         (0x41)
#define COMM_R_DATA         (0x42)
#define COMM_R_IO_CTRL1     (0x43)
#define COMM_R_IO_CTRL2     (0x44)
#define COMM_R_ID           (0x45)
#define COMM_R_ERR          (0x46)
#define COMM_R_ERR_EN       (0x47)
#define COMM_R_MCOUNT       (0x48)
#define COMM_R_CHANNEL      (0x49)
#define COMM_R_CONFIG       (0x59)
#define COMM_R_FILTER       (0x61)

// write commands
#define COMM_W_CTRL         (0x01)
#define COMM_W_IO_CTRL1     (0x03)
#define COMM_W_IO_CTRL2     (0x04)
#define COMM_W_ERR_EN       (0x07)
#define COMM_W_CHANNEL      (0x09)
#define COMM_W_CONFIG       (0x19)
#define COMM_W_FILTER       (0x21)
#define COMM_W_OFFSET       (0x29)
#define COMM_W_GAIN         (0x31)

// ADC control definitions / register locations
#define CTRL_CLK_SEL        (0x00)
#define CTRL_MODE           (0x02)
#define CTRL_POWER          (0x06)
#define CTRL_REF_EN         (0x08)
#define CTRL_CS_EN          (0x09)
#define CTRL_DATA_STATUS    (0x0A)
#define CTRL_CONT_READ      (0x0B)
#define CTRL_DOUT           (0x0C)

// channel definitions / register locations
#define CHN_AINM            (0x00)
#define CHN_AINP            (0x05)
#define CHN_SETUP           (0x0C)
#define CHN_ENABLE          (0x0F)

// function definitions

class adspiClass {
public:
  void      start(void);
  int       verify(void);
  int       comm(int command, int value); 
  uint8_t   status(void);
  uint8_t   err(void);
  uint16_t  getconfig(int setup_n);
  int       getfilter(int setup_n);
  int       data(void);
  void      channel_cfg(int channel_n, int c_en, int c_setup, int c_ainp, int c_ainm);
  void      setup_cfg(int setup_n);
  void      diag_cfg(int config_n);
  void      control_cfg(int clk_sel, int mode, int power, int ref_en, int cs_en, int data_status, int cont_read, int dout_rdy);
private:
  void      start_exclk(int pin);
  void      start_timer(void);
  void      TIMER_IRQHandler(void);

};

extern adspiClass adspi;

#endif