#include "ads1299-x.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
/**@headers for �s delay:*/
#include "compiler_abstraction.h"
#include "nrf.h"
#include <stdio.h>

#define SPI_INSTANCE 0

//NOTE: ADS1299 Default Registers
uint8_t ads1299_default_registers[] = {
    ADS1299_REGDEFAULT_CONFIG1,
    ADS1299_REGDEFAULT_CONFIG2,
    ADS1299_REGDEFAULT_CONFIG3,
    ADS1299_REGDEFAULT_LOFF,
    ADS1299_REGDEFAULT_CH1SET,
    ADS1299_REGDEFAULT_CH2SET,
    ADS1299_REGDEFAULT_CH3SET,
    ADS1299_REGDEFAULT_CH4SET,
    ADS1299_REGDEFAULT_CH5SET,
    ADS1299_REGDEFAULT_CH6SET,
    ADS1299_REGDEFAULT_CH7SET,
    ADS1299_REGDEFAULT_CH8SET,
    ADS1299_REGDEFAULT_BIAS_SENSP,
    ADS1299_REGDEFAULT_BIAS_SENSN,
    ADS1299_REGDEFAULT_LOFF_SENSP,
    ADS1299_REGDEFAULT_LOFF_SENSN,
    ADS1299_REGDEFAULT_LOFF_FLIP,
    ADS1299_REGDEFAULT_LOFF_STATP,
    ADS1299_REGDEFAULT_LOFF_STATN,
    ADS1299_REGDEFAULT_GPIO,
    ADS1299_REGDEFAULT_MISC1,
    ADS1299_REGDEFAULT_MISC2,
    ADS1299_REGDEFAULT_CONFIG4};

/**@SPI HANDLERS:
 * @brief SPI user event handler.
 * @param event
 */

static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(0);
static volatile bool spi_xfer_done;

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const *p_event,
    void *p_context) {
  spi_xfer_done = true;
  //  NRF_LOG_INFO("Transfer completed.\r\n");
  //  if (m_rx_buf[0] != 0) {
  //    NRF_LOG_INFO(" Received: \r\n");
  //    NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
  //  }
}

void ads_spi_init(void) {
  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
  spi_config.bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST;
  //SCLK = 1MHz is right speed because fCLK = (1/2)*SCLK, and fMOD = fCLK/4, and fMOD MUST BE 128kHz. Do the math.
  spi_config.frequency = NRF_DRV_SPI_FREQ_4M;
  spi_config.irq_priority = APP_IRQ_PRIORITY_HIGHEST;//APP_IRQ_PRIORITY_HIGHEST;
  spi_config.mode = NRF_DRV_SPI_MODE_1; //CPOL = 0 (Active High); CPHA = TRAILING (1)
  spi_config.miso_pin = ADS1299_SPI_MISO_PIN;
  spi_config.sck_pin = ADS1299_SPI_SCLK_PIN;
  spi_config.mosi_pin = ADS1299_SPI_MOSI_PIN;
  spi_config.ss_pin = ADS1299_SPI_CS_PIN;
  spi_config.orc = 0x55;
//  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, NULL, NULL));
  APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL));
  NRF_LOG_INFO(" SPI Initialized..\r\n");
}

void ads1299_powerdn(void) {
#if defined(BOARD_PCA10028) | defined(BOARD_NRF_BREAKOUT) | defined(BOARD_PCA10040)
  nrf_gpio_pin_clear(ADS1299_PWDN_RST_PIN);
#endif
#if defined(BOARD_FULL_EEG_V1)
  nrf_gpio_pin_clear(ADS1299_RESET_PIN);
  nrf_gpio_pin_clear(ADS1299_PWDN_PIN);
#endif
  nrf_delay_us(20);
  NRF_LOG_INFO(" ADS1299-x POWERED DOWN..\r\n");
}

void ads1299_powerup_reset(void) {
#if defined(BOARD_PCA10028) | defined(BOARD_NRF_BREAKOUT) | defined(BOARD_PCA10040)
  nrf_gpio_pin_clear(ADS1299_PWDN_RST_PIN);
#endif
#if defined(BOARD_FULL_EEG_V1)
  nrf_gpio_pin_clear(ADS1299_RESET_PIN);
  nrf_gpio_pin_clear(ADS1299_PWDN_PIN);
#endif
  nrf_delay_ms(50);
  NRF_LOG_INFO(" ADS1299-x POWERED UP AND RESET..\r\n");
}

void ads1299_powerup(void) {
#if defined(BOARD_PCA10028) | defined(BOARD_NRF_BREAKOUT) | defined(BOARD_PCA10040)
  nrf_gpio_pin_set(ADS1299_PWDN_RST_PIN);
#endif
#if defined(BOARD_FULL_EEG_V1)
  nrf_gpio_pin_set(ADS1299_RESET_PIN);
  nrf_gpio_pin_set(ADS1299_PWDN_PIN);
#endif
  nrf_delay_ms(1000); // Allow time for power-on reset
  NRF_LOG_INFO(" ADS1299-x POWERED UP...\r\n");
}

void ads1299_standby(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1299_OPC_STANDBY;
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) { __WFE(); }
  NRF_LOG_INFO(" ADS1299-x placed in standby mode...\r\n");
}

void ads1299_wake(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1299_OPC_WAKEUP;
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) { __WFE(); }
  nrf_delay_ms(10); // Allow time to wake up - 10ms
  NRF_LOG_INFO(" ADS1299-x Wakeup..\r\n");
}

void ads1299_soft_start_conversion(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1299_OPC_START;
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) { __WFE(); }
  NRF_LOG_INFO(" Start ADC conversion..\r\n");
}

void ads1299_stop_rdatac(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1299_OPC_SDATAC;
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) { __WFE(); }
  NRF_LOG_INFO(" Continuous Data Output Disabled..\r\n");
}

void ads1299_start_rdatac(void) {
  uint8_t tx_data_spi;
  uint8_t rx_data_spi;

  tx_data_spi = ADS1299_OPC_RDATAC;
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, &tx_data_spi, 1, &rx_data_spi, 1));
  while (!spi_xfer_done) { __WFE(); }
  NRF_LOG_INFO(" Continuous Data Output Enabled..\r\n");
}

void ads1299_check_id(void) {
  uint8_t device_id_reg_value;
  uint8_t tx_data_spi[3];
  uint8_t rx_data_spi[4];
  memset(rx_data_spi, 0, 4);
  tx_data_spi[0] = 0x20; // First command byte = 001r rrrr (r rrrr = register start address)
  tx_data_spi[1] = 0x00; // Intend to read 1 byte: (Bytes to read)-1 = 0
  tx_data_spi[2] = 0x00; //This will be replaced by Reg Data
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data_spi, 3, rx_data_spi, 4)); 
//  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data_spi, 3, rx_data_spi, 3)); 
  while (!spi_xfer_done) { __WFE(); }
  //NOTE: CHANGES FROM [2] to [3] for EASY DMA
#if SPI0_USE_EASY_DMA==1
  device_id_reg_value = rx_data_spi[3];
#else
  device_id_reg_value = rx_data_spi[2];
#endif
  bool is_ads_1299_4 = (device_id_reg_value & 0x1F) == (ADS1299_4_DEVICE_ID);
  bool is_ads_1299_6 = (device_id_reg_value & 0x1F) == (ADS1299_6_DEVICE_ID);
  bool is_ads_1299 = (device_id_reg_value & 0x1F) == (ADS1299_DEVICE_ID);
  uint8_t revisionVersion = (device_id_reg_value & 0xE0) >> 5;
  if (is_ads_1299 || is_ads_1299_6 || is_ads_1299_4) {
    NRF_LOG_INFO("Device Address Matches!\r\n");
  } else {
    NRF_LOG_INFO("********SPI I/O Error, Device Not Detected! *********** \r\n");
    NRF_LOG_INFO("SPI Transfer Dump: \r\n");
    NRF_LOG_INFO("ID[b0->3]: [0x%x | 0x%x | 0x%x] \r\n", rx_data_spi[0], rx_data_spi[1], rx_data_spi[2], rx_data_spi[3]);
//    NRF_LOG_INFO("ID[b3->6]: [0x%x | 0x%x | 0x%x | 0x%x] \r\n", rx_data_spi[3], rx_data_spi[4], rx_data_spi[5], rx_data_spi[6]);
  }
  if (is_ads_1299) {
    NRF_LOG_INFO("Device Name: ADS1299 \r\n");
  } else if (is_ads_1299_6) {
    NRF_LOG_INFO("Device Name: ADS1299-6 \r\n");
  } else if (is_ads_1299_4) {
    NRF_LOG_INFO("Device Name: ADS1299-4 \r\n");
  }
  if (is_ads_1299 || is_ads_1299_6 || is_ads_1299_4) {
    NRF_LOG_INFO("Device Revision #%d\r\n", revisionVersion);
    NRF_LOG_INFO("Device ID: 0x%x \r\n", device_id_reg_value);
  }
}

void ads1299_init_regs(void) {
  uint8_t err_code;
  uint8_t num_registers = 23;
  uint8_t txrx_size = num_registers + 2;
  uint8_t tx_data_spi[txrx_size]; //Size = 14 bytes
  uint8_t rx_data_spi[txrx_size]; //Size = 14 bytes
  uint8_t wreg_init_opcode = 0x41;
  for (int i = 0; i < txrx_size; ++i) {
    tx_data_spi[i] = 0;
    rx_data_spi[i] = 0;
  }
  tx_data_spi[0] = wreg_init_opcode;
  tx_data_spi[1] = num_registers - 1;
  for (int j = 0; j < num_registers; ++j) {
    tx_data_spi[j + 2] = ads1299_default_registers[j];
  }
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_data_spi, num_registers, rx_data_spi, num_registers));
  while (!spi_xfer_done) { __WFE(); }
  nrf_delay_ms(150);
  NRF_LOG_INFO(" Power-on reset and initialization procedure.. EC: %d \r\n", err_code);
}

//NOTE: DATA RETRIEVAL
/**@brief Function for acquiring a EEG Voltage Measurement samples.
 *
 * @details Uses SPI
 *          
 */
void get_eeg_voltage_samples(int32_t *eeg1, int32_t *eeg2, int32_t *eeg3, int32_t *eeg4) {
  uint8_t tx_rx_data[15] = {0x00, 0x00, 0x00,
      0x00, 0x00, 0x00,
      0x00, 0x00, 0x00,
      0x00, 0x00, 0x00,
      0x00, 0x00, 0x00};
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_rx_data, 15, tx_rx_data, 15));
  uint8_t cnt = 0;
  do {
    if (tx_rx_data[0] == 0xC0) {
      *eeg1 = ((tx_rx_data[3] << 16) | (tx_rx_data[4] << 8) | (tx_rx_data[5]));
      *eeg2 = ((tx_rx_data[6] << 16) | (tx_rx_data[7] << 8) | (tx_rx_data[8]));
      *eeg3 = ((tx_rx_data[9] << 16) | (tx_rx_data[10] << 8) | (tx_rx_data[11]));
      *eeg4 = ((tx_rx_data[12] << 16) | (tx_rx_data[13] << 8) | (tx_rx_data[14]));
      break;
    }
    cnt++;
    nrf_delay_us(1);
  } while (cnt < 255);
  //NRF_LOG_INFO("B0-2 = [0x%x 0x%x 0x%x | cnt=%d]\r\n",tx_rx_data[0],tx_rx_data[1],tx_rx_data[2],cnt);
  //NRF_LOG_INFO("DATA:[0x%x 0x%x]\r\n",*eeg1,*eeg2);
  //NRF_LOG_INFO("DATA:[0x%x 0x%x 0x%x 0x%x]\r\n",*eeg1,*eeg2,*eeg3,*eeg4);
}
/*
void get_eeg_voltage_sample(int32_t *eeg1) {
  uint8_t tx_rx_data[6];
  memset(tx_rx_data,0,6);
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_rx_data, 6, tx_rx_data, 6));
  while (!spi_xfer_done) { __WFE(); }
  *eeg1 = ((tx_rx_data[3] << 16) | (tx_rx_data[4] << 8) | (tx_rx_data[5]));
//      NRF_LOG_INFO("[0x[%x|%x|%x|%x|%x|%x]\r\n",tx_rx_data[0],tx_rx_data[1],tx_rx_data[2],tx_rx_data[3],tx_rx_data[4],tx_rx_data[5]);
}*/

void get_eeg_voltage_sample(int32_t *eeg1) {
  uint8_t tx_rx_data[6]; uint8_t cnt = 0;
  memset(tx_rx_data,0,6);
  spi_xfer_done = false;
  APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, tx_rx_data, 6, tx_rx_data, 6));
  while (!spi_xfer_done) { __WFE(); }
  do {
    if (((tx_rx_data[0] << 16) | (tx_rx_data[1] << 8) | (tx_rx_data[2]))==0xC00000) {
        *eeg1 = ((tx_rx_data[3] << 16) | (tx_rx_data[4] << 8) | (tx_rx_data[5]));
//        NRF_LOG_INFO("[%x] | [%x%x%x] \r\n",*eeg1,tx_rx_data[3], tx_rx_data[4],tx_rx_data[5]);
      break;
    }
    cnt++;
  } while (cnt < 255);
}

