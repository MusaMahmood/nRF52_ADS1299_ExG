/* Copyright (c) 2016 Musa Mahmood
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "ble_eeg.h"
#include "ads1299-x.h"
#include "app_error.h"
#include "app_util.h"
#include "ble_srv_common.h"
#include "nordic_common.h"
#include "nrf_log.h"
#include <string.h>

#define MAX_LEN_BLE_PACKET_BYTES 246 //20*3bytes																						 /**< Maximum size in bytes of a transmitted Body Voltage Measurement. */

void ble_eeg_on_ble_evt(ble_eeg_t *p_eeg, ble_evt_t *p_ble_evt) {
  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    p_eeg->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    p_eeg->conn_handle = BLE_CONN_HANDLE_INVALID;
    break;

  case BLE_GATTS_EVT_HVN_TX_COMPLETE:
    if (p_eeg->busy) {
      p_eeg->busy = false;
    }
    break;
  default:
    break;
  }
}

static uint8_t bvm_encode_24(ble_eeg_t *p_eeg, uint8_t *p_encoded_buffer, int encodeChannel) {
  uint8_t len = 0;
  int i;
  // Encode body voltage measurement
  
    for (i = 0; i < p_eeg->eeg_ch1_count; i++) {
      if (len + sizeof(uint16_t) + sizeof(uint8_t) > MAX_LEN_BLE_PACKET_BYTES) {
        // Not all stored voltage values can fit into the packet, so
        // move the remaining values to the start of the buffer.
        memmove(&p_eeg->eeg_ch1_buffer[0],
            &p_eeg->eeg_ch1_buffer[i],
            (p_eeg->eeg_ch1_count - i) * sizeof(uint32_t));
        break;
      }
      len += uint24_encode(p_eeg->eeg_ch1_buffer[i], &p_encoded_buffer[len]); //len+=3;
    }
    p_eeg->eeg_ch1_count -= i;
    
  return len;
}

static uint32_t eeg_ch1_char_add(ble_eeg_t *p_eeg) {
  uint32_t err_code = 0;
  ble_uuid_t char_uuid;
  uint8_t encoded_initial_eeg[MAX_LEN_BLE_PACKET_BYTES];
  BLE_UUID_BLE_ASSIGN(char_uuid, BLE_UUID_EEG_CH1_CHAR);

  ble_gatts_char_md_t char_md;

  memset(&char_md, 0, sizeof(char_md));
  char_md.char_props.read = 1;
  char_md.char_props.write = 0;

  ble_gatts_attr_md_t cccd_md;
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
  cccd_md.vloc = BLE_GATTS_VLOC_STACK;
  char_md.p_cccd_md = &cccd_md;
  char_md.char_props.notify = 1;
  ble_gatts_attr_md_t attr_md;
  memset(&attr_md, 0, sizeof(attr_md));
  attr_md.vloc = BLE_GATTS_VLOC_STACK;
  attr_md.vlen = 1;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

  ble_gatts_attr_t attr_char_value;
  memset(&attr_char_value, 0, sizeof(attr_char_value));
  attr_char_value.p_uuid = &char_uuid;
  attr_char_value.p_attr_md = &attr_md;
  attr_char_value.init_len = bvm_encode_24(p_eeg, encoded_initial_eeg, 1);
  attr_char_value.init_offs = 0;
  attr_char_value.max_len = MAX_LEN_BLE_PACKET_BYTES;
  attr_char_value.p_value = encoded_initial_eeg;
  err_code = sd_ble_gatts_characteristic_add(p_eeg->service_handle,
      &char_md,
      &attr_char_value,
      &p_eeg->eeg_ch1_handles);
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

/**@brief Function for initiating our new service.
 *
 * @param[in]   p_mpu        Our Service structure.
 *
 */
void ble_eeg_service_init(ble_eeg_t *p_eeg) {
  uint32_t err_code; // Variable to hold return codes from library and softdevice functions
  uint16_t service_handle;
  ble_uuid_t service_uuid;
  ble_uuid128_t base_uuid = {BMS_UUID_BASE};

  err_code = sd_ble_uuid_vs_add(&base_uuid, &(p_eeg->uuid_type));
  APP_ERROR_CHECK(err_code);

  service_uuid.type = p_eeg->uuid_type;
  service_uuid.uuid = BLE_UUID_BIOPOTENTIAL_EEG_MEASUREMENT_SERVICE;

  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &service_uuid, &service_handle);
  APP_ERROR_CHECK(err_code);
  //Add Characteristic:
  eeg_ch1_char_add(p_eeg);

  p_eeg->busy = false;
  p_eeg->packets_sent = 0;
}

#if defined(ADS1299)
/**@Update adds single int16_t voltage value: */
void ble_eeg_update_1ch(ble_eeg_t *p_eeg, int32_t *eeg1) {
  //add new value:
  p_eeg->eeg_ch1_buffer[p_eeg->eeg_ch1_count++] = *eeg1; // Add new value to 32-bit buffer
  if (p_eeg->eeg_ch1_count == BLE_EEG_MAX_BUFFERED_MEASUREMENTS) {
    ble_eeg_send_24bit_array_ch1(p_eeg);
  }
}

uint32_t ble_eeg_send_24bit_array_ch1(ble_eeg_t *p_eeg) {
  uint32_t err_code;
  if (p_eeg->conn_handle != BLE_CONN_HANDLE_INVALID) {
    uint8_t encoded_eeg[MAX_LEN_BLE_PACKET_BYTES];
    uint16_t len;
    uint16_t hvx_len;
    ble_gatts_hvx_params_t hvx_params;
    len = bvm_encode_24(p_eeg, encoded_eeg, 1);
    hvx_len = len;
    memset(&hvx_params, 0, sizeof(hvx_params));
    hvx_params.handle = p_eeg->eeg_ch1_handles.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    hvx_params.p_len = &hvx_len;
    hvx_params.p_data = encoded_eeg;
    err_code = sd_ble_gatts_hvx(p_eeg->conn_handle, &hvx_params);
  }
  return err_code;
}

#endif //(defined(ADS1299)