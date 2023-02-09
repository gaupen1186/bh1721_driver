#include "bh1721fvc.h"
#include "i2c.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


#define BH1721_DEV_ADDR   0x23

#define bh1721_delay_ms   sys_sleep_ms

#define BH1721_I2C_WAIT_FREE_MS_MAX     100


static bh1721_res_mode_t res_mode;


static int8_t bh1721_i2c_write_cmd(uint8_t dev, uint8_t cmd)
{
  // timeout for wait free, return error
  if(i2c_wait_for_free(BH1721_I2C_WAIT_FREE_MS_MAX) != true)
    return -1;

  // set i2c busy
  i2c_set_busy();
  // nrf twi xfer tx
  ret_code_t ret = i2c_xfer_tx(dev, &cmd, 1);
  // set i2c free
  i2c_set_free();

  if(ret != NRF_SUCCESS)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
    return -1;
  }

  return 0;
}

static int8_t bh1721_i2c_read_raw(uint8_t dev, uint16_t *pdata)
{
  // timeout for wait free, return error
  if(i2c_wait_for_free(BH1721_I2C_WAIT_FREE_MS_MAX) != true)
    return -1;

  // set i2c busy
  i2c_set_busy();
  uint8_t rx[2];
  // nrf twi xfer tx rx
  ret_code_t ret = i2c_xfer_rx(dev, rx, 2);
  // set i2c free
  i2c_set_free();

  if(ret != NRF_SUCCESS)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n", (uint32_t)__func__, __LINE__);
    return -1;
  }

  *pdata = ((uint16_t)rx[0] << 8) | rx[1];
  return 0;
}

bool bh1721_power_down(void)
{
  int8_t ret = bh1721_i2c_write_cmd(BH1721_DEV_ADDR, CMD_POWER_DOWN);
  if(ret != 0)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n",
                  (uint32_t)__func__, __LINE__);
    return false;
  }

  return true;
}

bool bh1721_power_on(void)
{
  int8_t ret = bh1721_i2c_write_cmd(BH1721_DEV_ADDR, CMD_POWER_ON);
  if(ret != 0)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n",
                  (uint32_t)__func__, __LINE__);
    return false;
  }

  return true;
}

bool bh1721_set_resolution_mode(bh1721_res_mode_t mode)
{
  if(mode != MODE_AUTO_RES && mode != MODE_HIGH_RES && mode != MODE_LOW_RES)
  {
    NRF_LOG_ERROR("[%s]: Param invalid! L%u\r\n",
                  (uint32_t)__func__, __LINE__);
    return false;
  }

  int8_t ret = bh1721_i2c_write_cmd(BH1721_DEV_ADDR, mode);
  if(ret != 0)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n",
                  (uint32_t)__func__, __LINE__);
    return false;
  }

  res_mode = mode;
  return true;
}

bool bh1721_get_resolution_mode(void)
{
  return res_mode;
}

bool bh1721_set_meaurement_time(uint16_t mtreg)
{
  if(mtreg < 140 || mtreg > 1020)
  {
    NRF_LOG_ERROR("[%s]: Param invalid! L%u\r\n",
                  (uint32_t)__func__, __LINE__);
    return false;
  }

  if(bh1721_power_down() == true)
  {
    uint8_t high = (uint8_t)((mtreg >> 5) & 0xff);
    high |= 0x40;
    if(bh1721_i2c_write_cmd(BH1721_DEV_ADDR, high) == 0)
    {
      uint8_t low = (uint8_t)(mtreg & 0x1f);
      low |= 0x60;
      if(bh1721_i2c_write_cmd(BH1721_DEV_ADDR, low) == 0)
      {
        if(bh1721_power_on() == 0)
        {
          return true;
        }
      }
    }
  }

  NRF_LOG_ERROR("[%s]: failed! L%u\r\n",
                (uint32_t)__func__, __LINE__);
  return false;
}

bool bh1721_read_lux(uint16_t *lux)
{
  uint16_t raw_data;
  int8_t ret = bh1721_i2c_read_raw(BH1721_DEV_ADDR, &raw_data);
  if(ret != 0)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n",
                  (uint32_t)__func__, __LINE__);
    return false;
  }

  *lux = (uint16_t)(raw_data / 1.2);
  return true;
}

bool bh1721_init(void)
{
  int8_t ret = 0;

  ret += !i2c_init();
  ret += !bh1721_power_down();
  bh1721_delay_ms(10);
  ret += !bh1721_power_on();
  ret += !bh1721_set_resolution_mode(MODE_AUTO_RES);

  if(ret != 0)
  {
    NRF_LOG_ERROR("[%s]: Failed! L%u\r\n",
                  (uint32_t)__func__, __LINE__);
    return false;
  }
  return true;
}
