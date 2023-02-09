#ifndef __BH1721_H__
#define __BH1721_H__

#include <stdint.h>
#include <stdbool.h>


// opecode defines
#define CMD_POWER_DOWN          0x00
#define CMD_POWER_ON            0x01
#define CMD_CHANGE_MSU_TIME_H   0x02
#define CMD_CHANGE_MSU_TIME_L   0x03


typedef enum
{
  MODE_AUTO_RES   = 0x10,
  MODE_HIGH_RES   = 0x12,
  MODE_LOW_RES    = 0x13,
}
bh1721_res_mode_t;


bool bh1721_power_down(void);
bool bh1721_power_on(void);
bool bh1721_set_resolution_mode(bh1721_res_mode_t mode);
bool bh1721_get_resolution_mode(void);
bool bh1721_set_meaurement_time(uint16_t mtreg);
bool bh1721_read_lux(uint16_t *lux);
bool bh1721_init(void);
bool bh1721_read_lux(uint16_t *lux);

#endif // __BH1721_H__
