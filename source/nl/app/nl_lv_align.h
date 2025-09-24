/**
 * @file nl_lv_align.h
 * @brief Navigation Library Level Alignment Functions
 */

#ifndef _NL_LV_ALIGN_H
#define _NL_LV_ALIGN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nl.h"
#include "nl_imu.h"

int nl_lv_align(ins_t *ins, nl_imu_mgr_t *mgr);
int nl_set_yaw_from_mag(ins_t *ins, nl_t *mag, nl_t *heading);


#ifdef __cplusplus
}
#endif

#endif /* _NL_LV_ALIGN_H */
