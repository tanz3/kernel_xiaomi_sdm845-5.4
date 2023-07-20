/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2019-2020, The Linux Foundation. All rights reserved.
 */

#ifndef __SOC_QCOM_SOCINFO_H__
#define __SOC_QCOM_SOCINFO_H__

#include <linux/types.h>

#define HARDWARE_PLATFORM_UNKNOWN 0
#define HARDWARE_PLATFORM_POLARIS 2
#define HARDWARE_PLATFORM_DIPPERN 3
#define HARDWARE_PLATFORM_BERYLLIUM 4
#define HARDWARE_PLATFORM_URSA 5
#define HARDWARE_PLATFORM_PERSEUS 6
#define HARDWARE_PLATFORM_EQUULEUS 7

#define HW_MAJOR_VERSION_SHIFT 16
#define HW_MAJOR_VERSION_MASK  0xFFFF0000
#define HW_MINOR_VERSION_SHIFT 0
#define HW_MINOR_VERSION_MASK  0x0000FFFF

uint32_t get_hw_version_platform(void);
uint32_t get_hw_version_major(void);
uint32_t get_hw_version_minor(void);

#if IS_ENABLED(CONFIG_QCOM_SOCINFO)
uint32_t socinfo_get_id(void);
uint32_t socinfo_get_serial_number(void);
const char *socinfo_get_id_string(void);
#else
static inline uint32_t socinfo_get_id(void)
{
	return 0;
}

static inline uint32_t socinfo_get_serial_number(void)
{
	return 0;
}

static inline const char *socinfo_get_id_string(void)
{
	return "N/A";
}
#endif /* CONFIG_QCOM_SOCINFO */

#endif /* __SOC_QCOM_SOCINFO_H__ */
