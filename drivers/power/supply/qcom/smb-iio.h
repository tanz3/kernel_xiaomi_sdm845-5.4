/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2023 The Linux Foundation. All rights reserved.
 */

#ifndef __SMB2_IIO_H
#define __SMB2_IIO_H

#include <linux/iio/iio.h>
#include <dt-bindings/iio/qti_power_supply_iio.h>
#include "iio-channels.h"

struct smb2_iio_prop_channels {
	const char *datasheet_name;
	int channel_num;
	enum iio_chan_type type;
	long info_mask;
};

#define PARAM(chan) PSY_IIO_##chan

#define SMB2_CHAN(_dname, _chan, _type, _mask)		\
	{								\
		.datasheet_name = _dname,				\
		.channel_num = _chan,				\
		.type = _type,						\
		.info_mask = _mask,					\
	},								\

#define SMB2_CHAN_VOLT(_dname, chan)					\
	[PARAM(chan)] = SMB2_CHAN(_dname, PARAM(chan),		\
			IIO_VOLTAGE, BIT(IIO_CHAN_INFO_PROCESSED))	\

#define SMB2_CHAN_CUR(_dname, chan)					\
	[PARAM(chan)] = SMB2_CHAN(_dname, PARAM(chan),		\
			IIO_CURRENT, BIT(IIO_CHAN_INFO_PROCESSED))	\

#define SMB2_CHAN_RES(_dname, chan)					\
	[PARAM(chan)] = SMB2_CHAN(_dname, PARAM(chan),		\
			IIO_RESISTANCE, BIT(IIO_CHAN_INFO_PROCESSED))	\

#define SMB2_CHAN_TEMP(_dname, chan)					\
	[PARAM(chan)] = SMB2_CHAN(_dname, PARAM(chan),		\
			IIO_TEMP, BIT(IIO_CHAN_INFO_PROCESSED))	\

#define SMB2_CHAN_POWER(_dname, chan)					\
	[PARAM(chan)] = SMB2_CHAN(_dname, PARAM(chan),		\
			IIO_POWER, BIT(IIO_CHAN_INFO_PROCESSED))	\

#define SMB2_CHAN_CAP(_dname, chan)					\
	[PARAM(chan)] = SMB2_CHAN(_dname, PARAM(chan),		\
			IIO_CAPACITANCE, BIT(IIO_CHAN_INFO_PROCESSED))	\

#define SMB2_CHAN_COUNT(_dname, chan)					\
	[PARAM(chan)] = SMB2_CHAN(_dname, PARAM(chan),		\
			IIO_COUNT, BIT(IIO_CHAN_INFO_PROCESSED))	\

#define SMB2_CHAN_INDEX(_dname, chan)					\
	[PARAM(chan)] = SMB2_CHAN(_dname, PARAM(chan),		\
			IIO_INDEX, BIT(IIO_CHAN_INFO_PROCESSED))	\

#define SMB2_CHAN_ACTIVITY(_dname, chan)				\
	[PARAM(chan)] = SMB2_CHAN(_dname, PARAM(chan),		\
			IIO_ACTIVITY, BIT(IIO_CHAN_INFO_PROCESSED))	\

static const struct smb2_iio_prop_channels smb2_chans_pmic[] = {
	SMB2_CHAN_CUR("usb_pd_current_max", PD_CURRENT_MAX)
	SMB2_CHAN_INDEX("usb_typec_mode", TYPEC_MODE)
	SMB2_CHAN_INDEX("usb_typec_power_role", TYPEC_POWER_ROLE)
	SMB2_CHAN_INDEX("usb_typec_cc_orientation", TYPEC_CC_ORIENTATION)
	SMB2_CHAN_INDEX("usb_pd_active", PD_ACTIVE)
	SMB2_CHAN_CUR("usb_input_current_settled", USB_INPUT_CURRENT_SETTLED)
	SMB2_CHAN_ACTIVITY("usb_pe_start", PE_START)
	SMB2_CHAN_CUR("usb_ctm_current_max", CTM_CURRENT_MAX)
	SMB2_CHAN_CUR("usb_hw_current_max", HW_CURRENT_MAX)
	SMB2_CHAN_INDEX("usb_real_type", USB_REAL_TYPE)
	SMB2_CHAN_VOLT("usb_pd_voltage_max", PD_VOLTAGE_MAX)
	SMB2_CHAN_VOLT("usb_pd_voltage_min", PD_VOLTAGE_MIN)
	SMB2_CHAN_INDEX("usb_connector_type", CONNECTOR_TYPE)
	SMB2_CHAN_INDEX("usb_moisture_detected", MOISTURE_DETECTED)
	SMB2_CHAN_ACTIVITY("usb_apsd_rerun", APSD_RERUN)
	SMB2_CHAN_ACTIVITY("usb_typec_src_rp", TYPEC_SRC_RP)
	SMB2_CHAN_ACTIVITY("usb_pd_in_hard_reset", PD_IN_HARD_RESET)
	SMB2_CHAN_INDEX("usb_pd_usb_suspend_supported",
			PD_USB_SUSPEND_SUPPORTED)
	SMB2_CHAN_ACTIVITY("usb_pr_swap", PR_SWAP)
	SMB2_CHAN_CUR("main_input_current_settled", MAIN_INPUT_CURRENT_SETTLED)
	SMB2_CHAN_VOLT("main_input_voltage_settled", MAIN_INPUT_VOLTAGE_SETTLED)
	SMB2_CHAN_CUR("main_fcc_delta", FCC_DELTA)
	SMB2_CHAN_ACTIVITY("main_toggle_stat", TOGGLE_STAT)
	SMB2_CHAN_VOLT("main_voltage_max", VOLTAGE_MAX)
	SMB2_CHAN_CUR("main_constant_charge_current_max",
			CONSTANT_CHARGE_CURRENT_MAX)
	SMB2_CHAN_CUR("main_current_max", CURRENT_MAX)
	SMB2_CHAN_INDEX("main_health", HEALTH)
	SMB2_CHAN_INDEX("dc_real_type", DC_REAL_TYPE)
	SMB2_CHAN_TEMP("battery_charger_temp", CHARGER_TEMP)
	SMB2_CHAN_TEMP("battery_charger_temp_max", CHARGER_TEMP_MAX)
	SMB2_CHAN_CUR("battery_input_current_limited", INPUT_CURRENT_LIMITED)
	SMB2_CHAN_ACTIVITY("battery_sw_jeita_enabled", SW_JEITA_ENABLED)
	SMB2_CHAN_ACTIVITY("battery_charge_done", CHARGE_DONE)
	SMB2_CHAN_ACTIVITY("battery_parallel_disable", PARALLEL_DISABLE)
	SMB2_CHAN_ACTIVITY("battery_set_ship_mode", SET_SHIP_MODE)
	SMB2_CHAN_INDEX("battery_die_health", DIE_HEALTH)
	SMB2_CHAN_ACTIVITY("battery_rerun_aicl", RERUN_AICL)
	SMB2_CHAN_COUNT("battery_dp_dm", DP_DM)
	SMB2_CHAN_ACTIVITY("battery_fcc_stepper_enable", FCC_STEPPER_ENABLE)
	SMB5_CHAN_INDEX("usb_typec_accessory_mode", TYPEC_ACCESSORY_MODE)
};

struct iio_channel **get_ext_channels(struct device *dev,
	const char *const *channel_map, int size);
#endif
