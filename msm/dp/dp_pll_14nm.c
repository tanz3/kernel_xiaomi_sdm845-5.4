// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2016-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

/*
 ***************************************************************************
 ******** Display Port PLL driver block diagram for branch clocks **********
 ***************************************************************************

			+--------------------------+
			|       DP_VCO_CLK         |
			|			   |
			|  +-------------------+   |
			|  |   (DP PLL/VCO)    |   |
			|  +---------+---------+   |
			|	     v		   |
			| +----------+-----------+ |
			| | hsclk_divsel_clk_src | |
			| +----------+-----------+ |
			+--------------------------+
				     |
				     v
	   +------------<------------|------------>-------------+
	   |                         |                          |
+----------v----------+	  +----------v----------+    +----------v----------+
|   dp_link_2x_clk    |	  | vco_divided_clk_src	|    | vco_divided_clk_src |
|     divsel_five     |	  |			|    |			   |
v----------+----------v	  |	divsel_two	|    |	   divsel_four	   |
	   |		  +----------+----------+    +----------+----------+
	   |                         |                          |
	   v			     v				v
				     |	+---------------------+	|
  Input to MMSSCC block		     |	|    (aux_clk_ops)    |	|
  for link clk, crypto clk	     +-->   vco_divided_clk   <-+
  and interface clock			|	_src_mux      |
					+----------+----------+
						   |
						   v
					 Input to MMSSCC block
					 for DP pixel clock

 ******************************************************************************
 */

#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/clk.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include "clk-regmap-mux.h"
#include "dp_hpd.h"
#include "dp_debug.h"

#include <dt-bindings/clock/mdss-14nm-pll-clk.h>

#include "dp_pll.h"

#define DP_PHY_CFG				0x0010
#define DP_PHY_CFG_1				0x0014
#define DP_PHY_PD_CTL				0x0018
#define DP_PHY_MODE				0x001C

#define DP_PHY_VCO_DIV				0x0068
#define DP_PHY_TX0_TX1_LANE_CTL			0x006C

#define DP_PHY_TX2_TX3_LANE_CTL			0x0088
#define DP_PHY_SPARE0				0x00AC
#define DP_PHY_STATUS				0x00C0

/* Tx registers */
#define QSERDES_TX0_OFFSET			0x0400
#define QSERDES_TX1_OFFSET			0x0800

#define TXn_BIST_MODE_LANENO			0x0000
#define TXn_CLKBUF_ENABLE			0x0008
#define TXn_TX_EMP_POST1_LVL			0x000C

#define TXn_TX_DRV_LVL				0x001C

#define TXn_RESET_TSYNC_EN			0x0024
#define TXn_PRE_STALL_LDO_BOOST_EN		0x0028
#define TXn_TX_BAND				0x002C
#define TXn_SLEW_CNTL				0x0030
#define TXn_INTERFACE_SELECT			0x0034

#define TXn_RES_CODE_LANE_TX			0x003C
#define TXn_RES_CODE_LANE_RX			0x0040
#define TXn_RES_CODE_LANE_OFFSET_TX		0x0044
#define TXn_RES_CODE_LANE_OFFSET_RX		0x0048

#define TXn_DEBUG_BUS_SEL			0x0058
#define TXn_TRANSCEIVER_BIAS_EN			0x005C
#define TXn_HIGHZ_DRVR_EN			0x0060
#define TXn_TX_POL_INV				0x0064
#define TXn_PARRATE_REC_DETECT_IDLE_EN		0x0068

#define TXn_LANE_MODE_1				0x008C

#define TXn_TRAN_DRVR_EMP_EN			0x00C0
#define TXn_TX_INTERFACE_MODE			0x00C4

#define TXn_VMODE_CTRL1				0x00F0

/* PLL register offset */
#define QSERDES_COM_ATB_SEL1			0x0000
#define QSERDES_COM_ATB_SEL2			0x0004
#define QSERDES_COM_FREQ_UPDATE			0x0008
#define QSERDES_COM_BG_TIMER			0x000C
#define QSERDES_COM_SSC_EN_CENTER		0x0010
#define QSERDES_COM_SSC_ADJ_PER1		0x0014
#define QSERDES_COM_SSC_ADJ_PER2		0x0018
#define QSERDES_COM_SSC_PER1			0x001C
#define QSERDES_COM_SSC_PER2			0x0020
#define QSERDES_COM_SSC_STEP_SIZE1		0x0024
#define QSERDES_COM_SSC_STEP_SIZE2		0x0028
#define QSERDES_COM_POST_DIV			0x002C
#define QSERDES_COM_POST_DIV_MUX		0x0030
#define QSERDES_COM_BIAS_EN_CLKBUFLR_EN		0x0034
#define QSERDES_COM_CLK_ENABLE1			0x0038
#define QSERDES_COM_SYS_CLK_CTRL		0x003C
#define QSERDES_COM_SYSCLK_BUF_ENABLE		0x0040
#define QSERDES_COM_PLL_EN			0x0044
#define QSERDES_COM_PLL_IVCO			0x0048
#define QSERDES_COM_LOCK_CMP1_MODE0		0x004C
#define QSERDES_COM_LOCK_CMP2_MODE0		0x0050
#define QSERDES_COM_LOCK_CMP3_MODE0		0x0054

#define QSERDES_COM_CP_CTRL_MODE0		0x0078
#define QSERDES_COM_CP_CTRL_MODE1		0x007C
#define QSERDES_COM_PLL_RCTRL_MODE0		0x0084
#define QSERDES_COM_PLL_CCTRL_MODE0		0x0090
#define QSERDES_COM_PLL_CNTRL			0x009C

#define QSERDES_COM_SYSCLK_EN_SEL		0x00AC
#define QSERDES_COM_CML_SYSCLK_SEL		0x00B0
#define QSERDES_COM_RESETSM_CNTRL		0x00B4
#define QSERDES_COM_RESETSM_CNTRL2		0x00B8
#define QSERDES_COM_LOCK_CMP_EN			0x00C8
#define QSERDES_COM_LOCK_CMP_CFG		0x00CC

#define QSERDES_COM_DEC_START_MODE0		0x00D0
#define QSERDES_COM_DEC_START_MODE1		0x00D4
#define QSERDES_COM_DIV_FRAC_START1_MODE0	0x00DC
#define QSERDES_COM_DIV_FRAC_START2_MODE0	0x00E0
#define QSERDES_COM_DIV_FRAC_START3_MODE0	0x00E4

#define QSERDES_COM_INTEGLOOP_GAIN0_MODE0	0x0108
#define QSERDES_COM_INTEGLOOP_GAIN1_MODE0	0x010C
#define QSERDES_COM_VCO_TUNE_CTRL		0x0124
#define QSERDES_COM_VCO_TUNE_MAP		0x0128
#define QSERDES_COM_VCO_TUNE1_MODE0		0x012C
#define QSERDES_COM_VCO_TUNE2_MODE0		0x0130

#define QSERDES_COM_CMN_STATUS			0x015C
#define QSERDES_COM_RESET_SM_STATUS		0x0160

#define QSERDES_COM_BG_CTRL			0x0170
#define QSERDES_COM_CLK_SELECT			0x0174
#define QSERDES_COM_HSCLK_SEL			0x0178
#define QSERDES_COM_CORECLK_DIV			0x0184
#define QSERDES_COM_SW_RESET			0x0188
#define QSERDES_COM_CORE_CLK_EN			0x018C
#define QSERDES_COM_C_READY_STATUS		0x0190
#define QSERDES_COM_CMN_CONFIG			0x0194
#define QSERDES_COM_SVS_MODE_CLK_SEL		0x019C

#define DP_PLL_POLL_SLEEP_US			500
#define DP_PLL_POLL_TIMEOUT_US			10000

#define DP_PHY_POLL_SLEEP_US			500
#define DP_PHY_POLL_TIMEOUT_US			10000

#define DP_VCO_RATE_8100MHZDIV1000		8100000UL
#define DP_VCO_RATE_10800MHZDIV1000		10800000UL

#define DP_VCO_HSCLK_RATE_1620MHZDIV1000	1620000UL
#define DP_VCO_HSCLK_RATE_2700MHZDIV1000	2700000UL
#define DP_VCO_HSCLK_RATE_5400MHZDIV1000	5400000UL

struct dp_pll_14nm_db {
	struct dp_pll *pll;

	/* lane and orientation settings */
	u8 lane_cnt;
	u8 orientation;

	/* COM PHY settings */
	u32 hsclk_sel;
	u32 dec_start_mode0;
	u32 div_frac_start1_mode0;
	u32 div_frac_start2_mode0;
	u32 div_frac_start3_mode0;
	u32 lock_cmp1_mode0;
	u32 lock_cmp2_mode0;
	u32 lock_cmp3_mode0;

	/* PHY vco divider */
	u32 phy_vco_div;

	/* TX settings */
	u32 lane_mode_1;
};

static struct clk_ops mux_clk_ops;

static struct regmap_config dp_pll_14nm_cfg = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register = 0x910,
};

static struct clk_fixed_factor dp_phy_pll_link_clk = {
	.div = 10,
	.mult = 1,

	.hw.init = &(struct clk_init_data){
		.name = "dp_phy_pll_link_clk",
		.parent_names =
			(const char *[]){ "dp_vco_clk" },
		.num_parents = 1,
		.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
		.ops = &clk_fixed_factor_ops,
	},
};

static struct clk_fixed_factor dp_vco_divsel_two_clk_src = {
	.div = 2,
	.mult = 1,

	.hw.init = &(struct clk_init_data){
		.name = "dp_vco_divsel_two_clk_src",
		.parent_names =
			(const char *[]){ "dp_vco_clk" },
		.num_parents = 1,
		.flags = (CLK_GET_RATE_NOCACHE),
		.ops = &clk_fixed_factor_ops,
	},
};

static struct clk_fixed_factor dp_vco_divsel_four_clk_src = {
	.div = 4,
	.mult = 1,

	.hw.init = &(struct clk_init_data){
		.name = "dp_vco_divsel_four_clk_src",
		.parent_names =
			(const char *[]){ "dp_vco_clk" },
		.num_parents = 1,
		.flags = (CLK_GET_RATE_NOCACHE),
		.ops = &clk_fixed_factor_ops,
	},
};

static int clk_mux_determine_rate(struct clk_hw *hw,
				     struct clk_rate_request *req)
{
	int ret = 0;

	ret = __clk_mux_determine_rate_closest(hw, req);
	if (ret)
		return ret;

	/* Set the new parent of mux if there is a new valid parent */
	if (hw->clk && req->best_parent_hw->clk)
		clk_set_parent(hw->clk, req->best_parent_hw->clk);

	return 0;
}


static unsigned long mux_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct clk *div_clk = NULL, *vco_clk = NULL;
	struct dp_pll_vco_clk *vco = NULL;

	div_clk = clk_get_parent(hw->clk);
	if (!div_clk)
		return 0;

	vco_clk = clk_get_parent(div_clk);
	if (!vco_clk)
		return 0;

	vco = to_dp_vco_hw(__clk_get_hw(vco_clk));
	if (!vco)
		return 0;

	if (vco->rate == DP_VCO_HSCLK_RATE_5400MHZDIV1000)
		return (vco->rate / 4);
	else
		return (vco->rate / 2);
}

static struct clk_regmap_mux dp_phy_pll_vco_div_clk = {
	.reg = 0x64,
	.shift = 0,
	.width = 1,

	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "dp_phy_pll_vco_div_clk",
			.parent_names =
				(const char *[]){"dp_vco_divsel_two_clk_src",
					"dp_vco_divsel_four_clk_src"},
			.num_parents = 2,
			.ops = &mux_clk_ops,
			.flags = (CLK_GET_RATE_NOCACHE | CLK_SET_RATE_PARENT),
		},
	},
};

int dp_mux_set_parent_14nm(void *context, unsigned int reg, unsigned int val)
{
	struct dp_pll *pll = context;
	u32 auxclk_div;

	if (!context) {
		DP_ERR("invalid input parameters\n");
		return -EINVAL;
	}

	auxclk_div = dp_pll_read(dp_phy, DP_PHY_VCO_DIV);
	auxclk_div &= ~0x03;	/* bits 0 to 1 */

	if (val == 0) /* mux parent index = 0 */
		auxclk_div |= 1;
	else if (val == 1) /* mux parent index = 1 */
		auxclk_div |= 2;

	dp_pll_write(dp_phy,
			DP_PHY_VCO_DIV, auxclk_div);
	/* Make sure the PHY registers writes are done */
	wmb();
	pr_debug("mux=%d auxclk_div=%x\n", val, auxclk_div);

	return 0;
}

int dp_mux_get_parent_14nm(void *context, unsigned int reg, unsigned int *val)
{
	u32 auxclk_div = 0;
	struct dp_pll *pll = context;

	if (!context || !val) {
		DP_ERR("invalid input parameters\n");
		return -EINVAL;
	}

	auxclk_div = dp_pll_read(dp_phy, DP_PHY_VCO_DIV);
	auxclk_div &= 0x03;

	if (auxclk_div == 1) /* Default divider */
		*val = 0;
	else if (auxclk_div == 2)
		*val = 1;

	pr_debug("auxclk_div=%d, val=%d\n", auxclk_div, *val);

	return 0;
}

static int dp_vco_pll_init_db_14nm(struct dp_pll_14nm_db *pdb,
		unsigned long rate)
{
	struct dp_pll *pll = pdb->pll;
	u32 spare_value = 0;

	spare_value = dp_pll_read(dp_phy, DP_PHY_SPARE0);
	pdb->lane_cnt = spare_value & 0x0F;
	pdb->orientation = (spare_value & 0xF0) >> 4;

	pr_debug("spare_value=0x%x, ln_cnt=0x%x, orientation=0x%x\n",
			spare_value, pdb->lane_cnt, pdb->orientation);

	switch (rate) {
	case DP_VCO_HSCLK_RATE_1620MHZDIV1000:
		pdb->hsclk_sel = 0x2c;
		pdb->dec_start_mode0 = 0x69;
		pdb->div_frac_start1_mode0 = 0x00;
		pdb->div_frac_start2_mode0 = 0x80;
		pdb->div_frac_start3_mode0 = 0x07;
		pdb->lock_cmp1_mode0 = 0xbf;
		pdb->lock_cmp2_mode0 = 0x21;
		pdb->lock_cmp3_mode0 = 0x00;
		pdb->phy_vco_div = 0x1;
		pdb->lane_mode_1 = 0xc6;
		break;
	case DP_VCO_HSCLK_RATE_2700MHZDIV1000:
		pdb->hsclk_sel = 0x24;
		pdb->dec_start_mode0 = 0x69;
		pdb->div_frac_start1_mode0 = 0x00;
		pdb->div_frac_start2_mode0 = 0x80;
		pdb->div_frac_start3_mode0 = 0x07;
		pdb->lock_cmp1_mode0 = 0x3f;
		pdb->lock_cmp2_mode0 = 0x38;
		pdb->lock_cmp3_mode0 = 0x00;
		pdb->phy_vco_div = 0x1;
		pdb->lane_mode_1 = 0xc4;
		break;
	case DP_VCO_HSCLK_RATE_5400MHZDIV1000:
		pdb->hsclk_sel = 0x20;
		pdb->dec_start_mode0 = 0x8c;
		pdb->div_frac_start1_mode0 = 0x00;
		pdb->div_frac_start2_mode0 = 0x00;
		pdb->div_frac_start3_mode0 = 0x0a;
		pdb->lock_cmp1_mode0 = 0x7f;
		pdb->lock_cmp2_mode0 = 0x70;
		pdb->lock_cmp3_mode0 = 0x00;
		pdb->phy_vco_div = 0x2;
		pdb->lane_mode_1 = 0xc4;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int dp_config_vco_rate_14nm(struct dp_pll_vco_clk *vco,
		unsigned long rate)
{
	u32 res = 0;
	struct dp_pll *pll = vco->priv;
	struct dp_pll_14nm_db *pdb = (struct dp_pll_14nm_db *)pll->priv;

	res = dp_vco_pll_init_db_14nm(pdb, rate);
	if (res) {
		pr_err("VCO Init DB failed\n");
		return res;
	}

	dp_pll_write(dp_phy, DP_PHY_PD_CTL, 0x3d);

	/* Make sure the PHY register writes are done */
	wmb();

	dp_pll_write(dp_pll,
		QSERDES_COM_SVS_MODE_CLK_SEL, 0x01);
	dp_pll_write(dp_pll,
		QSERDES_COM_SYSCLK_EN_SEL, 0x37);
	dp_pll_write(dp_pll,
		QSERDES_COM_CLK_SELECT, 0x00);
	dp_pll_write(dp_pll,
		QSERDES_COM_SYS_CLK_CTRL, 0x06);
	dp_pll_write(dp_pll,
		QSERDES_COM_BIAS_EN_CLKBUFLR_EN, 0x3f);
	dp_pll_write(dp_pll,
		QSERDES_COM_CLK_ENABLE1, 0x0e);
	dp_pll_write(dp_pll,
		QSERDES_COM_BG_CTRL, 0x0f);
	dp_pll_write(dp_pll,
		QSERDES_COM_SYSCLK_BUF_ENABLE, 0x06);
	dp_pll_write(dp_pll,
		QSERDES_COM_CLK_SELECT, 0x30);
	dp_pll_write(dp_pll,
		QSERDES_COM_PLL_IVCO, 0x0f);
	dp_pll_write(dp_pll,
		QSERDES_COM_PLL_CCTRL_MODE0, 0x28);
	dp_pll_write(dp_pll,
		QSERDES_COM_PLL_RCTRL_MODE0, 0x16);
	dp_pll_write(dp_pll,
		QSERDES_COM_CP_CTRL_MODE0, 0x0b);

	/* Parameters dependent on vco clock frequency */
	dp_pll_write(dp_pll,
		QSERDES_COM_HSCLK_SEL, pdb->hsclk_sel);
	dp_pll_write(dp_pll,
		QSERDES_COM_DEC_START_MODE0, pdb->dec_start_mode0);
	dp_pll_write(dp_pll,
		QSERDES_COM_DIV_FRAC_START1_MODE0, pdb->div_frac_start1_mode0);
	dp_pll_write(dp_pll,
		QSERDES_COM_DIV_FRAC_START2_MODE0, pdb->div_frac_start2_mode0);
	dp_pll_write(dp_pll,
		QSERDES_COM_DIV_FRAC_START3_MODE0, pdb->div_frac_start3_mode0);
	dp_pll_write(dp_pll,
		QSERDES_COM_LOCK_CMP1_MODE0, pdb->lock_cmp1_mode0);
	dp_pll_write(dp_pll,
		QSERDES_COM_LOCK_CMP2_MODE0, pdb->lock_cmp2_mode0);
	dp_pll_write(dp_pll,
		QSERDES_COM_LOCK_CMP3_MODE0, pdb->lock_cmp3_mode0);

	dp_pll_write(dp_pll,
		QSERDES_COM_INTEGLOOP_GAIN0_MODE0, 0x40);
	dp_pll_write(dp_pll,
		QSERDES_COM_INTEGLOOP_GAIN1_MODE0, 0x00);
	dp_pll_write(dp_pll,
		QSERDES_COM_VCO_TUNE_MAP, 0x00);
	dp_pll_write(dp_pll,
		QSERDES_COM_BG_TIMER, 0x08);
	dp_pll_write(dp_pll,
		QSERDES_COM_CORECLK_DIV, 0x05);
	dp_pll_write(dp_pll,
		QSERDES_COM_VCO_TUNE_CTRL, 0x00);
	dp_pll_write(dp_pll,
		QSERDES_COM_VCO_TUNE1_MODE0, 0x00);
	dp_pll_write(dp_pll,
		QSERDES_COM_VCO_TUNE2_MODE0, 0x00);
	dp_pll_write(dp_pll,
		QSERDES_COM_VCO_TUNE_CTRL, 0x00);
	wmb(); /* make sure write happens */

	dp_pll_write(dp_pll,
		QSERDES_COM_CORE_CLK_EN, 0x0f);
	wmb(); /* make sure write happens */

	if (pdb->orientation == ORIENTATION_CC2)
		dp_pll_write(dp_phy, DP_PHY_MODE, 0xc9);
	else
		dp_pll_write(dp_phy, DP_PHY_MODE, 0xd9);
	wmb(); /* make sure write happens */

	/* TX Lane configuration */
	dp_pll_write(dp_phy,
		DP_PHY_TX0_TX1_LANE_CTL, 0x05);
	dp_pll_write(dp_phy,
		DP_PHY_TX2_TX3_LANE_CTL, 0x05);

	/* TX-0 register configuration */
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_TRANSCEIVER_BIAS_EN, 0x1a);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_VMODE_CTRL1, 0x40);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_PRE_STALL_LDO_BOOST_EN, 0x30);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_INTERFACE_SELECT, 0x3d);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_CLKBUF_ENABLE, 0x0f);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_RESET_TSYNC_EN, 0x03);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_TRAN_DRVR_EMP_EN, 0x03);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_PARRATE_REC_DETECT_IDLE_EN, 0x00);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_TX_INTERFACE_MODE, 0x00);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_TX_EMP_POST1_LVL, 0x2b);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_TX_DRV_LVL, 0x2f);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_TX_BAND, 0x4);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_RES_CODE_LANE_OFFSET_TX, 0x12);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_RES_CODE_LANE_OFFSET_RX, 0x12);

	/* TX-1 register configuration */
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_TRANSCEIVER_BIAS_EN, 0x1a);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_VMODE_CTRL1, 0x40);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_PRE_STALL_LDO_BOOST_EN, 0x30);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_INTERFACE_SELECT, 0x3d);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_CLKBUF_ENABLE, 0x0f);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_RESET_TSYNC_EN, 0x03);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_TRAN_DRVR_EMP_EN, 0x03);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_PARRATE_REC_DETECT_IDLE_EN, 0x00);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_TX_INTERFACE_MODE, 0x00);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_TX_EMP_POST1_LVL, 0x2b);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_TX_DRV_LVL, 0x2f);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_TX_BAND, 0x4);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_RES_CODE_LANE_OFFSET_TX, 0x12);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_RES_CODE_LANE_OFFSET_RX, 0x12);
	wmb(); /* make sure write happens */

	/* PHY VCO divider programming */
	dp_pll_write(dp_phy,
		DP_PHY_VCO_DIV, pdb->phy_vco_div);
	wmb(); /* make sure write happens */

	dp_pll_write(dp_pll,
		QSERDES_COM_CMN_CONFIG, 0x02);
	wmb(); /* make sure write happens */

	return res;
}

static bool dp_14nm_pll_lock_status(struct dp_pll *pll)
{
	u32 status;
	bool pll_locked;

	/* poll for PLL lock status */
	if (readl_poll_timeout_atomic((dp_pll_get_base(dp_pll) +
			QSERDES_COM_C_READY_STATUS),
			status,
			((status & BIT(0)) > 0),
			DP_PLL_POLL_SLEEP_US,
			DP_PLL_POLL_TIMEOUT_US)) {
		pr_err("C_READY status is not high. Status=%x\n", status);
		pll_locked = false;
	} else {
		pll_locked = true;
	}

	return pll_locked;
}

static bool dp_14nm_phy_rdy_status(struct dp_pll *pll)
{
	u32 status;
	bool phy_ready = true;

	/* poll for PHY ready status */
	if (readl_poll_timeout_atomic((dp_pll_get_base(dp_phy) +
			DP_PHY_STATUS),
			status,
			((status & (BIT(1) | BIT(0))) > 0),
			DP_PHY_POLL_SLEEP_US,
			DP_PHY_POLL_TIMEOUT_US)) {
		pr_err("Phy_ready is not high. Status=%x\n", status);
		phy_ready = false;
	}

	return phy_ready;
}

static int dp_pll_enable_14nm(struct clk_hw *hw)
{
	int rc = 0;
	struct dp_pll_vco_clk *vco = to_dp_vco_hw(hw);
	struct dp_pll *pll = vco->priv;

	dp_pll_write(dp_phy, DP_PHY_CFG, 0x01);
	dp_pll_write(dp_phy, DP_PHY_CFG, 0x05);
	dp_pll_write(dp_phy, DP_PHY_CFG, 0x01);
	dp_pll_write(dp_phy, DP_PHY_CFG, 0x09);
	wmb(); /* Make sure the PHY register writes are done */

	dp_pll_write(dp_pll,
		QSERDES_COM_RESETSM_CNTRL, 0x20);
	wmb();	/* Make sure the PLL register writes are done */

	udelay(900); /* hw recommended delay for full PU */

	if (!dp_14nm_pll_lock_status(pll)) {
		rc = -EINVAL;
		goto lock_err;
	}

	dp_pll_write(dp_phy, DP_PHY_CFG, 0x19);
	wmb();	/* Make sure the PHY register writes are done */

	udelay(10); /* hw recommended delay */

	if (!dp_14nm_phy_rdy_status(pll)) {
		rc = -EINVAL;
		goto lock_err;
	}

	pr_debug("PLL is locked\n");

	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_TRANSCEIVER_BIAS_EN, 0x3f);
	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_HIGHZ_DRVR_EN, 0x10);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_TRANSCEIVER_BIAS_EN, 0x3f);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_HIGHZ_DRVR_EN, 0x10);

	dp_pll_write(dp_phy,
		QSERDES_TX0_OFFSET + TXn_TX_POL_INV, 0x0a);
	dp_pll_write(dp_phy,
		QSERDES_TX1_OFFSET + TXn_TX_POL_INV, 0x0a);

	/*
	 * Switch DP Mainlink clock (cc_dpphy_link_clk) from DP
	 * controller side with final frequency
	 */
	dp_pll_write(dp_phy, DP_PHY_CFG, 0x18);
	wmb();	/* Make sure the PHY register writes are done */
	dp_pll_write(dp_phy, DP_PHY_CFG, 0x19);
	wmb();	/* Make sure the PHY register writes are done */

lock_err:
	return rc;
}

static int dp_pll_disable_14nm(struct clk_hw *hw)
{
	struct dp_pll_vco_clk *vco = to_dp_vco_hw(hw);
	struct dp_pll *pll = vco->priv;

	/* Assert DP PHY power down */
	dp_pll_write(dp_phy, DP_PHY_PD_CTL, 0x2);
	/*
	 * Make sure all the register writes to disable PLL are
	 * completed before doing any other operation
	 */
	wmb();

	return 0;
}

int dp_vco_set_rate_14nm(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate);

int dp_vco_prepare_14nm(struct clk_hw *hw)
{
	int rc = 0;
	struct dp_pll_vco_clk *vco;
	struct dp_pll *pll;

	if (!hw) {
		DP_ERR("invalid input parameters\n");
		return -EINVAL;
	}

	vco = to_dp_vco_hw(hw);
	pll = vco->priv;

	pr_debug("rate=%ld\n", vco->rate);

	if ((pll->vco_cached_rate != 0)
		&& (pll->vco_cached_rate == vco->rate)) {
		rc = dp_vco_set_rate_14nm(hw,
			pll->vco_cached_rate, pll->vco_cached_rate);
		if (rc) {
			pr_err("index=%d vco_set_rate failed. rc=%d\n",
				rc, pll->index);
			goto error;
		}
	}

	rc = dp_pll_enable_14nm(hw);
	if (rc) {
		pr_err("ndx=%d failed to enable dp pll\n",
					pll->index);
		goto error;
	}

error:
	return rc;
}

void dp_vco_unprepare_14nm(struct clk_hw *hw)
{
	struct dp_pll_vco_clk *vco;
	struct dp_pll *pll;

	if (!hw) {
		DP_ERR("invalid input parameters\n");
		return;
	}

	vco = to_dp_vco_hw(hw);
	pll = vco->priv;

	if (!pll) {
		DP_ERR("invalid input parameter\n");
		return;
	}

	pll->vco_cached_rate = vco->rate;
	dp_pll_disable_14nm(hw);
}

int dp_vco_set_rate_14nm(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	struct dp_pll_vco_clk *vco;
	int rc;
	struct dp_pll *pll;

	if (!hw) {
		DP_ERR("invalid input parameters\n");
		return -EINVAL;
	}

	vco = to_dp_vco_hw(hw);
	pll = vco->priv;

	pr_debug("DP lane CLK rate=%ld\n", rate);

	rc = dp_config_vco_rate_14nm(vco, rate);
	if (rc)
		pr_err("Failed to set clk rate\n");

	vco->rate = rate;

	return 0;
}

unsigned long dp_vco_recalc_rate_14nm(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct dp_pll_vco_clk *vco;
	u32 div, hsclk_div;
	u64 vco_rate;
	struct dp_pll *pll;

	if (!hw) {
		DP_ERR("invalid input parameters\n");
		return 0;
	}

	vco = to_dp_vco_hw(hw);
	pll = vco->priv;

	if (is_gdsc_disabled(pll))
		return 0;

	div = dp_pll_read(dp_pll, QSERDES_COM_HSCLK_SEL);
	div &= 0x0f;

	if (div == 12)
		hsclk_div = 5; /* Default */
	else if (div == 4)
		hsclk_div = 3;
	else if (div == 0)
		hsclk_div = 2;
	else {
		pr_debug("unknown divider. forcing to default\n");
		hsclk_div = 5;
	}

	if (hsclk_div == 5)
		vco_rate = DP_VCO_HSCLK_RATE_1620MHZDIV1000;
	else if (hsclk_div == 3)
		vco_rate = DP_VCO_HSCLK_RATE_2700MHZDIV1000;
	else
		vco_rate = DP_VCO_HSCLK_RATE_5400MHZDIV1000;

	pr_debug("returning vco rate = %lu\n", (unsigned long)vco_rate);

	pll->vco_cached_rate = vco->rate = vco_rate;
	return (unsigned long)vco_rate;
}

long dp_vco_round_rate_14nm(struct clk_hw *hw, unsigned long rate,
			unsigned long *parent_rate)
{
	unsigned long rrate = rate;
	struct dp_pll_vco_clk *vco;

	if (!hw) {
		DP_ERR("invalid input parameters\n");
		return 0;
	}

	vco = to_dp_vco_hw(hw);

	if (rate <= vco->min_rate)
		rrate = vco->min_rate;
	else if (rate <= DP_VCO_HSCLK_RATE_2700MHZDIV1000)
		rrate = DP_VCO_HSCLK_RATE_2700MHZDIV1000;
	else
		rrate = vco->max_rate;

	pr_debug("rrate=%ld\n", rrate);

	if (parent_rate)
		*parent_rate = rrate;
	return rrate;
}

static struct regmap_bus dp_pixel_mux_regmap_ops = {
	.reg_write = dp_mux_set_parent_14nm,
	.reg_read = dp_mux_get_parent_14nm,
};

/* Op structures */
static const struct clk_ops dp_14nm_vco_clk_ops = {
	.recalc_rate = dp_vco_recalc_rate_14nm,
	.set_rate = dp_vco_set_rate_14nm,
	.round_rate = dp_vco_round_rate_14nm,
	.prepare = dp_vco_prepare_14nm,
	.unprepare = dp_vco_unprepare_14nm,
};

static struct dp_pll_vco_clk dp_vco_clk = {
	.min_rate = DP_VCO_HSCLK_RATE_1620MHZDIV1000,
	.max_rate = DP_VCO_HSCLK_RATE_5400MHZDIV1000,
	.hw.init = &(struct clk_init_data){
		.name = "dp_vco_clk",
		.parent_names = (const char *[]){ "xo_board" },
		.num_parents = 1,
		.ops = &dp_14nm_vco_clk_ops,
	},
};

static struct clk_hw *mdss_dp_pllcc_14nm[] = {
	[DP_VCO_CLK] = &dp_vco_clk.hw,
	[DP_PHY_PLL_LINK_CLK] = &dp_phy_pll_link_clk.hw,
	[DP_VCO_DIVSEL_FOUR_CLK_SRC] = &dp_vco_divsel_four_clk_src.hw,
	[DP_VCO_DIVSEL_TWO_CLK_SRC] = &dp_vco_divsel_two_clk_src.hw,
	[DP_PHY_PLL_VCO_DIV_CLK] = &dp_phy_pll_vco_div_clk.clkr.hw,
};

static struct dp_pll_14nm_db dp_pdb;

int dp_pll_clock_register_14nm(struct dp_pll *pll)
{
	int rc = -ENOTSUPP, i = 0;
	struct platform_device *pdev;
	struct clk *clk;
	struct regmap *regmap;
	int num_clks = ARRAY_SIZE(mdss_dp_pllcc_14nm);

	if (!pll) {
		DP_ERR("pll data not initialized\n");
		return -EINVAL;
	}
	pdev = pll->pdev;

	pll->clk_data = devm_kzalloc(&pdev->dev, sizeof(*pll->clk_data), GFP_KERNEL);
	if (!pll->clk_data)
		return -ENOMEM;

	pll->clk_data->clks = devm_kcalloc(&pdev->dev, num_clks,
				sizeof(struct clk *), GFP_KERNEL);
	if (!pll->clk_data->clks) {
		kfree(pll->clk_data);
		return -ENOMEM;
	}

	pll->clk_data->clk_num = num_clks;

	pll->priv = &dp_pdb;
	dp_pdb.pll = pll;

	/* Set client data for vco, mux and div clocks */
	regmap = devm_regmap_init(&pdev->dev, &dp_pixel_mux_regmap_ops,
			pll, &dp_pll_14nm_cfg);
	mux_clk_ops = clk_regmap_mux_closest_ops;
	mux_clk_ops.determine_rate = clk_mux_determine_rate;
	mux_clk_ops.recalc_rate = mux_recalc_rate;

	dp_vco_clk.priv = pll;
	dp_phy_pll_vco_div_clk.clkr.regmap = regmap;

	for (i = DP_VCO_CLK; i <= DP_PHY_PLL_VCO_DIV_CLK; i++) {
		pr_debug("reg clk: %d index: %d\n", i, pll->index);
		clk = clk_register(&pdev->dev, mdss_dp_pllcc_14nm[i]);
		if (IS_ERR(clk)) {
			pr_err("clk registration failed for DP: %d\n",
					pll->index);
			rc = -EINVAL;
			goto clk_reg_fail;
		}
		pll->clk_data->clks[i] = clk;
	}

	rc = of_clk_add_provider(pdev->dev.of_node,
			of_clk_src_onecell_get, pll->clk_data);
	if (rc) {
		pr_err("Clock register failed rc=%d\n", rc);
		rc = -EPROBE_DEFER;
		goto clk_reg_fail;
	} else {
		pr_debug("SUCCESS\n");
	}
	return rc;
clk_reg_fail:
	dp_pll_clock_unregister_14nm(pll);
	return rc;
}

void dp_pll_clock_unregister_14nm(struct dp_pll *pll)
{
	kfree(pll->clk_data->clks);
	kfree(pll->clk_data);
}
