// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/*
 * Copyright (c) 2016-2020, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifndef _DT_BINDINGS_CLK_QCOM_GCC_IPQ5332_H
#define _DT_BINDINGS_CLK_QCOM_GCC_IPQ5332_H

#define GCC_ADSS_PWM_CLK				0
#define GCC_ADSS_PWM_CLK_SRC				1
#define GCC_AHB_CLK					2
#define GCC_APSS_AHB_CLK				3
#define GCC_APSS_AHB_CLK_SRC				4
#define GCC_APSS_AXI_CLK				5
#define GCC_APSS_AXI_CLK_SRC				6
#define GCC_BLSP1_AHB_CLK				7
#define GCC_BLSP1_QUP1_I2C_APPS_CLK			8
#define GCC_BLSP1_QUP1_SPI_APPS_CLK			9
#define GCC_BLSP1_QUP1_SPI_APPS_CLK_SRC			10
#define GCC_BLSP1_QUP2_I2C_APPS_CLK			11
#define GCC_BLSP1_QUP2_SPI_APPS_CLK			12
#define GCC_BLSP1_QUP2_SPI_APPS_CLK_SRC			13
#define GCC_BLSP1_QUP3_I2C_APPS_CLK			14
#define GCC_BLSP1_QUP3_SPI_APPS_CLK			15
#define GCC_BLSP1_QUP3_SPI_APPS_CLK_SRC			16
#define GCC_BLSP1_SLEEP_CLK				17
#define GCC_BLSP1_UART1_APPS_CLK			18
#define GCC_BLSP1_UART1_APPS_CLK_SRC			19
#define GCC_BLSP1_UART2_APPS_CLK			20
#define GCC_BLSP1_UART2_APPS_CLK_SRC			21
#define GCC_BLSP1_UART3_APPS_CLK			22
#define GCC_BLSP1_UART3_APPS_CLK_SRC			23
#define GCC_CE_AHB_CLK					24
#define GCC_CE_AXI_CLK					25
#define GCC_CE_PCNOC_AHB_CLK				26
#define GCC_CMN_12GPLL_AHB_CLK				27
#define GCC_CMN_12GPLL_APU_CLK				28
#define GCC_CMN_12GPLL_SYS_CLK				29
#define GCC_DCC_CLK					30
#define GCC_DCC_XO_CLK					31
#define GCC_GP1_CLK					32
#define GCC_GP1_CLK_SRC					33
#define GCC_GP2_CLK					34
#define GCC_GP2_CLK_SRC					35
#define GCC_LPASS_CORE_AXIM_CLK				36
#define GCC_LPASS_SWAY_CLK				37
#define GCC_LPASS_SWAY_CLK_SRC				38
#define GCC_MDIO_AHB_CLK				39
#define GCC_MDIO_SLAVE_AHB_CLK				40
#define GCC_MEM_NOC_Q6_AXI_CLK				41
#define GCC_NSS_TS_CLK					44
#define GCC_NSS_TS_CLK_SRC				45
#define GCC_NSSCC_CLK					46
#define GCC_NSSCFG_CLK					47
#define GCC_NSSNOC_ATB_CLK				48
#define GCC_NSSNOC_NSSCC_CLK				49
#define GCC_NSSNOC_QOSGEN_REF_CLK			50
#define GCC_NSSNOC_SNOC_1_CLK				51
#define GCC_NSSNOC_SNOC_CLK				52
#define GCC_NSSNOC_TIMEOUT_REF_CLK			53
#define GCC_NSSNOC_XO_DCD_CLK				54
#define GCC_PCIE3X1_0_AHB_CLK				55
#define GCC_PCIE3X1_0_AUX_CLK				56
#define GCC_PCIE3X1_0_AXI_CLK_SRC			57
#define GCC_PCIE3X1_0_AXI_M_CLK				58
#define GCC_PCIE3X1_0_AXI_S_BRIDGE_CLK			59
#define GCC_PCIE3X1_0_AXI_S_CLK				60
#define GCC_PCIE3X1_0_PIPE_CLK				61
#define GCC_PCIE3X1_0_RCHG_CLK_SRC			62
#define GCC_PCIE3X1_1_AHB_CLK				63
#define GCC_PCIE3X1_1_AUX_CLK				64
#define GCC_PCIE3X1_1_AXI_CLK_SRC			65
#define GCC_PCIE3X1_1_AXI_M_CLK				66
#define GCC_PCIE3X1_1_AXI_S_BRIDGE_CLK			67
#define GCC_PCIE3X1_1_AXI_S_CLK				68
#define GCC_PCIE3X1_1_PIPE_CLK				69
#define GCC_PCIE3X1_1_RCHG_CLK_SRC			70
#define GCC_PCIE3X1_PHY_AHB_CLK				71
#define GCC_PCIE3X2_AHB_CLK				72
#define GCC_PCIE3X2_AUX_CLK				73
#define GCC_PCIE3X2_AXI_M_CLK				74
#define GCC_PCIE3X2_AXI_M_CLK_SRC			75
#define GCC_PCIE3X2_AXI_S_BRIDGE_CLK			76
#define GCC_PCIE3X2_AXI_S_CLK				77
#define GCC_PCIE3X2_AXI_S_CLK_SRC			78
#define GCC_PCIE3X2_PHY_AHB_CLK				79
#define GCC_PCIE3X2_PIPE_CLK				80
#define GCC_PCIE3X2_RCHG_CLK_SRC			81
#define GCC_PCIE_AUX_CLK_SRC				82
#define GCC_PCNOC_AT_CLK				83
#define GCC_PCNOC_BFDCD_CLK_SRC				84
#define GCC_PCNOC_DCC_CLK				85
#define GCC_PCNOC_LPASS_CLK				86
#define GCC_PCNOC_TS_CLK				87
#define GCC_PRNG_AHB_CLK				88
#define GCC_Q6_AHB_CLK					89
#define GCC_Q6_AHB_S_CLK				90
#define GCC_Q6_AXIM_CLK					91
#define GCC_Q6_AXIM_CLK_SRC				92
#define GCC_Q6_AXIS_CLK					93
#define GCC_Q6_TSCTR_1TO2_CLK				94
#define GCC_Q6SS_ATBM_CLK				95
#define GCC_Q6SS_PCLKDBG_CLK				96
#define GCC_Q6SS_TRIG_CLK				97
#define GCC_QDSS_APB2JTAG_CLK				98
#define GCC_QDSS_AT_CLK					99
#define GCC_QDSS_AT_CLK_SRC				100
#define GCC_QDSS_CFG_AHB_CLK				101
#define GCC_QDSS_DAP_AHB_CLK				102
#define GCC_QDSS_DAP_CLK				103
#define GCC_QDSS_DAP_DIV_CLK_SRC			104
#define GCC_QDSS_ETR_USB_CLK				105
#define GCC_QDSS_EUD_AT_CLK				106
#define GCC_QDSS_STM_CLK				107
#define GCC_QDSS_STM_CLK_SRC				108
#define GCC_QDSS_TRACECLKIN_CLK				109
#define GCC_QDSS_TRACECLKIN_CLK_SRC			110
#define GCC_QDSS_TS_CLK					111
#define GCC_QDSS_TSCTR_CLK_SRC				112
#define GCC_QDSS_TSCTR_DIV16_CLK			113
#define GCC_QDSS_TSCTR_DIV2_CLK				114
#define GCC_QDSS_TSCTR_DIV3_CLK				115
#define GCC_QDSS_TSCTR_DIV4_CLK				116
#define GCC_QDSS_TSCTR_DIV8_CLK				117
#define GCC_QPIC_AHB_CLK				118
#define GCC_QPIC_CLK					119
#define GCC_QPIC_IO_MACRO_CLK				120
#define GCC_QPIC_IO_MACRO_CLK_SRC			121
#define GCC_QPIC_SLEEP_CLK				122
#define GCC_SDCC1_AHB_CLK				123
#define GCC_SDCC1_APPS_CLK				124
#define GCC_SDCC1_APPS_CLK_SRC				125
#define GCC_SLEEP_CLK_SRC				126
#define GCC_SNOC_LPASS_CFG_CLK				127
#define GCC_SNOC_NSSNOC_1_CLK				128
#define GCC_SNOC_NSSNOC_CLK				129
#define GCC_SNOC_PCIE3_1LANE_1_M_CLK			130
#define GCC_SNOC_PCIE3_1LANE_1_S_CLK			131
#define GCC_SNOC_PCIE3_1LANE_M_CLK			132
#define GCC_SNOC_PCIE3_1LANE_S_CLK			133
#define GCC_SNOC_PCIE3_2LANE_M_CLK			134
#define GCC_SNOC_PCIE3_2LANE_S_CLK			135
#define GCC_SNOC_PCNOC_AHB_CLK				136
#define GCC_SNOC_TS_CLK					137
#define GCC_SNOC_USB_CLK				138
#define GCC_SYS_NOC_AT_CLK				139
#define GCC_SYS_NOC_AXI_CLK				140
#define GCC_SYS_NOC_QDSS_STM_AXI_CLK			141
#define GCC_SYS_NOC_WCSS_AHB_CLK			142
#define GCC_SYSTEM_NOC_BFDCD_CLK_SRC			143
#define GCC_UNIPHY0_AHB_CLK				144
#define GCC_UNIPHY0_SYS_CLK				145
#define GCC_UNIPHY1_AHB_CLK				146
#define GCC_UNIPHY1_SYS_CLK				147
#define GCC_UNIPHY_SYS_CLK_SRC				148
#define GCC_USB0_AUX_CLK				149
#define GCC_USB0_AUX_CLK_SRC				150
#define GCC_USB0_EUD_AT_CLK				151
#define GCC_USB0_LFPS_CLK				152
#define GCC_USB0_LFPS_CLK_SRC				153
#define GCC_USB0_MASTER_CLK				154
#define GCC_USB0_MASTER_CLK_SRC				155
#define GCC_USB0_MOCK_UTMI_CLK				156
#define GCC_USB0_MOCK_UTMI_CLK_SRC			157
#define GCC_USB0_MOCK_UTMI_DIV_CLK_SRC			158
#define GCC_USB0_PHY_CFG_AHB_CLK			159
#define GCC_USB0_PIPE_CLK				160
#define GCC_USB0_SLEEP_CLK				161
#define GCC_WCSS_AHB_CLK_SRC				162
#define GCC_WCSS_AXIM_CLK				163
#define GCC_WCSS_AXIS_CLK				164
#define GCC_WCSS_DBG_IFC_APB_BDG_CLK			165
#define GCC_WCSS_DBG_IFC_APB_CLK			166
#define GCC_WCSS_DBG_IFC_ATB_BDG_CLK			167
#define GCC_WCSS_DBG_IFC_ATB_CLK			168
#define GCC_WCSS_DBG_IFC_NTS_BDG_CLK			169
#define GCC_WCSS_DBG_IFC_NTS_CLK			170
#define GCC_WCSS_ECAHB_CLK				171
#define GCC_WCSS_MST_ASYNC_BDG_CLK			172
#define GCC_WCSS_SLV_ASYNC_BDG_CLK			173
#define GCC_XO_CLK					174
#define GCC_XO_CLK_SRC					175
#define GCC_XO_DIV4_CLK					176
#define GCC_IM_SLEEP_CLK				177
#define GCC_NSSNOC_PCNOC_1_CLK				178
#define GCC_SNOC_QOSGEN_EXTREF_DIV_CLK_SRC		181
#define GCC_PCIE3X2_PIPE_CLK_SRC			183
#define GCC_PCIE3X1_0_PIPE_CLK_SRC			184
#define GCC_PCIE3X1_1_PIPE_CLK_SRC			185
#define GCC_USB0_PIPE_CLK_SRC				186
#define GPLL0_MAIN					187
#define GPLL0						188
#define GPLL2_MAIN					189
#define GPLL2						190
#define GPLL4_MAIN					191
#define GPLL4						192
#define GCC_PCIE3X2_RCHG_CLK				193
#define GCC_PCIE3X1_0_RCHG_CLK				194
#define GCC_PCIE3X1_1_RCHG_CLK				195


#endif
