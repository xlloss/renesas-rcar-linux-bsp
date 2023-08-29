/*
 * Copyright (C) 2017 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _TP2854_H
#define _TP2854_H

#define DRIVER_VERSION "For tp2854B Driver Ver 3.1"

#define TP2854_ADDR 0x44
#define TP2854_ID_1 0x28
#define TP2854_ID_2 0x54

/* MIPI PAGE = 0 */
#define TP2854_TVP_CLK_74M 0
#define TP2854_TVP_CLK_148M 1

#define TP2854_PAGE 0x40
#define MIPI_PAGE_ENABLE 0x0C
#define MIPI_PAGE_DISABLE 0x04
#define MIPI_PAGE_DISABLE_AW 0x04
#define MIPI_PAGE_DISABLE_AR 0x00

#define TP2854_INPUT_STATUS 0x01

#define TP2854_DECODE_CTRL 0x02
#define TP2854_EQ2_CTRL 0x07
#define TP2854_EQ1_CTRL 0x0B

#define TP2854_EQ1_HYSTER 0x0C
#define EQ_CLK_FSEL (1 << 4)
#define EQ_CLK_148M 0
#define EQ_CLK_74M 1

#define TP2854_COMBFILTER_SDFMT_CTL 0x0D
#define TP2854_DECODE_STATUS 0x02

#define TP2854_NPXL_H 0x1C
#define TP2854_NPXL_L 0x1D
#define TP2854_CLK_DATA_OUT 0x4E
#define TP2854_SYS_CLK_CTRL 0xF5
#define TP2854_COL_H_PLL_FR_CTL 0x2A
#define BLUE_SCREEN (4 << 0)

#define TP2854_ID1_REG 0xFE
#define TP2854_ID2_REG 0xFF
#define TP2854_OUT_H_DELAY_H 0x15
#define TP2854_OUT_H_DELAY_L 0x16
#define TP2854_OUT_H_ACTIVE_L 0x17
#define TP2854_OUT_V_DELAY_L 0x18
#define TP2854_OUT_V_ACTIVE_L 0x19
#define TP2854_OUT_V_H_ACTIVE_H 0x1A

#define TP2854_MISC_CTL 0x35
#define FSL_74MHZ_148MHZ_SYS_CLK (1 << 5)
#define FSL_74MHZ 1
#define FSL_148MHZ 0

#define TP2854_PCLAMP_CTRL 0x20

#define TP2854_ClAMP_GAIN_CTRL 0x21
#define WPGN 4
#define CLPGN 0

#define TP2854_SYNC_AMPACG_CTRL 0x22
#define TP2854_COLOR_KILL_TH 0x2B
#define CKILLHY 6
#define CGAINMAX 0

#define TP2854_SYNC_CLAMP_CTRL 0x23

#define TP2854_T_LPX 0x25
#define TP2854_T_PREP 0x26
#define TP2854_T_TRAIL 0x27
#define TP2854_T_EXIT 0x29

#define TP2854B_TX_TLPX_4LAN 0x04
#define TP2854B_TX_PREP_4LAN 0x03
#define TP2854B_TX_TRAIL_4LAN 0x09
#define TP2854B_TX_EXIT_4LAN 0x02

#define TP2854B_TX_TLPX_2LAN 0x08
#define TP2854B_TX_PREP_2LAN 0x06
#define TP2854B_TX_TRAIL_2LAN 0x11
#define TP2854B_TX_EXIT_2LAN 0x0a


#define TP2854_COLOR_PLL 0x2C
#define TP2854_COLOR_BURST_GAIN 0x2D
#define TP2854_COLOR_GAIN_REF 0x2E
#define TP2854_COLOR_CARR_DDS_BFSTD_4 0x30
#define TP2854_COLOR_CARR_DDS_BFSTD_3 0x31
#define TP2854_COLOR_CARR_DDS_BFSTD_2 0x32
#define TP2854_COLOR_CARR_DDS 0x33
#define BFSTD_2 4
#define HS 3
#define HPRNG 2
#define FGAP 0

#define TP2854_LPF_CTRL 0x39

/* MIPI PAGE = 1 */
#define TP2854_MIPI_CLK_LAEN_CTRL 0x01
#define TP2854_MIPI_CLK_EN 0x02
#define CLK_LANE_PWD 1 << 1
#define MIPI_CLK_EN 1 << 0

#define TP2854_MIPI_OUTPUT_EN 0x08
#define ENABLE_ALL_LANES 0x0F
#define ENABLE_LANE12 0x03
#define DISABLE_ALL_LANES 0x00

#define TP2854_MIPI_PLL_CTRL4 0x13
#define OUT_DIV_EN (1 << 5)
#define OUT_VCO_OUT 0
#define OUT_DIVOUT_XIN 1

#define TP2854_MIPI_PLL_CTRL5 0x14
#define RST_CLK_GEN (1 << 7)
#define DIV_BIT_CLK_FPLL_2 (4 << 4)
#define DIV_PHY_CLK_FPLL_7 (7 << 0)
#define DIV_PHY_CLK_FPLL_6 (6 << 0)

#define TP2854_MIPI_PLL_CTRL6 0x15
#define DIV_CSI_CLK_16 (1 << 0)

#define TP2854_MIPI_NUM_LAN 0x20
#define NUM_CHANNELS(x) (x << 4)
#define NUM_LANES(x) (x << 0)

#define TP2854_MIPI_VERTUAL_CHID 0x34
#define VCID_CH4VC0_CH3VC1_CH2VC2_CH1VC3 0x1b
#define VCID_CH4VC4_CH3VC3_CH2VC2_CH1VC1 0xe4

#define TP2854_MIPI_STOPCLK 0x23
#define MIPI_CLK_NORMAL 0x0
#define MIPI_CLK_STOP 0x2

#define CHANNEL_NUM 4

#define NPXL_720P_60 0
#define NPXL_720P_50 1
#define NPXL_720P_30 2
#define NPXL_720P_25 3
#define NPXL_1080P_30 4
#define NPXL_1080P_25 5
#define NPXL_480I 6
#define NPXL_576I 7

#define TP2854_SUBCAM_0_FAKE_SLAV 0x40
#define TP2854_SUBCAM_1_FAKE_SLAV 0x41
#define TP2854_SUBCAM_2_FAKE_SLAV 0x42
#define TP2854_SUBCAM_3_FAKE_SLAV 0x43

#define TP2854_SUBCAM_4_FAKE_SLAV 0x50
#define TP2854_SUBCAM_5_FAKE_SLAV 0x51
#define TP2854_SUBCAM_6_FAKE_SLAV 0x52
#define TP2854_SUBCAM_7_FAKE_SLAV 0x53

#endif
