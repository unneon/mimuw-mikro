#pragma once

#define LIS35DE_CTRL1 0x20
#define LIS35DE_CTRL1_PD (1 << 6)
#define LIS35DE_CTRL1_ZEN (1 << 2)
#define LIS35DE_CTRL1_YEN (1 << 1)
#define LIS35DE_CTRL1_XEN (1 << 0)

#define LIS35DE_CTRL3 0x22
#define LIS35DE_CTRL3_I1CFG_CLICK 0b111

#define LIS35DE_OUTX 0x29

#define LIS35DE_OUTY 0x2B

#define LIS35DE_OUTZ 0x2D

#define LIS35DE_CLICKCFG 0x38
#define LIS35DE_CLICKCFG_LIR (1 << 6)
#define LIS35DE_CLICKCFG_DOUBLEZ (1 << 5)
#define LIS35DE_CLICKCFG_SINGLEZ (1 << 4)
#define LIS35DE_CLICKCFG_DOUBLEY (1 << 3)
#define LIS35DE_CLICKCFG_SINGLEY (1 << 2)
#define LIS35DE_CLICKCFG_DOUBLEX (1 << 1)
#define LIS35DE_CLICKCFG_SINGLEX (1 << 0)

#define LIS35DE_CLICKSRC 0x39

#define LIS35DE_CLICKTIMELIMIT 0x3D

#define LIS35DE_CLICKLATENCY 0x3E

#define LIS35DE_CLICKWINDOW 0x3F
