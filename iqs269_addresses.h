/**
	*****************************************************************************
	* @file			iqs269_addresses.h

	* @brief 		IQS269 memory map addresses to be used with IQS269 Driver module.
  * @note     All memory map addresses are preceded with 'IQS269_MM_' to prevent
  *           possible conflicts with other IQS device memory maps.
	*****************************************************************************/
  
#ifndef __IQS269_ADDRESSES_H
#define __IQS269_ADDRESSES_H

/** @name     IQS269 Memory Map
  * @{
  */
/* Device Information - Read Only */
#define IQS269_MM_PRODUCT_NUMBER              0x00  // Byte0
#define IQS269_MM_SOFTWARE_VERSION            0x00  // Byte1
#define IQS269_MM_HWFN                        0x01
#define IQS269_MM_DEVELOPMENT_VERSION         0x01

/* Global Flags - Read Only */
#define IQS269_MM_SYSTEM_FLAGS                0x02
#define IQS269_MM_GLOBAL_EVENTS               0x02
#define IQS269_MM_GESTURE_EVENTS              0x03
#define IQS269_MM_RESERVED0                   0x03
#define IQS269_MM_PROX_STATE                  0x04
#define IQS269_MM_PROX_DIRECTION              0x04
#define IQS269_MM_TOUCH_STATE                 0x05
#define IQS269_MM_DEEP_TOUCH_STATE            0x05
#define IQS269_MM_FOLLOWER_ACTIVE             0x06
#define IQS269_MM_FOLLOWER_MOVEMENT           0x06
#define IQS269_MM_MOVEMENT_DIRECTION          0x07
#define IQS269_MM_MULTI_DIRECTION             0x07

/* Channel Counts and LTA - Read Only */
#define IQS269_MM_CNT_CH0_L                   0x08
#define IQS269_MM_CNT_CH0_H                   0x08
#define IQS269_MM_LTA_CH0_L                   0x09
#define IQS269_MM_LTA_CH0_H                   0x09
#define IQS269_MM_CNT_CH1_L                   0x0A
#define IQS269_MM_CNT_CH1_H                   0x0A
#define IQS269_MM_LTA_CH1_L                   0x0B
#define IQS269_MM_LTA_CH1_H                   0x0B
#define IQS269_MM_CNT_CH2_L                   0x0C
#define IQS269_MM_CNT_CH2_H                   0x0C
#define IQS269_MM_LTA_CH2_L                   0x0D
#define IQS269_MM_LTA_CH2_H                   0x0D
#define IQS269_MM_CNT_CH3_L                   0x0E
#define IQS269_MM_CNT_CH3_H                   0x0E
#define IQS269_MM_LTA_CH3_L                   0x0F
#define IQS269_MM_LTA_CH3_H                   0x0F
#define IQS269_MM_CNT_CH4_L                   0x10
#define IQS269_MM_CNT_CH4_H                   0x10
#define IQS269_MM_LTA_CH4_L                   0x11
#define IQS269_MM_LTA_CH4_H                   0x11
#define IQS269_MM_CNT_CH5_L                   0x12
#define IQS269_MM_CNT_CH5_H                   0x12
#define IQS269_MM_LTA_CH5_L                   0x13
#define IQS269_MM_LTA_CH5_H                   0x13
#define IQS269_MM_CNT_CH6_L                   0x14
#define IQS269_MM_CNT_CH6_H                   0x14
#define IQS269_MM_LTA_CH6_L                   0x15
#define IQS269_MM_LTA_CH6_H                   0x15
#define IQS269_MM_CNT_CH7_L                   0x16
#define IQS269_MM_CNT_CH7_H                   0x16
#define IQS269_MM_LTA_CH7_L                   0x17
#define IQS269_MM_LTA_CH7_H                   0x17

/* Channel Delta Values -Read Only */
#define IQS269_MM_CH0_DTA_L                    0x18
#define IQS269_MM_CH0_DTA_H                    0x18
#define IQS269_MM_CH1_DTA_L                    0x19
#define IQS269_MM_CH1_DTA_H                    0x19
#define IQS269_MM_CH2_DTA_L                    0x1A
#define IQS269_MM_CH2_DTA_H                    0x1A
#define IQS269_MM_CH3_DTA_L                    0x1B
#define IQS269_MM_CH3_DTA_H                    0x1B
#define IQS269_MM_CH4_DTA_L                    0x1C
#define IQS269_MM_CH4_DTA_H                    0x1C
#define IQS269_MM_CH5_DTA_L                    0x1D
#define IQS269_MM_CH5_DTA_H                    0x1D
#define IQS269_MM_CH6_DTA_L                    0x1E
#define IQS269_MM_CH6_DTA_H                    0x1E
#define IQS269_MM_CH7_DTA_L                    0x1F
#define IQS269_MM_CH7_DTA_H                    0x1F

/* Follower UI - Read Only */
#define IQS269_MM_DCF_DTA_CH0_L                0x20
#define IQS269_MM_DCF_DTA_CH0_H                0x20
#define IQS269_MM_DCF_DTA_CH1_L                0x21
#define IQS269_MM_DCF_DTA_CH1_H                0x21
#define IQS269_MM_DCF_DTA_CH2_L                0x22
#define IQS269_MM_DCF_DTA_CH2_H                0x22
#define IQS269_MM_DCF_DTA_CH3_L                0x23
#define IQS269_MM_DCF_DTA_CH3_H                0x23
#define IQS269_MM_DCF_DTA_CH4_L                0x24
#define IQS269_MM_DCF_DTA_CH4_H                0x24
#define IQS269_MM_DCF_DTA_CH5_L                0x25
#define IQS269_MM_DCF_DTA_CH5_H                0x25
#define IQS269_MM_DCF_DTA_CH6_L                0x26
#define IQS269_MM_DCF_DTA_CH6_H                0x26
#define IQS269_MM_DCF_DTA_CH7_L                0x27
#define IQS269_MM_DCF_DTA_CH7_H                0x27

/* Movement UI - Read Only */
#define IQS269_MM_MOV_CNTR_CH0                0x28
#define IQS269_MM_CNTR_TMR_CH0                0x28
#define IQS269_MM_MOV_CNTR_CH1                0x29
#define IQS269_MM_CNTR_TMR_CH1                0x29
#define IQS269_MM_MOV_CNTR_CH2                0x2A
#define IQS269_MM_CNTR_TMR_CH2                0x2A
#define IQS269_MM_MOV_CNTR_CH3                0x2B
#define IQS269_MM_CNTR_TMR_CH3                0x2B
#define IQS269_MM_MOV_CNTR_CH4                0x2C
#define IQS269_MM_CNTR_TMR_CH4                0x2C
#define IQS269_MM_MOV_CNTR_CH5                0x2D
#define IQS269_MM_CNTR_TMR_CH5                0x2D
#define IQS269_MM_MOV_CNTR_CH6                0x2E
#define IQS269_MM_CNTR_TMR_CH6                0x2E
#define IQS269_MM_MOV_CNTR_CH7                0x2F
#define IQS269_MM_CNTR_TMR_CH7                0x2F

/* Slider UI - Read Only */
#define IQS269_MM_COORDINATE_SLDR0            0x30
#define IQS269_MM_COORDINATE_SLDR1            0x30

/* Absolute Capacitance UI - Read Only */
#define IQS269_MM_ABSCAP_BASE_L               0x31
#define IQS269_MM_ABSCAP_BASE_H               0x31
#define IQS269_MM_ABSCAP_CNT_L                0x32
#define IQS269_MM_ABSCAP_CNT_H                0x32
#define IQS269_MM_ABSCAP_CALCAP0_L            0x33
#define IQS269_MM_ABSCAP_CALCAP0_H            0x33
#define IQS269_MM_ABSCAP_CALCAP1_L            0x34
#define IQS269_MM_ABSCAP_CALCAP1_H            0x34

/* Calibration Values - Read Only */
#define IQS269_MM_CALIB_VALUE0_L              0x35
#define IQS269_MM_CALIB_VALUE0_H              0x35
#define IQS269_MM_CALIB_VALUE1_L              0x36
#define IQS269_MM_CALIB_VALUE1_H              0x36

/* Device Settings - Read/Write */
#define IQS269_MM_PMU_SETTINGS                0x80
#define IQS269_MM_I2C_SETTINGS                0x80
#define IQS269_MM_ACTIVE_CHANNELS             0x81
#define IQS269_MM_FILTER_SETTINGS             0x81
#define IQS269_MM_RESEED_ENABLE               0x82
#define IQS269_MM_GLOBAL_EVENTS_MASK          0x82
#define IQS269_MM_REPORT_RATE_NP              0x83
#define IQS269_MM_REPORT_RATE_LP              0x83
#define IQS269_MM_REPORT_RATE_ULP             0x84
#define IQS269_MM_MODE_TIMEOUT                0x84
#define IQS269_MM_WINDOW_TIMEOUT              0x85
#define IQS269_MM_HALT_TIMEOUT                0x85
#define IQS269_MM_PXS_GLBL_SETTINGS_0         0x86
#define IQS269_MM_PXS_GLBL_SETTINGS_1         0x86
#define IQS269_MM_ABS_CAP_SETTINGS            0x87
#define IQS269_MM_UI_GLBL_SETTINGS            0x87
#define IQS269_MM_BLOCK_FOLLOW_EVENT          0x88
#define IQS269_MM_MOVEMENT_CHANNELS           0x88
#define IQS269_MM_SLIDER0_CHANNELS            0x89
#define IQS269_MM_SLIDER1_CHANNELS            0x89
#define IQS269_MM_TAP_TIMEOUT                 0x8A
#define IQS269_MM_SWIPE_TIMEOUT               0x8A
#define IQS269_MM_GESTURE_THRESHOLD           0x8B
#define IQS269_MM_RESEED_CHANNELS             0x8B

/* Channel 0 Settings - Read/Write */
#define IQS269_MM_CH0_SELECT_CRX              0x8C
#define IQS269_MM_CH0_SELECT_CTX              0x8C
#define IQS269_MM_CH0_PROXCTL                 0x8D
#define IQS269_MM_CH0_PROXCG0                 0x8D
#define IQS269_MM_CH0_PROXCFG1                0x8E
#define IQS269_MM_CH0_ATI_TARGETS             0x8E
#define IQS269_MM_CH0_MULTIPLIERS             0x8F
#define IQS269_MM_CH0_PCC                     0x8F
#define IQS269_MM_CH0_THRESHOLD_P             0x90
#define IQS269_MM_CH0_THRESHOLD_T             0x90
#define IQS269_MM_CH0_THRESHOLD_DT            0x91
#define IQS269_MM_CH0_HYSTERESIS              0x91
#define IQS269_MM_CH0_FOLLOW_CHANNELS         0x92
#define IQS269_MM_CH0_FOLLOW_WEIGHT           0x92

/* Channel 1 Settings - Read/Write */
#define IQS269_MM_CH1_SELECT_CRX              0x93
#define IQS269_MM_CH1_SELECT_CTX              0x93
#define IQS269_MM_CH1_PROXCTL                 0x94
#define IQS269_MM_CH1_PROXCG0                 0x94
#define IQS269_MM_CH1_PROXCFG1                0x95
#define IQS269_MM_CH1_ATI_TARGETS             0x95
#define IQS269_MM_CH1_MULTIPLIERS             0x96
#define IQS269_MM_CH1_PCC                     0x96
#define IQS269_MM_CH1_THRESHOLD_P             0x97
#define IQS269_MM_CH1_THRESHOLD_T             0x97
#define IQS269_MM_CH1_THRESHOLD_DT            0x98
#define IQS269_MM_CH1_HYSTERESIS              0x98
#define IQS269_MM_CH1_FOLLOW_CHANNELS         0x99
#define IQS269_MM_CH1_FOLLOW_WEIGHT           0x99

/* Channel 2 Settings - Read/Write */
#define IQS269_MM_CH2_SELECT_CRX              0x9A
#define IQS269_MM_CH2_SELECT_CTX              0x9A
#define IQS269_MM_CH2_PROXCTL                 0x9B
#define IQS269_MM_CH2_PROXCG0                 0x9B
#define IQS269_MM_CH2_PROXCFG1                0x9C
#define IQS269_MM_CH2_ATI_TARGETS             0x9C
#define IQS269_MM_CH2_MULTIPLIERS             0x9D
#define IQS269_MM_CH2_PCC                     0x9D
#define IQS269_MM_CH2_THRESHOLD_P             0x9E
#define IQS269_MM_CH2_THRESHOLD_T             0x9E
#define IQS269_MM_CH2_THRESHOLD_DT            0x9F
#define IQS269_MM_CH2_HYSTERESIS              0x9F
#define IQS269_MM_CH2_FOLLOW_CHANNELS         0xA0
#define IQS269_MM_CH2_FOLLOW_WEIGHT           0xA0

/* Channel 3 Settings - Read/Write */
#define IQS269_MM_CH3_SELECT_CRX              0xA1
#define IQS269_MM_CH3_SELECT_CTX              0xA1
#define IQS269_MM_CH3_PROXCTL                 0xA2
#define IQS269_MM_CH3_PROXCG0                 0xA2
#define IQS269_MM_CH3_PROXCFG1                0xA3
#define IQS269_MM_CH3_ATI_TARGETS             0xA3
#define IQS269_MM_CH3_MULTIPLIERS             0xA4
#define IQS269_MM_CH3_PCC                     0xA4
#define IQS269_MM_CH3_THRESHOLD_P             0xA5
#define IQS269_MM_CH3_THRESHOLD_T             0xA5
#define IQS269_MM_CH3_THRESHOLD_DT            0xA6
#define IQS269_MM_CH3_HYSTERESIS              0xA6
#define IQS269_MM_CH3_FOLLOW_CHANNELS         0xA7
#define IQS269_MM_CH3_FOLLOW_WEIGHT           0xA7

/* Channel 4 Settings - Read/Write */
#define IQS269_MM_CH4_SELECT_CRX              0xA8
#define IQS269_MM_CH4_SELECT_CTX              0xA8
#define IQS269_MM_CH4_PROXCTL                 0xA9
#define IQS269_MM_CH4_PROXCG0                 0xA9
#define IQS269_MM_CH4_PROXCFG1                0xAA
#define IQS269_MM_CH4_ATI_TARGETS             0xAA
#define IQS269_MM_CH4_MULTIPLIERS             0xAB
#define IQS269_MM_CH4_PCC                     0xAB
#define IQS269_MM_CH4_THRESHOLD_P             0xAC
#define IQS269_MM_CH4_THRESHOLD_T             0xAC
#define IQS269_MM_CH4_THRESHOLD_DT            0xAD
#define IQS269_MM_CH4_HYSTERESIS              0xAD
#define IQS269_MM_CH4_FOLLOW_CHANNELS         0xAE
#define IQS269_MM_CH4_FOLLOW_WEIGHT           0xAE

/* Channel 5 Settings - Read/Write */
#define IQS269_MM_CH5_SELECT_CRX              0xAF
#define IQS269_MM_CH5_SELECT_CTX              0xAF
#define IQS269_MM_CH5_PROXCTL                 0xB0
#define IQS269_MM_CH5_PROXCG0                 0xB0
#define IQS269_MM_CH5_PROXCFG1                0xB1
#define IQS269_MM_CH5_ATI_TARGETS             0xB1
#define IQS269_MM_CH5_MULTIPLIERS             0xB2
#define IQS269_MM_CH5_PCC                     0xB2
#define IQS269_MM_CH5_THRESHOLD_P             0xB3
#define IQS269_MM_CH5_THRESHOLD_T             0xB3
#define IQS269_MM_CH5_THRESHOLD_DT            0xB4
#define IQS269_MM_CH5_HYSTERESIS              0xB4
#define IQS269_MM_CH5_FOLLOW_CHANNELS         0xB5
#define IQS269_MM_CH5_FOLLOW_WEIGHT           0xB5

/* Channel 6 Settings - Read/Write */
#define IQS269_MM_CH6_SELECT_CRX              0xB6
#define IQS269_MM_CH6_SELECT_CTX              0xB6
#define IQS269_MM_CH6_PROXCTL                 0xB7
#define IQS269_MM_CH6_PROXCG0                 0xB7
#define IQS269_MM_CH6_PROXCFG1                0xB8
#define IQS269_MM_CH6_ATI_TARGETS             0xB8
#define IQS269_MM_CH6_MULTIPLIERS             0xB9
#define IQS269_MM_CH6_PCC                     0xB9
#define IQS269_MM_CH6_THRESHOLD_P             0xBA
#define IQS269_MM_CH6_THRESHOLD_T             0xBA
#define IQS269_MM_CH6_THRESHOLD_DT            0xBB
#define IQS269_MM_CH6_HYSTERESIS              0xBB
#define IQS269_MM_CH6_FOLLOW_CHANNELS         0xBC
#define IQS269_MM_CH6_FOLLOW_WEIGHT           0xBC

/* Channel 7 Settings - Read/Write */
#define IQS269_MM_CH7_SELECT_CRX              0xBD
#define IQS269_MM_CH7_SELECT_CTX              0xBD
#define IQS269_MM_CH7_PROXCTL                 0xBE
#define IQS269_MM_CH7_PROXCG0                 0xBE
#define IQS269_MM_CH7_PROXCFG1                0xBF
#define IQS269_MM_CH7_ATI_TARGETS             0xBF
#define IQS269_MM_CH7_MULTIPLIERS             0xC0
#define IQS269_MM_CH7_PCC                     0xC0
#define IQS269_MM_CH7_THRESHOLD_P             0xC1
#define IQS269_MM_CH7_THRESHOLD_T             0xC1
#define IQS269_MM_CH7_THRESHOLD_DT            0xC2
#define IQS269_MM_CH7_HYSTERESIS              0xC2
#define IQS269_MM_CH7_FOLLOW_CHANNELS         0xC3
#define IQS269_MM_CH7_FOLLOW_WEIGHT           0xC3

/** @}
  * End group IQS269 Memory Map
  */
  
#endif /* __IQS269_ADDRESSES_H */