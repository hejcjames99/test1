/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#ifndef __APPS_H__
#define __APPS_H__

#include "app_status_ind.h"

#define STACK_READY_BT  0x01
#define STACK_READY_BLE 0x02

#ifdef __cplusplus
extern "C" {
#endif

#include "plat_types.h"



int app_init(void);

int app_deinit(int deinit_case);

int app_shutdown(void);

int app_reset(void);

int app_status_battery_report(uint8_t level);

int app_voice_report( APP_STATUS_INDICATION_T status,uint8_t device_id);
int app_voice_report_generic(APP_STATUS_INDICATION_T status, uint8_t device_id, uint8_t isMerging);
int app_voice_stop(APP_STATUS_INDICATION_T status, uint8_t device_id);


/*FixME*/
void app_status_set_num(const char* p);

////////////10 second tiemr///////////////
#define APP_FAST_PAIRING_TIMEOUT_IN_SECOND  120

#define APP_PAIR_TIMER_ID       0
#define APP_POWEROFF_TIMER_ID   1
#define APP_FASTPAIR_LASTING_TIMER_ID   2

void app_stop_10_second_timer(uint8_t timer_id);
void app_start_10_second_timer(uint8_t timer_id);

void app_notify_stack_ready(uint8_t ready_flag);

void app_start_postponed_reset(void);

bool app_is_power_off_in_progress(void);

#define CHIP_ID_C     1
#define CHIP_ID_D     2

void app_disconnect_all_bt_connections(void);
bool app_is_stack_ready(void);

#ifdef GFPS_ENABLED
enum GFPS_RING_MODE {
    GFPS_RING_MODE_BOTH_OFF,
    GFPS_RING_MODE_LEFT_ON,
    GPFS_RING_MODE_RIGHT_ON,
    GFPS_RING_MODE_BOTH_ON,
};
void app_voice_start_gfps_find(void);
void app_voice_stop_gfps_find(void);
bool app_voice_gfps_find_state(void);
void app_gfps_battery_show_timer_start();
void app_gfps_battery_show_timer_stop();
void app_gfps_find_sm(bool find_on_off);
void app_gfps_ring_mode_set(uint8_t ring_mode);
uint8_t app_gfps_ring_mode_get();
#endif


void app_bt_streaming_hold_frequency_run_timer_create(void);
void app_bt_streaming_hold_frequency_run_timer_start(uint8_t ratio);
////////////////////

#ifdef __BES2500_HAILINGWEI_UART__
void app_ibrt_uart_func_enable_close(bool en);
#endif

#ifdef __PUTIN_PUTOUT_SWITCH_ROLE__
extern uint8_t current_box_status;
extern uint8_t left_earheadset_status;
extern uint8_t right_earheadset_status;
extern bool out_of_box_poweron_indication_flag;

void app_putin_putout_switch_role_init(void);

//void app_putin_putout_box_process(uint8_t putin_putout);

void app_detect_switch_role_timer_start_stop(bool timer_en);

#endif

#ifdef __BES2500_HAILINGWEI_CHC__
extern uint8_t  app_poweroff_flag;

extern uint8_t tws_connect_disconnect_status;
extern uint8_t tws_discon_linkloss_reconnect_cnt;
extern bool phone_linkloss_tws_disconnect_flag;
extern uint8_t phone_connect_disconnect_status;
extern uint8_t phone_discon_linkloss_reconnect_cnt;
extern bool phone_from_connected_to_disconnected;
extern uint8_t earheadset_fixed_left_right_type;
extern bool tws_slave_key_function_flag;

extern uint8_t HLWei_hfp_end_call_type;
extern bool HLWei_hfp_sync_ring_flag;
extern bool HLWei_hfp_three_way_exist_flag;
extern bool HLWei_hfp_incoming_call_flag;
extern bool HLWei_hfp_out_call_flag;
extern bool HLWei_hfp_answer_voice_play_flag;
extern bool HLWei_hfp_answer_call_flag;
extern bool HLWei_hfp_key_call_voice_flag;

extern uint8_t HLWei_curr_battery_level;
extern uint8_t HLWei_last_battery_level;
extern uint8_t HLWei_slave_battery_level;
extern bool app_battery_lowbat_flag;
extern uint8_t app_battery_lowbat_play_voice_type;

extern bool app_battery_charging_flag;
extern bool app_battery_plugin_init_flag;
extern bool app_battery_plugout_init_flag;
extern bool app_battery_send_reconnect_flag;
//extern bool app_battery_open_box_poweron_flag;
//extern bool app_reset_poweron_flag;
extern bool app_reboot_poweron_do_not_play_voice_flag;

extern uint8_t app_led_recover_display_type;
extern bool tws_role_switch_flag;

extern uint8_t tws_master_search_count;
extern bool delay_play_connected_voice_flag;
extern bool clear_pairlist_reset_poweron_flag;
extern bool tws_master_delay_to_switch_eq_flag;
extern bool tws_a2dp_low_latency_open_close_flag;
extern bool clear_tws_pairlist_reset_poweron_flag;
extern uint8_t key_siri_open_close_opration;
extern bool delay_detect_touch_flag;
extern bool clear_poweroff_flag;

#ifdef __BES2500_DAKANG_TWS_PAIR__
extern bool enter_tws_pair_mode_flag;
extern bool enter_tws_pair_mode_enable_flag;
extern bool tws_pair_phone_pair_switch_flag;
extern bool slave_enter_tws_pairmode_flag;
extern uint8_t slave_enter_tws_pairmode_cnt;
extern bool master_enter_tws_pairmode_flag;
extern uint8_t master_enter_tws_pairmode_cnt;
extern bool master_switch_role_shutdown_flag;
#endif
extern bool ntc_temper_no_shutdown_flag;
extern uint8_t ntc_temper_mode;
extern uint8_t ntc_temper_low_high_cnt;
extern uint8_t ntc_temper_normal_cnt;
extern uint8_t app_recover_freq;
extern uint8_t app_freq_change_cnt;
extern bool app_freq_change_flag;
extern uint8_t software_version_shi;
extern uint8_t software_version_ge;
extern bool app_auto_play_nv_flag;
extern bool app_siri_open_close_flag;
extern bool spk_mute_flag;

/*
struct HaiLingWei_Variable{
	uint8_t tws_connect_disconnect_status;
	uint8_t tws_discon_linkloss_reconnect_cnt;
	bool phone_linkloss_tws_disconnect_flag;
	uint8_t phone_connect_disconnect_status;
	uint8_t phone_discon_linkloss_reconnect_cnt;
	bool phone_from_connected_to_disconnected;
	uint8_t earheadset_fixed_left_right_type;
	bool tws_slave_key_function_flag;
	
	uint8_t HLWei_hfp_end_call_type;
	bool HLWei_hfp_sync_ring_flag;
	bool HLWei_hfp_three_way_exist_flag;
	bool HLWei_hfp_incoming_call_flag;
	bool HLWei_hfp_out_call_flag;
	bool HLWei_hfp_answer_voice_play_flag;
	bool HLWei_hfp_answer_call_flag;

	uint8_t HLWei_curr_battery_level;
	uint8_t HLWei_last_battery_level;
	uint8_t HLWei_slave_battery_level;
	bool app_battery_lowbat_flag;
	uint8_t app_battery_lowbat_play_voice_type;

	bool app_battery_charging_flag;
	bool app_battery_plugin_init_flag;
	bool app_battery_plugout_init_flag;

	uint8_t app_led_recover_display_type;
	bool tws_role_switch_flag;

	uint8_t tws_master_search_count;
	bool delay_play_connected_voice_flag;
	bool clear_pairlist_reset_poweron_flag;
	bool tws_master_delay_to_switch_eq_flag;
	bool tws_a2dp_low_latency_open_close_flag;
	uint8_t key_siri_open_close_opration;

	bool enter_tws_pair_mode_flag;
	bool enter_tws_pair_mode_enable_flag;
};

extern HaiLingWei_Variable HLWei_Variable;
*/
void stop_sniff(void);
//void app_detect_ntc_timer_start_stop(bool timer_en);

//void app_set_reset(void);
void app_timer_start_stop(bool timer_en);

bool app_get_charge_status(void);

void app_store_auto_play_proc(bool auto_play);

void app_store_shut_down_proc(bool shutdown_flag);

uint8_t app_get_curr_master_role(void);

void app_hailingwei_variable_init(void);

void app_charging_stop_tws_search(void);

uint8_t app_earheadset_fixed_type_get(void);

void app_led_sync_hfp_proc(bool send_flag, uint8_t status);

#ifdef __BES2500_HAILINGWEI_LOWLATENCY__
bool app_get_curr_latency_mode_status(void);

void app_switch_latency_mode(bool switch_mode_flag);

void app_set_curr_latency_mode_status(bool mode);
#endif

bool app_get_led_display_recover_enable(void);

void app_hailingwei_earheadset_fixed_type(void);

#ifdef __BES2500_HAILINGWEI_SWITCH_EQ__
void app_tws_switch_eq(bool switch_flag, uint8_t eq_mode);
#endif

void app_a2dp_auto_play_timer_start_stop(bool timer_en);


void app_freq_timer_start_stop(bool timer_en);

void app_spk_open_close_control(bool open_close);

#ifndef __BES2500_HAILINGWEI_CHC__
void app_hailingwei_ntc_temperate_proc(uint16_t curr_volt);
#endif

void app_earheadset_enter_dut_test_rf_param(void);

void app_phone_connected_report_battery_level(void);

void app_earheadset_enter_otamode_update_software(void);

void app_master_recive_battery_level_proc(uint8_t slave_level);

void app_disconnected_tws_battery_deal(bool connect_disconnect);

void app_spp_cmd_received_data_prec(uint8_t* ptrParam, uint32_t paramLen);

void app_clear_all_pairlst_reset_poweron(bool clear_tws, bool clear_phone);

void app_hailingwei_poweroff_prec(uint8_t shutdown_type, bool slave_shutdown_flag);

void app_hfp_end_call_voice_play(APP_STATUS_INDICATION_T status, uint8_t device_id, bool lowbat_flag);

void app_charge_clear_all_pairlist(bool clear_tws, bool clear_phone);

void app_delay_to_deal_timer_start_stop(bool timer_en);

void app_putin_box_handler(void);

void app_putout_box_handler(void);

//void app_open_box_close_box_timer_start_stop(bool timer_en);

#ifdef __BES2500_DAKANG_TWS_PAIR__
//void app_slave_exit_tws_pairmode_timer_start_stop(bool timer_en);
#endif
void app_bt_key_software_version(void);

void app_nv_update_timer_start_stop(bool timer_en);

void app_bt_software_version_timer_start_stop(bool timer_en);

void app_play_voice_delay_open_siri_timer_start_stop(bool timer_en);


void app_end_call_tran_way_to_headset_timer_start_stop(bool timer_en);

void app_enter_phone_pairmode_play_voice_timer_start_stop(bool timer_en);

void app_tws_pairfail_enter_phone_pairmode_timer_start_stop(bool timer_en);

void app_recover_led_display_timer_start_stop(bool timer_en, uint8_t display_type);

void app_phone_connected_or_disconnected_timer_start_stop(bool timer_en, bool connected_flag);

void app_earheadset_tws_connect_or_disconnect_timer_start_stop(bool timer_en, bool connect_flag);
#endif



#ifdef __BES2500_I2C_STANDARD__
#define ICP1205_DEVICE_ADDRESS     0xc2
/*******************************************************
寄存器：0x00  可读可写
功能：bit7:保留位
	  bit4~bit6:cv_set 恒压充电设置         
	  bit3:iterm_sel 截止电流设置1/10C        1/5C
	  bit2:ntc_en 恒温充电使能，使能后，快充模式下当温度过高时（38℃），将充电电流自动切换为default_cc_set
	  bit1:recharge_en 再充电使能，停止充电后，若充电使能且再充电也使能，ICP1205每隔15分钟检查一次电池是否需要再充电
	  bit0:charge_en 充电使能
	  

********************************************************/
#define ICP1205_CHRG_CON1      0x00 
/*******************************************************
寄存器：0x01  可读可写
功能：
	  bit4~bit7: fast_cc_set 恒流充电设置(fast_cc_set) 0:25ma 1:30ma 2:35ma 3:20ma 4:45ma 5:50ma 6:55ma
	                                      7:60ma 8:75ma 9:90ma a:105ma b:120ma c:135ma d:150ma e:165ma f:180ma
	  bit0~bit3:default_cc_set 慢充充电电流
	  

********************************************************/
#define ICP1205_CHRG_CON2      0x01
/*******************************************************
寄存器：0x02  可读可写
功能：
	  bit4~bit7:保留位
	  bit2~bit3:charge_time_thd 充电超时阈值 0:120min 1:180min 2:240min 3:无充电时间限制
	  
	  bit0~bit2:deialy_time 充电延时时间，0:4min 1:8min 2:16min 3:无充电延时
	  

********************************************************/
#define ICP1205_CHRG_CON3      0x02
/*******************************************************
寄存器：0x04 只读
功能：
	  bit4~bit7:保留位
	  bit3:chrg_err 充电错误状态 当发生欠压、短路保护、过压、过温、欠温时，该位置为1
	  
	  bit0~bit2:chrg_sts 充电状态：0:空闲 2:涓流模式 3:快充模式 4:慢充模式 5:充电延时 6:充满 7:错误状态
	  

********************************************************/
#define ICP1205_CHRG_STS1      0x04
/*******************************************************
寄存器：0x05 只读
功能：
	  bit7:保留位
	  bit6:ovp_sts Vin输入过压状态指示 1:过压              0:未过压
	  bit5:sc_sts 电池短路状态           1:短路           0:未短路
	  bit4:bat_3p3_cmp 电池电压是否低于3.3V           1: >3.3v   0<3.3v
	  bit3:Vin_4p4_cmp Vin 输入电压是否大于4.4V            1:>4.4v    0:<4.4v
	  bit2:Vin_3p3_cmp Vin 输入电压是否大于3.3V            1:>3.3v   0:<3.3v
	  bit1:保留位
	  bit0:Vin_det   Vin状态         1:vin插入    0:vin拔出
	  

********************************************************/
#define ICP1205_CHRG_STS2      0x05
/*******************************************************
寄存器：0x06 只读
功能：
	  bit7:ntc_h_sts NTC高温状态      1:高温状态，无法充电             0:未触发高温保护
	  bit6:ntc_l_sts NTC低温状态      1:低温状态，无法充电             0:未触发低温保护
	  bit5:ntc_sts 过热状态(对应NTC10K3950电阻，超过38℃)               1:温度过高，建议降低充电电流               0:未触发过热状态
	  bit4:trickle_sts 涓流充电状态            1:涓流          0:非涓流
	  bit3:cv_sts    恒压充电状态         1：恒压               0：非恒压
	  bit2:vin_0p35_vbat_cmp       1: Vin与Vbat压差>0.35V          0: Vin与Vbat压差<0.35V
	  bit1:vin_0p15_vbat_cmp       1: Vin与Vbat压差>0.15V       0: Vin与Vbat压差<0.15V
	  bit0:iterm_sts   截止电流状态         1: 截止       0: 未截止
	  

********************************************************/
#define ICP1205_CHRG_STS3      0x06
/*******************************************************
寄存器：0x08  可读可写
功能：bit4~bit7:MASK        只写
	  bit2~bit3:wdt_time   看门狗超时时间            00: 4s        01: 8s          10: 16s       11: 32s
	  bit1:wdt_feed      喂狗信号       写0x22喂狗
	  bit0:wdt_en        看门狗使能          1: enable       0: disable

	  

********************************************************/
#define ICP1205_WDT_CON        0x08
/*******************************************************
寄存器：0x09  可读可写
功能：bit4~bit7:MASK        只写
	  bit2~bit3:ship_ackmsk_tm   进入运输模式后，延时关机时间                00:1S  01:4S   10:16S    11:32S
	  bit1:ship_mode      模式选择       默认为1，（该必须为1，不能写0）
	  bit0:ship_en        运输模式使能         1: enable     0: disable      
	  
	  

********************************************************/
#define ICP1205_SHIP_CON       0x09
/*******************************************************
寄存器：0x0a可读可写
功能：bit4~bit7:MASK        只读
	  bit2~bit3:保留  配置写0
	  bit1:tx_en      载波发送使能          1: enable   0: disable
	  bit0:rx_en        载波接收使能         1: enable     0: disable      
	  
	  

********************************************************/
#define ICP1205_CRCOM_CON1     0x0a
/*******************************************************
寄存器：0x0b可读可写
功能：bit2~bit7:MASK        只读

	  bit1:intdata_updt      数据2（整型）更新                 1: 有新数据
	  bit0:chrdata_updt        数据1（CHAR）更新         1: 有新数据   
	  
	  

********************************************************/
#define ICP1205_CRCOM_CON2     0x0b
/*******************************************************
寄存器：0x0b只读
功能：bit0~bit7:rx_data1        接收数据1 (CHAR) 存储寄存器    
********************************************************/
#define ICP1205_CRCOM_CHR_DAT  0x0c
/*******************************************************
寄存器：0x0c只读
功能：bit0~bit7:rx_int_dat_h        载波接收16位整型数据高8位  
********************************************************/
#define ICP1205_CRCOM_INT_DATH 0x0d
/*******************************************************
寄存器：0x0e只读
功能：bit0~bit7:rx_int_dat_l        载波接收16位整型数据低8位  
********************************************************/
#define ICP1205_CRCOM_INT_DATL 0x0e
/*******************************************************
寄存器：0x0f可读可写
功能：bit0~bit7:req_data     回复数据寄存器           当收到载波命令0x9时，回复该寄存器的数据
********************************************************/
#define	ICP1205_CRCOM_TDAT     0x0f
/*******************************************************
寄存器：0x0b可读可写
功能：bit5~bit7:MASK        只读
	  bit4:comm_busy   透传模式指示
	  1: 透传模式
	  0: 非透传模式
	  bit3:保留位   
	  bit2:comm_dir 透传方向
	  1: 发送模式
	  0: 接收模式
	  bit1:comm_sw   透传方向控制模式  
	  1: 使用寄存器内的comm_dir，控制透传方向
	  0: 使用RTMD引脚控制透传方向
	  bit0:comm_en  透传模式使能   
	  1: enable
	  0: disable
********************************************************/
#define ICP1205_COMM_CON       0x10
/*******************************************************
寄存器：0x0b可读可写
功能：bit2~bit7:MASK        只读
	  bit1:key_sts   按键状态
	  1：按键按下
	  0：按键松开
	  bit0:GP0/GP1  GP0/GP1状态控制  
	  1：输出高电平
	  0：输出低电平
********************************************************/
#define ICP1205_GPIO_CON       0x11
/*******************************************************
寄存器：0x0b可读可写
功能：bit7:cv_set_fine_en           CV电压设置
	  1: 在CHRG_CON1的cv_set基础上增加25mV
	  0: 直接使用cv_set电压
	  bit6:保留位
	  bit5:出仓唤醒使能功能            当KEY/GP1引脚出厂默认为输出时，该位控制支持  Vin掉电为0时，GP1输出为电池电压。
	  0:使能掉电唤醒功能（默认）
	  1:关闭掉电唤醒功能

	  bit4:KEY/GP1方向控制寄存器
	  1：输出
	  0：输入   注意：出厂默认为输出时，切换到输入按键模式，需要先写1再写0

	  bit3:direct_comm_en  透传直通模式使能               从Vin至comm的level shifter直通模式
	  1：enable
	  0：disable

	  bit0~bit2:reserved
********************************************************/
#define ICP1205_REV_ANA1       0x12
//保留寄存器
#define ICP1205_REV_DAT1       0x14
#define ICP1205_REV_DAT2       0x15
#define ICP1205_REV_DAT3       0x16
#define ICP1205_REV_DAT4       0x17
/*******************************************************
寄存器：0x0b可读可写
功能：		bit1~bit7:reserved
		bit0:int_en    中断使能
		1: enable
		0: disable
********************************************************/
#define ICP1205_INT_EN         0x20
/*******************************************************
寄存器：0x0b可读可写
功能：		
		bit7:chrg_fin  充电完成中断
		1：充电完成
		bit6:gp0_ch  gp0改变中断
		1：GP0状态改变
		bit5:bat_low 电池低电中断
		1：电池电压低
		bit4: wdt_timeout   wdt超时中断
		1：看门狗超时
		bit3:保留
		bit2:chrg_err  充电错误中断
		1：充电错误
		bit1:Vin_plug out Vin拔出中断
		1：Vin 拔出
		bit0:Vin_plug in Vin插入中断
		1:Vin 插入
********************************************************/
#define ICP1205_INT_STAT1      0x21
/*******************************************************
寄存器：0x0b可读可写
功能：		
		bit7:iterm_cmp  充电截止中断
		1：充电截止
		bit6:Vin_0p35_vbat_cmp  
		1: Vin 与 Vbat压差与0.35V比较状态改变中断
		bit5:Vin_0p15_vbat_cmp
		1: Vin 与 Vbat压差与0.15V比较状态改变中断
		bit4: cv_stat   恒压模式
		1：进入恒压充电模式中断
		bit3:chrg_ntc充电过热中断
		1：充电过热
		bit2:chrg_trickle_timeout  涓流充电超时（45分钟）中断
		1：涓流充电超时
		bit1:chrg_timeout  充电超时中断
		1：充电超时
		bit0:chrg_state_ch 充电状态改变中断
		1：充电状态改变
********************************************************/
#define ICP1205_INT_STAT2      0x22
/*******************************************************
寄存器：0x0b可读可写
功能：		
		bit5~bit7:reserved

		bit4:comm_mode  透传模式
		1：进入透传模式中断
		bit3:rx_failed 载波接收校验错误
		1：载波接收校验错误中断
		bit2:rx_cmd   载波收到命令
		1：载波收到命令中断
		bit1:key_up  按键松开  1：按键松开中断
		bit0:key_dn  按键按下 1：按键按下中断
********************************************************/
#define ICP1205_INT_STAT3      0x23
/*******************************************************
寄存器：0x0b可读可写
功能：		
		bit7:chrg_fin 
		1：充电完成中断屏蔽
		bit6:gp0_ch  
		1：GP0状态改变中断屏蔽
		bit5:bat_low
		1：电池电压低中断屏蔽
		bit4: wdt_timeout  
		1：看门狗超时中断屏蔽
		bit3:保留

		bit2:chrg_err
		1：充电错误中断屏蔽
		bit1:Vin_plug out
		1：Vin 拔出中断屏蔽
		bit0:Vin_plug in 
		1:Vin 插入中断屏蔽
********************************************************/
#define ICP1205_INT_MASK1      0x24
/*******************************************************
寄存器：0x0b可读可写
功能：		
		bit7:iterm_cmp 
		1：充电截止中断屏蔽
		bit6:Vin_0p35_vbat_cmp  
		1: Vin 与 Vbat压差与0.35V比较状态改变中断屏蔽
		bit5:Vin_0p15_vbat_cmp
		1: Vin 与 Vbat压差与0.15V比较状态改变中断屏蔽
		bit4: cv_stat  
		1：进入恒压充电模式中断屏蔽
		bit3:chrg_ntc
		1：充电过热中断屏蔽
		bit2:chrg_trickle_timeout
		1：涓流充电超时（45分钟）中断屏蔽
		bit1:chrg_timeout
		1：充电超时中断屏蔽
		bit0:chrg_state_ch
		1：充电状态改变中断屏蔽
********************************************************/
#define ICP1205_INT_MASK2      0x25
/*******************************************************
寄存器：0x0b可读可写
功能：		
		bit5~bit7:reserved 
		bit4: comm_mode  
		1：进入透传模式中断屏蔽
		bit3:  rx_failed
		1：载波接收校验错误中断屏蔽
		bit2:rx_cmd
		1：载波收到命令中断屏蔽
		bit1:key_up
		1：按键松开中断屏蔽
		bit0:key_dn
		1：按键按下中断屏蔽
********************************************************/
#define ICP1205_INT_MASK3      0x26
#endif


#ifdef __BES2500_IQS269_TOUCH__
extern bool app_putout_touch_send_flag;
extern uint8_t app_putout_touch_send_cnt;

extern bool app_putin_touch_send_flag;
extern uint8_t app_putin_touch_send_cnt;


#define I2C_IQS62x_ADDR					0x88

#define APP_INIT_RESET          0
#define APP_INIT_TOUCH          1
#define APP_INIT_ATI_LOOP       2
#define APP_INIT_TOUCH_SUCCESS  3


typedef union {
    struct
    {
    	uint8_t NP_SEG_LT_BLOCK 	:1;
    	uint8_t EVENT				:1;
    	uint8_t IN_ATI				:1;
    	uint8_t powerMode			:2;
    	uint8_t res					:2;
    	//uint8_t res					:1;
    	uint8_t SHOW_RESET			:1;
    };
    uint8_t SysFlags;
} SysFlags_t;


typedef union {
    struct
    {
    	uint8_t PXS_EVENT			:1;
		uint8_t TOUCH				:1;
		uint8_t DTOUCH				:1;
		uint8_t GESTURE				:1;
		uint8_t res					:1;
		uint8_t REFCHAN				:1;
		uint8_t SYSTEM				:1;
		uint8_t POWERMODE			:1;
	};
	uint8_t GlobalFlags;
} GlobalFlags_t;

typedef union {
    struct
    {
    	uint8_t ch0					:1;
		uint8_t ch1					:1;
		uint8_t ch2					:1;
		uint8_t ch3					:1;
		uint8_t ch4					:1;
		uint8_t ch5					:1;
		uint8_t ch6					:1;
		uint8_t ch7					:1;
	};
	uint8_t PXSFlags;
} PXSFlags_t;

typedef union {
    struct
    {
    	uint8_t ch0					:1;
    	uint8_t ch1					:1;
    	uint8_t ch2					:1;
    	uint8_t ch3					:1;
		uint8_t ch4					:1;
		uint8_t ch5					:1;
		uint8_t ch6					:1;
		uint8_t ch7					:1;
	};
	uint8_t TCHFlags;
} TCHFlags_t;

// PXS CH  Value
typedef union
{
    struct
    {
    	uint8_t Ch_Low;
    	uint8_t Ch_High;
    };
    int16_t Ch;

}Ch_t;



// "Object" for IQS6xx
typedef struct {

	// System Flags
	SysFlags_t SysFlags;

	// Global Events
	GlobalFlags_t GlobalFlags;

	// Proxsense UI Flags
	PXSFlags_t PXSFlags;

	TCHFlags_t TCHFlags;

	// Channel data

	Ch_t DCF[2];
	Ch_t Ch[12]; //0,1 cnts lta
	Ch_t LTA[12]; //0,1 cnts lta

} IQS62x_Data_t;

void app_touch_init_reset(void);

void app_poweroff_set_touch_channel(void);

void app_earheadset_plugin_plugout_detect_timer_start_stop(bool en);

#endif
#ifdef __cplusplus
}
#endif
#endif//__FMDEC_H__
