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
�Ĵ�����0x00  �ɶ���д
���ܣ�bit7:����λ
	  bit4~bit6:cv_set ��ѹ�������         
	  bit3:iterm_sel ��ֹ��������1/10C        1/5C
	  bit2:ntc_en ���³��ʹ�ܣ�ʹ�ܺ󣬿��ģʽ�µ��¶ȹ���ʱ��38�棩�����������Զ��л�Ϊdefault_cc_set
	  bit1:recharge_en �ٳ��ʹ�ܣ�ֹͣ���������ʹ�����ٳ��Ҳʹ�ܣ�ICP1205ÿ��15���Ӽ��һ�ε���Ƿ���Ҫ�ٳ��
	  bit0:charge_en ���ʹ��
	  

********************************************************/
#define ICP1205_CHRG_CON1      0x00 
/*******************************************************
�Ĵ�����0x01  �ɶ���д
���ܣ�
	  bit4~bit7: fast_cc_set �����������(fast_cc_set) 0:25ma 1:30ma 2:35ma 3:20ma 4:45ma 5:50ma 6:55ma
	                                      7:60ma 8:75ma 9:90ma a:105ma b:120ma c:135ma d:150ma e:165ma f:180ma
	  bit0~bit3:default_cc_set ���������
	  

********************************************************/
#define ICP1205_CHRG_CON2      0x01
/*******************************************************
�Ĵ�����0x02  �ɶ���д
���ܣ�
	  bit4~bit7:����λ
	  bit2~bit3:charge_time_thd ��糬ʱ��ֵ 0:120min 1:180min 2:240min 3:�޳��ʱ������
	  
	  bit0~bit2:deialy_time �����ʱʱ�䣬0:4min 1:8min 2:16min 3:�޳����ʱ
	  

********************************************************/
#define ICP1205_CHRG_CON3      0x02
/*******************************************************
�Ĵ�����0x04 ֻ��
���ܣ�
	  bit4~bit7:����λ
	  bit3:chrg_err ������״̬ ������Ƿѹ����·��������ѹ�����¡�Ƿ��ʱ����λ��Ϊ1
	  
	  bit0~bit2:chrg_sts ���״̬��0:���� 2:���ģʽ 3:���ģʽ 4:����ģʽ 5:�����ʱ 6:���� 7:����״̬
	  

********************************************************/
#define ICP1205_CHRG_STS1      0x04
/*******************************************************
�Ĵ�����0x05 ֻ��
���ܣ�
	  bit7:����λ
	  bit6:ovp_sts Vin�����ѹ״ָ̬ʾ 1:��ѹ              0:δ��ѹ
	  bit5:sc_sts ��ض�·״̬           1:��·           0:δ��·
	  bit4:bat_3p3_cmp ��ص�ѹ�Ƿ����3.3V           1: >3.3v   0<3.3v
	  bit3:Vin_4p4_cmp Vin �����ѹ�Ƿ����4.4V            1:>4.4v    0:<4.4v
	  bit2:Vin_3p3_cmp Vin �����ѹ�Ƿ����3.3V            1:>3.3v   0:<3.3v
	  bit1:����λ
	  bit0:Vin_det   Vin״̬         1:vin����    0:vin�γ�
	  

********************************************************/
#define ICP1205_CHRG_STS2      0x05
/*******************************************************
�Ĵ�����0x06 ֻ��
���ܣ�
	  bit7:ntc_h_sts NTC����״̬      1:����״̬���޷����             0:δ�������±���
	  bit6:ntc_l_sts NTC����״̬      1:����״̬���޷����             0:δ�������±���
	  bit5:ntc_sts ����״̬(��ӦNTC10K3950���裬����38��)               1:�¶ȹ��ߣ����齵�ͳ�����               0:δ��������״̬
	  bit4:trickle_sts ������״̬            1:���          0:�����
	  bit3:cv_sts    ��ѹ���״̬         1����ѹ               0���Ǻ�ѹ
	  bit2:vin_0p35_vbat_cmp       1: Vin��Vbatѹ��>0.35V          0: Vin��Vbatѹ��<0.35V
	  bit1:vin_0p15_vbat_cmp       1: Vin��Vbatѹ��>0.15V       0: Vin��Vbatѹ��<0.15V
	  bit0:iterm_sts   ��ֹ����״̬         1: ��ֹ       0: δ��ֹ
	  

********************************************************/
#define ICP1205_CHRG_STS3      0x06
/*******************************************************
�Ĵ�����0x08  �ɶ���д
���ܣ�bit4~bit7:MASK        ֻд
	  bit2~bit3:wdt_time   ���Ź���ʱʱ��            00: 4s        01: 8s          10: 16s       11: 32s
	  bit1:wdt_feed      ι���ź�       д0x22ι��
	  bit0:wdt_en        ���Ź�ʹ��          1: enable       0: disable

	  

********************************************************/
#define ICP1205_WDT_CON        0x08
/*******************************************************
�Ĵ�����0x09  �ɶ���д
���ܣ�bit4~bit7:MASK        ֻд
	  bit2~bit3:ship_ackmsk_tm   ��������ģʽ����ʱ�ػ�ʱ��                00:1S  01:4S   10:16S    11:32S
	  bit1:ship_mode      ģʽѡ��       Ĭ��Ϊ1�����ñ���Ϊ1������д0��
	  bit0:ship_en        ����ģʽʹ��         1: enable     0: disable      
	  
	  

********************************************************/
#define ICP1205_SHIP_CON       0x09
/*******************************************************
�Ĵ�����0x0a�ɶ���д
���ܣ�bit4~bit7:MASK        ֻ��
	  bit2~bit3:����  ����д0
	  bit1:tx_en      �ز�����ʹ��          1: enable   0: disable
	  bit0:rx_en        �ز�����ʹ��         1: enable     0: disable      
	  
	  

********************************************************/
#define ICP1205_CRCOM_CON1     0x0a
/*******************************************************
�Ĵ�����0x0b�ɶ���д
���ܣ�bit2~bit7:MASK        ֻ��

	  bit1:intdata_updt      ����2�����ͣ�����                 1: ��������
	  bit0:chrdata_updt        ����1��CHAR������         1: ��������   
	  
	  

********************************************************/
#define ICP1205_CRCOM_CON2     0x0b
/*******************************************************
�Ĵ�����0x0bֻ��
���ܣ�bit0~bit7:rx_data1        ��������1 (CHAR) �洢�Ĵ���    
********************************************************/
#define ICP1205_CRCOM_CHR_DAT  0x0c
/*******************************************************
�Ĵ�����0x0cֻ��
���ܣ�bit0~bit7:rx_int_dat_h        �ز�����16λ�������ݸ�8λ  
********************************************************/
#define ICP1205_CRCOM_INT_DATH 0x0d
/*******************************************************
�Ĵ�����0x0eֻ��
���ܣ�bit0~bit7:rx_int_dat_l        �ز�����16λ�������ݵ�8λ  
********************************************************/
#define ICP1205_CRCOM_INT_DATL 0x0e
/*******************************************************
�Ĵ�����0x0f�ɶ���д
���ܣ�bit0~bit7:req_data     �ظ����ݼĴ���           ���յ��ز�����0x9ʱ���ظ��üĴ���������
********************************************************/
#define	ICP1205_CRCOM_TDAT     0x0f
/*******************************************************
�Ĵ�����0x0b�ɶ���д
���ܣ�bit5~bit7:MASK        ֻ��
	  bit4:comm_busy   ͸��ģʽָʾ
	  1: ͸��ģʽ
	  0: ��͸��ģʽ
	  bit3:����λ   
	  bit2:comm_dir ͸������
	  1: ����ģʽ
	  0: ����ģʽ
	  bit1:comm_sw   ͸���������ģʽ  
	  1: ʹ�üĴ����ڵ�comm_dir������͸������
	  0: ʹ��RTMD���ſ���͸������
	  bit0:comm_en  ͸��ģʽʹ��   
	  1: enable
	  0: disable
********************************************************/
#define ICP1205_COMM_CON       0x10
/*******************************************************
�Ĵ�����0x0b�ɶ���д
���ܣ�bit2~bit7:MASK        ֻ��
	  bit1:key_sts   ����״̬
	  1����������
	  0�������ɿ�
	  bit0:GP0/GP1  GP0/GP1״̬����  
	  1������ߵ�ƽ
	  0������͵�ƽ
********************************************************/
#define ICP1205_GPIO_CON       0x11
/*******************************************************
�Ĵ�����0x0b�ɶ���д
���ܣ�bit7:cv_set_fine_en           CV��ѹ����
	  1: ��CHRG_CON1��cv_set����������25mV
	  0: ֱ��ʹ��cv_set��ѹ
	  bit6:����λ
	  bit5:���ֻ���ʹ�ܹ���            ��KEY/GP1���ų���Ĭ��Ϊ���ʱ����λ����֧��  Vin����Ϊ0ʱ��GP1���Ϊ��ص�ѹ��
	  0:ʹ�ܵ��绽�ѹ��ܣ�Ĭ�ϣ�
	  1:�رյ��绽�ѹ���

	  bit4:KEY/GP1������ƼĴ���
	  1�����
	  0������   ע�⣺����Ĭ��Ϊ���ʱ���л������밴��ģʽ����Ҫ��д1��д0

	  bit3:direct_comm_en  ͸��ֱͨģʽʹ��               ��Vin��comm��level shifterֱͨģʽ
	  1��enable
	  0��disable

	  bit0~bit2:reserved
********************************************************/
#define ICP1205_REV_ANA1       0x12
//�����Ĵ���
#define ICP1205_REV_DAT1       0x14
#define ICP1205_REV_DAT2       0x15
#define ICP1205_REV_DAT3       0x16
#define ICP1205_REV_DAT4       0x17
/*******************************************************
�Ĵ�����0x0b�ɶ���д
���ܣ�		bit1~bit7:reserved
		bit0:int_en    �ж�ʹ��
		1: enable
		0: disable
********************************************************/
#define ICP1205_INT_EN         0x20
/*******************************************************
�Ĵ�����0x0b�ɶ���д
���ܣ�		
		bit7:chrg_fin  �������ж�
		1��������
		bit6:gp0_ch  gp0�ı��ж�
		1��GP0״̬�ı�
		bit5:bat_low ��ص͵��ж�
		1����ص�ѹ��
		bit4: wdt_timeout   wdt��ʱ�ж�
		1�����Ź���ʱ
		bit3:����
		bit2:chrg_err  �������ж�
		1��������
		bit1:Vin_plug out Vin�γ��ж�
		1��Vin �γ�
		bit0:Vin_plug in Vin�����ж�
		1:Vin ����
********************************************************/
#define ICP1205_INT_STAT1      0x21
/*******************************************************
�Ĵ�����0x0b�ɶ���д
���ܣ�		
		bit7:iterm_cmp  ����ֹ�ж�
		1������ֹ
		bit6:Vin_0p35_vbat_cmp  
		1: Vin �� Vbatѹ����0.35V�Ƚ�״̬�ı��ж�
		bit5:Vin_0p15_vbat_cmp
		1: Vin �� Vbatѹ����0.15V�Ƚ�״̬�ı��ж�
		bit4: cv_stat   ��ѹģʽ
		1�������ѹ���ģʽ�ж�
		bit3:chrg_ntc�������ж�
		1��������
		bit2:chrg_trickle_timeout  �����糬ʱ��45���ӣ��ж�
		1�������糬ʱ
		bit1:chrg_timeout  ��糬ʱ�ж�
		1����糬ʱ
		bit0:chrg_state_ch ���״̬�ı��ж�
		1�����״̬�ı�
********************************************************/
#define ICP1205_INT_STAT2      0x22
/*******************************************************
�Ĵ�����0x0b�ɶ���д
���ܣ�		
		bit5~bit7:reserved

		bit4:comm_mode  ͸��ģʽ
		1������͸��ģʽ�ж�
		bit3:rx_failed �ز�����У�����
		1���ز�����У������ж�
		bit2:rx_cmd   �ز��յ�����
		1���ز��յ������ж�
		bit1:key_up  �����ɿ�  1�������ɿ��ж�
		bit0:key_dn  �������� 1�����������ж�
********************************************************/
#define ICP1205_INT_STAT3      0x23
/*******************************************************
�Ĵ�����0x0b�ɶ���д
���ܣ�		
		bit7:chrg_fin 
		1���������ж�����
		bit6:gp0_ch  
		1��GP0״̬�ı��ж�����
		bit5:bat_low
		1����ص�ѹ���ж�����
		bit4: wdt_timeout  
		1�����Ź���ʱ�ж�����
		bit3:����

		bit2:chrg_err
		1���������ж�����
		bit1:Vin_plug out
		1��Vin �γ��ж�����
		bit0:Vin_plug in 
		1:Vin �����ж�����
********************************************************/
#define ICP1205_INT_MASK1      0x24
/*******************************************************
�Ĵ�����0x0b�ɶ���д
���ܣ�		
		bit7:iterm_cmp 
		1������ֹ�ж�����
		bit6:Vin_0p35_vbat_cmp  
		1: Vin �� Vbatѹ����0.35V�Ƚ�״̬�ı��ж�����
		bit5:Vin_0p15_vbat_cmp
		1: Vin �� Vbatѹ����0.15V�Ƚ�״̬�ı��ж�����
		bit4: cv_stat  
		1�������ѹ���ģʽ�ж�����
		bit3:chrg_ntc
		1���������ж�����
		bit2:chrg_trickle_timeout
		1�������糬ʱ��45���ӣ��ж�����
		bit1:chrg_timeout
		1����糬ʱ�ж�����
		bit0:chrg_state_ch
		1�����״̬�ı��ж�����
********************************************************/
#define ICP1205_INT_MASK2      0x25
/*******************************************************
�Ĵ�����0x0b�ɶ���д
���ܣ�		
		bit5~bit7:reserved 
		bit4: comm_mode  
		1������͸��ģʽ�ж�����
		bit3:  rx_failed
		1���ز�����У������ж�����
		bit2:rx_cmd
		1���ز��յ������ж�����
		bit1:key_up
		1�������ɿ��ж�����
		bit0:key_dn
		1�����������ж�����
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
