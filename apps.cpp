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
#include "stdio.h"
#include "cmsis_os.h"
#include "list.h"
#include "string.h"
#include "hal_timer.h"
#include "hal_trace.h"
#include "hal_bootmode.h"
#include "hal_sleep.h"
#include "pmu.h"
#include "audioflinger.h"
#include "apps.h"
#include "app_thread.h"
#include "app_key.h"
#include "app_bt_media_manager.h"
#include "app_pwl.h"
#include "app_audio.h"
#include "app_overlay.h"
#include "app_battery.h"
#include "app_utils.h"
#include "app_status_ind.h"
#include "bt_drv_interface.h"
#include "besbt.h"
#include "norflash_api.h"
#include "nvrecord.h"
#include "nvrecord_dev.h"
#include "nvrecord_env.h"
#include "crash_dump_section.h"
#include "log_section.h"
#include "factory_section.h"
#include "a2dp_api.h"
#include "me_api.h"
#include "os_api.h"
#include "btapp.h"
#include "app_bt.h"
#include "bt_if.h"
#include "gapm_task.h"
#include "app_ble_include.h"
#include "app_bt_func.h"
#ifdef __AI_VOICE__
#include "app_ai_if.h"
#include "app_ai_tws.h"
#include "ai_manager.h"
#include "app_ai_manager_api.h"
#endif

#include "audio_process.h"

#ifdef __PC_CMD_UART__
#include "app_cmd.h"
#endif

#ifdef __FACTORY_MODE_SUPPORT__
#include "app_factory.h"
#include "app_factory_bt.h"
#endif

#ifdef __INTERCONNECTION__
#include "app_interconnection.h"
#include "app_interconnection_ble.h"
#include "app_interconnection_logic_protocol.h"
#include "app_ble_mode_switch.h"
#endif

#ifdef __INTERACTION__
#include "app_interaction.h"
#endif

#ifdef BISTO_ENABLED
#include "app_ai_manager_api.h"
#include "gsound_custom_reset.h"
#include "nvrecord_gsound.h"
#include "gsound_custom_actions.h"
#include "gsound_custom_ota.h"
#endif

#ifdef IBRT_OTA
#include "ota_bes.h"
#endif

#ifdef MEDIA_PLAYER_SUPPORT
#include "resources.h"
#include "app_media_player.h"
#endif

#ifdef VOICE_DATAPATH
#include "app_voicepath.h"
#endif

#ifdef BT_USB_AUDIO_DUAL_MODE
#include "btusb_audio.h"
#include "usbaudio_thread.h"
#endif

#ifdef TILE_DATAPATH
#include "tile_target_ble.h"
#endif

#if defined(IBRT)
#include "app_ibrt_if.h"
#include "app_ibrt_customif_ui.h"
#include "app_ibrt_ui_test.h"
#include "app_ibrt_voice_report.h"
#include "app_tws_if.h"
#endif

#ifdef GFPS_ENABLED
#include "app_gfps.h"
#endif

#ifdef BTIF_BLE_APP_DATAPATH_SERVER
#include "app_ble_cmd_handler.h"
#endif

#ifdef ANC_APP
#include "app_anc.h"
#endif

#ifdef __THIRDPARTY
#include "app_thirdparty.h"
#endif

#ifdef OTA_ENABLED
#include "nvrecord_ota.h"
#include "ota_common.h"
#endif

#include "watchdog/watchdog.h"

#if defined(ANC_ASSIST_ENABLED)
#include "app_anc_assist.h"
#include "app_voice_assist_ai_voice.h"
#include "app_voice_assist_wd.h"
#endif
#ifdef AUDIO_OUTPUT_DC_AUTO_CALIB
#include "codec_calib.h"
#endif

#ifdef __BES2500_IQS269_TOUCH__
#include "IQS269_SETTINGS.h"
#include "IQS269_ADDRESSES.h"
#endif

#ifdef __BES2500_HAILINGWEI_CHC__
#include "app_a2dp.h"
#include "app_ibrt_customif_cmd.h"
#endif



#ifdef __BES2500_HAILINGWEI_UART__
#include "communication_svr.h"
#endif
#ifdef AUDIO_DEBUG_V0_1_0
extern "C" int32_t audio_test_cmd_init(void);
extern "C" int speech_tuning_init(void);
#endif

#if (defined(BTUSB_AUDIO_MODE) || defined(BT_USB_AUDIO_DUAL_MODE))
extern "C"  bool app_usbaudio_mode_on(void) ;
#endif

#ifdef BES_OTA_BASIC
extern "C" void ota_flash_init(void);
#endif

#define APP_BATTERY_LEVEL_LOWPOWERTHRESHOLD (1)
#define POWERON_PRESSMAXTIME_THRESHOLD_MS  (3000)

#ifdef FPGA
uint32_t __ota_upgrade_log_start[100];
#endif

enum APP_POWERON_CASE_T {
    APP_POWERON_CASE_NORMAL = 0,
    APP_POWERON_CASE_DITHERING,
    APP_POWERON_CASE_REBOOT,
    APP_POWERON_CASE_ALARM,
    APP_POWERON_CASE_CALIB,
    APP_POWERON_CASE_BOTHSCAN,
    APP_POWERON_CASE_CHARGING,
    APP_POWERON_CASE_FACTORY,
    APP_POWERON_CASE_TEST,
    APP_POWERON_CASE_INVALID,

    APP_POWERON_CASE_NUM
};

#ifdef RB_CODEC
extern int rb_ctl_init();
extern bool rb_ctl_is_init_done(void);
extern void app_rbplay_audio_reset_pause_status(void);
#endif

#ifdef GFPS_ENABLED
extern "C" void app_fast_pairing_timeout_timehandler(void);
#endif

bool out_of_box_poweron_indication_flag = false;
uint8_t  app_poweroff_flag = 0;
static enum APP_POWERON_CASE_T g_pwron_case = APP_POWERON_CASE_INVALID;

#ifdef __BES2500_IQS269_TOUCH__
bool app_putout_touch_send_flag = true;
uint8_t app_putout_touch_send_cnt = 0;

bool app_putin_touch_send_flag = true;
uint8_t app_putin_touch_send_cnt = 0;
bool app_send_flag = false;

bool app_powerkey_down_flag = true;
bool app_powerkey_up_flag = false;


osTimerId app_earheadset_plugin_plugout_detect_timer = NULL;

static void app_earheadset_plugin_plugout_detect_handler(void const *parma)
{
	if(app_send_flag)
	{
		if(app_putout_touch_send_flag)
		{
			//TRACE(1,"putout cnt %d",app_putout_touch_send_cnt);
			if(++app_putout_touch_send_cnt >= 3)
			{
				app_putout_touch_send_cnt = 0;
				//app_putout_touch_send_flag = false;
				app_send_flag = false;
				//if(!app_need_clear_key_flag)
				{
					send_enter_ear_key_event(HAL_KEY_CODE_FN10, HAL_KEY_EVENT_DOWN);
				}
			}
		}
		else if(app_putin_touch_send_flag)
		{
			//TRACE(1,"putin cnt %d",app_putin_touch_send_cnt);
			if(++app_putin_touch_send_cnt >= 3)
			{
				app_putin_touch_send_cnt = 0;
				//app_putin_touch_send_flag = false;
				app_send_flag = false;
				//if(!app_need_clear_key_flag)
				{
					send_enter_ear_key_event(HAL_KEY_CODE_FN10, HAL_KEY_EVENT_UP);
				}

				//app_need_clear_key_flag = false;
			}
		}
	}
}

osTimerDef (APP_IQS269_EARHEADSET_PLUGIN_PLUGOUT_DETECT, app_earheadset_plugin_plugout_detect_handler);

void app_earheadset_plugin_plugout_detect_timer_start_stop(bool timer_en)
{
	TRACE(1,"app_earheadset_plugin_plugout_detect_timer_start_stop %d",timer_en);

	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}

	if(app_earheadset_plugin_plugout_detect_timer == NULL)
	{
    	app_earheadset_plugin_plugout_detect_timer = osTimerCreate(osTimer(APP_IQS269_EARHEADSET_PLUGIN_PLUGOUT_DETECT),osTimerPeriodic,NULL);
	}
	
	if(timer_en)
	{
		osTimerStart(app_earheadset_plugin_plugout_detect_timer, 200);
	}
	else
	{
		osTimerStop(app_earheadset_plugin_plugout_detect_timer);
	}
}

#endif //end __BES2500_IQS269_TOUCH__


#ifdef __BES2500_HAILINGWEI_CHC__
//HaiLingWei_Variable HLWei_Variable;
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
bool HLWei_hfp_key_call_voice_flag;

uint8_t HLWei_curr_battery_level;
uint8_t HLWei_last_battery_level;
uint8_t HLWei_slave_battery_level;
bool app_battery_lowbat_flag;
uint8_t app_battery_lowbat_play_voice_type;

bool app_battery_charging_flag;
bool app_battery_plugin_init_flag;
bool app_battery_plugout_init_flag;
bool app_battery_send_reconnect_flag;
//bool app_battery_open_box_poweron_flag;
//bool app_reset_poweron_flag;

uint8_t app_led_recover_display_type;
bool tws_role_switch_flag;

uint8_t tws_master_search_count;
bool delay_play_connected_voice_flag;
bool clear_pairlist_reset_poweron_flag;
bool tws_master_delay_to_switch_eq_flag;
bool tws_a2dp_low_latency_open_close_flag;
bool clear_tws_pairlist_reset_poweron_flag;
uint8_t key_siri_open_close_opration;
bool clear_poweroff_flag;
bool app_reboot_poweron_do_not_play_voice_flag;

#ifdef __BES2500_DAKANG_TWS_PAIR__
bool enter_tws_pair_mode_flag;
bool enter_tws_pair_mode_enable_flag;
bool tws_pair_phone_pair_switch_flag;
bool slave_enter_tws_pairmode_flag;
uint8_t slave_enter_tws_pairmode_cnt;
bool master_enter_tws_pairmode_flag;
uint8_t master_enter_tws_pairmode_cnt;
bool master_switch_role_shutdown_flag;
#endif

bool ntc_temper_no_shutdown_flag = false;
uint8_t ntc_temper_mode = NORMAL_MODE;
uint8_t ntc_temper_low_high_cnt = 0;
uint8_t ntc_temper_normal_cnt = 0;
bool spk_mute_flag = false;


/*******************************************************************
美格信SPP通信测试
********************************************************************/
static uint8_t spp_cmd[][5] =
{
	{0xAA, 0x55, 0x06, 0x01, 0xFE},//打开主麦，关闭副麦
	{0xAA, 0x55, 0x06, 0x02, 0xFD},//打开副麦，关闭主麦
	{0xAA, 0x55, 0x06, 0x03, 0xFC},//打开主副MIC
	{0xAA, 0x55, 0x06, 0x04, 0xFB},
	{0xAA, 0x00, 0x01, 0x05, 0xFA},
};

static uint8_t spp_send_data[5] = {0};

extern void app_tota_gen_send_data_via_spp(uint8_t* ptrData, uint32_t length);


int8_t app_spp_data_compare(uint8_t *spp_data)
{
	uint8_t i = 0;

	for(i = 0; i < 5; i++)
	{
		if(memcmp(spp_data, spp_cmd[i], 5) == 0)
		{
			TRACE(1,"app_spp_data_compare %d", i);
			return i;
		}
	}

	return -1;
}

void app_spp_cmd_received_data_prec(uint8_t* ptrParam, uint32_t paramLen)
{
	int8_t ret = -1;
	TRACE(1,"%s %d",__func__, paramLen);
	
	//DUMP8("0x%02x ", ptrParam, paramLen > 32 ? 32 : paramLen);

	if(paramLen == 5)
	{
		ret = app_spp_data_compare(ptrParam);

		memset(ptrParam, 0x00, paramLen);

		spp_send_data[0] = 0xAA;
		spp_send_data[1] = 0x55;
		spp_send_data[2] = 0x08;

		if(ret == -1)
		{
			//TRACE(0,"data is invild");
		}
		else
		{
			if(ret == 0)
			{
				//打开主MIC，关闭副MIC
				if(app_bt_pcm_stream_trigger_change_ch(2))
				{
					TRACE(0,"open MIC1  master MIC");
					spp_send_data[3] = 0x01;
					spp_send_data[4] = 0xFE;
				}
				//app_tota_send_data_via_spp(spp_send_data, 5);
				//DUMP8("0x%02x ", spp_send_data, 5);
				app_tota_gen_send_data_via_spp(spp_send_data, 5);
			}
			else if(ret == 1)
			{
				//打开副MIC，关闭主MIC
				if(app_bt_pcm_stream_trigger_change_ch(1))
				{
					TRACE(0,"open MIC2  slave MIC");
					spp_send_data[3] = 0x02;
					spp_send_data[4] = 0xFD;
					//app_tota_send_data_via_spp(spp_send_data, 5);
					app_tota_gen_send_data_via_spp(spp_send_data, 5);
				}
			}
			else if(ret == 2)
			{
				// 打开双MIC
				TRACE(0,"open two MIC  MIC1   MIC2");
				if(app_bt_pcm_stream_trigger_change_ch(3))
				{
					spp_send_data[3] = 0x03;
					spp_send_data[4] = 0xFC;
					//app_tota_send_data_via_spp(spp_send_data, 5);
					app_tota_gen_send_data_via_spp(spp_send_data, 5);
				}
			}
			else if(ret == 3)
			{
				app_bes_enc_enable_flag = true;
				if(app_bt_pcm_stream_trigger_change_ch(app_bt_stream_ch_change_num))
				{					
					spp_send_data[0] = 0x20;
					spp_send_data[1] = 0x22;
					spp_send_data[2] = 0x04;
					spp_send_data[3] = 0x21;
					spp_send_data[4] = 0x11;
					//app_tota_send_data_via_spp(spp_send_data, 5);
					app_tota_gen_send_data_via_spp(spp_send_data, 5);
				}
			}
			/*
			else if(ret == 4)
			{
				app_bes_enc_enable_flag = false;
				if(app_bt_pcm_stream_trigger_change_ch(app_bt_stream_ch_change_num))
				{
					spp_send_data[2] = 0x01;
					spp_send_data[3] = 0x05;
					app_tota_send_data_via_spp(spp_send_data, 4);
				}
			}
			*/
		}
	}
	else if(paramLen > 5)
	{
		//TRACE(0,"data len is large");
	}
	else
	{
		//TRACE(0,"data len is small");
	}
}



void app_hailingwei_variable_init(void)
{
	tws_connect_disconnect_status = TWS_DEFAULT;
	tws_discon_linkloss_reconnect_cnt = 0;
	phone_connect_disconnect_status = PHONE_DEFAULT;
	phone_linkloss_tws_disconnect_flag = false;
	phone_discon_linkloss_reconnect_cnt = 0;
	phone_from_connected_to_disconnected = false;
	//HLWei_Variable.earheadset_fixed_left_right_type = EARHEADSET_DEFAULT;
	tws_slave_key_function_flag = false;

	HLWei_hfp_end_call_type = DEFAULT_END_CALL;
	HLWei_hfp_sync_ring_flag = true;
	HLWei_hfp_three_way_exist_flag = false;
	HLWei_hfp_incoming_call_flag = false;
	HLWei_hfp_out_call_flag = false;
	HLWei_hfp_answer_voice_play_flag = false;
	HLWei_hfp_answer_call_flag = false;
	HLWei_hfp_key_call_voice_flag = false;

	HLWei_curr_battery_level = 10;
	HLWei_last_battery_level = 10;
	HLWei_slave_battery_level = 10;
	app_battery_lowbat_flag = false;
	app_battery_lowbat_play_voice_type = LOWBAT_DEFAULT;

	app_battery_charging_flag = false;
	app_battery_plugin_init_flag = false;
	app_battery_plugout_init_flag = false;
	app_battery_send_reconnect_flag = false;
	//app_battery_open_box_poweron_flag = false;
	//app_reset_poweron_flag = false;

	app_led_recover_display_type = LED_RECOVER_DEFAULT;
	tws_role_switch_flag = false;

	tws_master_search_count = 0;
	delay_play_connected_voice_flag = false;
	clear_pairlist_reset_poweron_flag = false;
	tws_master_delay_to_switch_eq_flag = false;
	tws_a2dp_low_latency_open_close_flag = false;
	clear_tws_pairlist_reset_poweron_flag = false;
	key_siri_open_close_opration = SIRI_CANCEL_COMP;
	delay_detect_touch_flag = false;
	clear_poweroff_flag = false;
	#ifdef __BES2500_DAKANG_TWS_PAIR__
	enter_tws_pair_mode_flag = false;
	enter_tws_pair_mode_enable_flag = false;
	tws_pair_phone_pair_switch_flag = false;
	slave_enter_tws_pairmode_flag = false;
	slave_enter_tws_pairmode_cnt = 0;
	master_enter_tws_pairmode_flag = false;
	master_enter_tws_pairmode_cnt = 0;
	#endif
	ntc_temper_no_shutdown_flag = false;
	
	app_bt_stream_ch_change_num = 3;
	app_bes_enc_enable_flag = true;
	//spk_mute_flag = false;

	app_reboot_poweron_do_not_play_voice_flag = false;
}

void stop_sniff(void)
{
	app_bt_stop_sniff(0);
}

/*
void app_set_reset(void)
{
	uint32_t delay_time = 6000;
	
	if (app_fixed_left_right_earheadset_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_set_reset_pin_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    }

    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 0);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
	hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 0);
	hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
	hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 0);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
	hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 0);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 0);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 0);
    hal_sys_timer_delay_us(delay_time);
	hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 0);
    hal_sys_timer_delay_us(delay_time);
	hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
    hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 0);
    hal_sys_timer_delay_us(delay_time);
	hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
	hal_sys_timer_delay_us(delay_time);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 0);
    hal_sys_timer_delay_us(delay_time);
	hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_set_reset_pin_cfg.pin, HAL_GPIO_DIR_OUT, 1);
	hal_sys_timer_delay_us(delay_time);
}
*/
extern bool a2dp_is_music_ongoing(void);
extern bool app_keyhandle_a2dp_play(void);
extern bool app_keyhandle_hfp_audio_connected(void);
extern bool app_keyhandle_hfp_incomingcall(void);

#ifndef __BES2500_HAILINGWEI_CHC__
void app_hailingwei_ntc_temperate_proc(uint16_t curr_volt)
{
	//TRACE(4, "%s %d %d %d %d", __func__, app_battery_charging_flag, ntc_temper_normal_cnt, ntc_temper_low_high_cnt, curr_volt);
	if(app_battery_charging_flag)
	{
		//charging  0掳c 1336  ~  -2掳c 1360  2掳c 1305
		if(curr_volt > 1258)
		{
			// < 0掳c
			ntc_temper_normal_cnt = 0;
			if(++ntc_temper_low_high_cnt >= 3)
			{
				//TRACE(0,"SET CLOSE  CHARGE < 0");
				ntc_temper_low_high_cnt = 0;
				ntc_temper_no_shutdown_flag = true;
				app_battery_charge_enable_control(false);
			}
		}
		else if(curr_volt > 435)// 43掳c 520    45掳c 493   47掳c  460
		{
			// 0掳c~45掳c
			ntc_temper_low_high_cnt = 0;
			if(++ntc_temper_normal_cnt >= 3)
			{
				//TRACE(0,"SET CHARGE");
				ntc_temper_no_shutdown_flag = false;
				ntc_temper_normal_cnt = 0;
				app_battery_charge_enable_control(true);
			}
		}
		else
		{
			// >45掳c
			ntc_temper_normal_cnt = 0;
			if(++ntc_temper_low_high_cnt >= 3)
			{
				//TRACE(0,"SET CLOSE  CHARGE > 45");
				ntc_temper_no_shutdown_flag = true;
				ntc_temper_low_high_cnt = 0;
				app_battery_charge_enable_control(false);
			}
		}
	}
	else
	{
		//no charging
		if(a2dp_is_music_ongoing() || app_keyhandle_a2dp_play() || app_keyhandle_hfp_audio_connected())
		{
			//active
			if(curr_volt >= 1540)
			{
				ntc_temper_normal_cnt = 0;
				if(++ntc_temper_low_high_cnt >= 3)
				{
					//TRACE(0,"POWEROFF -20");
					ntc_temper_low_high_cnt = 0;
					app_hailingwei_poweroff_prec(AUTO_POWEROFF, false);
				}
			}
			else if(curr_volt >= 320)
			{
				//nothing
				ntc_temper_normal_cnt = 0;
				ntc_temper_low_high_cnt = 0;
			}
			else
			{
				ntc_temper_normal_cnt = 0;
				if(++ntc_temper_low_high_cnt >= 3)
				{
					//TRACE(0,"POWEROFF > 60");
					ntc_temper_low_high_cnt = 0;
					app_hailingwei_poweroff_prec(AUTO_POWEROFF, false);
				}
			}
		}
		else
		{
			ntc_temper_normal_cnt = 0;
			ntc_temper_low_high_cnt = 0;
		}
	}
}
#endif

void app_hailingwei_earheadset_fixed_type(void)
{
	uint8_t low_level_cnt = 0;
	uint8_t high_level_cnt = 0;
	
	if (app_fixed_left_right_earheadset_cfg.pin != HAL_IOMUX_PIN_NUM)
    {
        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_fixed_left_right_earheadset_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_fixed_left_right_earheadset_cfg.pin, HAL_GPIO_DIR_IN, 1);
    }

    while(1)
    {
    	if(hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_fixed_left_right_earheadset_cfg.pin))
    	{
    		high_level_cnt++;
    		low_level_cnt = 0;
    	}
    	else
    	{
    		low_level_cnt++;
    		high_level_cnt = 0;
    	}

    	if((high_level_cnt >= 3) || (low_level_cnt >= 3))
    	{
    		break;
    	}

    	osDelay(20);
    }

    if(high_level_cnt >= 3)
    {
    	earheadset_fixed_left_right_type = RIGHT_EARHEADSET;
    }
    else if(low_level_cnt >= 3)
    {
    	earheadset_fixed_left_right_type = LEFT_EARHEADSET;
    }

    TRACE(1,"left_right_type = %d", earheadset_fixed_left_right_type);
}


uint8_t app_earheadset_fixed_type_get(void)
{
	return earheadset_fixed_left_right_type;
}


uint8_t app_get_curr_master_role(void)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	if(app_tws_ibrt_tws_link_connected())
	{
		if(p_ibrt_ctrl->current_role == IBRT_MASTER)
		{
			return IBRT_MASTER;
		}
		else
		{
			return IBRT_SLAVE;
		}
	}
	else
	{
		return IBRT_UNKNOW;
	}
}


void app_charging_stop_tws_search(void)
{
	app_stop_tws_serching_direactly();
}


bool app_get_charge_status(void)
{
	return app_battery_charging_flag;
}


bool app_get_led_display_recover_enable(void)
{
	if((app_led_recover_display_type == LED_POWERON_RECOVER)
	||(app_led_recover_display_type == LED_NORMAL_RECOVER))
	{
		return false;
	}

	return true;
}


void app_led_sync_hfp_proc(bool send_flag, uint8_t status)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	if(app_tws_ibrt_tws_link_connected())
	{
		if((p_ibrt_ctrl->current_role == IBRT_MASTER)&&(send_flag))
		{
			app_ibrt_customif_cmd_sync_led(status);
		}
		else
		{
			app_status_indication_set((APP_STATUS_INDICATION_T)status);
		}
	}
}


/* handle the power off before role switch */
void app_hailingwei_poweroff_prec(uint8_t shutdown_type, bool slave_shutdown_flag)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	TRACE(2,"%s shutdown %d tws connect %d", __func__, shutdown_type, app_tws_ibrt_tws_link_connected());
	
	if(app_tws_ibrt_tws_link_connected())
	{
		TRACE(2,"phone connect %d slave phone %d", app_tws_ibrt_mobile_link_connected(), app_tws_ibrt_slave_ibrt_link_connected());
		if(app_tws_ibrt_mobile_link_connected() || app_tws_ibrt_slave_ibrt_link_connected())
		{
			if(shutdown_type == AUTO_POWEROFF)
			{
				app_ibrt_customif_cmd_poweroff(AUTO_POWEROFF, true, IBRT_MASTER);
				osDelay(100);
				app_shutdown();
			}
			else if(shutdown_type == LOWBATTERY_POWEROFF)
			{
				if(p_ibrt_ctrl->current_role == IBRT_MASTER)
				{
					app_tws_if_trigger_role_switch();
					master_switch_role_shutdown_flag = true;
					app_delay_to_deal_timer_start_stop(true);
				}
				else if(p_ibrt_ctrl->current_role == IBRT_SLAVE)
				{
					app_shutdown();
				}
			}
			else if(shutdown_type == KEY_POWEROFF)
			{
				slave_shutdown_flag = 0;	//make compiler happy
				/*if(slave_shutdown_flag)
				{
					app_shutdown();
					//app_ibrt_customif_cmd_poweroff(KEY_POWEROFF, true, IBRT_MASTER);
				}
				else
				{
					//app_ibrt_customif_cmd_poweroff(AUTO_POWEROFF, true, IBRT_MASTER);
					app_tws_if_trigger_role_switch();
					master_switch_role_shutdown_flag = true;
					app_delay_to_deal_timer_start_stop(true);
				}*/
				/*
				app_ibrt_customif_cmd_poweroff(AUTO_POWEROFF, true, IBRT_MASTER);
				osDelay(100);
				app_shutdown();
				*/
				if(p_ibrt_ctrl->current_role == IBRT_MASTER)
				{
					app_tws_if_trigger_role_switch();
					master_switch_role_shutdown_flag = true;
					app_delay_to_deal_timer_start_stop(true);
				}
				else if(p_ibrt_ctrl->current_role == IBRT_SLAVE)
				{
					app_shutdown();
				}				
			}
		}
		else
		{
			if(shutdown_type != KEY_POWEROFF)
			{	
				//sync shutdown
				app_ibrt_customif_cmd_poweroff(AUTO_POWEROFF, true, IBRT_MASTER);
			}	
			osDelay(100);
			app_shutdown();
		}
	}
	else
	{
		//shutdown
		app_shutdown();
	}
}





// battery level master
void app_master_recive_battery_level_proc(uint8_t slave_level)
{
	uint8_t curr_level = 10;

	//TRACE(0, "app_master_recive_battery_level_proc");
	if(app_get_curr_master_role() == IBRT_MASTER)
	{
		app_battery_get_info(NULL, &curr_level, NULL);
		if(curr_level >= slave_level)
		{
			// nothing
			HLWei_curr_battery_level = slave_level;
			if(slave_level > 1)
			{
				app_status_battery_report(slave_level);
			}
			else
			{
				if(!app_battery_lowbat_flag)
				{
					app_battery_lowbat_set_status();
					app_battery_lowbat_play_prec();
				}
			}
		}
		else
		{
			//nothing
		}
	}
}




//hfp end voice play
void app_hfp_end_call_voice_play(APP_STATUS_INDICATION_T status, uint8_t device_id, bool lowbat_flag)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	uint16_t aud_pram = 0;

	if((HLWei_hfp_key_call_voice_flag)||(lowbat_flag))
	{
		if(app_tws_ibrt_tws_link_connected())
		{
			if(p_ibrt_ctrl->current_role == IBRT_MASTER)
			{
				app_voice_report(status,device_id);
			}
		}
		else
		{
			//uint16_t aud_pram = 0;
			aud_pram |=PROMOT_ID_BIT_MASK_CHNLSEl_ALL;
			aud_pram |= PROMOT_ID_BIT_MASK_MERGING;
			//app_voice_report(APP_STATUS_INDICATION_TONE_DU, 0);
			trigger_media_play(AUDIO_ID_BT_TONE_DU, 0, aud_pram);
			//app_voice_report(status,device_id);
		}

		HLWei_hfp_key_call_voice_flag = false;
	}
	else
	{
		if(app_poweroff_flag)
		{
			if(app_tws_ibrt_tws_link_connected())
			{
				if(p_ibrt_ctrl->current_role == IBRT_MASTER)
				{
					app_voice_report(status,device_id);
				}
			}
			else
			{
				//uint16_t aud_pram = 0;
				aud_pram |=PROMOT_ID_BIT_MASK_CHNLSEl_ALL;
				aud_pram |= PROMOT_ID_BIT_MASK_MERGING;
				//app_voice_report(APP_STATUS_INDICATION_TONE_DU, 0);
				trigger_media_play(AUDIO_ID_BT_TONE_DU, 0, aud_pram);
				///app_voice_report(status,device_id);
			}
		}
	}
}





#ifdef __BES2500_HAILINGWEI_LOWLATENCY__
bool app_get_curr_latency_mode_status(void)
{
	TRACE(1,"%s low latency flag %d",__func__, tws_a2dp_low_latency_open_close_flag);
	return tws_a2dp_low_latency_open_close_flag;
}


void app_set_curr_latency_mode_status(bool mode)
{
	tws_a2dp_low_latency_open_close_flag = mode;
	TRACE(1,"%s low latency flag %d",__func__, tws_a2dp_low_latency_open_close_flag);
}


void app_switch_latency_mode(bool switch_mode_flag)
{
	if(switch_mode_flag)
	{
		if(tws_a2dp_low_latency_open_close_flag)
		{
			tws_a2dp_low_latency_open_close_flag = false;
			app_voice_report(APP_STATUS_INDICATION_NORMAL_LATENCY,0);
		}
		else
		{
			tws_a2dp_low_latency_open_close_flag = true;
			app_voice_report(APP_STATUS_INDICATION_LOW_LATENCY,0);
		}
	}
	else
	{
		//tws connected sync latency
		if(tws_a2dp_low_latency_open_close_flag)
		{
			app_voice_report(APP_STATUS_INDICATION_LOW_LATENCY,0);
		}
		else
		{
			app_voice_report(APP_STATUS_INDICATION_NORMAL_LATENCY,0);
		}
	}
}
#endif

// clear phone and tws pairlist
void app_clear_all_pairlst_reset_poweron(bool clear_tws, bool clear_phone)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();

	TRACE(2,"%s clear_tws %d clear_phone %d",__func__, clear_tws, clear_phone);

	app_bt_set_default_volume();
	
	if(!app_tws_ibrt_mobile_link_connected())	
	{
		
		if(app_tws_ibrt_tws_link_connected())
		{
			if(p_ibrt_ctrl->current_role == IBRT_MASTER)
			{
				//send cmd to slave
				if(!clear_tws_pairlist_reset_poweron_flag)
				{
					app_ibrt_customif_cmd_clear_pairlist(clear_tws, clear_phone);
				}
			}

			//reset poweron
			clear_pairlist_reset_poweron_flag = true;
			if(p_ibrt_ctrl->current_role == IBRT_SLAVE)
			{
				//hal_sw_bootmode_set(HAL_SW_BOOTMODE_CLEAR_PAIRLIST_POWERON);
				//app_reset();
				if(clear_tws_pairlist_reset_poweron_flag)
				{
					// clear tws pairlist
					if(clear_tws)
					{
						app_ibrt_remove_history_tws_paired_device();
					}
					// clear phone pairlist
					if(clear_phone)
					{
						app_ibrt_remove_history_phone_paired_device();
					}
					hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT_ENTER_PHONE_PAIR);
					app_reset();
				}
				else
				{
					//clear_pairlist_reset_poweron_flag = false;
					//app_shutdown();
					//app_delay_to_deal_timer_start_stop(true);
					//hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT_ENTER_TWS_PAIR);
					//app_reset();
					// clear tws pairlist
					if(clear_tws)
					{
						app_ibrt_remove_history_tws_paired_device();
					}
					// clear phone pairlist
					if(clear_phone)
					{
						app_ibrt_remove_history_phone_paired_device();
					}
					//app_status_indication_set(APP_STATUS_INDICATION_INITIAL);
					//clear_pairlist_reset_poweron_flag = false;
					//clear_poweroff_flag = true;
					//app_shutdown();
					app_delay_to_deal_timer_start_stop(false);
					delay_detect_touch_flag = false;
					app_delay_to_deal_timer_start_stop(true);
				}
			}
			else if(p_ibrt_ctrl->current_role == IBRT_MASTER)
			{
				app_delay_to_deal_timer_start_stop(false);
				delay_detect_touch_flag = false;
				app_delay_to_deal_timer_start_stop(true);
				// clear tws pairlist
				if(clear_tws)
				{
					app_ibrt_remove_history_tws_paired_device();
				}
				// clear phone pairlist
				if(clear_phone)
				{
					app_ibrt_remove_history_phone_paired_device();
				}
			}
		}
		else
		{
			//reset poweron
			clear_pairlist_reset_poweron_flag = true;
			//hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT_ENTER_TWS_PAIR);
			//app_reset();
			delay_detect_touch_flag = false;
			app_delay_to_deal_timer_start_stop(false);
			app_delay_to_deal_timer_start_stop(true);
			// clear tws pairlist
			if(clear_tws)
			{
				app_ibrt_remove_history_tws_paired_device();
			}
			// clear phone pairlist
			if(clear_phone)
			{
				app_ibrt_remove_history_phone_paired_device();
			}
		}
	}
}


#ifdef __BES2500_HAILINGWEI_SWITCH_EQ__
//SWITCH EQ
extern uint8_t bt_audio_get_eq_index(AUDIO_EQ_TYPE_T audio_eq_type,uint8_t anc_status);
extern void bt_audio_set_eq_index(uint8_t index_eq);

void app_tws_switch_eq(bool switch_flag, uint8_t eq_mode)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	uint8_t curr_eq_mode;

	TRACE(2,"%s switch_flag %d eq_mode %d",__func__, switch_flag, eq_mode);
	
	if(app_tws_ibrt_tws_link_connected())
	{
		if(p_ibrt_ctrl->current_role == IBRT_MASTER)
		{
			//send cmd to slave switch eq
			if(switch_flag)
			{
				curr_eq_mode = bt_audio_get_eq_index(AUDIO_EQ_TYPE_HW_DAC_IIR, 0);

				app_ibrt_customif_cmd_switch_eqmode(switch_flag, curr_eq_mode);

				//delay switch eq
				tws_master_delay_to_switch_eq_flag = true;
				app_delay_to_deal_timer_start_stop(true);
			}
			else
			{
				//tws connected sync master eq
				curr_eq_mode = bt_audio_get_eq_index(AUDIO_EQ_TYPE_HW_DAC_IIR, 0);
				app_ibrt_customif_cmd_switch_eqmode(switch_flag, curr_eq_mode);
			}
		}
		else if(p_ibrt_ctrl->current_role == IBRT_SLAVE)
		{
			if(switch_flag)
			{
				bt_audio_set_eq_index(eq_mode);
				//switch eq
				bt_audio_switch_eq_mode_and_get_curr_mode(true);
			}
			else
			{
				bt_audio_set_eq_index(eq_mode);
				// set eq
				bt_audio_switch_eq_mode_and_get_curr_mode(false);
			}
		}
	}
	else
	{
		//switch eq
		if(switch_flag)
		{
			bt_audio_switch_eq_mode_and_get_curr_mode(true);
		}
	}
}
#endif


//timer

// all
osTimerId app_delay_to_deal_timer = NULL;
extern void bt_key_handle_func_keyup(void);

void app_delay_to_deal_timehandler(void const *param)
{
	//ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	TRACE(0, "%s ", __func__);
	//app_battery_charge_enable_control(true);
	#ifdef __BES2500_HAILINGWEI_SWITCH_EQ__
	if(tws_master_delay_to_switch_eq_flag)
	{
		bt_audio_switch_eq_mode_and_get_curr_mode(true);
		tws_master_delay_to_switch_eq_flag = false;
	}
	#endif

	if(delay_detect_touch_flag)
	{
		delay_detect_touch_flag = false;
	}

	if(master_switch_role_shutdown_flag)
	{
		//TRACE(1,"p_ibrt_ctrl->cur role %d",p_ibrt_ctrl->current_role);
		master_switch_role_shutdown_flag = false;
		app_shutdown();
	}


	if(clear_tws_pairlist_reset_poweron_flag)
	{
		hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT_ENTER_PHONE_PAIR);
		app_reset();
	}

	if(clear_pairlist_reset_poweron_flag)
	{
		//hal_sw_bootmode_set(HAL_SW_BOOTMODE_CLEAR_PAIRLIST_POWERON);
		//app_reset();
		app_status_indication_set(APP_STATUS_INDICATION_INITIAL);
		clear_pairlist_reset_poweron_flag = false;
		clear_poweroff_flag = true;
		app_shutdown();
		//hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT_ENTER_TWS_PAIR);
		//app_reset();
	}

	if(HLWei_hfp_answer_voice_play_flag)
	{
		HLWei_hfp_answer_voice_play_flag = false;
		//answer tone
		//app_voice_report(APP_STATUS_INDICATION_TONE_DU, 0);
	}

	if(key_siri_open_close_opration == OPRATION_SIRI)
	{
		//bt_key_handle_func_keyup();
		key_siri_open_close_opration = SIRI_CANCEL_COMP;
	}
}

osTimerDef (APP_DELAY_TO_DEAL, (void (*)(void const *))app_delay_to_deal_timehandler);

void app_delay_to_deal_timer_start_stop(bool timer_en)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);

	/*
	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}
	*/
	
	if(app_delay_to_deal_timer == NULL)
	{
    	app_delay_to_deal_timer = osTimerCreate(osTimer(APP_DELAY_TO_DEAL),osTimerOnce,NULL);
	}

	if(timer_en)
	{
		if(HLWei_hfp_answer_voice_play_flag)
		{
			osTimerStart(app_delay_to_deal_timer, 1100);
		}
		else if(key_siri_open_close_opration == OPRATION_SIRI)
		{
			osTimerStart(app_delay_to_deal_timer, 500);
		}
		else if(master_switch_role_shutdown_flag)
		{
			osTimerStart(app_delay_to_deal_timer, 3000);
		}
		else if(delay_detect_touch_flag)
		{
			osTimerStart(app_delay_to_deal_timer, 4000);
		}
		else
		{
			osTimerStart(app_delay_to_deal_timer, 300);
			//osTimerStart(app_delay_to_deal_timer, 5000);
		}
	}
	else
	{
		osTimerStop(app_delay_to_deal_timer);
	}
}




// phone end three way call tran to earphone
osTimerId app_end_call_tran_way_to_headset_timer = NULL;

void app_end_call_tran_way_to_headset_timehandler(void const *param)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	TRACE(0, "%s ", __func__);

	
	if(app_tws_ibrt_tws_link_connected())
	{
		if((HLWei_hfp_three_way_exist_flag)&&(p_ibrt_ctrl->current_role == IBRT_MASTER))
		{
			hfp_handle_key(HFP_KEY_THREEWAY_HOLD_AND_ANSWER);
		}
		HLWei_hfp_three_way_exist_flag = false;
	}
	else
	{
		if(HLWei_hfp_three_way_exist_flag)
		{
			hfp_handle_key(HFP_KEY_THREEWAY_HOLD_AND_ANSWER);
		}
		HLWei_hfp_three_way_exist_flag = false;
	}
	
}

osTimerDef (APP_END_CALL_TRAN_WAY_TO_HEADSET_TIMER, (void (*)(void const *))app_end_call_tran_way_to_headset_timehandler);

void app_end_call_tran_way_to_headset_timer_start_stop(bool timer_en)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);
	
	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}

	
	if(app_end_call_tran_way_to_headset_timer == NULL)
	{
    	app_end_call_tran_way_to_headset_timer = osTimerCreate(osTimer(APP_END_CALL_TRAN_WAY_TO_HEADSET_TIMER),osTimerOnce,NULL);
	}

	if(timer_en)
	{
		osTimerStart(app_end_call_tran_way_to_headset_timer, 100);
	}
	else
	{
		osTimerStop(app_end_call_tran_way_to_headset_timer);
	}
}



osTimerId app_a2dp_auto_play_timer = NULL;
//extern bool a2dp_is_music_ongoing(void);
#ifdef USER_REBOOT_PLAY_MUSIC_AUTO
extern bool a2dp_need_to_play;
uint8_t a2dp_auto_play_cnt = 0;
#endif

void app_a2dp_auto_play_timehandler(void const *param)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	TRACE(0, "%s ", __func__);

	if((!a2dp_is_music_ongoing()) && (!app_keyhandle_hfp_audio_connected())
		&& (!app_keyhandle_hfp_incomingcall()))
		{
			if(a2dp_need_to_play)
			{
				if(p_ibrt_ctrl->current_role == IBRT_MASTER)
				{
					if(++a2dp_auto_play_cnt <= 6)
					{
						a2dp_handleKey(AVRCP_KEY_PLAY);
						app_a2dp_auto_play_timer_start_stop(true);
					}
				}
			}
		}
		else
		{
			a2dp_auto_play_cnt = 0;
			a2dp_need_to_play = false;
		}

	if(p_ibrt_ctrl->current_role == IBRT_SLAVE)
	{
		if(a2dp_need_to_play)
		{
			if(++a2dp_auto_play_cnt <= 6)
			{
				send_enter_ear_key_event(HAL_KEY_CODE_PWR, HAL_KEY_EVENT_DOUBLECLICK);	
				app_a2dp_auto_play_timer_start_stop(true);
			}
		}
	}
	
}

osTimerDef (APP_A2DP_AUTO_PLAY_TIMER, (void (*)(void const *))app_a2dp_auto_play_timehandler);

void app_a2dp_auto_play_timer_start_stop(bool timer_en)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);
	
	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}

	
	if(app_a2dp_auto_play_timer == NULL)
	{
    	app_a2dp_auto_play_timer = osTimerCreate(osTimer(APP_A2DP_AUTO_PLAY_TIMER),osTimerOnce,NULL);
	}

	if(timer_en)
	{
		osTimerStart(app_a2dp_auto_play_timer, 3000);
	}
	else
	{
		a2dp_auto_play_cnt = 0;
		osTimerStop(app_a2dp_auto_play_timer);
	}
}




// phone end three way call tran to earphone
uint8_t app_freq_change_cnt = 0;
bool app_freq_change_flag = false;
uint8_t app_recover_freq = HAL_CMU_FREQ_32K;
osTimerId app_freq_timer = NULL;


void app_freq_timehandler(void const *param)
{
	//TRACE(0, "%s ", __func__);
	if(app_bt_freq_change_proc())
	{
		app_freq_timer_start_stop(false);
	}
}

osTimerDef (APP_FREQ_TIMER, (void (*)(void const *))app_freq_timehandler);

void app_freq_timer_start_stop(bool timer_en)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);
	
	//if((app_battery_charging_flag)&&(timer_en))
	//{
	//	return;
	//}

	
	if(app_freq_timer == NULL)
	{
    	app_freq_timer = osTimerCreate(osTimer(APP_FREQ_TIMER),osTimerOnce,NULL);
	}

	if(timer_en)
	{
		app_freq_change_cnt = 0;
		app_freq_change_flag = true;
		osTimerStart(app_freq_timer, 12000);
	}
	else
	{
		app_freq_change_cnt = 0;
		app_freq_change_flag = false;
		osTimerStop(app_freq_timer);
	}
}

// LED 
osTimerId app_recover_led_display_timer = NULL;

void app_recover_led_display_timehandler(void const *param)
{
	TRACE(0, "%s", __func__);

	if(!tws_role_switch_flag)
	{
		app_led_recover_display_type = LED_RECOVER_DEFAULT;
		app_status_indication_set(APP_STATUS_INDICATION_INITIAL);
		app_status_indication_set(app_last_status_indication_get());
	}
	else
	{
		//shutdown afther tws role switch 
		tws_role_switch_flag = false;
		app_shutdown();
	}
}

osTimerDef (APP_RECOVER_LED_DISPLAY_TIMER, (void (*)(void const *))app_recover_led_display_timehandler);

void app_recover_led_display_timer_start_stop(bool timer_en, uint8_t display_type)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);
	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}

	if(app_recover_led_display_timer == NULL)
	{
    	app_recover_led_display_timer = osTimerCreate(osTimer(APP_RECOVER_LED_DISPLAY_TIMER),osTimerOnce,NULL);
	}
	
	if(timer_en)
	{
		app_led_recover_display_type = display_type;
		if(tws_role_switch_flag)
		{
			osTimerStart(app_recover_led_display_timer, 1500);
		}
		else
		{
			osTimerStart(app_recover_led_display_timer, 1200);
		}
	}
	else
	{
		app_led_recover_display_type = display_type;
		osTimerStop(app_recover_led_display_timer);
	}
}


// open close box
void app_key_init(void);
void app_key_init_on_charging(void);

void app_putin_box_handler(void)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();

	if(app_battery_charging_flag)
	{
		TRACE(0,"close box");
		//close box
		if (p_ibrt_ctrl->nv_role == IBRT_UNKNOW)
		{
			app_ibrt_ui_event_entry(IBRT_CLOSE_BOX_EVENT);
		}
		else
		{
			// do nothing
		}

		app_status_indication_set(APP_STATUS_INDICATION_INITIAL);
		app_status_indication_set(APP_STATUS_INDICATION_CHARGING);
		app_key_init_on_charging();

		pmu_sleep_en(false);
	}
}

void app_putout_box_handler(void)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	if(!app_battery_charging_flag)
	{
		pmu_sleep_en(true);
		//if((app_battery_open_box_poweron_flag)/*||(app_reset_poweron_flag)*/)
		{
			//app_battery_open_box_poweron_flag = false;
			//open box
			//app_status_indication_set(APP_STATUS_INDICATION_INITIAL);
			//app_status_indication_set(APP_STATUS_INDICATION_POWERON);
			//app_recover_led_display_timer_start_stop(true, LED_POWERON_RECOVER);
				
			if(p_ibrt_ctrl->nv_role == IBRT_UNKNOW)
			{
				TRACE(0,"open box ibrt_ui_log:power on unknow mode");
	        	#ifdef __BES2500_DAKANG_TWS_PAIR__
				if(enter_tws_pair_mode_flag)
				{
					app_ibrt_enter_limited_mode();
	            	#ifdef __BES2500_HAILINGWEI_CHC__
					if(app_earheadset_fixed_type_get() == LEFT_EARHEADSET)
					{
						//master
						app_start_tws_serching_direactly();
					}
					else
					{
						//slave
						app_status_indication_set(APP_STATUS_INDICATION_CONNECTING);
						//app_slave_exit_tws_pairmode_timer_start_stop(true);
						#ifdef __BES2500_DAKANG_TWS_PAIR__
						slave_enter_tws_pairmode_flag = true;
						slave_enter_tws_pairmode_cnt = 0;
						app_tws_pairfail_enter_phone_pairmode_timer_start_stop(true);
						#endif
					}
	            	#endif
				}
				else
				{
					enter_tws_pair_mode_enable_flag = true;
					app_ibrt_if_enter_freeman_pairing();
				}
	        	#else
				app_ibrt_enter_limited_mode();
				if(app_earheadset_fixed_type_get() == LEFT_EARHEADSET)
				{
					//master
					app_start_tws_serching_direactly();
				}
				else
				{
					//slave
					app_status_indication_set(APP_STATUS_INDICATION_CONNECTING);
				}
	        	#endif
			}
			else
			{
				//enter openreconnect
				if(app_tws_ibrt_tws_link_connected())
				{
					if(p_ibrt_ctrl->current_role == IBRT_SLAVE)
					{
						app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
					}
					else if(p_ibrt_ctrl->current_role == IBRT_MASTER)
					{
						if(app_tws_ibrt_mobile_link_connected())
						{
							app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
						}
						else
						{
							if(app_ibrt_ui_get_enter_pairing_mode())
							{
								app_status_indication_set(APP_STATUS_INDICATION_BOTHSCAN);
							}
							else
							{
								app_status_indication_set(APP_STATUS_INDICATION_PAGESCAN);
							}
						}
					}
				}
				else
				{
					if(app_tws_ibrt_mobile_link_connected())
					{
						app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
					}
					else
					{
						if(app_ibrt_ui_get_enter_pairing_mode())
						{
							app_status_indication_set(APP_STATUS_INDICATION_BOTHSCAN);
						}
						else
						{
							app_status_indication_set(APP_STATUS_INDICATION_PAGESCAN);
						}
					}
				}
			}

			app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
			//app_key_init();
			//open uart
			#ifdef __BES2500_HAILINGWEI_UART__
			if(current_box_status == IN_BOX_STATUS)
			{
				app_ibrt_uart_func_enable_close(true);
			}
			#endif
		}
		//else
		//{
		//	app_shutdown();
		//}
	}
}

#if 0
osTimerId app_open_box_close_box_timer = NULL;

void app_open_box_close_box_timehandler(void const *param)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();

	TRACE(1, "%s nv_role %d", __func__, p_ibrt_ctrl->nv_role);

	if(app_battery_charging_flag)
	{
		TRACE(0,"close box");
		//close box
		if (p_ibrt_ctrl->nv_role == IBRT_UNKNOW)
		{
			app_ibrt_ui_event_entry(IBRT_CLOSE_BOX_EVENT);
		}
		else
		{
			// do nothing
		}

		app_status_indication_set(APP_STATUS_INDICATION_CHARGING);
		app_key_init_on_charging();
	}
	else
	{
		//open box
		app_status_indication_set(APP_STATUS_INDICATION_POWERON);
		app_recover_led_display_timer_start_stop(true, LED_POWERON_RECOVER);
		
		if(p_ibrt_ctrl->nv_role == IBRT_UNKNOW)
		{
            TRACE(0,"open box ibrt_ui_log:power on unknow mode");
            #ifdef __BES2500_DAKANG_TWS_PAIR__
            if(enter_tws_pair_mode_flag)
            {
                app_ibrt_enter_limited_mode();
                #ifdef __BES2500_HAILINGWEI_CHC__
                if(app_earheadset_fixed_type_get() == LEFT_EARHEADSET)
                {
                	//master
                	app_start_tws_serching_direactly();
                }
                else
                {
                	//slave
                	app_status_indication_set(APP_STATUS_INDICATION_CONNECTING);
                	//app_slave_exit_tws_pairmode_timer_start_stop(true);
                	slave_enter_tws_pairmode_flag = true;
                	slave_enter_tws_pairmode_cnt = 0;
                	app_enter_phone_pairmode_play_voice_timer_start_stop(true);
                	
                }
                #endif
            }
            else
            {
            	enter_tws_pair_mode_enable_flag = true;
            	app_ibrt_if_enter_freeman_pairing();
            }
            #else
            app_ibrt_enter_limited_mode();
            if(app_earheadset_fixed_type_get() == LEFT_EARHEADSET)
            {
            	//master
            	app_start_tws_serching_direactly();
            }
            else
            {
            	//slave
            	app_status_indication_set(APP_STATUS_INDICATION_CONNECTING);
            }
            #endif
		}
		else
		{
			//enter openreconnect
			app_status_indication_set(APP_STATUS_INDICATION_PAGESCAN);
		}

		app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
		app_key_init();
		//open uart
		#ifdef __BES2500_HAILINGWEI_UART__
		app_ibrt_uart_func_enable_close(true);
		#endif
	}
}


osTimerDef (APP_OPEN_BOX_CLOSE_BOX_TIMER, (void (*)(void const *))app_open_box_close_box_timehandler);

void app_open_box_close_box_timer_start_stop(bool timer_en)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);

	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}
	/*
	if(app_open_box_close_box_timer == NULL)
	{
    	app_open_box_close_box_timer = osTimerCreate(osTimer(APP_OPEN_BOX_CLOSE_BOX_TIMER),osTimerOnce,NULL);
	}
	
	if(timer_en)
	{
		osTimerStart(app_open_box_close_box_timer, 100);
	}
	else
	{
		osTimerStop(app_open_box_close_box_timer);
	}
	*/
}
#endif

// earheadset enter pairmode
osTimerId app_enter_phone_pairmode_play_voice_timer = NULL;

void app_enter_phone_pairmode_play_voice_timehandler(void const *param)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	TRACE(2, "%s pairing_mode %d access_mode%d", __func__, app_ibrt_ui_get_enter_pairing_mode(), p_ibrt_ctrl->access_mode);

	#ifndef __BES2500_DAKANG_TWS_PAIR__
	if(slave_enter_tws_pairmode_flag)
	{
		if(++slave_enter_tws_pairmode_cnt >= 9)
		{
			slave_enter_tws_pairmode_flag = false;
			slave_enter_tws_pairmode_cnt = 0;
			tws_pair_phone_pair_switch_flag = true;
			hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT_ENTER_PHONE_PAIR);
			app_reset();
		}
		return;
	}
	#endif
	
	if(app_ibrt_ui_get_enter_pairing_mode()/* &&(p_ibrt_ctrl->access_mode == BTIF_BAM_GENERAL_ACCESSIBLE)*/)
	{
		//app_sysfreq_req(APP_SYSFREQ_USER_APP_INIT, APP_SYSFREQ_13M);
		app_status_indication_set(APP_STATUS_INDICATION_BOTHSCAN);
		app_voice_report(APP_STATUS_INDICATION_BOTHSCAN,0);
		app_start_10_second_timer(APP_PAIR_TIMER_ID);
		app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
		//if(!app_tws_ibrt_tws_link_connected())
		{
			enter_tws_pair_mode_enable_flag = true;
		}
		app_enter_phone_pairmode_play_voice_timer_start_stop(false);
	}
}

osTimerDef (APP_ENTER_PHONE_PAIRMODE_PLAY_VOICE_TIMER, (void (*)(void const *))app_enter_phone_pairmode_play_voice_timehandler);

void app_enter_phone_pairmode_play_voice_timer_start_stop(bool timer_en)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);
	
	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}

	if(app_enter_phone_pairmode_play_voice_timer == NULL)
	{
    	app_enter_phone_pairmode_play_voice_timer = osTimerCreate(osTimer(APP_ENTER_PHONE_PAIRMODE_PLAY_VOICE_TIMER),osTimerOnce,NULL);
	}

	if(timer_en)
	{
		osTimerStart(app_enter_phone_pairmode_play_voice_timer, 1200);
	}
	else
	{
		osTimerStop(app_enter_phone_pairmode_play_voice_timer);
	}
}


// tws pairfail enter phone pair
osTimerId app_tws_pairfail_enter_phone_pairmode_timer = NULL;

void app_tws_pairfail_enter_phone_pairmode_timehandler(void const *param)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	TRACE(2, "%s pairing_mode %d access_mode%d", __func__, app_ibrt_ui_get_enter_pairing_mode(), p_ibrt_ctrl->access_mode);

	#ifdef __BES2500_DAKANG_TWS_PAIR__
	if(slave_enter_tws_pairmode_flag)
	{
		if(++slave_enter_tws_pairmode_cnt >= 9)
		{
			slave_enter_tws_pairmode_flag = false;
			slave_enter_tws_pairmode_cnt = 0;
			tws_pair_phone_pair_switch_flag = true;
			hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT_ENTER_PHONE_PAIR);
			app_reset();
		}
		return;
	}

	if(master_enter_tws_pairmode_flag)
	{
		if(++master_enter_tws_pairmode_cnt >= 9)
		{
			master_enter_tws_pairmode_flag = false;
			master_enter_tws_pairmode_cnt = 0;
			tws_pair_phone_pair_switch_flag = true;
			hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT_ENTER_PHONE_PAIR);
			app_reset();
		}
		return;
	}
	#endif
}

osTimerDef (APP_TWS_PAIRFAIL_ENTER_PHONE_PAIRMODE_TIMER, (void (*)(void const *))app_tws_pairfail_enter_phone_pairmode_timehandler);

void app_tws_pairfail_enter_phone_pairmode_timer_start_stop(bool timer_en)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);
	
	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}

	if(app_tws_pairfail_enter_phone_pairmode_timer == NULL)
	{
    	app_tws_pairfail_enter_phone_pairmode_timer = osTimerCreate(osTimer(APP_TWS_PAIRFAIL_ENTER_PHONE_PAIRMODE_TIMER),osTimerPeriodic,NULL);
	}

	if(timer_en)
	{
		osTimerStart(app_tws_pairfail_enter_phone_pairmode_timer, 1200);
	}
	else
	{
		osTimerStop(app_tws_pairfail_enter_phone_pairmode_timer);
	}
}


// 
osTimerId app_timer = NULL;
extern struct BT_DEVICE_T  app_bt_device;

void app_timehandler(void const *param)
{
	//ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	//TRACE(2, "%s pairing_mode %d access_mode%d", __func__, app_ibrt_ui_get_enter_pairing_mode(), p_ibrt_ctrl->access_mode);
	TRACE(0,"app_timehandler");
	app_bt_device.hf_mute_flag = 0;
}

osTimerDef (APP_TIMER, (void (*)(void const *))app_timehandler);

void app_timer_start_stop(bool timer_en)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);
	
	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}

	if(app_timer == NULL)
	{
    	app_timer = osTimerCreate(osTimer(APP_TIMER),osTimerPeriodic,NULL);
	}

	if(timer_en)
	{
		osTimerStart(app_timer, 10000);
	}
	else
	{
		osTimerStop(app_timer);
	}
}



// tws connected or disconnected
osTimerId app_earheadset_tws_connect_or_disconnec_timer = NULL;

void app_hfp_set_battery_level(uint8_t level);

void app_phone_connected_report_battery_level(void)
{
	uint8_t curr_level;
	uint16_t curr_volt;

	app_battery_get_info(&curr_volt, NULL, NULL);

	curr_level = app_battery_get_curr_level(curr_volt);

	if(app_tws_ibrt_tws_link_connected())
	{
		TRACE(1,"slave_battery_level %d",HLWei_slave_battery_level);
		if(HLWei_slave_battery_level != 10)
		{
			if(curr_level > HLWei_slave_battery_level)
			{
				app_hfp_set_battery_level(HLWei_slave_battery_level);
			}
			else
			{
				app_hfp_set_battery_level(curr_level);
			}
		}
		else
		{
			app_hfp_set_battery_level(curr_level);
		}
	}
	else
	{
		app_hfp_set_battery_level(curr_level);
	}
}

void app_disconnected_tws_battery_deal(bool connect_disconnect)
{
	uint8_t curr_level;
	uint16_t curr_volt;
	
	bool need_report = false;

	app_battery_get_info(&curr_volt, NULL, NULL);

	curr_level = app_battery_get_curr_level(curr_volt);

	app_battery_get_info(NULL, &curr_level, NULL);

	if(connect_disconnect)
	{
		if(app_get_curr_master_role() == 0)
		{
			if(curr_level > HLWei_slave_battery_level)
			{
				if(HLWei_slave_battery_level > 1)
				{
					app_battery_lowbat_flag = 0;
					app_battery_lowbat_play_voice_type = LOWBAT_DEFAULT;
					app_status_battery_report(HLWei_slave_battery_level);
				}
				else
				{
					app_battery_lowbat_flag = 0;
					app_battery_lowbat_play_voice_type = LOWBAT_DEFAULT;
					app_status_battery_report(HLWei_slave_battery_level);
				}
			}
			else
			{
				//do nothing
			}
		}
	}
	else
	{
		if(app_battery_lowbat_flag)
		{
			if(curr_level > 1)
			{
				need_report = true;
			}
		}
		else
		{
			if(curr_level > 1)
			{
				need_report = true;
			}
			else
			{
				need_report = true;
			}
		}

		if(need_report)
		{
			if(app_battery_lowbat_flag)
			{
				app_status_indication_set(app_last_status_indication_get());
			}
			app_battery_lowbat_flag = false;
			app_battery_lowbat_play_voice_type = LOWBAT_DEFAULT;
			app_status_battery_report(curr_level);
		}
	}
}

void app_battery_report_connect_disconnect(bool connected)
{
	uint8_t curr_level;
	uint16_t curr_volt;

	TRACE(2,"%s connected = %d, master_role %d",__func__,connected, app_get_curr_master_role());
	if(connected)
	{
		if(app_get_curr_master_role() == 1)
		{
			app_battery_get_info(&curr_volt, NULL, NULL);
			//slave report battery level to master
			//TRACE(1, "curr_volt = %d", curr_volt);
			curr_level = app_battery_get_curr_level(curr_volt);
			//TRACE(1, "curr_level = %d", curr_level);
			app_ibrt_customif_cmd_battery_deal(curr_level, true);
		}
	}
	else
	{
		app_disconnected_tws_battery_deal(false);
	}
}






static void app_earheadset_tws_connect_or_disconnect_handler(void const *parma)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	TRACE(2,"%s current_role %d status = %d", __func__, p_ibrt_ctrl->current_role, tws_connect_disconnect_status);

	if(p_ibrt_ctrl->current_role == IBRT_MASTER)
	{
		TRACE(2,"twsconnected %d, master phone connected %d",app_tws_ibrt_tws_link_connected(), app_tws_ibrt_mobile_link_connected());
	}
	else if(p_ibrt_ctrl->current_role == IBRT_SLAVE)
	{
		TRACE(2,"twsconnected %d, slave phone connected %d",app_tws_ibrt_tws_link_connected(), app_tws_ibrt_slave_ibrt_link_connected());
	}

	if(tws_connect_disconnect_status == TWS_CONNECTED)
	{
		#ifdef __BES2500_DAKANG_TWS_PAIR__
		if(p_ibrt_ctrl->current_role == IBRT_SLAVE)
		{
			app_enter_phone_pairmode_play_voice_timer_start_stop(false);
		}
		
		if(slave_enter_tws_pairmode_flag)
		{
			app_enter_phone_pairmode_play_voice_timer_start_stop(false);
	        slave_enter_tws_pairmode_flag = false;
	    	slave_enter_tws_pairmode_cnt = 0;
    	}
		//app_slave_exit_tws_pairmode_timer_start_stop(false);
		#endif
		tws_master_search_count = 0;
		tws_connect_disconnect_status = TWS_DEFAULT;
		
		if(delay_play_connected_voice_flag)
		{
			// delay play , fangzhi play only one
			delay_play_connected_voice_flag = false;
			app_voice_report(APP_STATUS_INDICATION_CONNECTED, 0);
			return;
		}

		#ifdef __PUTIN_PUTOUT_SWITCH_ROLE__
		app_ibrt_customif_cmd_current_status(current_box_status, app_earheadset_fixed_type_get(),software_version_shi,software_version_ge);
		#endif
		
		if(phone_linkloss_tws_disconnect_flag)
		{
			if(p_ibrt_ctrl->current_role == IBRT_MASTER)
			{
				app_voice_report(APP_STATUS_INDICATION_TONE_DU, 0);
				if(app_tws_ibrt_mobile_link_connected())
				{
					//app_voice_report(APP_STATUS_INDICATION_CONNECTED, 0);
					app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
					app_stop_10_second_timer(APP_POWEROFF_TIMER_ID);
					app_ibrt_customif_cmd_set_slave_phone_connect(true);
				}
				else
				{
					app_status_indication_set(APP_STATUS_INDICATION_LINKLOSS);
					app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
				}
			}
			else if(p_ibrt_ctrl->current_role == IBRT_SLAVE)
			{
				app_enter_phone_pairmode_play_voice_timer_start_stop(false);
				app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
				app_stop_10_second_timer(APP_PAIR_TIMER_ID);
				app_stop_10_second_timer(APP_POWEROFF_TIMER_ID);
			}

			phone_linkloss_tws_disconnect_flag = false;
		}
		else
		{
			if(p_ibrt_ctrl->current_role == IBRT_MASTER)
			{
				#if 0
				if(a2dp_is_music_ongoing() || app_keyhandle_a2dp_play() || app_keyhandle_hfp_audio_connected())
				{
					TRACE(0,"TWS CONNECT DELAY RING CONNECTED");
					delay_play_connected_voice_flag = true;
					tws_connect_disconnect_status = TWS_CONNECTED;
					osTimerStart(app_earheadset_tws_connect_or_disconnec_timer, 1000);
				}
				else
				#endif
				{
					//app_voice_report(APP_STATUS_INDICATION_CONNECTED, 0);
				}
				if(app_ibrt_ui_get_enter_pairing_mode())
				{
					// nothing
					//app_voice_report(APP_STATUS_INDICATION_CONNECTED, 0);
					//app_stop_10_second_timer(APP_POWEROFF_TIMER_ID);
					//app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
				}
				else
				{
					if(app_tws_ibrt_mobile_link_connected())
					{
						app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
						app_stop_10_second_timer(APP_POWEROFF_TIMER_ID);
						app_ibrt_customif_cmd_set_slave_phone_connect(true);
					}
					else
					{
						//app_voice_report(APP_STATUS_INDICATION_CONNECTED, 0);
						if(phone_connect_disconnect_status == PHONE_DISCONNECTED_LINK)
						{
							app_status_indication_set(APP_STATUS_INDICATION_LINKLOSS);
						}
						else
						{
							//app_stop_10_second_timer(APP_POWEROFF_TIMER_ID);
							//app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
							app_status_indication_set(APP_STATUS_INDICATION_PAGESCAN);
						}
					}
				}
				#ifdef __BES2500_HAILINGWEI_SWITCH_EQ__
				app_tws_switch_eq(false, 0);
				#endif
			}
			else if(p_ibrt_ctrl->current_role == IBRT_SLAVE)
			{
				app_enter_phone_pairmode_play_voice_timer_start_stop(false);
				app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
				app_stop_10_second_timer(APP_PAIR_TIMER_ID);
				app_stop_10_second_timer(APP_POWEROFF_TIMER_ID);
			}

			app_battery_report_connect_disconnect(true);
		}
	}
	else if(tws_connect_disconnect_status == TWS_DISCONNECTED_NORMAL)
	{
		if(!phone_linkloss_tws_disconnect_flag)
		{
			//app_voice_report(APP_STATUS_INDICATION_DISCONNECTED, 0);
			app_battery_report_connect_disconnect(false);
			tws_connect_disconnect_status = TWS_DEFAULT;
		}

		if(p_ibrt_ctrl->nv_role == IBRT_SLAVE)
		{
			if(app_ibrt_ui_get_enter_pairing_mode())
			{
				app_enter_phone_pairmode_play_voice_timer_start_stop(true);
			}
		}
	}
	else if(tws_connect_disconnect_status == TWS_DISCONNECTED_LINK)
	{
		TRACE(1, "p_ibrt_ctrl->nv_role = %d", p_ibrt_ctrl->nv_role);
		if(!phone_linkloss_tws_disconnect_flag)
		{
			if(p_ibrt_ctrl->nv_role == IBRT_SLAVE)
			{
				//app_voice_report(APP_STATUS_INDICATION_DISCONNECTED, 0);
				//app_status_indication_set(APP_STATUS_INDICATION_PAGESCAN);
				app_status_indication_set(APP_STATUS_INDICATION_LINKLOSS);
				app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
			}
			else if(p_ibrt_ctrl->nv_role == IBRT_MASTER)
			{
				//app_voice_report(APP_STATUS_INDICATION_DISCONNECTED, 0);
				if(!app_tws_ibrt_mobile_link_connected())
				{
					if(!app_ibrt_ui_get_enter_pairing_mode())
					{
						//app_status_indication_set(APP_STATUS_INDICATION_PAGESCAN);
						app_status_indication_set(APP_STATUS_INDICATION_LINKLOSS);
	                    app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
					}
				}
			}
		}
		else
		{
			phone_linkloss_tws_disconnect_flag = false;
			if(p_ibrt_ctrl->nv_role == IBRT_MASTER)
			{
				app_voice_report(APP_STATUS_INDICATION_TONE_DU, 0);
			}
		}

		app_battery_report_connect_disconnect(false);
		//tws_connect_disconnect_status = TWS_DEFAULT;
	}
}

osTimerDef (APP_EARHEADSET_TWS_CONNECT_OR_DISCONNECT, app_earheadset_tws_connect_or_disconnect_handler);

void app_earheadset_tws_connect_or_disconnect_timer_start_stop(bool timer_en, bool connect_flag)
{
	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}

	TRACE(1,"%s %d",__func__, timer_en);

	if(app_earheadset_tws_connect_or_disconnec_timer == NULL)
	{
		app_earheadset_tws_connect_or_disconnec_timer = osTimerCreate(osTimer(APP_EARHEADSET_TWS_CONNECT_OR_DISCONNECT),osTimerOnce,NULL);
	}
	
	if(timer_en)
	{
		if(connect_flag)
		{
			osTimerStart(app_earheadset_tws_connect_or_disconnec_timer, 1000);
		}
		else
		{
			osTimerStart(app_earheadset_tws_connect_or_disconnec_timer, 300);
		}
	}
	else
	{
		osTimerStop(app_earheadset_tws_connect_or_disconnec_timer);
	}
}



// phone disconnected or connected
osTimerId app_phone_connected_or_disconnected_timer = NULL;


void app_phone_connected_or_disconnected_timehandler(void const *param)
{
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	TRACE(3,"%s phone connected_status: %d current_role %d", __func__, phone_connect_disconnect_status, p_ibrt_ctrl->current_role);

	if(p_ibrt_ctrl->current_role == IBRT_MASTER)
	{
		TRACE(2,"twsconnected %d, master phone connected %d",app_tws_ibrt_tws_link_connected(), app_tws_ibrt_mobile_link_connected());
	}
	else if(p_ibrt_ctrl->current_role == IBRT_SLAVE)
	{
		app_reboot_poweron_do_not_play_voice_flag = false;
		TRACE(2,"twsconnected %d, slave phone connected %d",app_tws_ibrt_tws_link_connected(), app_tws_ibrt_slave_ibrt_link_connected());
	}
	
	
	if(phone_connect_disconnect_status == PHONE_CONNECTED)
	{
		phone_connect_disconnect_status = PHONE_DEFAULT;
		if(delay_play_connected_voice_flag)
		{
			// delay play , fangzhi play only one
			delay_play_connected_voice_flag = false;
			app_voice_report(APP_STATUS_INDICATION_CONNECTED, 0);
			return;
		}
		
		if(app_tws_ibrt_tws_link_connected())
		{
			if(p_ibrt_ctrl->current_role == IBRT_MASTER)
			{
				/*
				if(a2dp_is_music_ongoing() || app_keyhandle_a2dp_play() || app_keyhandle_hfp_audio_connected())
				{
					TRACE(0,"PHONE CONNECT DELAY RING CONNECTED");
					app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
					//app_voice_report(APP_STATUS_INDICATION_CONNECTED, 0);
					app_ibrt_customif_cmd_set_slave_phone_connect(true);
					//delay_play_connected_voice_flag = true;
					//phone_connect_disconnect_status = PHONE_CONNECTED;
					//osTimerStart(app_phone_connected_or_disconnected_timer, 1500);
					app_enter_phone_pairmode_play_voice_timer_start_stop(false);
				}
				else
				*/
				{
					if(!app_keyhandle_hfp_incomingcall())
					{
						app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
					}
					/*
					if(!app_reboot_poweron_do_not_play_voice_flag)
					{
						app_voice_report(APP_STATUS_INDICATION_CONNECTED, 0);
					}
					else
					{
						app_reboot_poweron_do_not_play_voice_flag = false;
					}
					*/
					app_ibrt_customif_cmd_set_slave_phone_connect(true);
					app_enter_phone_pairmode_play_voice_timer_start_stop(false);
				}
			}
			else if(p_ibrt_ctrl->current_role == IBRT_SLAVE)
			{
				app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
			}
		}
		else
		{
			if(a2dp_is_music_ongoing() || app_keyhandle_a2dp_play() || app_keyhandle_hfp_audio_connected())
			{
				TRACE(0,"PHONE CONNECT DELAY RING CONNECTED");
				app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
				delay_play_connected_voice_flag = true;
				phone_connect_disconnect_status = PHONE_CONNECTED;
				osTimerStart(app_phone_connected_or_disconnected_timer, 1500);
				app_enter_phone_pairmode_play_voice_timer_start_stop(false);
			}
			else
			{
				app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
				/*
				if(!app_reboot_poweron_do_not_play_voice_flag)
				{
					app_voice_report(APP_STATUS_INDICATION_CONNECTED, 0);
				}
				else
				{
					app_reboot_poweron_do_not_play_voice_flag = false;
				}
				*/
				app_enter_phone_pairmode_play_voice_timer_start_stop(false);
			}
		}
		//phone connected, close APP_PAIR_TIMER_ID
		app_stop_10_second_timer(APP_PAIR_TIMER_ID);
		app_stop_10_second_timer(APP_POWEROFF_TIMER_ID);
	}
	else if(phone_connect_disconnect_status == PHONE_DISCONNECTED_NORMAL)
	{
		if(phone_from_connected_to_disconnected)
		{
			if(((p_ibrt_ctrl->current_role == IBRT_MASTER)&&app_tws_ibrt_tws_link_connected())||(!app_tws_ibrt_tws_link_connected()))
			{
				//app_voice_report(APP_STATUS_INDICATION_DISCONNECTED, 0);
				app_voice_report(APP_STATUS_INDICATION_TONE_DU, 0);
				app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
			}

			if((app_tws_ibrt_tws_link_connected())&&(p_ibrt_ctrl->current_role == IBRT_SLAVE))
			{
				app_status_indication_set(APP_STATUS_INDICATION_CONNECTED);
			}

			phone_from_connected_to_disconnected = false;
		}
		phone_connect_disconnect_status = PHONE_DEFAULT;
	}
	else if(phone_connect_disconnect_status == PHONE_DISCONNECTED_LINK)
	{
		if(!phone_linkloss_tws_disconnect_flag)
		{
			TRACE(1,"phone_from_connected_to_disconnected = %d",phone_from_connected_to_disconnected);
			if(phone_from_connected_to_disconnected)
			{
				if(p_ibrt_ctrl->current_role != IBRT_SLAVE)
				{
					uint16_t aud_pram = 0;
					aud_pram |=PROMOT_ID_BIT_MASK_CHNLSEl_ALL;
					aud_pram |= PROMOT_ID_BIT_MASK_MERGING;
					//app_voice_report(APP_STATUS_INDICATION_TONE_DU, 0);
					trigger_media_play(AUDIO_ID_BT_TONE_DU, 0, aud_pram);
					app_status_indication_set(APP_STATUS_INDICATION_LINKLOSS);
					app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
				}

			}
		}

		phone_from_connected_to_disconnected = false;
	}
}

osTimerDef (APP_PHONE_CONNECTED_OR_DISCONNECTED_TIMER, (void (*)(void const *))app_phone_connected_or_disconnected_timehandler);

void app_phone_connected_or_disconnected_timer_start_stop(bool timer_en, bool connected_flag)
{
	TRACE(2, "%s timer_en %d connected_flag%d", __func__, timer_en, connected_flag);
	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}
	
	if(app_phone_connected_or_disconnected_timer == NULL)
	{
		app_phone_connected_or_disconnected_timer = osTimerCreate(osTimer(APP_PHONE_CONNECTED_OR_DISCONNECTED_TIMER),osTimerOnce,NULL);
	}

	if(timer_en)
	{
		if(connected_flag)
		{
			osTimerStart(app_phone_connected_or_disconnected_timer, 500);
		}
		else
		{
			osTimerStart(app_phone_connected_or_disconnected_timer, 750);
		}
	}
	else
	{
		osTimerStop(app_phone_connected_or_disconnected_timer);
	}
}

#endif

#ifdef __BES2500_DAKANG_TWS_PAIR__
/*
osTimerId app_slave_exit_tws_pairmode_timer = NULL;

void app_slave_exit_tws_pairmode_timehandler(void const *param)
{
	tws_pair_phone_pair_switch_flag = true;
	hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT_ENTER_PHONE_PAIR);
	app_reset();
}


osTimerDef (APP_SLAVE_EXIT_TWS_PAIRMODE_TIMER, (void (*)(void const *))app_slave_exit_tws_pairmode_timehandler);

void app_slave_exit_tws_pairmode_timer_start_stop(bool timer_en)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);

	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}

	if(app_slave_exit_tws_pairmode_timer == NULL)
	{
    	//app_slave_exit_tws_pairmode_timer = osTimerCreate(osTimer(APP_SLAVE_EXIT_TWS_PAIRMODE_TIMER),osTimerOnce,NULL);
	}
	
	if(timer_en)
	{
		//osTimerStart(app_slave_exit_tws_pairmode_timer, 10000);
	}
	else
	{
		//osTimerStop(app_slave_exit_tws_pairmode_timer);
	}
}
*/
#endif



#ifdef __PUTIN_PUTOUT_SWITCH_ROLE__
uint8_t current_box_status = 0xff;
uint8_t left_earheadset_status = 0xff;
uint8_t right_earheadset_status = 0xff;
bool delay_detect_touch_flag = false;
void switch_role_rdy_irq_handler(enum HAL_GPIO_PIN_T pin);

//耳机入仓出仓P0.1中断设置
void app_switch_role_irq_set_enable_disenable(bool en)
{
	if(en)
	{
		//hal_sys_timer_delay_us(100);
		if(current_box_status == OUT_BOX_STATUS)
		{
			hal_gpio_enter_ear_enable_irq((enum HAL_GPIO_PIN_T)app_putin_putout_box_cfg.pin, HAL_GPIO_IRQ_POLARITY_LOW_FALLING, switch_role_rdy_irq_handler);
		}
		else if(current_box_status == IN_BOX_STATUS)
		{
			hal_gpio_enter_ear_enable_irq((enum HAL_GPIO_PIN_T)app_putin_putout_box_cfg.pin, HAL_GPIO_IRQ_POLARITY_HIGH_RISING, switch_role_rdy_irq_handler);
		}
	}
	else
	{
		hal_gpio_enter_ear_disable_irq((enum HAL_GPIO_PIN_T)app_putin_putout_box_cfg.pin);
	}
}

void switch_role_rdy_irq_handler(enum HAL_GPIO_PIN_T pin)
{
	TRACE(0,"%s",__func__);
	app_switch_role_irq_set_enable_disenable(false);
	//app_detect_switch_role_timer_start_stop(true);
	//耳机进出仓发出按键事件HAL_KEY_CODE_FN8
	send_enter_ear_key_event(HAL_KEY_CODE_FN8, HAL_KEY_EVENT_DOWN);
}


void app_putin_putout_switch_role_init(void)
{
	uint8_t high_level_cnt = 0;
	uint8_t low_level_cnt = 0;
	
	if (app_putin_putout_box_cfg.pin != HAL_IOMUX_PIN_NUM)
	{
		hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_putin_putout_box_cfg, 1);
		hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_putin_putout_box_cfg.pin, HAL_GPIO_DIR_IN, 1);
	}

	while(1)
	{
		if(hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_putin_putout_box_cfg.pin))
		{
			low_level_cnt = 0;
			high_level_cnt++;
		}
		else
		{
			low_level_cnt++;
			high_level_cnt = 0;
		}

		if((high_level_cnt >= 5)||(low_level_cnt >= 5))
		{
			break;
		}

		osDelay(20);
	}

	if(high_level_cnt >= 5)
	{
		current_box_status = OUT_BOX_STATUS;
		//inbox_mute_flag = false;
		app_spk_open_close_control(true);		
	}
	else if(low_level_cnt >= 5)
	{
		current_box_status = IN_BOX_STATUS;
		//inbox_mute_flag = true;
		app_spk_open_close_control(false);		
	}

	if(app_earheadset_fixed_type_get() == LEFT_EARHEADSET)
	{
		left_earheadset_status = current_box_status;
	}
	else if(app_earheadset_fixed_type_get() == RIGHT_EARHEADSET)
	{
		right_earheadset_status = current_box_status;
	}

	TRACE(1,"%s current_box_status %d",__func__, current_box_status);
	app_switch_role_irq_set_enable_disenable(true);
}


osTimerId app_detect_switch_role_timer = NULL;
uint8_t detect_switch_role_cnt = 0;

//耳机进仓出仓角色切换,发出按键事件APP_KEY_CODE_FN9
void app_detect_switch_role_timehandler(void const *param)
{
	//uint8_t level = 0;
	//level = hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_putin_putout_box_cfg.pin);
	//TRACE(1," level %d", level);
	//return;
	//TRACE(1,"%s  box_status %d", __func__, current_box_status);
	
	if(current_box_status == OUT_BOX_STATUS)
	{
		if(!hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_putin_putout_box_cfg.pin))
		{
			if(++detect_switch_role_cnt >= 3)
			{
				app_detect_switch_role_timer_start_stop(false);
				detect_switch_role_cnt = 0;
				//inbox_mute_flag = true;
				current_box_status = IN_BOX_STATUS;
				app_switch_role_irq_set_enable_disenable(true);
				//send key out
				send_enter_ear_key_event(HAL_KEY_CODE_FN9, HAL_KEY_EVENT_DOWN);	
			}
		}
		else
		{
			detect_switch_role_cnt = 0;
			app_detect_switch_role_timer_start_stop(false);
			app_switch_role_irq_set_enable_disenable(true);
		}
	}
	else if(current_box_status == IN_BOX_STATUS)
	{
		if(hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)app_putin_putout_box_cfg.pin))
		{
			if(++detect_switch_role_cnt >= 3)
			{
				app_detect_switch_role_timer_start_stop(false);
				detect_switch_role_cnt = 0;
				current_box_status = OUT_BOX_STATUS;
				//inbox_mute_flag = false;
				app_switch_role_irq_set_enable_disenable(true);
				//send key out
				send_enter_ear_key_event(HAL_KEY_CODE_FN9, HAL_KEY_EVENT_UP);
			}
		}
		else
		{
			detect_switch_role_cnt = 0;
			app_detect_switch_role_timer_start_stop(false);
			app_switch_role_irq_set_enable_disenable(true);
		}
	}
}


osTimerDef (APP_DETECT_SWITCH_ROLE_TIMER, (void (*)(void const *))app_detect_switch_role_timehandler);

void app_detect_switch_role_timer_start_stop(bool timer_en)
{
	//if((app_battery_charging_flag)&&(timer_en))
	//{
	//	return;
	//}

	TRACE(1, "%s timer_en %d", __func__, timer_en);


	if(app_detect_switch_role_timer == NULL)
	{
		//TRACE(0,"\n\n---->app_detect_switch_role_timer\n\n");
    	app_detect_switch_role_timer = osTimerCreate(osTimer(APP_DETECT_SWITCH_ROLE_TIMER),osTimerPeriodic,NULL);
	}

	if(timer_en)
	{
		//TRACE(0,"\n\n---->timer_en\n\n");
		detect_switch_role_cnt = 0;
		osTimerStart(app_detect_switch_role_timer, 50);
	}
	else
	{
		detect_switch_role_cnt = 0;
		osTimerStop(app_detect_switch_role_timer);
	}
}

void app_spk_open_close_control(bool open_close)
{
	TRACE(1,"%s %d", __func__, open_close);
	if(open_close)
	{
		// open spk
		bt_sco_spk_forcemute(false);
		bt_a2dp_spk_forcemute(false);
		spk_mute_flag = false;
	}
	else
	{
		// close spk
		bt_sco_spk_forcemute(true);
		bt_a2dp_spk_forcemute(true);
		spk_mute_flag = true;
	}
}

/**************************************************************************************************
鐢ㄦ潵鏄剧ず褰撳墠杞欢鐗堟湰锛岀敤LED鏄剧ず锛?姣斿 V1.6, 鍒欒摑鐏棯涓�娆★紝绾㈢伅闂?娆?***************************************************************************************************/
#define SOFTWARE_SHI_TOTAL   3
#define SOFTWARE_GE_TOTAL    1


uint8_t software_version_shi = SOFTWARE_SHI_TOTAL;
uint8_t software_version_ge = SOFTWARE_GE_TOTAL;



uint8_t app_bt_software_version_shi = 0;
uint8_t app_bt_software_version_ge = 0;
uint8_t app_bt_software_version_shi_cnt = 0;
uint8_t app_bt_software_version_ge_cnt = 0;
bool app_bt_software_version_display_flag = false;

osTimerId app_bt_software_version_display_timerid = NULL;


void app_bt_software_version_display_handle(void const *param)
{
	TRACE(1,"app_bt_software_version_display_flag = %d",app_bt_software_version_display_flag);
	if(app_bt_software_version_display_flag)
	{
		if(app_bt_software_version_shi > 0)
		{
			if(app_bt_software_version_shi_cnt%2 == 0)
			{
				// mie
				hal_gpio_pin_clr((enum HAL_GPIO_PIN_T)cfg_hw_pinmux_pwl[0].pin);
			}
			else
			{
				// liang
				hal_gpio_pin_set((enum HAL_GPIO_PIN_T)cfg_hw_pinmux_pwl[0].pin);
			}
			
			if(++app_bt_software_version_shi_cnt >= (SOFTWARE_SHI_TOTAL*2+1))
			{
				hal_gpio_pin_clr((enum HAL_GPIO_PIN_T)cfg_hw_pinmux_pwl[0].pin);
				app_bt_software_version_shi = 0;
				app_bt_software_version_shi_cnt = 0;
			}
		}
		else if(app_bt_software_version_ge > 0)
		{
			if(app_bt_software_version_ge_cnt%2 == 0)
			{
				// mie
				hal_gpio_pin_clr((enum HAL_GPIO_PIN_T)cfg_hw_pinmux_pwl[1].pin);
			}
			else
			{
				// liang
				hal_gpio_pin_set((enum HAL_GPIO_PIN_T)cfg_hw_pinmux_pwl[1].pin);
			}
			
			if(++app_bt_software_version_ge_cnt >= (SOFTWARE_GE_TOTAL*2+1))
			{
				hal_gpio_pin_clr((enum HAL_GPIO_PIN_T)cfg_hw_pinmux_pwl[1].pin);
				app_bt_software_version_ge = 0;
				app_bt_software_version_ge_cnt = 0;
			}
		}

		if((app_bt_software_version_ge == 0)&&(app_bt_software_version_shi == 0))
		{
			app_bt_software_version_display_flag = false;
			app_status_indication_set(APP_STATUS_INDICATION_INITIAL);
			#if 1
			app_status_indication_set(app_last_status_indication_get());
			#else
			if(app_battery_get_lowbat_status())
			{
				app_battery_set_lowbat_status(false);
				app_status_indication_set(APP_STATUS_INDICATION_CHARGENEED);
				app_battery_set_lowbat_status(true);
			}
			else
			{
				app_status_indication_set(app_last_status_indication_get());
			}
			#endif
			app_bt_software_version_timer_start_stop(false);
		}
	}
}


osTimerDef (APP_SOFTWARE_VERSION_DISPLAY, app_bt_software_version_display_handle);

void app_bt_software_version_timer_start_stop(bool timer_en)
{
	if((app_battery_charging_flag)&&(timer_en))
	{
		return;
	}

	TRACE(1, "%s timer_en %d", __func__, timer_en);


	if(app_bt_software_version_display_timerid == NULL)
	{
		//TRACE(0,"\n\n---->app_detect_switch_role_timer\n\n");
    	app_bt_software_version_display_timerid = osTimerCreate(osTimer(APP_SOFTWARE_VERSION_DISPLAY),osTimerPeriodic,NULL);
	}

	if(timer_en)
	{
		osTimerStart(app_bt_software_version_display_timerid, 500);
	}
	else
	{
		osTimerStop(app_bt_software_version_display_timerid);
	}
}


void app_bt_key_software_version(void)
{
	if(!app_tws_ibrt_tws_link_connected() && !app_tws_ibrt_mobile_link_connected())
	{
		if(app_ibrt_ui_get_enter_pairing_mode())
		{
			if(!app_bt_software_version_display_flag)
			{
				///TRACE("app_bt_key_software_version_deal");
				app_status_indication_set(APP_STATUS_INDICATION_INITIAL);
				app_bt_software_version_display_flag = true;
				app_bt_software_version_shi = SOFTWARE_SHI_TOTAL;
				app_bt_software_version_ge = SOFTWARE_GE_TOTAL;
				app_bt_software_version_shi_cnt = 0;
				app_bt_software_version_ge_cnt = 0;
				app_bt_software_version_timer_start_stop(true);
			}
		}
	}
}


osTimerId app_nv_update_timer = NULL;
bool app_auto_play_nv_flag = false;
void app_nv_update_timehandler(void const *param)
{
	TRACE(2, "%s", __func__);

	app_store_shut_down_proc(true);
	if(app_auto_play_nv_flag)
	{
		app_auto_play_nv_flag = false;
		app_store_auto_play_proc(false);
	}
}

osTimerDef (APP_NV_UPDATE_TIMER, (void (*)(void const *))app_nv_update_timehandler);

void app_nv_update_timer_start_stop(bool timer_en)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);


	if(app_nv_update_timer == NULL)
	{
    	app_nv_update_timer = osTimerCreate(osTimer(APP_NV_UPDATE_TIMER),osTimerOnce,NULL);
	}

	if(timer_en)
	{
		osTimerStart(app_nv_update_timer, 2000);
	}
	else
	{
		osTimerStop(app_nv_update_timer);
	}
}


osTimerId app_play_voice_delay_open_siri_timer = NULL;
bool app_siri_open_close_flag = false;
extern int app_hfp_siri_open_close_proc(bool en);

void app_play_voice_delay_open_siri_timehandler(void const *param)
{
	TRACE(1, "%s ", __func__);
	//ntc_capture_start();
	app_hfp_siri_open_close_proc(app_siri_open_close_flag);
	app_siri_open_close_flag = false;
}


osTimerDef (APP_PLAY_VOICE_DELAY_OPEN_SIRI_TIMER, (void (*)(void const *))app_play_voice_delay_open_siri_timehandler);

void app_play_voice_delay_open_siri_timer_start_stop(bool timer_en)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);


	if(app_play_voice_delay_open_siri_timer == NULL)
	{
		TRACE(0,"\n\n---->app_detect_switch_role_timer\n\n");
    	app_play_voice_delay_open_siri_timer = osTimerCreate(osTimer(APP_PLAY_VOICE_DELAY_OPEN_SIRI_TIMER),osTimerOnce,NULL);
	}

	if(timer_en)
	{
		TRACE(0,"\n\n---->timer_en\n\n");
		osTimerStart(app_play_voice_delay_open_siri_timer, 500);
	}
	else
	{
		osTimerStop(app_play_voice_delay_open_siri_timer);
	}
}


/*
osTimerId app_detect_ntc_timer = NULL;

void app_detect_ntc_timehandler(void const *param)
{
	TRACE(1, "%s ", __func__);
	ntc_capture_start();
}


osTimerDef (APP_DETECT_NTC_TIMER, (void (*)(void const *))app_detect_ntc_timehandler);

void app_detect_ntc_timer_start_stop(bool timer_en)
{
	TRACE(1, "%s timer_en %d", __func__, timer_en);


	if(app_detect_ntc_timer == NULL)
	{
		TRACE(0,"\n\n---->app_detect_switch_role_timer\n\n");
    	app_detect_ntc_timer = osTimerCreate(osTimer(APP_DETECT_NTC_TIMER),osTimerOnce,NULL);
	}

	if(timer_en)
	{
		TRACE(0,"\n\n---->timer_en\n\n");
		osTimerStart(app_detect_ntc_timer, 500);
	}
	else
	{
		osTimerStop(app_detect_ntc_timer);
	}
}
*/

/*
void app_putin_putout_box_process(uint8_t putin_putout)
{
	if(putin_putout == OUT_BOX_STATUS)
	{
		app_ibrt_open_close_box_process(0);
		app_putout_box_handler();
	}
	else if(putin_putout == IN_BOX_STATUS)
	{
		app_ibrt_open_close_box_process(1);
		app_putin_box_handler();
	}
}
*/
#endif
#ifndef APP_TEST_MODE
static uint8_t app_status_indication_init(void)
{
    struct APP_PWL_CFG_T cfg;
    memset(&cfg, 0, sizeof(struct APP_PWL_CFG_T));
    app_pwl_open();
    app_pwl_setup(APP_PWL_ID_0, &cfg);
    app_pwl_setup(APP_PWL_ID_1, &cfg);
    return 0;
}
#endif

#if defined(__BTIF_EARPHONE__) && defined(__BTIF_AUTOPOWEROFF__)

void PairingTransferToConnectable(void);

typedef void (*APP_10_SECOND_TIMER_CB_T)(void);

void app_pair_timerout(void);
void app_poweroff_timerout(void);
void CloseEarphone(void);

typedef struct
{
    uint8_t timer_id;
    uint8_t timer_en;
    uint8_t timer_count;
    uint8_t timer_period;
    APP_10_SECOND_TIMER_CB_T cb;
}APP_10_SECOND_TIMER_STRUCT;

#define INIT_APP_TIMER(_id, _en, _count, _period, _cb) \
    { \
        .timer_id = _id, \
        .timer_en = _en, \
        .timer_count = _count, \
        .timer_period = _period, \
        .cb = _cb, \
    }

APP_10_SECOND_TIMER_STRUCT app_10_second_array[] =
{
    //INIT_APP_TIMER(APP_PAIR_TIMER_ID, 0, 0, 2, PairingTransferToConnectable),
    //INIT_APP_TIMER(APP_POWEROFF_TIMER_ID, 0, 0, 2, CloseEarphone),
    INIT_APP_TIMER(APP_PAIR_TIMER_ID, 0, 0, 31, PairingTransferToConnectable),
   	INIT_APP_TIMER(APP_POWEROFF_TIMER_ID, 0, 0, 31, CloseEarphone),
#ifdef GFPS_ENABLED
    INIT_APP_TIMER(APP_FASTPAIR_LASTING_TIMER_ID, 0, 0, APP_FAST_PAIRING_TIMEOUT_IN_SECOND/10,
        app_fast_pairing_timeout_timehandler),
#endif
};

void app_stop_10_second_timer(uint8_t timer_id)
{
    APP_10_SECOND_TIMER_STRUCT *timer = &app_10_second_array[timer_id];
	TRACE(1,"%s %d", __func__, timer_id);
    timer->timer_en = 0;
    timer->timer_count = 0;
}

void app_start_10_second_timer(uint8_t timer_id)
{
    APP_10_SECOND_TIMER_STRUCT *timer = &app_10_second_array[timer_id];
	TRACE(1,"%s %d", __func__, timer_id);
    timer->timer_en = 1;
    timer->timer_count = 0;
}

void app_set_10_second_timer(uint8_t timer_id, uint8_t enable, uint8_t period)
{
    APP_10_SECOND_TIMER_STRUCT *timer = &app_10_second_array[timer_id];

    timer->timer_en = enable;
    timer->timer_count = period;
}

void app_10_second_timer_check(void)
{
    APP_10_SECOND_TIMER_STRUCT *timer = app_10_second_array;
    unsigned int i;

    for(i = 0; i < ARRAY_SIZE(app_10_second_array); i++) {
    	TRACE(2,"%d %d", i, timer->timer_en);
        if (timer->timer_en) {
            timer->timer_count++;
            TRACE(2, "i %d timer_count %d",timer->timer_count, i);
            if (timer->timer_count >= timer->timer_period) {
                timer->timer_en = 0;
                if (timer->cb)
                    timer->cb();
            }
        }
        timer++;
    }
}

void CloseEarphone(void)
{
    int activeCons;
    activeCons = btif_me_get_activeCons();

#ifdef ANC_APP
    if(app_anc_work_status()) {
        app_set_10_second_timer(APP_POWEROFF_TIMER_ID, 1, 30);
        return;
    }
#endif /* ANC_APP */
	#ifdef __BES2500_HAILINGWEI_CHC__
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	
	if((activeCons == 1)&&(app_tws_ibrt_tws_link_connected()))
	{
		//sync shutdown
		TRACE(1,"!!!CloseEarphone tws connected p_ibrt_ctrl->current_role %d\n", p_ibrt_ctrl->current_role);
		if(p_ibrt_ctrl->current_role == IBRT_MASTER)
		{
			app_hailingwei_poweroff_prec(AUTO_POWEROFF, false);
		}
	}
	else if(activeCons == 0)
	{
		TRACE(0,"!!!CloseEarphone\n");
        app_shutdown();
	}
	#else
    if(activeCons == 0) {
        TRACE(0,"!!!CloseEarphone\n");
        app_shutdown();
    }
    #endif
}
#endif /* #if defined(__BTIF_EARPHONE__) && defined(__BTIF_AUTOPOWEROFF__) */

int signal_send_to_main_thread(uint32_t signals);
uint8_t stack_ready_flag = 0;
void app_notify_stack_ready(uint8_t ready_flag)
{
    TRACE(2,"app_notify_stack_ready %d %d", stack_ready_flag, ready_flag);

    stack_ready_flag |= ready_flag;

#ifdef __IAG_BLE_INCLUDE__
    if(stack_ready_flag == (STACK_READY_BT|STACK_READY_BLE))
#endif
    {
        signal_send_to_main_thread(0x3);
    }
}

bool app_is_stack_ready(void)
{
    bool ret = false;

    if (stack_ready_flag == (STACK_READY_BT
#ifdef __IAG_BLE_INCLUDE__
                             | STACK_READY_BLE
#endif
                             ))
    {
        ret = true;
    }

    return ret;
}

static void app_stack_ready_cb(void)
{
    TRACE(0,"stack init done");
#ifdef BLE_ENABLE
    app_ble_stub_user_init();
    app_ble_start_connectable_adv(BLE_ADVERTISING_INTERVAL);
#endif
}

//#if (HF_CUSTOM_FEATURE_SUPPORT & HF_CUSTOM_FEATURE_BATTERY_REPORT) || (HF_SDK_FEATURES & HF_FEATURE_HF_INDICATORS)
#if defined(SUPPORT_BATTERY_REPORT) || defined(SUPPORT_HF_INDICATORS)
extern void app_hfp_set_battery_level(uint8_t level);
#endif

#ifdef __AMA_VOICE__
extern "C" void ama_query_alexa_state(uint16_t feature);
#endif

int app_status_battery_report(uint8_t level)
{
#if defined(__BTIF_EARPHONE__)
    if (app_is_stack_ready())
    {
        app_bt_state_checker();
    }
    app_10_second_timer_check();
#endif
    if (level <= APP_BATTERY_LEVEL_LOWPOWERTHRESHOLD)
    {
        //add something
    }

    if (app_is_stack_ready())
    {
// #if (HF_CUSTOM_FEATURE_SUPPORT & HF_CUSTOM_FEATURE_BATTERY_REPORT) || (HF_SDK_FEATURES & HF_FEATURE_HF_INDICATORS)
#if defined(SUPPORT_BATTERY_REPORT) || defined(SUPPORT_HF_INDICATORS)
#if defined(IBRT)
        if (app_tws_ibrt_mobile_link_connected())
        {
            app_hfp_set_battery_level(level);
        }
#else
        app_hfp_set_battery_level(level);
#endif
#else
        TRACE(1,"[%s] Can not enable SUPPORT_BATTERY_REPORT", __func__);
#endif
        osapi_notify_evm();
    }
#ifdef __AMA_VOICE__
    if (AI_SPEC_AMA == ai_manager_get_current_spec())
        ama_query_alexa_state(0x203);
#endif    
    return 0;
}

#ifdef MEDIA_PLAYER_SUPPORT

void app_status_set_num(const char* p)
{
    media_Set_IncomingNumber(p);
}

int app_voice_report_handler(APP_STATUS_INDICATION_T status, uint8_t device_id, uint8_t isMerging)
{
#if (defined(BTUSB_AUDIO_MODE) || defined(BT_USB_AUDIO_DUAL_MODE))
    if(app_usbaudio_mode_on()) return 0;
#endif
    uint16_t aud_pram = 0;    
#ifdef IBRT
    aud_pram |= PROMOT_ID_BIT_MASK_CHNLSEl_ALL;
#endif
    //TRACE(3,"%s %d%s",__func__, status, status2str((uint16_t)status));
	TRACE(2,"%s %d",__func__, status);
    
    #ifdef __BES2500_HAILINGWEI_CHC__
    if(spk_mute_flag)
    {
		//TRACE(0,"mute spk");
		//return 0;
    }	
    if(app_battery_charging_flag)
	{
		TRACE(0,"charge do not play voice");
		return 0;
	}
	#endif
    AUD_ID_ENUM id = MAX_RECORD_NUM;
    if (app_poweroff_flag == 1){
        switch (status) {
            case APP_STATUS_INDICATION_POWEROFF:
                id = AUD_ID_POWER_OFF;
                break;
            case APP_STATUS_INDICATION_TONE_DU:
				id = AUDIO_ID_BT_TONE_DU;
				break;
				
            default:
                return 0;
                break;
        }
    }else{
        switch (status) {
            case APP_STATUS_INDICATION_POWERON:
                id = AUD_ID_POWER_ON;
                break;
            case APP_STATUS_INDICATION_POWEROFF:
                id = AUD_ID_POWER_OFF;
                break;
            case APP_STATUS_INDICATION_CONNECTED:
                id = AUD_ID_BT_CONNECTED;
                break;
            case APP_STATUS_INDICATION_DISCONNECTED:
                id = AUD_ID_BT_DIS_CONNECT;
                break;
            case APP_STATUS_INDICATION_CALLNUMBER:
                id = AUD_ID_BT_CALL_INCOMING_NUMBER;
                break;
            case APP_STATUS_INDICATION_CHARGENEED:
                id = AUD_ID_BT_CHARGE_PLEASE;
                break;
            case APP_STATUS_INDICATION_FULLCHARGE:
                //id = AUD_ID_BT_CHARGE_FINISH;
                break;
            case APP_STATUS_INDICATION_PAIRSUCCEED:
                //id = AUD_ID_BT_PAIRING_SUC;
                break;
            case APP_STATUS_INDICATION_PAIRFAIL:
                //id = AUD_ID_BT_PAIRING_FAIL;
                break;

            case APP_STATUS_INDICATION_HANGUPCALL:
                id = AUD_ID_BT_CALL_HUNG_UP;
                break;

            case APP_STATUS_INDICATION_REFUSECALL:
                id = AUD_ID_BT_CALL_REFUSE;
                //isMerging = false;
                break;

            case APP_STATUS_INDICATION_ANSWERCALL:
                //id = AUD_ID_BT_CALL_ANSWER;
                break;

            case APP_STATUS_INDICATION_CLEARSUCCEED:
                //id = AUD_ID_BT_CLEAR_SUCCESS;
                break;

            case APP_STATUS_INDICATION_CLEARFAIL:
                //id = AUD_ID_BT_CLEAR_FAIL;
                break;
            case APP_STATUS_INDICATION_INCOMINGCALL:
                id = AUD_ID_BT_CALL_INCOMING_CALL;
                break;
            case APP_STATUS_INDICATION_BOTHSCAN:
                id = AUD_ID_BT_PAIR_ENABLE;
                break;
            case APP_STATUS_INDICATION_WARNING:
                //id = AUD_ID_BT_WARNING;
                break;
            case APP_STATUS_INDICATION_ALEXA_START:
                //id = AUDIO_ID_BT_ALEXA_START;
                break;
            case APP_STATUS_INDICATION_ALEXA_STOP:
                //id = AUDIO_ID_BT_ALEXA_STOP;
                break;
            case APP_STATUS_INDICATION_GSOUND_MIC_OPEN:
                //id = AUDIO_ID_BT_GSOUND_MIC_OPEN;
                break;
            case APP_STATUS_INDICATION_GSOUND_MIC_CLOSE:
                //id = AUDIO_ID_BT_GSOUND_MIC_CLOSE;
                break;
            case APP_STATUS_INDICATION_GSOUND_NC:
                //id = AUDIO_ID_BT_GSOUND_NC;
                break;
#ifdef __BT_WARNING_TONE_MERGE_INTO_STREAM_SBC__
            case APP_STATUS_RING_WARNING:
                id = AUD_ID_RING_WARNING;
                break;
#endif
			#ifdef __BES2500_HAILINGWEI_LOWLATENCY__
			case APP_STATUS_INDICATION_LOW_LATENCY:
				id = AUDIO_ID_BT_LOW_LATENCY;
				tws_a2dp_low_latency_open_close_flag = true;
				break;
			case APP_STATUS_INDICATION_NORMAL_LATENCY:
				id = AUDIO_ID_BT_NORMAL_LATENCY;
				tws_a2dp_low_latency_open_close_flag = false;
				break;
			#endif
            case APP_STATUS_INDICATION_MUTE:
                id = AUDIO_ID_BT_MUTE;
                break;
#ifdef __INTERACTION__
            case APP_STATUS_INDICATION_FINDME:
                id = AUD_ID_BT_FINDME;
                break;
#endif
            case APP_STATUS_INDICATION_FIND_MY_BUDS:
                //id = AUDIO_ID_FIND_MY_BUDS;
                //aud_pram &= (~PROMOT_ID_BIT_MASK_CHNLSEl_ALL);
                //isMerging = false;
                break;
            case APP_STATUS_INDICATION_TILE_FIND:
                //id = AUDIO_ID_FIND_TILE;
                break;
#ifdef __TENCENT_VOICE__
            case APP_STATUS_INDICATION_TENCENT_START:
                id = AUDIO_ID_BT_TENCENT_START;
                break;
            case APP_STATUS_INDICATION_TENCENT_STOP:
                id = AUDIO_ID_BT_TENCENT_STOP;
                break;
#endif
			#ifdef __BES2500_HAILINGWEI_CHC__
			case APP_STATUS_INDICATION_TONE_DU:
				id = AUDIO_ID_BT_TONE_DU;
				break;

			case APP_STATUS_INDICATION_TONE_DI:
				id = AUDIO_ID_BT_TONE_DI;
				break;
			#endif
            default:
                break;
        }
    }
    
    if (isMerging){
        aud_pram |= PROMOT_ID_BIT_MASK_MERGING;
    }
#ifdef BT_USB_AUDIO_DUAL_MODE
    if(!btusb_is_usb_mode())
    {
#if defined(IBRT)
        app_ibrt_if_voice_report_handler(id, aud_pram);
#else
        trigger_media_play(id, device_id, aud_pram);
#endif
    }

#else
#if defined(IBRT)
    app_ibrt_if_voice_report_handler(id, aud_pram);
#else
    trigger_media_play(id, device_id, aud_pram);
#endif
#endif

    return 0;
}

extern "C" int app_voice_report(APP_STATUS_INDICATION_T status, uint8_t device_id)
{
    return app_voice_report_handler(status, device_id, true);
}

extern "C" int app_voice_report_generic(APP_STATUS_INDICATION_T status, uint8_t device_id, uint8_t isMerging)
{
    return app_voice_report_handler(status, device_id, isMerging);
}

extern "C" int app_voice_stop(APP_STATUS_INDICATION_T status, uint8_t device_id)
{
    AUD_ID_ENUM id = MAX_RECORD_NUM;

    TRACE(2,"%s %d", __func__, status);

    if (status == APP_STATUS_INDICATION_FIND_MY_BUDS)
        id = AUDIO_ID_FIND_MY_BUDS;

    if (id != MAX_RECORD_NUM)
        trigger_media_stop(id, device_id);

    return 0;
}

#endif

static void app_poweron_normal(APP_KEY_STATUS *status, void *param)
{
    TRACE(3,"%s %d,%d",__func__, status->code, status->event);
    g_pwron_case = APP_POWERON_CASE_NORMAL;

    signal_send_to_main_thread(0x2);
}

#if !defined(BLE_ONLY_ENABLED)
static void app_poweron_scan(APP_KEY_STATUS *status, void *param)
{
    TRACE(3,"%s %d,%d",__func__, status->code, status->event);
    g_pwron_case = APP_POWERON_CASE_BOTHSCAN;

    signal_send_to_main_thread(0x2);
}
#endif

#ifdef __ENGINEER_MODE_SUPPORT__
#if !defined(BLE_ONLY_ENABLED)
static void app_poweron_factorymode(APP_KEY_STATUS *status, void *param)
{
	#ifdef __BES2500_HAILINGWEI_CHC__
	TRACE(0,"%s",__func__);
	#else
    TRACE(3,"%s %d,%d",__func__, status->code, status->event);
    #endif
    hal_sw_bootmode_clear(HAL_SW_BOOTMODE_REBOOT);
    app_factorymode_enter();
}

#ifdef __BES2500_HAILINGWEI_CHC__
void app_earheadset_enter_dut_test_rf_param(void)
{	
	if(!app_tws_ibrt_tws_link_connected())
	{
		if(app_ibrt_ui_get_enter_pairing_mode())
		{
			app_poweron_factorymode(NULL, NULL);
		}
	}
}
#endif

#endif
#endif


static bool g_pwron_finished = false;
static void app_poweron_finished(APP_KEY_STATUS *status, void *param)
{
    TRACE(3,"%s %d,%d",__func__, status->code, status->event);
    g_pwron_finished = true;
    signal_send_to_main_thread(0x2);
}

void app_poweron_wait_finished(void)
{
    if (!g_pwron_finished){
        osSignalWait(0x2, osWaitForever);
    }
}

#if  defined(__POWERKEY_CTRL_ONOFF_ONLY__)
void app_bt_key_shutdown(APP_KEY_STATUS *status, void *param);
const  APP_KEY_HANDLE  pwron_key_handle_cfg[] = {
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_UP},           "power on: shutdown"     , app_bt_key_shutdown, NULL},
};
#elif defined(__ENGINEER_MODE_SUPPORT__)
const  APP_KEY_HANDLE  pwron_key_handle_cfg[] = {
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_INITUP},           "power on: normal"     , app_poweron_normal, NULL},
#if !defined(BLE_ONLY_ENABLED)
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_INITLONGPRESS},    "power on: both scan"  , app_poweron_scan  , NULL},
    #ifndef __BES2500_HAILINGWEI_CHC__
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_INITLONGLONGPRESS},"power on: factory mode", app_poweron_factorymode  , NULL},
    #endif
#endif
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_INITFINISHED},     "power on: finished"   , app_poweron_finished  , NULL},
};
#else
const  APP_KEY_HANDLE  pwron_key_handle_cfg[] = {
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_INITUP},           "power on: normal"     , app_poweron_normal, NULL},
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_INITLONGPRESS},    "power on: both scan"  , app_poweron_scan  , NULL},
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_INITFINISHED},     "power on: finished"   , app_poweron_finished  , NULL},
};
#endif

#ifndef APP_TEST_MODE

static void app_poweron_key_init(void)
{
    uint8_t i = 0;
    TRACE(1,"%s",__func__);

    for (i=0; i<(sizeof(pwron_key_handle_cfg)/sizeof(APP_KEY_HANDLE)); i++){
        app_key_handle_registration(&pwron_key_handle_cfg[i]);
    }
}

static uint8_t app_poweron_wait_case(void)
{
    uint32_t stime = 0, etime = 0;

#ifdef __POWERKEY_CTRL_ONOFF_ONLY__
    g_pwron_case = APP_POWERON_CASE_NORMAL;
#else
	g_pwron_case = APP_POWERON_CASE_NORMAL;
    TRACE(1,"poweron_wait_case enter:%d", g_pwron_case);
    if (g_pwron_case == APP_POWERON_CASE_INVALID){
        stime = hal_sys_timer_get();
        osSignalWait(0x2, POWERON_PRESSMAXTIME_THRESHOLD_MS);
        etime = hal_sys_timer_get();
    }
    TRACE(2,"powon raw case:%d time:%d", g_pwron_case, TICKS_TO_MS(etime - stime));
#endif
    return g_pwron_case;

}
#endif

static void app_wait_stack_ready(void)
{
    uint32_t stime, etime;
    stime = hal_sys_timer_get();
    osSignalWait(0x3, 1000);
    etime = hal_sys_timer_get();
    TRACE(1,"app_wait_stack_ready: wait:%d ms", TICKS_TO_MS(etime - stime));

    app_stack_ready_cb();
}

extern "C" int system_shutdown(void);
int app_shutdown(void)
{
    system_shutdown();
    return 0;
}

int system_reset(void);
int app_reset(void)
{
    system_reset();
    return 0;
}

static void app_postponed_reset_timer_handler(void const *param);
osTimerDef(APP_POSTPONED_RESET_TIMER, app_postponed_reset_timer_handler);
static osTimerId app_postponed_reset_timer = NULL;
#define APP_RESET_PONTPONED_TIME_IN_MS  2000
static void app_postponed_reset_timer_handler(void const *param)
{
//    hal_cmu_sys_reboot();
	app_reset();
}

void app_start_postponed_reset(void)
{
    if (NULL == app_postponed_reset_timer)
    {
        app_postponed_reset_timer = osTimerCreate(osTimer(APP_POSTPONED_RESET_TIMER), osTimerOnce, NULL);
    }

    hal_sw_bootmode_set(HAL_SW_BOOTMODE_ENTER_HIDE_BOOT);

    osTimerStart(app_postponed_reset_timer, APP_RESET_PONTPONED_TIME_IN_MS);
}

void app_bt_key_shutdown(APP_KEY_STATUS *status, void *param)
{
    TRACE(3,"%s %d,%d",__func__, status->code, status->event);
#ifdef __POWERKEY_CTRL_ONOFF_ONLY__
    hal_sw_bootmode_clear(HAL_SW_BOOTMODE_REBOOT);
    app_reset();
#else
    app_shutdown();
#endif
}

void app_bt_key_enter_testmode(APP_KEY_STATUS *status, void *param)
{
    //TRACE(1,"%s\n",__FUNCTION__);

    if(app_status_indication_get() == APP_STATUS_INDICATION_BOTHSCAN){
#ifdef __FACTORY_MODE_SUPPORT__
        app_factorymode_bt_signalingtest(status, param);
#endif
    }
}

void app_bt_key_enter_nosignal_mode(APP_KEY_STATUS *status, void *param)
{
    //TRACE(1,"%s\n",__FUNCTION__);
    if(app_status_indication_get() == APP_STATUS_INDICATION_BOTHSCAN){
#ifdef __FACTORY_MODE_SUPPORT__
        app_factorymode_bt_nosignalingtest(status, param);
#endif
    }
}

void app_bt_singleclick(APP_KEY_STATUS *status, void *param)
{
    //TRACE(3,"%s %d,%d",__func__, status->code, status->event);
}

void app_bt_doubleclick(APP_KEY_STATUS *status, void *param)
{
    //TRACE(3,"%s %d,%d",__func__, status->code, status->event);
}

void app_power_off(APP_KEY_STATUS *status, void *param)
{
    //TRACE(0,"app_power_off\n");
}

/* enter single download mode api */
void app_single_dld_mode_enter(APP_KEY_STATUS *status, void *param)
{
    //TRACE(0, "app_single_dld_mode_enter");
    hal_sw_bootmode_set(HAL_SW_BOOTMODE_SINGLE_LINE_DOWNLOAD);
    pmu_reboot();
}

extern "C" void OS_NotifyEvm(void);

#define PRESS_KEY_TO_ENTER_OTA_INTERVEL    (15000)          // press key 15s enter to ota
#define PRESS_KEY_TO_ENTER_OTA_REPEAT_CNT    ((PRESS_KEY_TO_ENTER_OTA_INTERVEL - 2000) / 500)
void app_otaMode_enter(APP_KEY_STATUS *status, void *param)
{
    TRACE(1,"%s",__func__);

    hal_norflash_disable_protection(HAL_FLASH_ID_0);

    hal_sw_bootmode_set(HAL_SW_BOOTMODE_ENTER_HIDE_BOOT);
#ifdef __KMATE106__
	#ifndef __BES2500_HAILINGWEI_CHC__
    app_status_indication_set(APP_STATUS_INDICATION_OTA);
    app_voice_report(APP_STATUS_INDICATION_WARNING, 0);
    osDelay(1200);
    #endif
#endif
    pmu_reboot();
}

#ifdef __BES2500_HAILINGWEI_CHC__
void app_earheadset_enter_otamode_update_software(void)
{
	if(!app_tws_ibrt_tws_link_connected())
	{
		if(app_ibrt_ui_get_enter_pairing_mode())
		{
			app_otaMode_enter(NULL, NULL);
		}
	}
}
#endif


#ifdef OTA_ENABLED
const uint32_t ota_new_img_offset_get(void)
{
    return NEW_IMAGE_FLASH_OFFSET;
}

const uint32_t ota_app_img_offset_get(void)
{
    return __APP_IMAGE_FLASH_OFFSET__;
}
#endif

#ifdef __USB_COMM__
void app_usb_cdc_comm_key_handler(APP_KEY_STATUS *status, void *param)
{
    TRACE(3,"%s %d,%d", __func__, status->code, status->event);
    hal_sw_bootmode_clear(HAL_SW_BOOTMODE_REBOOT);
    hal_sw_bootmode_set(HAL_SW_BOOTMODE_CDC_COMM);
    pmu_usb_config(PMU_USB_CONFIG_TYPE_DEVICE);
    hal_cmu_reset_set(HAL_CMU_MOD_GLOBAL);
}
#endif

#if 0
void app_dfu_key_handler(APP_KEY_STATUS *status, void *param)
{
    TRACE(1,"%s ",__func__);
    hal_sw_bootmode_clear(HAL_SW_BOOTMODE_REBOOT);
    hal_sw_bootmode_set(HAL_SW_BOOTMODE_FORCE_USB_DLD);
    pmu_usb_config(PMU_USB_CONFIG_TYPE_DEVICE);
    hal_cmu_reset_set(HAL_CMU_MOD_GLOBAL);
}
#else
void app_dfu_key_handler(APP_KEY_STATUS *status, void *param)
{
    TRACE(1,"%s ",__func__);
    hal_sw_bootmode_clear(0xffffffff);
    hal_sw_bootmode_set(HAL_SW_BOOTMODE_FORCE_USB_DLD | HAL_SW_BOOTMODE_SKIP_FLASH_BOOT);
    pmu_usb_config(PMU_USB_CONFIG_TYPE_DEVICE);
    hal_cmu_reset_set(HAL_CMU_MOD_GLOBAL);
}
#endif

void app_ota_key_handler(APP_KEY_STATUS *status, void *param)
{
    static uint32_t time = hal_sys_timer_get();
    static uint16_t cnt = 0;

    TRACE(3,"%s %d,%d",__func__, status->code, status->event);

    if (TICKS_TO_MS(hal_sys_timer_get() - time) > 600) // 600 = (repeat key intervel)500 + (margin)100
        cnt = 0;
    else
        cnt++;

    if (cnt == PRESS_KEY_TO_ENTER_OTA_REPEAT_CNT) {
        app_otaMode_enter(NULL, NULL);
    }

    time = hal_sys_timer_get();
}
extern "C" void app_bt_key(APP_KEY_STATUS *status, void *param)
{
    TRACE(3,"%s %d,%d",__func__, status->code, status->event);
#define DEBUG_CODE_USE 0
    switch(status->event)
    {
        case APP_KEY_EVENT_CLICK:
            TRACE(0,"first blood!");
#if DEBUG_CODE_USE
            if (status->code == APP_KEY_CODE_PWR)
            {
#ifdef __INTERCONNECTION__
                // add favorite music
                // app_interconnection_handle_favorite_music_through_ccmp(1);

                // ask for ota update
                ota_update_request();
                return;
#else
                static int m = 0;
                if (m == 0) {
                    m = 1;
                    hal_iomux_set_analog_i2c();
                }
                else {
                    m = 0;
                    hal_iomux_set_uart0();
                }
#endif
            }
#endif
            break;
        case APP_KEY_EVENT_DOUBLECLICK:
            TRACE(0,"double kill");
#if DEBUG_CODE_USE
            if (status->code == APP_KEY_CODE_PWR)
            {
#ifdef __INTERCONNECTION__
                // play favorite music
                app_interconnection_handle_favorite_music_through_ccmp(2);
#else
                app_otaMode_enter(NULL, NULL);
#endif
                return;
            }
#endif
			break;
        case APP_KEY_EVENT_TRIPLECLICK:
            TRACE(0,"triple kill");
            if (status->code == APP_KEY_CODE_PWR)
            {
				#ifndef __BES2500_HAILINGWEI_CHC__
#ifndef __BT_ONE_BRING_TWO__
				if(btif_me_get_activeCons() < 1){
#else
	            if(btif_me_get_activeCons() < 2){
#endif
	                app_bt_accessmode_set(BTIF_BT_DEFAULT_ACCESS_MODE_PAIR);
#ifdef __INTERCONNECTION__
	                app_interceonnection_start_discoverable_adv(INTERCONNECTION_BLE_FAST_ADVERTISING_INTERVAL,
	                                                            APP_INTERCONNECTION_FAST_ADV_TIMEOUT_IN_MS);
	                return;
#endif
#ifdef GFPS_ENABLED
	                app_enter_fastpairing_mode();
#endif
					app_voice_report(APP_STATUS_INDICATION_BOTHSCAN,0);
            	}
                return;
            }
            #else
            }
            #endif
            break;
        case APP_KEY_EVENT_ULTRACLICK:
            TRACE(0,"ultra kill");
            break;
        case APP_KEY_EVENT_RAMPAGECLICK:
            TRACE(0,"rampage kill!you are crazy!");
            break;

        case APP_KEY_EVENT_UP:
            break;
    }
#ifdef __FACTORY_MODE_SUPPORT__
    if (app_status_indication_get() == APP_STATUS_INDICATION_BOTHSCAN && (status->event == APP_KEY_EVENT_DOUBLECLICK)){
		#ifdef __BES2500_HAILINGWEI_CHC__
		bt_key_send(status);
		#else
        app_factorymode_languageswitch_proc();
        #endif
    }else
#endif
    {
        bt_key_send(status);
    }
}

#ifdef RB_CODEC
extern bool app_rbcodec_check_hfp_active(void );
void app_switch_player_key(APP_KEY_STATUS *status, void *param)
{
    TRACE(3,"%s %d,%d",__func__, status->code, status->event);

    if(!rb_ctl_is_init_done()) {
        TRACE(0,"rb ctl not init done");
        return ;
    }

    if( app_rbcodec_check_hfp_active() ) {
        app_bt_key(status,param);
        return;
    }

    app_rbplay_audio_reset_pause_status();

    if(app_rbplay_mode_switch()) {
        app_voice_report(APP_STATUS_INDICATION_POWERON, 0);
        app_rbcodec_ctr_play_onoff(true);
    } else {
        app_rbcodec_ctr_play_onoff(false);
        app_voice_report(APP_STATUS_INDICATION_POWEROFF, 0);
    }
    return ;

}
#endif

void app_voice_assistant_key(APP_KEY_STATUS *status, void *param)
{
    TRACE(2,"%s event %d", __func__, status->event);
#if defined(BISTO_ENABLED) || defined(__AI_VOICE__)
    if (app_ai_manager_is_in_multi_ai_mode())
    {
        if (app_ai_manager_spec_get_status_is_in_invalid()) {
            TRACE(0,"AI feature has been diabled");
            return;
        }

#ifdef MAI_TYPE_REBOOT_WITHOUT_OEM_APP
        if (app_ai_manager_get_spec_update_flag()) {
            TRACE(0,"device reboot is ongoing...");
            return;
        }
#endif

        if (app_ai_manager_is_need_reboot())
        {
            TRACE(1, "%s ai need to reboot", __func__);
            return;
        }

        if(app_ai_manager_voicekey_is_enable()) {
            if (AI_SPEC_GSOUND == app_ai_manager_get_current_spec()) {
#ifdef BISTO_ENABLED
                gsound_custom_actions_handle_key(status, param);
#endif
            } else if(AI_SPEC_INIT != app_ai_manager_get_current_spec()) {
                app_ai_key_event_handle(status, 0);
            }
        }
    }
    else
    {
	    app_ai_key_event_handle(status, 0);
#ifdef BISTO_ENABLED
	    gsound_custom_actions_handle_key(status, param);
#endif
    }
#endif
}

#ifdef IS_MULTI_AI_ENABLED
void app_voice_gva_onoff_key(APP_KEY_STATUS *status, void *param)
{
    uint8_t current_ai_spec = app_ai_manager_get_current_spec();

    TRACE(2,"%s current_ai_spec %d", __func__, current_ai_spec);
    if (current_ai_spec == AI_SPEC_INIT)
    {
        app_ai_manager_enable(true, AI_SPEC_GSOUND);
    }
    else if(current_ai_spec == AI_SPEC_GSOUND)
    {
        app_ai_manager_enable(false, AI_SPEC_GSOUND);
    }
    else if(current_ai_spec == AI_SPEC_AMA)
    {
        app_ai_manager_switch_spec(AI_SPEC_GSOUND);
    }
    app_ble_refresh_adv_state(BLE_ADVERTISING_INTERVAL);
}

void app_voice_ama_onoff_key(APP_KEY_STATUS *status, void *param)
{
    uint8_t current_ai_spec = app_ai_manager_get_current_spec();

    TRACE(2,"%s current_ai_spec %d", __func__, current_ai_spec);
    if (current_ai_spec == AI_SPEC_INIT)
    {
        app_ai_manager_enable(true, AI_SPEC_AMA);
    }
    else if(current_ai_spec == AI_SPEC_AMA)
    {
        app_ai_manager_enable(false, AI_SPEC_AMA);
    }
    else if(current_ai_spec == AI_SPEC_GSOUND)
    {
        app_ai_manager_switch_spec(AI_SPEC_AMA);
    }
    app_ble_refresh_adv_state(BLE_ADVERTISING_INTERVAL);
}
#endif

#if defined(BT_USB_AUDIO_DUAL_MODE_TEST) && defined(BT_USB_AUDIO_DUAL_MODE)
extern "C" void test_btusb_switch(void);
void app_btusb_audio_dual_mode_test(APP_KEY_STATUS *status, void *param)
{
    TRACE(0,"test_btusb_switch");
    test_btusb_switch();
}
#endif

extern void switch_dualmic_status(void);

void app_switch_dualmic_key(APP_KEY_STATUS *status, void *param)
{
    switch_dualmic_status();
}

#if defined(ANC_APP)
void app_anc_key(APP_KEY_STATUS *status, void *param)
{
    //app_anc_loop_switch();
    app_anc_loop_switch_on_off();
}
#endif

#ifdef POWERKEY_I2C_SWITCH
extern void app_factorymode_i2c_switch(APP_KEY_STATUS *status, void *param);
#endif

#ifdef TILE_DATAPATH
extern "C" void app_tile_key_handler(APP_KEY_STATUS *status, void *param);
#endif

#ifdef __POWERKEY_CTRL_ONOFF_ONLY__
#if defined(__APP_KEY_FN_STYLE_A__)
const APP_KEY_HANDLE  app_key_handle_cfg[] = {
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_UP},"bt function key",app_bt_key_shutdown, NULL},
    {{APP_KEY_CODE_FN1,APP_KEY_EVENT_LONGPRESS},"bt function key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN1,APP_KEY_EVENT_UP},"bt function key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN1,APP_KEY_EVENT_DOUBLECLICK},"bt function key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN2,APP_KEY_EVENT_UP},"bt volume up key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN2,APP_KEY_EVENT_LONGPRESS},"bt play backward key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN3,APP_KEY_EVENT_UP},"bt volume down key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN3,APP_KEY_EVENT_LONGPRESS},"bt play forward key",app_bt_key, NULL},
#ifdef SUPPORT_SIRI
    {{APP_KEY_CODE_NONE ,APP_KEY_EVENT_NONE},"none function key",app_bt_key, NULL},
#endif

};
#else //#elif defined(__APP_KEY_FN_STYLE_B__)
const APP_KEY_HANDLE  app_key_handle_cfg[] = {
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_UP},"bt function key",app_bt_key_shutdown, NULL},
    {{APP_KEY_CODE_FN1,APP_KEY_EVENT_LONGPRESS},"bt function key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN1,APP_KEY_EVENT_UP},"bt function key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN1,APP_KEY_EVENT_DOUBLECLICK},"bt function key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN2,APP_KEY_EVENT_REPEAT},"bt volume up key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN2,APP_KEY_EVENT_UP},"bt play backward key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN3,APP_KEY_EVENT_REPEAT},"bt volume down key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN3,APP_KEY_EVENT_UP},"bt play forward key",app_bt_key, NULL},
#ifdef SUPPORT_SIRI
    {{APP_KEY_CODE_NONE ,APP_KEY_EVENT_NONE},"none function key",app_bt_key, NULL},
#endif

};
#endif
#else
#if defined(__APP_KEY_FN_STYLE_A__)
//--
const APP_KEY_HANDLE  app_key_handle_cfg[] = {
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_LONGLONGPRESS},"bt function key",app_bt_key_shutdown, NULL},
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_LONGPRESS},"bt function key",app_bt_key, NULL},
#if defined(BT_USB_AUDIO_DUAL_MODE_TEST) && defined(BT_USB_AUDIO_DUAL_MODE)
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_CLICK},"bt function key",app_bt_key, NULL},
#ifdef RB_CODEC
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_CLICK},"bt function key",app_switch_player_key, NULL},
#else
    //{{APP_KEY_CODE_PWR,APP_KEY_EVENT_CLICK},"btusb mode switch key.",app_btusb_audio_dual_mode_test, NULL},
#endif
#endif
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_DOUBLECLICK},"bt function key",app_bt_key, NULL},
#ifdef TILE_DATAPATH
     {{APP_KEY_CODE_PWR,APP_KEY_EVENT_TRIPLECLICK},"bt function key",app_tile_key_handler, NULL},
#else
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_TRIPLECLICK},"bt function key",app_bt_key, NULL},
#endif
#if RAMPAGECLICK_TEST_MODE
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_ULTRACLICK},"bt function key",app_bt_key_enter_nosignal_mode, NULL},
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_RAMPAGECLICK},"bt function key",app_bt_key_enter_testmode, NULL},
#endif
#ifdef POWERKEY_I2C_SWITCH
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_RAMPAGECLICK},"bt i2c key",app_factorymode_i2c_switch, NULL},
#endif
    //{{APP_KEY_CODE_FN1,APP_KEY_EVENT_UP},"bt volume up key",app_bt_key, NULL},
    //{{APP_KEY_CODE_FN1,APP_KEY_EVENT_LONGPRESS},"bt play backward key",app_bt_key, NULL},
#if defined(APP_LINEIN_A2DP_SOURCE)||defined(APP_I2S_A2DP_SOURCE)
    {{APP_KEY_CODE_FN1,APP_KEY_EVENT_DOUBLECLICK},"bt mode src snk key",app_bt_key, NULL},
#endif
    //{{APP_KEY_CODE_FN2,APP_KEY_EVENT_UP},"bt volume down key",app_bt_key, NULL},
    //{{APP_KEY_CODE_FN2,APP_KEY_EVENT_LONGPRESS},"bt play forward key",app_bt_key, NULL},
    //{{APP_KEY_CODE_FN15,APP_KEY_EVENT_UP},"bt volume down key",app_bt_key, NULL},
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_CLICK},"bt function key",app_bt_key, NULL},

#ifdef SUPPORT_SIRI
    {{APP_KEY_CODE_NONE ,APP_KEY_EVENT_NONE},"none function key",app_bt_key, NULL},
#endif
#if defined( __BT_ANC_KEY__)&&defined(ANC_APP)
	{{APP_KEY_CODE_PWR,APP_KEY_EVENT_CLICK},"bt anc key",app_anc_key, NULL},
#endif
#ifdef TILE_DATAPATH
    {{APP_KEY_CODE_TILE, APP_KEY_EVENT_DOWN}, "tile function key", app_tile_key_handler, NULL},
    {{APP_KEY_CODE_TILE, APP_KEY_EVENT_UP}, "tile function key", app_tile_key_handler, NULL},
#endif

#if defined(VOICE_DATAPATH) || defined(__AI_VOICE__)
    {{APP_KEY_CODE_GOOGLE, APP_KEY_EVENT_FIRST_DOWN}, "google assistant key", app_voice_assistant_key, NULL},
#if defined(IS_GSOUND_BUTTION_HANDLER_WORKAROUND_ENABLED) || defined(PUSH_AND_HOLD_ENABLED) || defined(__TENCENT_VOICE__)
    {{APP_KEY_CODE_GOOGLE, APP_KEY_EVENT_UP}, "google assistant key", app_voice_assistant_key, NULL},
#endif
    {{APP_KEY_CODE_GOOGLE, APP_KEY_EVENT_UP_AFTER_LONGPRESS}, "google assistant key", app_voice_assistant_key, NULL},
    {{APP_KEY_CODE_GOOGLE, APP_KEY_EVENT_LONGPRESS}, "google assistant key", app_voice_assistant_key, NULL},
    {{APP_KEY_CODE_GOOGLE, APP_KEY_EVENT_CLICK}, "google assistant key", app_voice_assistant_key, NULL},
    {{APP_KEY_CODE_GOOGLE, APP_KEY_EVENT_DOUBLECLICK}, "google assistant key", app_voice_assistant_key, NULL},
#endif
#ifdef IS_MULTI_AI_ENABLED
    {{APP_KEY_CODE_FN13, APP_KEY_EVENT_CLICK}, "gva on-off key", app_voice_gva_onoff_key, NULL},
    {{APP_KEY_CODE_FN14, APP_KEY_EVENT_CLICK}, "ama on-off key", app_voice_ama_onoff_key, NULL},
#endif
#if defined(BT_USB_AUDIO_DUAL_MODE_TEST) && defined(BT_USB_AUDIO_DUAL_MODE)
    {{APP_KEY_CODE_FN15, APP_KEY_EVENT_CLICK}, "btusb mode switch key.", app_btusb_audio_dual_mode_test, NULL},
#endif
};
#else //#elif defined(__APP_KEY_FN_STYLE_B__)
const APP_KEY_HANDLE  app_key_handle_cfg[] = {
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_LONGLONGPRESS},"bt function key",app_bt_key_shutdown, NULL},
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_LONGPRESS},"bt function key",app_bt_key, NULL},
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_CLICK},"bt function key",app_bt_key, NULL},
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_DOUBLECLICK},"bt function key",app_bt_key, NULL},
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_TRIPLECLICK},"bt function key",app_bt_key, NULL},
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_ULTRACLICK},"bt function key",app_bt_key_enter_nosignal_mode, NULL},
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_RAMPAGECLICK},"bt function key",app_bt_key_enter_testmode, NULL},
    {{APP_KEY_CODE_FN1,APP_KEY_EVENT_REPEAT},"bt volume up key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN1,APP_KEY_EVENT_UP},"bt play backward key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN2,APP_KEY_EVENT_REPEAT},"bt volume down key",app_bt_key, NULL},
    {{APP_KEY_CODE_FN2,APP_KEY_EVENT_UP},"bt play forward key",app_bt_key, NULL},
#ifdef SUPPORT_SIRI
    {{APP_KEY_CODE_NONE ,APP_KEY_EVENT_NONE},"none function key",app_bt_key, NULL},
#endif

    {{APP_KEY_CODE_GOOGLE, APP_KEY_EVENT_FIRST_DOWN}, "google assistant key", app_voice_assistant_key, NULL},
#if defined(IS_GSOUND_BUTTION_HANDLER_WORKAROUND_ENABLED) || defined(PUSH_AND_HOLD_ENABLED)
    {{APP_KEY_CODE_GOOGLE, APP_KEY_EVENT_UP}, "google assistant key", app_voice_assistant_key, NULL},
#endif
    {{APP_KEY_CODE_GOOGLE, APP_KEY_EVENT_UP_AFTER_LONGPRESS}, "google assistant key", app_voice_assistant_key, NULL},
    {{APP_KEY_CODE_GOOGLE, APP_KEY_EVENT_LONGPRESS}, "google assistant key", app_voice_assistant_key, NULL},
    {{APP_KEY_CODE_GOOGLE, APP_KEY_EVENT_CLICK}, "google assistant key", app_voice_assistant_key, NULL},
    {{APP_KEY_CODE_GOOGLE, APP_KEY_EVENT_DOUBLECLICK}, "google assistant key", app_voice_assistant_key, NULL},
};
#endif
#endif

void app_key_init(void)
{
#if defined(IBRT)
	TRACE(1,"%s",__func__);
    app_ibrt_ui_test_key_init();
#else
    uint8_t i = 0;
    TRACE(1,"%s",__func__);

    app_key_handle_clear();
    for (i=0; i<(sizeof(app_key_handle_cfg)/sizeof(APP_KEY_HANDLE)); i++){
        app_key_handle_registration(&app_key_handle_cfg[i]);
    }
#endif
}


void app_ibrt_ui_test_key(APP_KEY_STATUS *status, void *param);


void app_charge_clear_all_pairlst_reset_poweron(APP_KEY_STATUS *status, void *param)
{
	app_charge_clear_all_pairlist(true, true);
}

void app_charge_clear_all_pairlist(bool clear_tws, bool clear_phone)
{
	//ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();

	TRACE(2,"%s clear_tws %d clear_phone %d",__func__, clear_tws, clear_phone);

	clear_pairlist_reset_poweron_flag = true;
	
	if(clear_tws)
	{
		app_ibrt_remove_history_tws_paired_device();
	}
	// clear phone pairlist
	if(clear_phone)
	{
		app_ibrt_remove_history_phone_paired_device();
	}

	hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT_ENTER_PHONE_PAIR);
	app_reset();
}



void app_key_init_on_charging(void)
{
    uint8_t i = 0;
    const APP_KEY_HANDLE  key_cfg[] = {
    	#ifdef __BES2500_HAILINGWEI_CHC__
		{{HAL_KEY_CODE_FN8,APP_KEY_EVENT_DOWN},"app_ibrt_ui_test_key", app_ibrt_ui_test_key, NULL},
		{{APP_KEY_CODE_FN9,APP_KEY_EVENT_DOWN},"app_ibrt_ui_test_key", app_ibrt_ui_test_key, NULL},
    	{{APP_KEY_CODE_FN9,APP_KEY_EVENT_UP},"app_ibrt_ui_test_key", app_ibrt_ui_test_key, NULL},

    	{{APP_KEY_CODE_PWR,APP_KEY_EVENT_TRIPLECLICK},"bt function key",app_charge_clear_all_pairlst_reset_poweron, NULL},
    	#else
        {{APP_KEY_CODE_PWR,APP_KEY_EVENT_REPEAT},"ota function key",app_ota_key_handler, NULL},
        {{APP_KEY_CODE_PWR,APP_KEY_EVENT_CLICK},"bt function key",app_dfu_key_handler, NULL},
        #endif
#ifdef __USB_COMM__
        {{APP_KEY_CODE_PWR,APP_KEY_EVENT_LONGPRESS},"usb cdc key",app_usb_cdc_comm_key_handler, NULL},
#endif
    };

    TRACE(1,"%s",__func__);
    for (i=0; i<(sizeof(key_cfg)/sizeof(APP_KEY_HANDLE)); i++){
        app_key_handle_registration(&key_cfg[i]);
    }
}

extern bt_status_t LinkDisconnectDirectly(bool PowerOffFlag);
void a2dp_suspend_music_force(void);

bool app_is_power_off_in_progress(void)
{
    return app_poweroff_flag?TRUE:FALSE;
}

#define HOLD_FREQUENCY_TIMER_DELAY_TIME_MS  (10000)
static enum APP_SYSFREQ_USER_T hold_frequency_user;
static enum APP_SYSFREQ_FREQ_T hold_frequency;
static void app_bt_streaming_hold_frequency_run_timehandler(void const *param);
osTimerDef (BT_STREAM_FREQUENCY_HOLD_TIMER_ID, app_bt_streaming_hold_frequency_run_timehandler);
osTimerId bt_stream_frequency_hold_timer_id = NULL;
static void app_bt_streaming_hold_frequency_run_timehandler(void const *param)
{
    TRACE(1, "release hold frequency");

#ifdef __AI_VOICE__
    if((app_bt_stream_isrun(APP_BT_STREAM_AI_VOICE) == false))
#endif
    {
	    app_sysfreq_req(hold_frequency_user, APP_SYSFREQ_32K);
	}
}

void app_bt_streaming_hold_frequency_run_timer_create(void)
{
    if(bt_stream_frequency_hold_timer_id == NULL){
        bt_stream_frequency_hold_timer_id = osTimerCreate(osTimer(BT_STREAM_FREQUENCY_HOLD_TIMER_ID), osTimerOnce, NULL);
    }
}

void app_bt_streaming_hold_frequency_run_timer_start(uint8_t ratio)
{
    if(bt_stream_frequency_hold_timer_id != NULL){
        hold_frequency_user = APP_SYSFREQ_USER_BOOST;
        uint8_t freq = (uint8_t)APP_SYSFREQ_104M;
        freq = ratio+freq;
        hold_frequency = (enum APP_SYSFREQ_FREQ_T)freq;
        TRACE(1, "hold frequency to %d", freq);
        app_sysfreq_req(hold_frequency_user, hold_frequency);

        osTimerStart(bt_stream_frequency_hold_timer_id, HOLD_FREQUENCY_TIMER_DELAY_TIME_MS);
    }else{
        TRACE(1,"%s bt_stream_frequency_hold_timer_id == NULL",__func__);
    }
}

void app_bt_streaming_hold_frequency_run_timer_stop(void)
{
    if(bt_stream_frequency_hold_timer_id != NULL){
        osTimerStop(bt_stream_frequency_hold_timer_id);
    }
}

#if GFPS_ENABLED
#define APP_GFPS_BATTERY_TIMEROUT_VALUE             (10000)
static void app_gfps_battery_show_timeout_timer_cb(void const *n);
osTimerDef (GFPS_BATTERY_SHOW_TIMEOUT_TIMER, app_gfps_battery_show_timeout_timer_cb);
static osTimerId app_gfps_battery_show_timer_id = NULL;
#include "app_gfps.h"
static void app_gfps_battery_show_timeout_timer_cb(void const *n)
{
    TRACE(1,"%s", __func__);
    app_gfps_set_battery_datatype(HIDE_UI_INDICATION);
}

void app_gfps_battery_show_timer_start()
{
    if (app_gfps_battery_show_timer_id == NULL)
        app_gfps_battery_show_timer_id = osTimerCreate(osTimer(GFPS_BATTERY_SHOW_TIMEOUT_TIMER), osTimerOnce, NULL);
    osTimerStart(app_gfps_battery_show_timer_id, APP_GFPS_BATTERY_TIMEROUT_VALUE);
}

void app_gfps_battery_show_timer_stop()
{
    if (app_gfps_battery_show_timer_id)
        osTimerStop(app_gfps_battery_show_timer_id);
}

enum GFPS_FIND_BUDS_STATUS {
    GFPS_MUSIC_OFF_FIND_OFF,
    GFPS_MUSIC_OFF_FIND_ON,
    GFPS_MUSIC_ON_FIND_OFF,
    GFPS_MUSIC_ON_FIND_ON,
};

static void app_gfps_find_mybuds_timer_cb(void const *n);
osTimerDef (GFPS_FIND_MYBUDS_TIMER, app_gfps_find_mybuds_timer_cb);
static osTimerId app_gfps_find_mybuds_timer_id = NULL;
static uint8_t current_gfps_find_state = GFPS_MUSIC_OFF_FIND_OFF;
static bool gfps_find_state = 0;
extern struct BT_DEVICE_T  app_bt_device;
uint8_t gfps_ring_mode = GFPS_RING_MODE_BOTH_OFF;

void app_gfps_ring_mode_set(uint8_t ring_mode)
{
    gfps_ring_mode = ring_mode;
}

uint8_t app_gfps_ring_mode_get()
{
    return gfps_ring_mode;
}

void app_voice_start_gfps_find(void)
{
    TRACE(1, "%s", __func__);
    gfps_find_state=1;
    app_voice_report(APP_STATUS_INDICATION_FIND_MY_BUDS, 0);
}

void app_voice_stop_gfps_find(void)
{
    TRACE(1, "%s", __func__);
    gfps_find_state=0;
    app_voice_stop(APP_STATUS_INDICATION_FIND_MY_BUDS, 0);
}

bool app_voice_gfps_find_state(void)
{
    return gfps_find_state;
}

static void app_gfps_find_mybuds_timer_cb(void const *n)
{
    app_gfps_find_sm(1);
}

static void app_gfps_delay_to_start_find_mybuds()
{
    if (app_gfps_find_mybuds_timer_id == NULL)
        app_gfps_find_mybuds_timer_id = osTimerCreate(osTimer(GFPS_FIND_MYBUDS_TIMER), osTimerOnce, NULL);
    osTimerStart(app_gfps_find_mybuds_timer_id, 500);
}

void app_gfps_find_sm(bool find_on_off)
{
    bool music_on_off = app_bt_device.a2dp_streamming[0];
    osTimerStop(app_gfps_find_mybuds_timer_id);
    TRACE(3, "%s %d %d", __func__, find_on_off, current_gfps_find_state);
    switch(current_gfps_find_state)
    {
        case GFPS_MUSIC_OFF_FIND_OFF:
            if(find_on_off)
            {
                app_voice_start_gfps_find();
                if(music_on_off)
                    current_gfps_find_state=GFPS_MUSIC_ON_FIND_ON;
                else
                    current_gfps_find_state=GFPS_MUSIC_OFF_FIND_ON;
            }
            else
            {
                if(music_on_off)
                    current_gfps_find_state=GFPS_MUSIC_ON_FIND_OFF;
                if (AUDIO_ID_FIND_MY_BUDS == app_play_audio_get_aud_id())
                    app_voice_stop_gfps_find();
            }
            break;

        case GFPS_MUSIC_OFF_FIND_ON:
            if(find_on_off)
            {
                if(music_on_off)
                {
                    app_voice_stop_gfps_find();
                    app_gfps_delay_to_start_find_mybuds();
                    current_gfps_find_state=GFPS_MUSIC_ON_FIND_OFF;//DELAY TO SATRT SM,TO OPEN FIND
                }
                else
                {
                    app_voice_start_gfps_find();
                }
            }
            else
            {
                if (AUDIO_ID_FIND_MY_BUDS == app_play_audio_get_aud_id())
                    app_voice_stop_gfps_find();
                if(music_on_off)
                    current_gfps_find_state=GFPS_MUSIC_ON_FIND_OFF;
                else
                    current_gfps_find_state=GFPS_MUSIC_OFF_FIND_OFF;
            }
            break;

        case GFPS_MUSIC_ON_FIND_OFF:
            if(find_on_off)
            {
                if(music_on_off)
                {
                    app_audio_manager_sendrequest(APP_BT_STREAM_MANAGER_STOP,BT_STREAM_SBC,BT_DEVICE_ID_1,MAX_RECORD_NUM);
                    current_gfps_find_state=GFPS_MUSIC_ON_FIND_ON;
                }
                else
                {
                    current_gfps_find_state=GFPS_MUSIC_OFF_FIND_ON;
                }
                app_voice_start_gfps_find();
            }
            else
            {
                if(!music_on_off)
                    current_gfps_find_state=GFPS_MUSIC_OFF_FIND_OFF;
                if (AUDIO_ID_FIND_MY_BUDS == app_play_audio_get_aud_id())
                    app_voice_stop_gfps_find();
            }
            break;

        case GFPS_MUSIC_ON_FIND_ON:
            if(find_on_off)
            {
                if(!music_on_off)
                    current_gfps_find_state=GFPS_MUSIC_OFF_FIND_ON;
                app_voice_start_gfps_find();
            }
            else
            {
                if (AUDIO_ID_FIND_MY_BUDS == app_play_audio_get_aud_id())
                    app_voice_stop_gfps_find();
                if(music_on_off)
                {
                    current_gfps_find_state=GFPS_MUSIC_ON_FIND_OFF;
                    app_audio_manager_sendrequest(APP_BT_STREAM_MANAGER_START,BT_STREAM_SBC,BT_DEVICE_ID_1,MAX_RECORD_NUM);
                }
                else
                {
                    current_gfps_find_state=GFPS_MUSIC_OFF_FIND_OFF;
                }
            }
            break;
    }
}
#endif

int app_deinit(int deinit_case)
{
    int nRet = 0;
    TRACE(2,"%s case:%d",__func__, deinit_case);

#ifdef __PC_CMD_UART__
    app_cmd_close();
#endif
#if (defined(BTUSB_AUDIO_MODE) || defined(BT_USB_AUDIO_DUAL_MODE))
    if(app_usbaudio_mode_on()) return 0;
#endif

	/*
    if (app_NTC_mode_cfg.pin != HAL_IOMUX_PIN_NUM){
       // hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_NTC_mode_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_NTC_mode_cfg.pin, HAL_GPIO_DIR_OUT, 0);
    }
	
    if (app_chuan_yun_mode_cfg.pin != HAL_IOMUX_PIN_NUM){
       // hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&app_chuan_yun_mode_cfg, 1);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_chuan_yun_mode_cfg.pin, HAL_GPIO_DIR_OUT, 1);
        osDelay(350);
        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)app_chuan_yun_mode_cfg.pin, HAL_GPIO_DIR_OUT, 0);
    }
	*/
	//TRACE(0, "app_deinit poweroff");

	#if USER_REBOOT_PLAY_MUSIC_AUTO
    hal_sw_bootmode_clear(HAL_SW_BOOTMODE_LOCAL_PLAYER);
	#endif
	app_store_auto_play_proc(false);
	

    if (!deinit_case){
#if defined(ANC_APP)
        app_anc_deinit();
#endif
#if defined(ANC_ASSIST_ENABLED)
        app_anc_assist_deinit();
#endif
        app_poweroff_flag = 1;
        #ifdef __BES2500_HAILINGWEI_CHC__
        //鍏虫満浼樺厛鍏虫満LED
        app_bt_software_version_timer_start_stop(false);
        #endif
        #ifdef __BES2500_IQS269_TOUCH__
        app_poweroff_set_touch_channel();
        #endif

        #ifdef __BES2500_HAILINGWEI_CHC__
		if((app_battery_charging_flag)/*||(app_factorymode_get())*/
		||clear_pairlist_reset_poweron_flag||tws_pair_phone_pair_switch_flag||clear_tws_pairlist_reset_poweron_flag)
		{
			TRACE(0, "RESET SHUT");
			#if FPGA==0
	        nv_record_flash_flush();
	        norflash_api_flush_all();
			#if defined(DUMP_LOG_ENABLE)
	        log_dump_flush_all();
			#endif
			#endif
	        //osDelay(1000);
	        af_close();
	        return nRet;
		}
        #endif
        //if(!clear_poweroff_flag)
        {
			#if defined(APP_LINEIN_A2DP_SOURCE)
	        app_audio_sendrequest(APP_A2DP_SOURCE_LINEIN_AUDIO, (uint8_t)APP_BT_SETTING_CLOSE,0);
			#endif
			#if defined(APP_I2S_A2DP_SOURCE)
	        app_audio_sendrequest(APP_A2DP_SOURCE_I2S_AUDIO, (uint8_t)APP_BT_SETTING_CLOSE,0);
			#endif
	        app_status_indication_filter_set(APP_STATUS_INDICATION_BOTHSCAN);
	        app_audio_sendrequest(APP_BT_STREAM_INVALID, (uint8_t)APP_BT_SETTING_CLOSEALL, 0);
	        osDelay(500);
	        
	        //app_voice_report(APP_STATUS_INDICATION_TONE_DU, 0);
	        //app_voice_report(APP_STATUS_INDICATION_TONE_DU, 0);
	        //app_hfp_end_call_voice_play(APP_STATUS_INDICATION_TONE_DU, 0, false);
	        //app_hfp_end_call_voice_play(APP_STATUS_INDICATION_TONE_DU, 0, false);
	        LinkDisconnectDirectly(true);
	        osDelay(500);
        }
        app_status_indication_set(APP_STATUS_INDICATION_INITIAL);
        app_status_indication_set(APP_STATUS_INDICATION_POWEROFF);
#ifdef MEDIA_PLAYER_SUPPORT
        //app_voice_report(APP_STATUS_INDICATION_POWEROFF, 0);
        //app_voice_report(APP_STATUS_INDICATION_TONE_DU, 0);
        //app_voice_report(APP_STATUS_INDICATION_TONE_DU, 0);
        app_hfp_end_call_voice_play(APP_STATUS_INDICATION_TONE_DU, 0, false);
        app_hfp_end_call_voice_play(APP_STATUS_INDICATION_TONE_DU, 0, false);
#endif
#ifdef __THIRDPARTY
        app_thirdparty_specific_lib_event_handle(THIRDPARTY_FUNC_NO1,THIRDPARTY_DEINIT);
#endif
#if FPGA==0
        nv_record_flash_flush();
        norflash_api_flush_all();
#if defined(DUMP_LOG_ENABLE)
        log_dump_flush_all();
#endif
#endif
        osDelay(2000);
        af_close();
    }

    return nRet;
}

#ifdef APP_TEST_MODE
extern void app_test_init(void);
int app_init(void)
{
    int nRet = 0;
    //uint8_t pwron_case = APP_POWERON_CASE_INVALID;
    TRACE(1,"%s",__func__);
    app_poweroff_flag = 0;

#ifdef APP_TRACE_RX_ENABLE
    app_trace_rx_open();
#endif

    app_sysfreq_req(APP_SYSFREQ_USER_APP_INIT, APP_SYSFREQ_52M);
    list_init();
    af_open();
    app_os_init();
    app_pwl_open();
    app_audio_open();
    app_audio_manager_open();
    app_overlay_open();
    if (app_key_open(true))
    {
        nRet = -1;
        goto exit;
    }

    app_test_init();
exit:
    app_sysfreq_req(APP_SYSFREQ_USER_APP_INIT, APP_SYSFREQ_32K);
    return nRet;
}
#else /* !defined(APP_TEST_MODE) */

int app_bt_connect2tester_init(void)
{
    btif_device_record_t rec;
    bt_bdaddr_t tester_addr;
    uint8_t i;
    bool find_tester = false;
    struct nvrecord_env_t *nvrecord_env;
    btdevice_profile *btdevice_plf_p;
    nv_record_env_get(&nvrecord_env);

    if (nvrecord_env->factory_tester_status.status != NVRAM_ENV_FACTORY_TESTER_STATUS_DEFAULT)
        return 0;

    if (!nvrec_dev_get_dongleaddr(&tester_addr)){
        nv_record_open(section_usrdata_ddbrecord);
        for (i = 0; nv_record_enum_dev_records(i, &rec) == BT_STS_SUCCESS; i++) {
            if (!memcmp(rec.bdAddr.address, tester_addr.address, BTIF_BD_ADDR_SIZE)){
                find_tester = true;
            }
        }
        if(i==0 && !find_tester){
            memset(&rec, 0, sizeof(btif_device_record_t));
            memcpy(rec.bdAddr.address, tester_addr.address, BTIF_BD_ADDR_SIZE);
            nv_record_add(section_usrdata_ddbrecord, &rec);
            btdevice_plf_p = (btdevice_profile *)app_bt_profile_active_store_ptr_get(rec.bdAddr.address);
            nv_record_btdevicerecord_set_hfp_profile_active_state(btdevice_plf_p, true);
            nv_record_btdevicerecord_set_a2dp_profile_active_state(btdevice_plf_p, true);
        }
        if (find_tester && i>2){
            nv_record_ddbrec_delete(&tester_addr);
            nvrecord_env->factory_tester_status.status = NVRAM_ENV_FACTORY_TESTER_STATUS_TEST_PASS;
            nv_record_env_set(nvrecord_env);
        }
    }

    return 0;
}

int app_nvrecord_rebuild(void)
{
    struct nvrecord_env_t *nvrecord_env;
    nv_record_env_get(&nvrecord_env);

    nv_record_sector_clear();
    nv_record_env_init();
    nv_record_update_factory_tester_status(NVRAM_ENV_FACTORY_TESTER_STATUS_TEST_PASS);
    nv_record_env_set(nvrecord_env);
    nv_record_flash_flush();

    return 0;
}

#if (defined(BTUSB_AUDIO_MODE) || defined(BT_USB_AUDIO_DUAL_MODE))
#include "app_audio.h"
#include "usb_audio_frm_defs.h"
#include "usb_audio_app.h"

static bool app_usbaudio_mode = false;

extern "C" void btusbaudio_entry(void);
void app_usbaudio_entry(void)
{
    btusbaudio_entry();
    app_usbaudio_mode = true ;
}

bool app_usbaudio_mode_on(void)
{
    return app_usbaudio_mode;
}

void app_usb_key(APP_KEY_STATUS *status, void *param)
{
    TRACE(3,"%s %d,%d",__func__, status->code, status->event);

}

const APP_KEY_HANDLE  app_usb_handle_cfg[] = {
    {{APP_KEY_CODE_FN1,APP_KEY_EVENT_UP},"USB HID FN1 UP key",app_usb_key, NULL},
    {{APP_KEY_CODE_FN2,APP_KEY_EVENT_UP},"USB HID FN2 UP key",app_usb_key, NULL},
    {{APP_KEY_CODE_PWR,APP_KEY_EVENT_UP},"USB HID PWR UP key",app_usb_key, NULL},
};

void app_usb_key_init(void)
{
    uint8_t i = 0;
    TRACE(1,"%s",__func__);
    for (i=0; i<(sizeof(app_usb_handle_cfg)/sizeof(APP_KEY_HANDLE)); i++){
        app_key_handle_registration(&app_usb_handle_cfg[i]);
    }
}
#endif /* (defined(BTUSB_AUDIO_MODE) || defined(BT_USB_AUDIO_DUAL_MODE)) */

//#define OS_HAS_CPU_STAT 1
#if OS_HAS_CPU_STAT
extern "C" void rtx_show_all_threads_usage(void);
#define _CPU_STATISTICS_PEROID_ 6000
#define CPU_USAGE_TIMER_TMO_VALUE (_CPU_STATISTICS_PEROID_/3)
static void cpu_usage_timer_handler(void const *param);
osTimerDef(cpu_usage_timer, cpu_usage_timer_handler);
static osTimerId cpu_usage_timer_id = NULL;
static void cpu_usage_timer_handler(void const *param)
{
    rtx_show_all_threads_usage();
}
#endif

#ifdef USER_REBOOT_PLAY_MUSIC_AUTO
bool a2dp_need_to_play = false;
#endif

int btdrv_tportopen(void);


void app_ibrt_init(void)
{
        app_bt_global_handle_init();
#if defined(IBRT)
        ibrt_config_t config;
        app_tws_ibrt_init();
        app_ibrt_ui_init();
        app_ibrt_ui_test_init();
        app_ibrt_if_config_load(&config);
        app_ibrt_customif_ui_start();
        app_ibrt_ui_init_after_customif_ui_start();
#ifdef IBRT_SEARCH_UI
        app_tws_ibrt_start(&config, true);
        app_ibrt_search_ui_init(false,IBRT_NONE_EVENT);
#else
        app_tws_ibrt_start(&config, false);
#endif

#ifdef POWER_ON_ENTER_TWS_PAIRING_ENABLED
        app_ibrt_ui_event_entry(IBRT_TWS_PAIRING_EVENT);
#endif

#endif
}

#ifdef GFPS_ENABLED
static void app_tell_battery_info_handler(uint8_t *batteryValueCount,
                                          uint8_t *batteryValue)
{
    GFPS_BATTERY_STATUS_E status;
    if (app_battery_is_charging())
    {
        status = BATTERY_CHARGING;
    }
    else
    {
        status = BATTERY_NOT_CHARGING;
    }

    // TODO: add the charger case's battery level
#ifdef IBRT
    if (app_tws_ibrt_tws_link_connected())
    {
        *batteryValueCount = 2;
    }
    else
    {
        *batteryValueCount = 1;
    }
#else
    *batteryValueCount = 1;
#endif

    TRACE(2,"%s,*batteryValueCount is %d",__func__,*batteryValueCount);
    if (1 == *batteryValueCount)
    {
        batteryValue[0] = ((app_battery_current_level()+1) * 10) | (status << 7);
    }
    else
    {
        batteryValue[0] = ((app_battery_current_level()+1) * 10) | (status << 7);
        batteryValue[1] = ((app_battery_current_level()+1) * 10) | (status << 7);

    }
}
#endif

#ifdef __BES2500_HAILINGWEI_UART__

void app_ibrt_uart_cmd_longlong_press_clear_proc(void)
{
	TRACE(0,"%s", __func__);
	ibrt_ctrl_t *p_ibrt_ctrl = app_ibrt_if_get_bt_ctrl_ctx();
	tws_pair_phone_pair_switch_flag = true;
	app_ibrt_remove_history_tws_paired_device();
	// clear phone pairlist
	app_ibrt_remove_history_phone_paired_device();
	
    if (app_tws_ibrt_tws_link_connected())
    {
        app_tws_ibrt_disconnect_connection(btif_me_get_remote_device_by_handle(p_ibrt_ctrl->tws_conhandle));
    }
    
	osDelay(500);
	hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT_ENTER_TWS_PAIR);
	app_reset();
}

void app_ibrt_uart_cmd_handle_proc(uint8_t uart_cmd0, uint8_t uart_cmd1)
{
	TRACE(2,"%s  uart_cmd0 = %d uart_cmd1 = %d", __func__, uart_cmd0, uart_cmd1);

	if((uart_cmd0 == 0x81)&&(uart_cmd1 == 0x91))
	{
		//app_ibrt_uart_cmd_shutdown_proc();
	}
	else if((uart_cmd0 == 0x82)&&(uart_cmd1 == 0x92))
	{
		//shuangji
		//app_ibrt_uart_cmd_doubleclick_proc();
	}
	else if((uart_cmd0 == 0x83)&&(uart_cmd1 == 0x93))
	{
		//app_ibrt_uart_cmd_enter_ota_update_proc();
	}
	else if((uart_cmd0 == 0x84)&&(uart_cmd1 == 0x94))
	{
		//chang an 10s
		app_ibrt_uart_cmd_longlong_press_clear_proc();
	}
	else if((uart_cmd0 == 0x85)&&(uart_cmd1 == 0x95))
	{
		//app_ibrt_uart_cmd_enter_freeman_proc();
	}
	else if((uart_cmd0 == 0x86)&&(uart_cmd1 == 0x96))
	{
		//app_ibrt_uart_cmd_enter_dut_test_rf_param();
	}
	else if((uart_cmd0 == 0x87)&&(uart_cmd1 == 0x97))
	{
		//app_ibrt_uart_cmd_enter_reset_tws_pairmode();
	}
}

#if 0
void app_ibrt_uart_cmd_handle_proc(uint8_t uart_cmd0, uint8_t uart_cmd1)
{
	TRACE(2,"%s  uart_cmd0 = %d uart_cmd1 = %d", __func__, uart_cmd0, uart_cmd1);

	if((uart_cmd0 == 0xA1)&&(uart_cmd1 == 0xC1))
	{
		TRACE(0,"communication_send_data_to_chargebox 0");
		communication_send_data_to_chargebox(0);
	}
	else if((uart_cmd0 == 0xA2)&&(uart_cmd1 == 0xC2))
	{
		TRACE(0,"communication_send_data_to_chargebox 1");
		communication_send_data_to_chargebox(1);
	}
	else if((uart_cmd0 == 0xA3)&&(uart_cmd1 == 0xC3))
	{
		TRACE(0,"communication_send_data_to_chargebox 2");
		communication_send_data_to_chargebox(2);
	}
	else if((uart_cmd0 == 0xA4)&&(uart_cmd1 == 0xC4))
	{

	}
	else if((uart_cmd0 == 0xA5)&&(uart_cmd1 == 0xC5))
	{

	}
	else if((uart_cmd0 == 0xA6)&&(uart_cmd1 == 0xC6))
	{

	}
	else if((uart_cmd0 == 0xB4)&&(uart_cmd1 == 0xD4))
	{

	}
}
#endif

static int app_ibrt_uart_handle_process(APP_MESSAGE_BODY *msg_body)
{
	uint32_t evt = msg_body->message_id;
    uint32_t arg0 = msg_body->message_Param0;
    uint32_t arg1 = msg_body->message_Param1;

    TRACE(3," %s evt: %d, arg0: %d , arg1 :%d", __func__, evt, arg0, arg1);

    switch(evt)
    {
    	case IBRT_UART_CMD_EVENT:
			app_ibrt_uart_cmd_handle_proc((uint8_t)arg0, (uint8_t)arg1);
    		break;
    	default:

    		break;
    }

    return 0;
}


void app_communication_uart_open_moudle(void)
{
	app_set_threadhandle(APP_MODULE_UART, app_ibrt_uart_handle_process);
	communication_open();
}

void app_ibrt_uart_func_enable_close(bool en)
{
	if(en)
	{
		ibrt_communication_enable_uart_func();
	}
	else
	{
		ibrt_communication_disable_uart_func();
	}
}

#endif


void app_store_auto_play_proc(bool auto_play)
{
	struct nvrecord_env_t *nvrecord_env;
	nv_record_env_get(&nvrecord_env);
	nvrecord_env->auto_play.auto_play = auto_play;
	nv_record_env_set(nvrecord_env);
	nv_record_flash_flush();
    norflash_api_flush_all();
}


void app_store_shut_down_proc(bool shutdown_flag)
{
	struct nvrecord_env_t *nvrecord_env;
	nv_record_env_get(&nvrecord_env);
	nvrecord_env->reset_shutdown.reset_shutdown = shutdown_flag;
	nv_record_env_set(nvrecord_env);
	nv_record_env_get(&nvrecord_env);
	TRACE(1,"%s  %d",__func__, nvrecord_env->reset_shutdown.reset_shutdown);
    nv_record_flash_flush();
    norflash_api_flush_all();
}


extern uint32_t __coredump_section_start[];
extern uint32_t __ota_upgrade_log_start[];
extern uint32_t __log_dump_start[];
extern uint32_t __crash_dump_start[];
extern uint32_t __custom_parameter_start[];
extern uint32_t __aud_start[];
extern uint32_t __userdata_start[];
extern uint32_t __factory_start[];
//extern uint32_t __sndkey_start[];


int app_init(void)
{
    int nRet = 0;
    struct nvrecord_env_t *nvrecord_env;
#ifdef POWER_ON_ENTER_TWS_PAIRING_ENABLED
    bool need_check_key = false;
#else
    bool need_check_key = true;
#endif
    uint8_t pwron_case = APP_POWERON_CASE_NORMAL;
#ifdef BT_USB_AUDIO_DUAL_MODE
    uint8_t usb_plugin = 0;
#endif
#ifdef IBRT_SEARCH_UI
    bool is_charging_poweron=false;
#endif
	out_of_box_poweron_indication_flag = false; 

    //TRACE(0,"please check all sections sizes and heads is correct ........");
    //TRACE(2,"__coredump_section_start: %p length: 0x%x", __coredump_section_start, CORE_DUMP_SECTION_SIZE);
    //TRACE(2,"__ota_upgrade_log_start: %p length: 0x%x", __ota_upgrade_log_start, OTA_UPGRADE_LOG_SIZE);
    //TRACE(2,"__log_dump_start: %p length: 0x%x", __log_dump_start, LOG_DUMP_SECTION_SIZE);
    //TRACE(2,"__crash_dump_start: %p length: 0x%x", __crash_dump_start, CRASH_DUMP_SECTION_SIZE);
    //TRACE(2,"__custom_parameter_start: %p length: 0x%x", __custom_parameter_start, CUSTOM_PARAMETER_SECTION_SIZE);
    //TRACE(2,"__userdata_start: %p length: 0x%x", __userdata_start, USERDATA_SECTION_SIZE*2);
    //TRACE(2,"__aud_start: %p length: 0x%x", __aud_start, AUD_SECTION_SIZE);
    //TRACE(2,"__factory_start: %p length: 0x%x", __factory_start, FACTORY_SECTION_SIZE);
	//TRACE(1,"__sndkey_start: %p ", __sndkey_start);

    TRACE(0,"app_init\n");

#ifdef APP_TRACE_RX_ENABLE
    app_trace_rx_open();
    app_bt_cmd_init();
#endif

#ifdef __RPC_ENABLE__
extern int rpc_service_setup(void);
    rpc_service_setup();
#endif

#ifdef IBRT
    // init tws interface
    app_tws_if_init();
#endif // #ifdef IBRT

    nv_record_init();
    factory_section_init();

#ifdef AUDIO_OUTPUT_DC_AUTO_CALIB
    /* DAC DC and Dig gain calibration
     * This subroutine will try to load DC and gain calibration
     * parameters from user data section located at NV record;
     * If failed to loading parameters, It will open AF and start
     * to calibration DC and gain;
     * After accomplished new calibrtion parameters, it will try
     * to save these data into user data section at NV record if
     * CODEC_DAC_DC_NV_DATA defined;
     */
    codec_dac_dc_auto_load(true, false, false);
#endif
    app_sysfreq_req(APP_SYSFREQ_USER_APP_INIT, APP_SYSFREQ_104M);
#if defined(MCU_HIGH_PERFORMANCE_MODE)
   // TRACE(1,"sys freq calc : %d\n", hal_sys_timer_calc_cpu_freq(5, 0));
#endif
    list_init();
    nRet = app_os_init();
    if (nRet) {
        goto exit;
    }
#if OS_HAS_CPU_STAT
        cpu_usage_timer_id = osTimerCreate(osTimer(cpu_usage_timer), osTimerPeriodic, NULL);
        if (cpu_usage_timer_id != NULL) {
            osTimerStart(cpu_usage_timer_id, CPU_USAGE_TIMER_TMO_VALUE);
        }
#endif

    app_status_indication_init();

	//test jingdian
	//app_status_indication_set(APP_STATUS_INDICATION_POWERON);

   	#ifdef __BES2500_HAILINGWEI_CHC__
	app_hailingwei_variable_init();
	//app_reset_poweron_flag = true;
	#endif

#ifdef BTADDR_FOR_DEBUG
    gen_bt_addr_for_debug();
#endif

#ifdef FORCE_SIGNALINGMODE
    hal_sw_bootmode_clear(HAL_SW_BOOTMODE_TEST_NOSIGNALINGMODE);
    hal_sw_bootmode_set(HAL_SW_BOOTMODE_TEST_MODE | HAL_SW_BOOTMODE_TEST_SIGNALINGMODE);
#elif defined FORCE_NOSIGNALINGMODE
    hal_sw_bootmode_clear(HAL_SW_BOOTMODE_TEST_SIGNALINGMODE);
    hal_sw_bootmode_set(HAL_SW_BOOTMODE_TEST_MODE | HAL_SW_BOOTMODE_TEST_NOSIGNALINGMODE);
#endif

    if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_REBOOT_FROM_CRASH){
        hal_sw_bootmode_clear(HAL_SW_BOOTMODE_REBOOT_FROM_CRASH);
        TRACE(0,"Crash happened!!!");
    #ifdef VOICE_DATAPATH
        gsound_dump_set_flag(true);
    #endif
    }

    if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_REBOOT){
        hal_sw_bootmode_clear(HAL_SW_BOOTMODE_REBOOT);
        pwron_case = APP_POWERON_CASE_REBOOT;
        need_check_key = false;
        
        TRACE(0,"Initiative REBOOT happens!!!");
#ifdef USER_REBOOT_PLAY_MUSIC_AUTO
        if(hal_sw_bootmode_get() & HAL_SW_BOOTMODE_LOCAL_PLAYER)
        {
            hal_sw_bootmode_clear(HAL_SW_BOOTMODE_LOCAL_PLAYER);
            a2dp_need_to_play = true;
            TRACE(0,"a2dp_need_to_play = true");
        }
#endif
    }

    if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_TEST_MODE){
        hal_sw_bootmode_clear(HAL_SW_BOOTMODE_TEST_MODE);
        pwron_case = APP_POWERON_CASE_TEST;
        need_check_key = false;
        TRACE(0,"To enter test mode!!!");
    }

    #ifdef __BES2500_HAILINGWEI_CHC__
    if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_CLEAR_PAIRLIST_POWERON){
        hal_sw_bootmode_clear(HAL_SW_BOOTMODE_CLEAR_PAIRLIST_POWERON);
        pwron_case = APP_POWERON_CASE_NORMAL;
        need_check_key = false;
        clear_pairlist_reset_poweron_flag = true;
        TRACE(0,"clear pairlist reset poweron");
    }
    #endif

    #ifdef __BES2500_DAKANG_TWS_PAIR__
    if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_REBOOT_ENTER_TWS_PAIR){
        hal_sw_bootmode_clear(HAL_SW_BOOTMODE_REBOOT_ENTER_TWS_PAIR);
        pwron_case = APP_POWERON_CASE_NORMAL;
        need_check_key = false;
        enter_tws_pair_mode_flag = true;
        tws_pair_phone_pair_switch_flag = true;
        TRACE(0,"key enter tws pair mode");
    }

    if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_REBOOT_ENTER_PHONE_PAIR){
        hal_sw_bootmode_clear(HAL_SW_BOOTMODE_REBOOT_ENTER_PHONE_PAIR);
        pwron_case = APP_POWERON_CASE_NORMAL;
        need_check_key = false;
        enter_tws_pair_mode_flag = false;
        tws_pair_phone_pair_switch_flag = true;
        TRACE(0,"return phone pair mode");
    }
    #endif

	/*
    //if(pwron_case == APP_POWERON_CASE_REBOOT)
    {
    	app_status_indication_set(APP_STATUS_INDICATION_TESTMODE);
    	while(1)
    	{
    		;
    	}
    }
	*/

#ifdef BT_USB_AUDIO_DUAL_MODE
    usb_os_init();
#endif
    nRet = app_battery_open();
    TRACE(1,"BATTERY %d",nRet);
    if (pwron_case != APP_POWERON_CASE_TEST){
#ifdef USER_REBOOT_PLAY_MUSIC_AUTO
        TRACE(0,"hal_sw_bootmode_clear HAL_SW_BOOTMODE_LOCAL_PLAYER!!!!!!");
        hal_sw_bootmode_clear(HAL_SW_BOOTMODE_LOCAL_PLAYER);
#endif
        switch (nRet) {
            case APP_BATTERY_OPEN_MODE_NORMAL:
                nRet = 0;
                break;
            case APP_BATTERY_OPEN_MODE_CHARGING:
            	#ifndef __BES2500_HAILINGWEI_CHC__
                app_status_indication_set(APP_STATUS_INDICATION_CHARGING);
                #endif
                TRACE(0,"CHARGING!");
                app_battery_start();

                app_key_open(false);
                app_key_init_on_charging();
                nRet = 0;
#if defined(BT_USB_AUDIO_DUAL_MODE)
                usb_plugin = 1;
#elif defined(BTUSB_AUDIO_MODE)
                goto exit;
#endif
                break;
            case APP_BATTERY_OPEN_MODE_CHARGING_PWRON:
                TRACE(0,"CHARGING PWRON!");
#ifdef IBRT_SEARCH_UI
                is_charging_poweron=true;
#endif
                need_check_key = false;
                nRet = 0;
                break;
            case APP_BATTERY_OPEN_MODE_INVALID:
            default:
                nRet = -1;
                goto exit;
                break;
        }
    }

	//need_check_key = false;
	
    if (app_key_open(need_check_key)){
        TRACE(0,"PWR KEY DITHER!");
        nRet = -1;
        goto exit;
    }
	
    /*if((need_check_key)&&(app_battery_get_5V_status() == 0))
    {
    	//powerkey do not poweron
    	TRACE(0,"powerkey do not poweron!");
        nRet = -1;
        goto exit;
    }*/
	
    hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT);
    app_poweron_key_init();
#if defined(_AUTO_TEST_)
    AUTO_TEST_SEND("Power on.");
#endif
    app_bt_init();
    af_open();
    app_audio_open();
    app_audio_manager_open();
    app_overlay_open();

    nv_record_env_init();
    nvrec_dev_data_open();
    factory_section_open();
//    app_bt_connect2tester_init();
    nv_record_env_get(&nvrecord_env);

	TRACE(1,"AUTO_PLAY : %d",nvrecord_env->auto_play.auto_play);
	if((nvrecord_env->auto_play.auto_play == 1)&&(!is_charging_poweron))
	{
		a2dp_need_to_play = true;
		//app_status_indication_set(APP_STATUS_INDICATION_POWEROFF);
		//osDelay(2000);
	}


	if(a2dp_need_to_play)
	{
		//app_status_indication_set(APP_STATUS_INDICATION_TESTMODE);
		//osDelay(2000);
	}

	TRACE(1,"reset_shutdown %d",nvrecord_env->reset_shutdown.reset_shutdown);
	if(nvrecord_env->reset_shutdown.reset_shutdown == 1)
	{
		app_store_shut_down_proc(false);
		if((app_battery_get_5V_status() == 0)&&(!tws_pair_phone_pair_switch_flag))
		{
			TRACE(0,"charge full or low shutdown");
	        nRet = -1;
	        goto exit;
		}
	}
	/*
	else
	{
		if(!a2dp_need_to_play)
		{
			if((app_battery_get_5V_status() == 0)&&(!tws_pair_phone_pair_switch_flag)&&(pwron_case == APP_POWERON_CASE_NORMAL))
			{
				TRACE(0,"no reset pair no test do not poweron!");
				nRet = -1;
	        	goto exit;
			}
		}
	}
	*/

#ifdef BISTO_ENABLED
    nv_record_gsound_rec_init();
#endif

#ifdef BLE_ENABLE
    app_ble_mode_init();
    app_ble_customif_init();
#ifdef IBRT
    app_ble_force_switch_adv(BLE_SWITCH_USER_IBRT, false);
#endif // #ifdef IBRT
    app_ble_start_connectable_adv(100);
#endif

    audio_process_init();
#ifdef __PC_CMD_UART__
    app_cmd_open();
#endif
#ifdef AUDIO_DEBUG
    speech_tuning_init();
    #ifndef SNDP_DUMP_TX_DATA
    //audio_test_cmd_init();
    #endif
#endif
#if defined(ANC_ASSIST_ENABLED)
    app_anc_assist_init();
    app_voice_assist_wd_init();
    app_voice_assist_ai_voice_init();
#if defined(VOICE_ASSIST_WD_ENABLED)
    app_voice_assist_wd_open();
#endif
#endif
#ifdef ANC_APP
    app_anc_init();
#endif

	#ifdef __BES2500_HAILINGWEI_UART__
	app_communication_uart_open_moudle();
	#endif

#ifdef MEDIA_PLAYER_SUPPORT
    app_play_audio_set_lang(nvrecord_env->media_language.language);
#endif
    app_bt_stream_volume_ptr_update(NULL);

#ifdef __THIRDPARTY
    app_thirdparty_init();
    app_thirdparty_specific_lib_event_handle(THIRDPARTY_FUNC_NO2,THIRDPARTY_INIT);
#endif

    // TODO: freddie->unify all of the OTA modules
#if defined(IBRT_OTA)
    ota_flash_init();
#endif

#ifdef OTA_ENABLED
    /// init OTA common module
    ota_common_init_handler();
#endif // OTA_ENABLED

#ifdef IBRT
    // for TWS side decision, the last bit is 1:right, 0:left
    if (app_tws_is_unknown_side())
    {
        app_tws_set_side_from_addr(factory_section_get_bt_address());
    }
#endif


    btdrv_start_bt();
#if defined (__GMA_VOICE__) && defined(IBRT_SEARCH_UI)
	app_ibrt_reconfig_btAddr_from_nv();
#endif

	app_hailingwei_earheadset_fixed_type();


    if (pwron_case != APP_POWERON_CASE_TEST) {
        BesbtInit();
        app_wait_stack_ready();
        bt_drv_extra_config_after_init();
        bt_generate_ecdh_key_pair();
        app_bt_start_custom_function_in_bt_thread((uint32_t)0,
            0, (uint32_t)app_ibrt_init);
    }
#if defined(BLE_ENABLE) && defined(IBRT)
    app_ble_force_switch_adv(BLE_SWITCH_USER_IBRT, true);
#endif
    app_sysfreq_req(APP_SYSFREQ_USER_APP_INIT, APP_SYSFREQ_52M);
    TRACE(1,"\n\n\nbt_stack_init_done:%d\n\n\n", pwron_case);

	osDelay(5);
	
    #ifdef __BES2500_HAILINGWEI_CHC__
    // get left right type
    
    #endif

    if (pwron_case == APP_POWERON_CASE_REBOOT){
    	app_reboot_poweron_do_not_play_voice_flag = true;
		#ifndef __BES2500_HAILINGWEI_CHC__
        app_status_indication_set(APP_STATUS_INDICATION_POWERON);
#ifdef MEDIA_PLAYER_SUPPORT
        app_voice_report(APP_STATUS_INDICATION_POWERON, 0);
#endif
		#endif
        app_bt_sleep_init();

#if defined(IBRT_OTA)
        bes_ota_init();
#endif

#if defined(IBRT)
#ifdef IBRT_SEARCH_UI
		if(is_charging_poweron==false)
		{
			#ifdef __BES2500_HAILINGWEI_CHC__
			app_recover_led_display_timer_start_stop(true, LED_POWERON_RECOVER);
			#endif
			if(IBRT_UNKNOW == nvrecord_env->ibrt_mode.mode)
			{
				TRACE(0,"ibrt_ui_log:power on unknow mode");
				#ifdef __BES2500_HAILINGWEI_CHC__
				#ifdef __BES2500_DAKANG_TWS_PAIR__
				if(enter_tws_pair_mode_flag)
				{
					app_ibrt_enter_limited_mode();
					#ifdef __BES2500_HAILINGWEI_CHC__
					if(app_earheadset_fixed_type_get() == LEFT_EARHEADSET)
					{
						//master
						app_start_tws_serching_direactly();
					}
					else
					{
						//slave
						app_status_indication_set(APP_STATUS_INDICATION_CONNECTING);
						//app_slave_exit_tws_pairmode_timer_start_stop(true);
						#ifdef __BES2500_DAKANG_TWS_PAIR__
						slave_enter_tws_pairmode_flag = true;
						slave_enter_tws_pairmode_cnt = 0;
						app_tws_pairfail_enter_phone_pairmode_timer_start_stop(true);
						#endif
					}
					#endif
					tws_pair_phone_pair_switch_flag = false;
					enter_tws_pair_mode_flag = false;
				}
				else
				{
					enter_tws_pair_mode_enable_flag = true;
					app_ibrt_if_enter_freeman_pairing();
				}
				#else
				app_ibrt_enter_limited_mode();
				if(app_earheadset_fixed_type_get() == LEFT_EARHEADSET)
				{
					//master
					app_start_tws_serching_direactly();
				}
				else
				{
					//slave
					app_status_indication_set(APP_STATUS_INDICATION_CONNECTING);
				}
				#endif
				#else
				app_ibrt_enter_limited_mode();
				#endif
			}
			else
			{
				TRACE(1,"ibrt_ui_log:power on %d fetch out", nvrecord_env->ibrt_mode.mode);
				#ifdef __BES2500_DAKANG_TWS_PAIR__
				if(enter_tws_pair_mode_flag)
				{
					app_ibrt_enter_limited_mode();
					#ifdef __BES2500_HAILINGWEI_CHC__
					if(app_earheadset_fixed_type_get() == LEFT_EARHEADSET)
					{
						//master
						app_start_tws_serching_direactly();
					}
					else
					{
						//slave
						app_status_indication_set(APP_STATUS_INDICATION_CONNECTING);
						//app_slave_exit_tws_pairmode_timer_start_stop(true);
						#ifdef __BES2500_DAKANG_TWS_PAIR__
						slave_enter_tws_pairmode_flag = true;
						slave_enter_tws_pairmode_cnt = 0;
						app_tws_pairfail_enter_phone_pairmode_timer_start_stop(true);
						#endif
					}
					#endif
					tws_pair_phone_pair_switch_flag = false;
					enter_tws_pair_mode_flag = false;
				}
				else
				{
					if(tws_pair_phone_pair_switch_flag)
					{
						tws_pair_phone_pair_switch_flag = false;
						enter_tws_pair_mode_enable_flag = true;
						app_ibrt_if_enter_freeman_pairing();
					}
					else
					{
						app_ibrt_ui_event_entry(IBRT_FETCH_OUT_EVENT);
						#ifdef __BES2500_HAILINGWEI_CHC__
						app_status_indication_set(APP_STATUS_INDICATION_PAGESCAN);
						#endif
					}
				}
				#else
				app_ibrt_ui_event_entry(IBRT_FETCH_OUT_EVENT);
				#ifdef __BES2500_HAILINGWEI_CHC__
				app_status_indication_set(APP_STATUS_INDICATION_PAGESCAN);
				#endif
				#endif
			}
		}

#elif defined(IS_MULTI_AI_ENABLED)
        //when ama and bisto switch, earphone need reconnect with peer, master need reconnect with phone
        uint8_t box_action = app_ai_tws_reboot_get_box_action();
        if (box_action != 0xFF)
        {
            TRACE(2, "%s box_actionstate %d", __func__, box_action);
            app_ibrt_ui_event_entry(box_action|IBRT_SKIP_FALSE_TRIGGER_MASK);
        }
#endif
#else
        app_bt_accessmode_set(BTIF_BAM_NOT_ACCESSIBLE);
#endif

        app_key_init();
        app_battery_start();
#if defined(__BTIF_EARPHONE__) && defined(__BTIF_AUTOPOWEROFF__)
        app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
#endif

		#ifdef __PUTIN_PUTOUT_SWITCH_ROLE__
		app_putin_putout_switch_role_init();
		#endif


#if defined(__IAG_BLE_INCLUDE__) && defined(BTIF_BLE_APP_DATAPATH_SERVER)
        BLE_custom_command_init();
#endif
#ifdef __THIRDPARTY
        app_thirdparty_specific_lib_event_handle(THIRDPARTY_FUNC_NO1,THIRDPARTY_INIT);
        app_thirdparty_specific_lib_event_handle(THIRDPARTY_FUNC_NO1,THIRDPARTY_START);
        app_thirdparty_specific_lib_event_handle(THIRDPARTY_FUNC_NO2,THIRDPARTY_BT_CONNECTABLE);
        app_thirdparty_specific_lib_event_handle(THIRDPARTY_FUNC_NO3,THIRDPARTY_START);
#endif
#if defined( __BTIF_EARPHONE__) && defined(__BTIF_BT_RECONNECT__)
#if !defined(IBRT)
        app_bt_profile_connect_manager_opening_reconnect();
#endif
#endif
    }
#ifdef __ENGINEER_MODE_SUPPORT__
    else if(pwron_case == APP_POWERON_CASE_TEST){
        app_factorymode_set(true);
		#ifdef __BES2500_HAILINGWEI_UART__
		app_ibrt_uart_func_enable_close(false);
		#endif		
        #ifndef __BES2500_HAILINGWEI_CHC__
        app_status_indication_set(APP_STATUS_INDICATION_POWERON);
#ifdef MEDIA_PLAYER_SUPPORT
        app_voice_report(APP_STATUS_INDICATION_POWERON, 0);
#endif
		#endif
#ifdef __WATCHER_DOG_RESET__
        app_wdt_close();
#endif
        TRACE(0,"!!!!!ENGINEER_MODE!!!!!\n");
        nRet = 0;
        #ifndef __BES2500_HAILINGWEI_CHC__
        app_nvrecord_rebuild();
        #endif
        app_factorymode_key_init();
        if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_TEST_SIGNALINGMODE){
            hal_sw_bootmode_clear(HAL_SW_BOOTMODE_TEST_MASK);
            app_factorymode_bt_signalingtest(NULL, NULL);
        }
        if (hal_sw_bootmode_get() & HAL_SW_BOOTMODE_TEST_NOSIGNALINGMODE){
            hal_sw_bootmode_clear(HAL_SW_BOOTMODE_TEST_MASK);
            app_factorymode_bt_nosignalingtest(NULL, NULL);
        }
    }
#endif
    else{
        #ifdef __BES2500_HAILINGWEI_CHC__
        if(is_charging_poweron)
        {
        	if(app_battery_get_5V_status() == 0)
			{
				is_charging_poweron = false;
			}
        }

		if(clear_pairlist_reset_poweron_flag)
		{
			clear_pairlist_reset_poweron_flag = false;
			app_status_indication_set(APP_STATUS_INDICATION_CLEAR_PAIRLIST);
			app_voice_report(APP_STATUS_INDICATION_POWERON, 0);
		}
		else
		{
	        if(!is_charging_poweron)
	        {
	        	if(!tws_pair_phone_pair_switch_flag)
	        	{
					out_of_box_poweron_indication_flag = false;	
					app_status_indication_set(APP_STATUS_INDICATION_POWERON);
					app_voice_report(APP_STATUS_INDICATION_POWERON, 0);
					app_voice_report(APP_STATUS_INDICATION_TONE_DU, 0);
				}
	        }
        }
        #else
        app_status_indication_set(APP_STATUS_INDICATION_POWERON);
#ifdef MEDIA_PLAYER_SUPPORT
        app_voice_report(APP_STATUS_INDICATION_POWERON, 0);
#endif
		#endif
        if (need_check_key){
            pwron_case = app_poweron_wait_case();
        }
        else
        {
            pwron_case = APP_POWERON_CASE_NORMAL;
        }
        if (pwron_case != APP_POWERON_CASE_INVALID && pwron_case != APP_POWERON_CASE_DITHERING){
            TRACE(1,"power on case:%d\n", pwron_case);
            nRet = 0;
#ifndef __POWERKEY_CTRL_ONOFF_ONLY__
			#ifndef __BES2500_HAILINGWEI_CHC__
            app_status_indication_set(APP_STATUS_INDICATION_INITIAL);
            #endif
#endif
            app_bt_sleep_init();

#ifdef IBRT_OTA
            bes_ota_init();
#endif

#ifdef __INTERCONNECTION__
            app_interconnection_init();
#endif

#ifdef __INTERACTION__
            app_interaction_init();
#endif

#if defined(__IAG_BLE_INCLUDE__) && defined(BTIF_BLE_APP_DATAPATH_SERVER)
            BLE_custom_command_init();
#endif
#ifdef GFPS_ENABLED
             app_gfps_set_battery_info_acquire_handler(app_tell_battery_info_handler);
             app_gfps_set_battery_datatype(SHOW_UI_INDICATION);
#endif
            switch (pwron_case) {
                case APP_POWERON_CASE_CALIB:
                    break;
                case APP_POWERON_CASE_BOTHSCAN:
                	#ifdef __BES2500_HAILINGWEI_CHC__
					app_ibrt_if_enter_freeman_pairing();
                	#else
                    app_status_indication_set(APP_STATUS_INDICATION_BOTHSCAN);
#ifdef MEDIA_PLAYER_SUPPORT
                    app_voice_report(APP_STATUS_INDICATION_BOTHSCAN,0);
#endif
#if defined( __BTIF_EARPHONE__)
#if defined(IBRT)
#ifdef IBRT_SEARCH_UI
                    if(false==is_charging_poweron)
                        app_ibrt_enter_limited_mode();
#endif
#else
                    app_bt_accessmode_set(BTIF_BT_DEFAULT_ACCESS_MODE_PAIR);
#endif
#ifdef GFPS_ENABLED
                    app_enter_fastpairing_mode();
#endif
#if defined(__BTIF_AUTOPOWEROFF__)
                    app_start_10_second_timer(APP_PAIR_TIMER_ID);
#endif
#endif
#ifdef __THIRDPARTY
                    app_thirdparty_specific_lib_event_handle(THIRDPARTY_FUNC_NO2,THIRDPARTY_BT_DISCOVERABLE);
#endif
					#endif
                    break;
                case APP_POWERON_CASE_NORMAL:
#if defined( __BTIF_EARPHONE__ ) && !defined(__EARPHONE_STAY_BOTH_SCAN__)
#if defined(IBRT)
#ifdef IBRT_SEARCH_UI
					#ifdef __BES2500_HAILINGWEI_CHC__
					if(is_charging_poweron)
					{
						if(app_battery_get_5V_status() == 0)
						{
							is_charging_poweron = false;
						}
					}
					#endif
					
                    if(is_charging_poweron==false)
                    {
                    	#ifdef __BES2500_HAILINGWEI_CHC__
                    	app_recover_led_display_timer_start_stop(true, LED_POWERON_RECOVER);
                    	#endif
                        if(IBRT_UNKNOW == nvrecord_env->ibrt_mode.mode)
                        {
                            TRACE(0,"ibrt_ui_log:power on unknow mode");
							#ifdef __BES2500_HAILINGWEI_CHC__
							#ifdef __BES2500_DAKANG_TWS_PAIR__
                            if(enter_tws_pair_mode_flag)
                            {
	                            app_ibrt_enter_limited_mode();
	                            #ifdef __BES2500_HAILINGWEI_CHC__
	                            if(app_earheadset_fixed_type_get() == LEFT_EARHEADSET)
	                            {
	                            	//master
	                            	app_start_tws_serching_direactly();
	                            }
	                            else
	                            {
	                            	//slave
	                            	app_status_indication_set(APP_STATUS_INDICATION_CONNECTING);
	                            	//app_slave_exit_tws_pairmode_timer_start_stop(true);
	                            	#ifdef __BES2500_DAKANG_TWS_PAIR__
	                            	slave_enter_tws_pairmode_flag = true;
							    	slave_enter_tws_pairmode_cnt = 0;
	                            	app_tws_pairfail_enter_phone_pairmode_timer_start_stop(true);
	                            	#endif
	                            }
	                            #endif
								tws_pair_phone_pair_switch_flag = false;
	                            enter_tws_pair_mode_flag = false;
                            }
                            else
                            {
                            	enter_tws_pair_mode_enable_flag = true;
                            	app_ibrt_if_enter_freeman_pairing();
                            }
                            #else
							app_ibrt_enter_limited_mode();
							if(app_earheadset_fixed_type_get() == LEFT_EARHEADSET)
				            {
				            	//master
				            	app_start_tws_serching_direactly();
				            }
				            else
				            {
				            	//slave
				            	app_status_indication_set(APP_STATUS_INDICATION_CONNECTING);
				            }
                            #endif
                            #else
                            app_ibrt_enter_limited_mode();
                            #endif
                        }
                        else
                        {
                            TRACE(1,"ibrt_ui_log:power on %d fetch out", nvrecord_env->ibrt_mode.mode);
                         	#ifdef __BES2500_DAKANG_TWS_PAIR__
                            if(enter_tws_pair_mode_flag)
                            {
	                            app_ibrt_enter_limited_mode();
	                            #ifdef __BES2500_HAILINGWEI_CHC__
	                            if(app_earheadset_fixed_type_get() == LEFT_EARHEADSET)
	                            {
	                            	//master
	                            	app_start_tws_serching_direactly();
	                            }
	                            else
	                            {
	                            	//slave
	                            	app_status_indication_set(APP_STATUS_INDICATION_CONNECTING);
	                            	//app_slave_exit_tws_pairmode_timer_start_stop(true);
	                            	#ifdef __BES2500_DAKANG_TWS_PAIR__
	                            	slave_enter_tws_pairmode_flag = true;
							    	slave_enter_tws_pairmode_cnt = 0;
	                            	app_tws_pairfail_enter_phone_pairmode_timer_start_stop(true);
	                            	#endif
	                            }
	                            #endif
								tws_pair_phone_pair_switch_flag = false;
	                            enter_tws_pair_mode_flag = false;
                            }
                            else
                            {
                            	if(tws_pair_phone_pair_switch_flag)
                            	{
                            		tws_pair_phone_pair_switch_flag = false;
                            		enter_tws_pair_mode_enable_flag = true;
                            		app_ibrt_if_enter_freeman_pairing();
                            	}
                            	else
                            	{
									app_ibrt_ui_event_entry(IBRT_FETCH_OUT_EVENT);
									#ifdef __BES2500_HAILINGWEI_CHC__
									app_status_indication_set(APP_STATUS_INDICATION_PAGESCAN);
									#endif
								}
                            }
                            #else
                            app_ibrt_ui_event_entry(IBRT_FETCH_OUT_EVENT);
                            #ifdef __BES2500_HAILINGWEI_CHC__
                            app_status_indication_set(APP_STATUS_INDICATION_PAGESCAN);
							#endif
							#endif
                        }
                    }
#elif defined(IS_MULTI_AI_ENABLED)
                    //when ama and bisto switch, earphone need reconnect with peer, master need reconnect with phone
                    //app_ibrt_ui_event_entry(IBRT_OPEN_BOX_EVENT);
                    //TRACE(1,"ibrt_ui_log:power on %d fetch out", nvrecord_env->ibrt_mode.mode);
                    //app_ibrt_ui_event_entry(IBRT_FETCH_OUT_EVENT);
#endif
#else
                    app_bt_accessmode_set(BTIF_BAM_NOT_ACCESSIBLE);
#endif
#endif
                    /* FALL THROUGH*/
                case APP_POWERON_CASE_REBOOT:
                case APP_POWERON_CASE_ALARM:
                default:
                	#ifndef __BES2500_HAILINGWEI_CHC__
                    app_status_indication_set(APP_STATUS_INDICATION_PAGESCAN);
                    #endif
#if defined( __BTIF_EARPHONE__) && defined(__BTIF_BT_RECONNECT__) && !defined(IBRT)
                    app_bt_profile_connect_manager_opening_reconnect();
#endif
#ifdef __THIRDPARTY
                    app_thirdparty_specific_lib_event_handle(THIRDPARTY_FUNC_NO2,THIRDPARTY_BT_CONNECTABLE);
#endif

                    break;
            }
            if (need_check_key)
            {
#ifndef __POWERKEY_CTRL_ONOFF_ONLY__
                app_poweron_wait_finished();
#endif
            }
	
			#ifdef __BES2500_IQS269_TOUCH__
			app_putout_touch_send_flag = true;
			app_putout_touch_send_cnt = 0;
			app_putin_touch_send_flag = true;
			app_putin_touch_send_cnt = 0;
			app_earheadset_plugin_plugout_detect_timer_start_stop(true);
			app_touch_init_reset();
            #endif

    		#ifdef __BES2500_HAILINGWEI_CHC__
			if(is_charging_poweron)
			{
				app_battery_charging_flag = true;
				app_battery_plugin_init_flag = false;
				app_battery_plugout_init_flag = true;
				app_battery_send_reconnect_flag = true;
				//app_reset_poweron_flag = false;
				//TRACE(0,"11111111");
				//hal_sw_bootmode_set(HAL_SW_BOOTMODE_CUSTOM_CHARGING_RESET);
				//TRACE(1," bootmode %d",hal_sw_bootmode_get());
				app_store_shut_down_proc(true);
				#ifdef __BES2500_HAILINGWEI_UART__
				app_ibrt_uart_func_enable_close(false);
				#endif
				app_battery_charge_enable_control(true);
				app_status_indication_set(APP_STATUS_INDICATION_CHARGING);
				app_battery_start();
				app_key_init_on_charging();
				pmu_sleep_en(false);
			}
			else
			{
				app_battery_charging_flag = false;
				app_battery_plugout_init_flag = false;
				app_battery_send_reconnect_flag = false;
				app_battery_plugin_init_flag = true;
				app_key_init();
				app_battery_start();
				app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
			}

			#ifdef __PUTIN_PUTOUT_SWITCH_ROLE__
		    app_putin_putout_switch_role_init();
		    #endif
            #else
            app_key_init();
            app_battery_start();
#if defined(__BTIF_EARPHONE__) && defined(__BTIF_AUTOPOWEROFF__)
            app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
#endif
			#endif
#ifdef __THIRDPARTY
            app_thirdparty_specific_lib_event_handle(THIRDPARTY_FUNC_NO1,THIRDPARTY_INIT);
            app_thirdparty_specific_lib_event_handle(THIRDPARTY_FUNC_NO1,THIRDPARTY_START);
            app_thirdparty_specific_lib_event_handle(THIRDPARTY_FUNC_NO3,THIRDPARTY_START);
#endif

#ifdef RB_CODEC
            rb_ctl_init();
#endif

			//ASSERT(0, "Unknown ldac frame format:  !!!!!");
        }else{
            af_close();
            app_key_close();
            nRet = -1;
        }
    }
exit:

#ifdef IS_MULTI_AI_ENABLED
    app_ai_tws_clear_reboot_box_state();
#endif

#ifdef __BT_DEBUG_TPORTS__
    {
       extern void bt_enable_tports(void);
       bt_enable_tports();
       //hal_iomux_tportopen();
    }
#endif

#ifdef BT_USB_AUDIO_DUAL_MODE
    if(usb_plugin)
    {
        btusb_switch(BTUSB_MODE_USB);
    }
    else
    {
        btusb_switch(BTUSB_MODE_BT);
    }
#else //BT_USB_AUDIO_DUAL_MODE
#if defined(BTUSB_AUDIO_MODE)
    if(pwron_case == APP_POWERON_CASE_CHARGING) {
        app_wdt_close();
        af_open();
        app_key_handle_clear();
        app_usb_key_init();
        app_usbaudio_entry();
    }

#endif // BTUSB_AUDIO_MODE
#endif // BT_USB_AUDIO_DUAL_MODE
    app_sysfreq_req(APP_SYSFREQ_USER_APP_INIT, APP_SYSFREQ_32K);

    return nRet;
}

#endif /* APP_TEST_MODE */


#ifdef __BES2500_I2C_COMMUNICATION__
/*********************************************************************************
I2C
**********************************************************************************/
#define IQS269_I2C_SCL_HIGH     hal_gpio_pin_set((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[0].pin)
#define IQS269_I2C_SCL_LOW      hal_gpio_pin_clr((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[0].pin)

#define IQS269_I2C_SDA_HIGH     hal_gpio_pin_set((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[1].pin)
#define IQS269_I2C_SDA_LOW      hal_gpio_pin_clr((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[1].pin)


// i2c delay
void IQS269_I2C_delay(void)
{
	hal_sys_timer_delay_us(5);
}

// i2c init
void IQS269_I2C_init(void)
{
	hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)iqs269_i2c_cfg, sizeof(iqs269_i2c_cfg)/sizeof(struct HAL_IOMUX_PIN_FUNCTION_MAP));
	hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[0].pin, HAL_GPIO_DIR_OUT, 1);
	hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[1].pin, HAL_GPIO_DIR_OUT, 1);
}

// SCL set input
void IQS269_I2C_SCL_set_input(void)
{
	hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[0].pin, HAL_GPIO_DIR_IN, 1);
}

// SCL set output
void IQS269_I2C_SCL_set_output(bool high_en)
{
	if(high_en)
	{
		hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[0].pin, HAL_GPIO_DIR_OUT, 1);
	}
	else
	{
		hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[0].pin, HAL_GPIO_DIR_OUT, 0);
	}
}

// get SCL input value
uint8_t IQS269_I2C_SCL_get_input_value(void)
{
	return hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[0].pin);
}


// SDA set input
void IQS269_I2C_SDA_set_input(void)
{
	hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[1].pin, HAL_GPIO_DIR_IN, 1);
}

// SDA set output
void IQS269_I2C_SDA_set_output(bool high_en)
{
	if(high_en)
	{
		hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[1].pin, HAL_GPIO_DIR_OUT, 1);
	}
	else
	{
		hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[1].pin, HAL_GPIO_DIR_OUT, 0);
	}
}

// get SDA input value ========= ack
uint8_t IQS269_I2C_SDA_get_input_value(void)
{
	return hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)iqs269_i2c_cfg[1].pin);
}

//(wait scl turn high)
void app_scl_wait_turn_high_level(void)
{
	uint16_t get_count = 500;
	
	IQS269_I2C_SCL_set_input();

	while(get_count)
	{
		if(IQS269_I2C_SCL_get_input_value())
		{
			break;
		}

		get_count--;

		if(get_count == 0)
		{
			break;
		}
		hal_sys_timer_delay_us(2);
	}

	//TRACE(1,"scl_level %d",IQS269_I2C_SCL_get_input_value());
}

// i2c start
void IQS269_I2C_start(void)
{

	IQS269_I2C_SDA_set_output(true);
	IQS269_I2C_SCL_set_output(true);
		
	IQS269_I2C_SDA_HIGH;
	#ifdef __BES2500_I2C_STANDARD__
	IQS269_I2C_SCL_HIGH;
	#else
	//IQS269_I2C_SCL_HIGH;
	app_scl_wait_turn_high_level();
	#endif
	IQS269_I2C_delay();
	IQS269_I2C_SDA_LOW;
	IQS269_I2C_delay();
	#ifdef __BES2500_I2C_STANDARD__
	IQS269_I2C_SCL_LOW;
	#else
	//IQS269_I2C_SCL_LOW;
	IQS269_I2C_SCL_set_output(false);
	#endif
	IQS269_I2C_delay();
}

// i2c stop 
void IQS269_I2C_stop(void)
{
	IQS269_I2C_SDA_set_output(false);
		
	//IQS269_I2C_SCL_LOW;
	IQS269_I2C_SCL_set_output(false);
	IQS269_I2C_delay();
	IQS269_I2C_SDA_LOW;
	IQS269_I2C_delay();
	#ifdef __BES2500_I2C_STANDARD__
	IQS269_I2C_SCL_HIGH;
	#else
	//IQS269_I2C_SCL_HIGH;
	app_scl_wait_turn_high_level();
	#endif
	IQS269_I2C_delay();
	IQS269_I2C_SDA_HIGH;
	IQS269_I2C_delay();
}

// i2c send high level
void IQS269_I2C_send_high(void)
{
	IQS269_I2C_SDA_HIGH;
	//IQS269_I2C_SCL_LOW;
	IQS269_I2C_delay();
	#ifdef __BES2500_I2C_STANDARD__
	IQS269_I2C_SCL_HIGH;
	#else
	//IQS269_I2C_SCL_HIGH;
	app_scl_wait_turn_high_level();
	#endif
	IQS269_I2C_delay();
	//IQS269_I2C_SCL_LOW;
	IQS269_I2C_SCL_set_output(false);
	IQS269_I2C_delay();
}

// i2c send low level
void IQS269_I2C_send_low(void)
{
	IQS269_I2C_SDA_LOW;
	//IQS269_I2C_SCL_LOW;
	IQS269_I2C_delay();
	#ifdef __BES2500_I2C_STANDARD__
	IQS269_I2C_SCL_HIGH;
	#else
	//IQS269_I2C_SCL_HIGH;
	app_scl_wait_turn_high_level();
	#endif
	IQS269_I2C_delay();
	//IQS269_I2C_SCL_LOW;
	IQS269_I2C_SCL_set_output(false);
	IQS269_I2C_delay();
}

bool IQS269_I2C_get_Ackknowledge(void)
{
	bool ack;
	uint8_t cnt = 200;
	
	IQS269_I2C_SDA_set_input();
	IQS269_I2C_delay();
	#ifdef __BES2500_I2C_STANDARD__
	IQS269_I2C_SCL_HIGH;
	#else
	//IQS269_I2C_SCL_HIGH;
	IQS269_I2C_SCL_set_input();
	#endif

	while(cnt)
	{
		hal_sys_timer_delay_us(2);
		if(IQS269_I2C_SDA_get_input_value())
		{
			ack = false;
		}
		else
		{
			ack = true;
			break;
		}

		cnt--;
	}

	IQS269_I2C_delay();
	//IQS269_I2C_SCL_LOW;
	IQS269_I2C_SCL_set_output(false);
	//IQS269_I2C_SDA_set_output();
	//IQS269_I2C_SDA_LOW;
	#ifdef __BES2500_I2C_STANDARD__
	IQS269_I2C_delay();
	IQS269_I2C_SDA_set_output(true);
	#endif
	TRACE(1,"IQS269_I2C_get_Ackknowledge ACK = %d",ack);
	return ack;
}


void IQS269_I2C_send_Ackknowledge(uint8_t ack)
{
	IQS269_I2C_SDA_set_output(false);
	IQS269_I2C_delay();
	
	if(ack == 0)
	{
		IQS269_I2C_SDA_HIGH;
	}
	else
	{
		IQS269_I2C_SDA_LOW;
	}
	IQS269_I2C_delay();
	#ifdef __BES2500_I2C_STANDARD__
	IQS269_I2C_SCL_HIGH;
	#else
	//IQS269_I2C_SCL_HIGH;
	app_scl_wait_turn_high_level();
	#endif
	IQS269_I2C_delay();
	//IQS269_I2C_SCL_LOW;
	IQS269_I2C_SCL_set_output(false);
}

void IQS269_I2C_WriteByte(uint8_t write_data)
{
	uint8_t i;
	
	IQS269_I2C_SDA_set_output(false);
	
	for(i = 0; i < 8; i++)
	{
		//TRACE("%0x",write_data);
		//TRACE("%d",i);
		if((write_data << i) & 0x80)
		{
			//TRACE("HIGH");
			IQS269_I2C_send_high();
		}
		else
		{
			//TRACE("LOW");
			IQS269_I2C_send_low();
		}
	}
}

uint8_t SC7A20_I2C_ReadByte(void)
{
	uint8_t read_data = 0;
	uint8_t i;

	IQS269_I2C_SDA_set_input();
	#ifndef __BES2500_I2C_STANDARD__
	IQS269_I2C_delay();
	#endif
	
	for(i = 0; i < 8; i++)// 0 1 2 3 4 5 6 7
	{
		read_data <<= 1;

		#ifdef __BES2500_I2C_STANDARD__
		IQS269_I2C_SCL_HIGH;
		#else
		//IQS269_I2C_SCL_HIGH;
		app_scl_wait_turn_high_level();
		#endif
		IQS269_I2C_delay();
		if(IQS269_I2C_SDA_get_input_value())
		{
			read_data++;
		}
		TRACE(1,"****read_data = 0x%x", read_data);
		//IQS269_I2C_SCL_LOW;
		#ifdef __BES2500_I2C_STANDARD__
		IQS269_I2C_delay();
		IQS269_I2C_SCL_set_output(false);
		IQS269_I2C_delay();
		#else
		IQS269_I2C_SCL_set_output(false);
		IQS269_I2C_delay();
		#endif
	}

	IQS269_I2C_SDA_set_output(false);
	IQS269_I2C_delay();

	return read_data;
}


bool IQS269_I2C_get_ack_ready(void)
{
	return true;
	#if 0
	uint16_t cnt = 1000;
	
	IQS269_I2C_SCL_set_input();

	while(cnt)
	{
		if(IQS269_I2C_SCL_get_input_value())
		{
			break;
		}
		
		hal_sys_timer_delay_us(2);
		cnt--;
	}

	
	if(cnt == 0)
	{
		TRACE("SCL IS LOW");
		return false;
	}
	
	IQS269_I2C_SCL_set_output();
	TRACE("SCL IS high");
	return true;
	#endif
}


uint8_t IQS269_I2C_Write_Byte(uint8_t device_addr, uint8_t reg_addr, uint8_t* write_data_buffer, uint8_t length, bool stop_en)
{
	uint8_t ret = 0;
	
	if(IQS269_I2C_get_ack_ready())
	{
		IQS269_I2C_start();
		IQS269_I2C_WriteByte(device_addr);
		if(IQS269_I2C_get_ack_ready())
		{
			if(IQS269_I2C_get_Ackknowledge())
			{
				IQS269_I2C_WriteByte(reg_addr);
				if(IQS269_I2C_get_ack_ready())
				{
					if(IQS269_I2C_get_Ackknowledge())
					{
						do
						{
							IQS269_I2C_WriteByte(*write_data_buffer);
							if(!IQS269_I2C_get_Ackknowledge())
							{
								break;
							}
							write_data_buffer++;
							length--;
						}while(length != 0);

						if(length == 0)
						{
							ret = 1;
						}
						else
						{
							ret = 0;
						}
					}
				}
				else
				{
					ret = 0;
				}
			}
		}
		
		if(stop_en)
		{
			IQS269_I2C_stop();
		}
	}
	return ret;
}

uint8_t IQS269_I2C_Read_Byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *read_data, uint8_t length, bool stop_en)
{
	uint8_t ret = 0;
	uint8_t *p;
	//TRACE(0,"IQS269_I2C_Read_Byte");
	p = read_data;
	if(IQS269_I2C_get_ack_ready())
	{
		IQS269_I2C_start();
		IQS269_I2C_WriteByte(device_addr);

		if(IQS269_I2C_get_ack_ready())
		{
			if(IQS269_I2C_get_Ackknowledge())
			{
				IQS269_I2C_WriteByte(reg_addr);
				if(IQS269_I2C_get_ack_ready())
				{
					if(IQS269_I2C_get_Ackknowledge())
					{
						IQS269_I2C_start();
						IQS269_I2C_WriteByte(device_addr+0x01);
						if(IQS269_I2C_get_Ackknowledge())
						{
							if(IQS269_I2C_get_ack_ready())
							{
								while(length != 0)
								{
									if(IQS269_I2C_get_ack_ready())
									{
										if(length == 1)
										{
											*p = SC7A20_I2C_ReadByte();
											IQS269_I2C_send_Ackknowledge(0);
										}
										else
										{
											*p = SC7A20_I2C_ReadByte();
											IQS269_I2C_send_Ackknowledge(1);
										}
										p++;
										length--;
									}
								}
							}
						}
						ret = 1;
					}
				}
				else
				{
					ret = 0;
				}
			}
		}
		if(stop_en)
		{
			IQS269_I2C_stop();
		}
	}
	
	return ret;
}
#endif

#ifdef __BES2500_I2C_STANDARD__
/********************************************************************************
charge and mcu
*********************************************************************************/

void app_read_data(void)
{
	uint8_t read_data = 0;
	
	IQS269_I2C_init();
	IQS269_I2C_Read_Byte(0xc2, 0x00, &read_data, 1, true);
	TRACE(1,"%s 1 read data 0x%x",__func__, read_data);
	IQS269_I2C_Read_Byte(0xc2, 0x01, &read_data, 1, true);
	TRACE(1,"%s 2 read data 0x%x",__func__, read_data);
	IQS269_I2C_Read_Byte(0xc2, 0x02, &read_data, 1, true);
	
	TRACE(1,"%s 3 read data 0x%x",__func__, read_data);
}

void app_ICP1205_charge_init(void)
{
					//对应寄存器         0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x07 0x08 0x09 0x0a 0x0b 0x0c 0x0d 0x0e 0x0f
	uint8_t Icp1205_init_set[16] = {0X23,0x32,0x00,0x99,0x00,0x00,0x00,0x00,0xf0,0xf0,0x03,0x07,0x00,0x00,0x00,0x00};
	uint8_t Icp1205_read[16] = {0};
	
	IQS269_I2C_init();

	IQS269_I2C_Write_Byte(ICP1205_DEVICE_ADDRESS, 0x00, &Icp1205_init_set[0], 16, true);
	IQS269_I2C_Read_Byte(ICP1205_DEVICE_ADDRESS, 0x00, &Icp1205_read[0], 16, true);
	DUMP8(" 0x%x ", &Icp1205_read[0], 16);
}

#endif


/*********************************************************************************
3rd touch drive IQS269
**********************************************************************************/
#ifdef __BES2500_IQS269_TOUCH__
uint8_t res=0;
uint8_t temp[42];
IQS62x_Data_t iqs_data;// = {0};
bool IQS269_init_success_flag = false;
uint8_t IQS269_init_step_index = APP_INIT_RESET;


void init_IQS269(void)
{
	
	uint8_t transferBytes[14];

	IQS269_init_success_flag = false;

	#if 0
	res = IQS269_I2C_Read_Byte(I2C_IQS62x_ADDR, IQS269_MM_PRODUCT_NUMBER, &temp[0], 2, true);
	#else
	
	//IQS269_I2C_init();
	
	//Wait for comms window to open
	//while(I2C_RDY_IN)
	{

	}

	//do
	//{
		//transferBytes[0] = 0x40;
		
		//res |= IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, 0xF2 , &transferBytes[0], 1);
		//i2c_start();
		//res = i2c_read_register(I2C_IQS62x_ADDR, IQS269_MM_PRODUCT_NUMBER, &temp[0], 2);
		//IQS269_I2C_start();
		transferBytes[0] = 0x40;
		#ifdef __BES2500_I2C_COMMUNICATION__
		IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, 0xF2 , &transferBytes[0], 1, false);
		
		res = IQS269_I2C_Read_Byte(I2C_IQS62x_ADDR, IQS269_MM_PRODUCT_NUMBER, &temp[0], 2, false);
		#endif
		TRACE(1,"TMP[0] = %d",temp[0]);
		TRACE(1,"TMP[1] = %d",temp[1]);


		transferBytes[0] = PMU_GENERAL_SETTINGS;
		transferBytes[1] = I2C_GENERAL_SETTINGS | 0x01; //ack reset
		transferBytes[2] = SYS_CHB_ACTIVE;
		transferBytes[3] = ACF_LTA_FILTER_SETTINGS;
		transferBytes[4] = LTA_CHB_RESEED_ENABLED;
		transferBytes[5] = UIS_GLOBAL_EVENTS_MASK;
		transferBytes[6] = PMU_REPORT_RATE_NP;
		transferBytes[7] = PMU_REPORT_RATE_LP;
		transferBytes[8] = PMU_REPORT_RATE_ULP;
		transferBytes[9] = PMU_MODE_TIMOUT;
		transferBytes[10] = I2C_WINDOW_TIMEOUT;
		transferBytes[11] = LTA_HALT_TIMEOUT;
	  // Now write the Device Settings settings to the device.

		//i2c_repeat_start();
		//res |= i2c_write_register(I2C_IQS62x_ADDR, IQS269_MM_PMU_SETTINGS , &transferBytes[0], 12);
		//IQS269_I2C_start();
		#ifdef __BES2500_I2C_COMMUNICATION__
		res |= IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, IQS269_MM_PMU_SETTINGS , &transferBytes[0], 12, false);
		#endif

	  // Save the next 12 bytes of the Global Settings to the transferBytes aray.
		// Addresses 0x86 Byte0 to 0x8B Byte1.
		transferBytes[0] = PXS_GENERAL_SETTINGS0;
		transferBytes[1] = PXS_GENERAL_SETTINGS1;
		transferBytes[2] = UIS_ABSOLUTE_CAPACITANCE;
		transferBytes[3] = UIS_DCF_GENERAL_SETTINGS;
		transferBytes[4] = GEM_CHB_BLOCK_NFOLLOW;
		transferBytes[5] = MOV_CHB_MOVEMENT_CHANNEL;
		transferBytes[6] = UIS_CHB_SLIDER0 ;
		transferBytes[7] = UIS_CHB_SLIDER1;
		transferBytes[8] = UIS_GESTURE_TAP_TIMEOUT;
		transferBytes[9] = UIS_GESTURE_SWIPE_TIMEOUT;
		transferBytes[10] = UIS_GESTURE_THRESHOLD;
		transferBytes[11] = LTA_CHB_RESEED ;
	  // Now write the Device Settings settings to the device.

		//i2c_repeat_start();
		//res |= i2c_write_register(I2C_IQS62x_ADDR, IQS269_MM_PXS_GLBL_SETTINGS_0 , &transferBytes[0], 12);
		//IQS269_I2C_start();
		#ifdef __BES2500_I2C_COMMUNICATION__
		res |= IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, IQS269_MM_PXS_GLBL_SETTINGS_0 , &transferBytes[0], 12, false);
		#endif
	  // Save the Channel 0 settings to the transferBytes array.
		// Addresses 0x8C Byte0 to 0x92 Byte1.
		transferBytes[0] = PXS_CRXSEL_CH0;
		transferBytes[1] = PXS_CTXSEL_CH0;
		transferBytes[2] = PXS_PROXCTRL_CH0;
		transferBytes[3] = PXS_PROXCFG0_CH0;
		transferBytes[4] = PXS_PROXCFG1_TESTREG0_CH0;
		transferBytes[5] = ATI_BASE_AND_TARGET_CH0;
		transferBytes[6] = ATI_MIRROR_CH0;
		transferBytes[7] = ATI_PCC_CH0;
		transferBytes[8] = PXS_PROX_THRESHOLD_CH0;
		transferBytes[9] = PXS_TOUCH_THRESHOLD_CH0;
		transferBytes[10] = PXS_DEEP_THRESHOLD_CH0;
		transferBytes[11] = PXS_HYSTERESIS_CH0 ;
		transferBytes[12] = DCF_CHB_ASSOCIATION_CH0;
		transferBytes[13] = DCF_WEIGHT_CH0;
		// Now write the Channel 0 settings to the device.

		//i2c_repeat_start();
		//res |= i2c_write_register(I2C_IQS62x_ADDR, IQS269_MM_CH0_SELECT_CRX , &transferBytes[0], 14);
		//IQS269_I2C_start();
		#ifdef __BES2500_I2C_COMMUNICATION__
		res |= IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, IQS269_MM_CH0_SELECT_CRX , &transferBytes[0], 14, false);
		#endif

	  // Save the Channel 1 settings to the transferBytes array.
		// Addresses 0x93 Byte0 to 0x99 Byte1.
		transferBytes[0] = PXS_CRXSEL_CH1;
		transferBytes[1] = PXS_CTXSEL_CH1;
		transferBytes[2] = PXS_PROXCTRL_CH1;
		transferBytes[3] = PXS_PROXCFG0_CH1;
		transferBytes[4] = PXS_PROXCFG1_TESTREG0_CH1;
		transferBytes[5] = ATI_BASE_AND_TARGET_CH1;
		transferBytes[6] = ATI_MIRROR_CH1;
		transferBytes[7] = ATI_PCC_CH1;
		transferBytes[8] = PXS_PROX_THRESHOLD_CH1;
		transferBytes[9] = PXS_TOUCH_THRESHOLD_CH1;
		transferBytes[10] = PXS_DEEP_THRESHOLD_CH1;
		transferBytes[11] = PXS_HYSTERESIS_CH1 ;
		transferBytes[12] = DCF_CHB_ASSOCIATION_CH1;
		transferBytes[13] = DCF_WEIGHT_CH1;
		// Now write the Channel 0 settings to the device.

		//i2c_repeat_start();
		//res |= i2c_write_register(I2C_IQS62x_ADDR, IQS269_MM_CH1_SELECT_CRX , &transferBytes[0], 14);
		//IQS269_I2C_start();
		#ifdef __BES2500_I2C_COMMUNICATION__
		res |= IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, IQS269_MM_CH1_SELECT_CRX , &transferBytes[0], 14, false);
		#endif

	  // Save the Channel 2 settings to the transferBytes array.
		// Addresses 0x9A Byte0 to 0xA0 Byte1.
		transferBytes[0] = PXS_CRXSEL_CH2;
		transferBytes[1] = PXS_CTXSEL_CH2;
		transferBytes[2] = PXS_PROXCTRL_CH2;
		transferBytes[3] = PXS_PROXCFG0_CH2;
		transferBytes[4] = PXS_PROXCFG1_TESTREG0_CH2;
		transferBytes[5] = ATI_BASE_AND_TARGET_CH2;
		transferBytes[6] = ATI_MIRROR_CH2;
		transferBytes[7] = ATI_PCC_CH2;
		transferBytes[8] = PXS_PROX_THRESHOLD_CH2;
		transferBytes[9] = PXS_TOUCH_THRESHOLD_CH2;
		transferBytes[10] = PXS_DEEP_THRESHOLD_CH2;
		transferBytes[11] = PXS_HYSTERESIS_CH2 ;
		transferBytes[12] = DCF_CHB_ASSOCIATION_CH2;
		transferBytes[13] = DCF_WEIGHT_CH2;
		// Now write the Channel 0 settings to the device.

		//i2c_repeat_start();
		//res |= i2c_write_register(I2C_IQS62x_ADDR, IQS269_MM_CH2_SELECT_CRX , &transferBytes[0], 14);
		//IQS269_I2C_start();
		#ifdef __BES2500_I2C_COMMUNICATION__
		res |= IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, IQS269_MM_CH2_SELECT_CRX , &transferBytes[0], 14, false);
		#endif

	  // Save the Channel 3 settings to the transferBytes array.
		// Addresses 0xA1 Byte0 to 0xA7 Byte1.
		transferBytes[0] = PXS_CRXSEL_CH3;
		transferBytes[1] = PXS_CTXSEL_CH3;
		transferBytes[2] = PXS_PROXCTRL_CH3;
		transferBytes[3] = PXS_PROXCFG0_CH3;
		transferBytes[4] = PXS_PROXCFG1_TESTREG0_CH3;
		transferBytes[5] = ATI_BASE_AND_TARGET_CH3;
		transferBytes[6] = ATI_MIRROR_CH3;
		transferBytes[7] = ATI_PCC_CH3;
		transferBytes[8] = PXS_PROX_THRESHOLD_CH3;
		transferBytes[9] = PXS_TOUCH_THRESHOLD_CH3;
		transferBytes[10] = PXS_DEEP_THRESHOLD_CH3;
		transferBytes[11] = PXS_HYSTERESIS_CH3 ;
		transferBytes[12] = DCF_CHB_ASSOCIATION_CH3;
		transferBytes[13] = DCF_WEIGHT_CH3;
		// Now write the Channel 0 settings to the device.

		//i2c_repeat_start();
		//res |= i2c_write_register(I2C_IQS62x_ADDR, IQS269_MM_CH3_SELECT_CRX , &transferBytes[0], 14);
		#ifdef __BES2500_I2C_COMMUNICATION__
		res |= IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, IQS269_MM_CH3_SELECT_CRX , &transferBytes[0], 14, false);
		#endif

	  // Save the Channel 4 settings to the transferBytes array.
		// Addresses 0xA8 Byte0 to 0xAE Byte1.
		transferBytes[0] = PXS_CRXSEL_CH4;
		transferBytes[1] = PXS_CTXSEL_CH4;
		transferBytes[2] = PXS_PROXCTRL_CH4;
		transferBytes[3] = PXS_PROXCFG0_CH4;
		transferBytes[4] = PXS_PROXCFG1_TESTREG0_CH4;
		transferBytes[5] = ATI_BASE_AND_TARGET_CH4;
		transferBytes[6] = ATI_MIRROR_CH4;
		transferBytes[7] = ATI_PCC_CH4;
		transferBytes[8] = PXS_PROX_THRESHOLD_CH4;
		transferBytes[9] = PXS_TOUCH_THRESHOLD_CH4;
		transferBytes[10] = PXS_DEEP_THRESHOLD_CH4;
		transferBytes[11] = PXS_HYSTERESIS_CH4 ;
		transferBytes[12] = DCF_CHB_ASSOCIATION_CH4;
		transferBytes[13] = DCF_WEIGHT_CH4;
		// Now write the Channel 0 settings to the device.

		//i2c_repeat_start();
		//res |= i2c_write_register(I2C_IQS62x_ADDR, IQS269_MM_CH4_SELECT_CRX , &transferBytes[0], 14);
		#ifdef __BES2500_I2C_COMMUNICATION__
		res |= IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, IQS269_MM_CH4_SELECT_CRX , &transferBytes[0], 14, false);
		#endif

	  // Save the Channel 5 settings to the transferBytes array.
		// Addresses 0xAF Byte0 to 0xB5 Byte1.
		transferBytes[0] = PXS_CRXSEL_CH5;
		transferBytes[1] = PXS_CTXSEL_CH5;
		transferBytes[2] = PXS_PROXCTRL_CH5;
		transferBytes[3] = PXS_PROXCFG0_CH5;
		transferBytes[4] = PXS_PROXCFG1_TESTREG0_CH5;
		transferBytes[5] = ATI_BASE_AND_TARGET_CH5;
		transferBytes[6] = ATI_MIRROR_CH5;
		transferBytes[7] = ATI_PCC_CH5;
		transferBytes[8] = PXS_PROX_THRESHOLD_CH5;
		transferBytes[9] = PXS_TOUCH_THRESHOLD_CH5;
		transferBytes[10] = PXS_DEEP_THRESHOLD_CH5;
		transferBytes[11] = PXS_HYSTERESIS_CH5 ;
		transferBytes[12] = DCF_CHB_ASSOCIATION_CH5;
		transferBytes[13] = DCF_WEIGHT_CH5;
		// Now write the Channel 0 settings to the device.

		//i2c_repeat_start();
		//res |= i2c_write_register(I2C_IQS62x_ADDR, IQS269_MM_CH5_SELECT_CRX , &transferBytes[0], 14);
		#ifdef __BES2500_I2C_COMMUNICATION__
		res |= IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, IQS269_MM_CH5_SELECT_CRX , &transferBytes[0], 14, false);
		#endif
	
	  // Save the Channel 6 settings to the transferBytes array.
		// Addresses 0xB6 Byte0 to 0xBC Byte1.
		transferBytes[0] = PXS_CRXSEL_CH6;
		transferBytes[1] = PXS_CTXSEL_CH6;
		transferBytes[2] = PXS_PROXCTRL_CH6;
		transferBytes[3] = PXS_PROXCFG0_CH6;
		transferBytes[4] = PXS_PROXCFG1_TESTREG0_CH6;
		transferBytes[5] = ATI_BASE_AND_TARGET_CH6;
		transferBytes[6] = ATI_MIRROR_CH6;
		transferBytes[7] = ATI_PCC_CH6;
		transferBytes[8] = PXS_PROX_THRESHOLD_CH6;
		transferBytes[9] = PXS_TOUCH_THRESHOLD_CH6;
		transferBytes[10] = PXS_DEEP_THRESHOLD_CH6;
		transferBytes[11] = PXS_HYSTERESIS_CH6 ;
		transferBytes[12] = DCF_CHB_ASSOCIATION_CH6;
		transferBytes[13] = DCF_WEIGHT_CH6;
		// Now write the Channel 0 settings to the device.

		//i2c_repeat_start();
		//res |= i2c_write_register(I2C_IQS62x_ADDR, IQS269_MM_CH6_SELECT_CRX , &transferBytes[0], 14);
		#ifdef __BES2500_I2C_COMMUNICATION__
		res |= IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, IQS269_MM_CH6_SELECT_CRX , &transferBytes[0], 14, false);
		#endif

	  // Save the Channel 7 settings to the transferBytes array.
		// Addresses 0xBD Byte0 to 0xC3 Byte1.
		transferBytes[0] = PXS_CRXSEL_CH7;
		transferBytes[1] = PXS_CTXSEL_CH7;
		transferBytes[2] = PXS_PROXCTRL_CH7;
		transferBytes[3] = PXS_PROXCFG0_CH7;
		transferBytes[4] = PXS_PROXCFG1_TESTREG0_CH7;
		transferBytes[5] = ATI_BASE_AND_TARGET_CH7;
		transferBytes[6] = ATI_MIRROR_CH7;
		transferBytes[7] = ATI_PCC_CH7;
		transferBytes[8] = PXS_PROX_THRESHOLD_CH7;
		transferBytes[9] = PXS_TOUCH_THRESHOLD_CH7;
		transferBytes[10] = PXS_DEEP_THRESHOLD_CH7;
		transferBytes[11] = PXS_HYSTERESIS_CH7 ;
		transferBytes[12] = DCF_CHB_ASSOCIATION_CH7;
		transferBytes[13] = DCF_WEIGHT_CH7;
	

		//i2c_repeat_start();
		//res |= i2c_write_register(I2C_IQS62x_ADDR, IQS269_MM_CH7_SELECT_CRX , &transferBytes[0], 14);
		//i2c_stop();
		#ifdef __BES2500_I2C_COMMUNICATION__
		res |= IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, IQS269_MM_CH7_SELECT_CRX , &transferBytes[0], 14, false);
		#endif

		transferBytes[0] = PMU_GENERAL_SETTINGS;
		transferBytes[1] = I2C_GENERAL_SETTINGS | 0x25; //ATI & EVENT MODE
		//res = i2c_write_register(I2C_IQS62x_ADDR, IQS269_MM_PMU_SETTINGS, &temp[0], 2);
		//i2c_stop();
		#ifdef __BES2500_I2C_COMMUNICATION__
		res = IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, IQS269_MM_PMU_SETTINGS, &transferBytes[0], 2, false);
		#endif

	//}while (res != 0x00);

	//Redo ATI
	//ati_loop();
	//delay_ms(100);
	//osDelay(100);

	#ifdef __BES2500_I2C_COMMUNICATION__
	IQS269_I2C_Read_Byte(I2C_IQS62x_ADDR, 0xf2, &temp[0], 1, false);
	#endif
	temp[0] |= 0x80;
	#ifdef __BES2500_I2C_COMMUNICATION__
	IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, 0xF2 , &temp[0], 1, true);
	#endif
	#endif
}



void ati_loop(void)
{
	uint8_t receive_data;

	
	IQS269_init_success_flag = false;
	//delay_ms(10);
	//osDelay(10);
	
	//i2c_start();
	//temp[0] = PMU_GENERAL_SETTINGS;
	//temp[1] = I2C_GENERAL_SETTINGS | 0x24; //ATI & EVENT MODE
	//res = i2c_write_register(I2C_IQS62x_ADDR, IQS269_MM_PMU_SETTINGS, &temp[0], 2);
	//i2c_stop();
	
	//res = IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, IQS269_MM_PMU_SETTINGS, &temp[0], 2, false);
	//Wait untill ATI completed
	#if 1
	res = IQS269_I2C_Read_Byte(I2C_IQS62x_ADDR, IQS269_MM_SYSTEM_FLAGS, &receive_data, 1, true);
	TRACE(1,"receive_data = 0x%x",receive_data);
	#else
	do
	{
		//delay_ms(10);
		//osDelay(10);
		//i2c_start();
		//res = i2c_read_register(I2C_IQS62x_ADDR, IQS269_MM_SYSTEM_FLAGS, &receive_data, 1);
		//i2c_stop();
		res = IQS269_I2C_Read_Byte(I2C_IQS62x_ADDR, IQS269_MM_SYSTEM_FLAGS, &receive_data, 1, false);
		TRACE("receive_data = 0x%x",receive_data);
	}while( ((receive_data & 0x04) == 0x04));
	#endif
	if( ((receive_data & 0x04) != 0x04))
	{
		TRACE(0,"iqs269a init OVER");
		IQS269_open_key_drv_flag = true;
		IQS269_init_success_flag = true;
		IQS269_init_step_index = APP_INIT_TOUCH_SUCCESS;
	}
}


void IQS269_rdy_irq_handler(enum HAL_GPIO_PIN_T pin);

void app_iqs269_rdy_irq_set_enable_disenable(bool en)
{
	if(en)
	{
		//hal_sys_timer_delay_us(100);
		hal_gpio_enter_ear_enable_irq((enum HAL_GPIO_PIN_T)iqs269_i2c_rdy_cfg.pin, HAL_GPIO_IRQ_POLARITY_LOW_FALLING, IQS269_rdy_irq_handler);
	}
	else
	{
		hal_gpio_enter_ear_disable_irq((enum HAL_GPIO_PIN_T)iqs269_i2c_rdy_cfg.pin);
	}
}


uint8_t poweroff_send_device_addr(void)
{
	uint8_t transferBytes[14];
	uint8_t ret = 0;
	
	transferBytes[0] = 0x40;
	ret = IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, 0xF2 , &transferBytes[0], 1, false);
	
	//TRACE(1,"poweroff_send_device_addr ret = %d",ret);
	
	return ret;
}

void app_touch_init_reset(void)
{
	#if 1
	uint8_t write_data[2] = {0x03, 0x02};
	uint8_t ret = 0;

	IQS269_I2C_init();

	if(IQS269_init_step_index == APP_INIT_RESET)
	{
		if (iqs269_i2c_rdy_cfg.pin != HAL_IOMUX_PIN_NUM)
	    {
	        hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)&iqs269_i2c_rdy_cfg, 1);
	        hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T)iqs269_i2c_rdy_cfg.pin, HAL_GPIO_DIR_IN, 1);
	    }
	    
	    //osDelay(10);
	    
		//reset command

		while(1)
		{
			ret = poweroff_send_device_addr();

			if(ret == 1)
				break;
		}
		
		ret = IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, 0x80, &write_data[0], 2, true);
		
		if(ret)
		{
			IQS269_init_step_index = APP_INIT_TOUCH;
		}
		else
		{
			IQS269_init_step_index = APP_INIT_RESET;
		}
		

		TRACE(1,"app_touch_init_reset %d",IQS269_init_step_index);
		
		app_iqs269_rdy_irq_set_enable_disenable(true);
	}
	#endif
}

//wait rdy low level
bool app_rdy_wait_turn_low_level(void)
{
	uint8_t cnt = 150;
	bool ret = false;
	
	while(cnt)
	{
		if(hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)iqs269_i2c_rdy_cfg.pin) == 0)
		{
			ret = true;
			break;
		}
		hal_sys_timer_delay_us(1);
		cnt--;
	}

	TRACE(1,"%s  ret = %d",__func__, ret);
	return ret;
}


uint8_t app_iqs269_touch_data[8];


void app_get_iqs269_touch_status(void)
{
	#if 1
	uint8_t read_data[8];
	/*
	if(!app_iqs269_init_success_flag)
	{
		ati_loop();
	}
	else
	*/
	{
		//memset(&iqs_data.SysFlags, 0x00, 4);

		res = IQS269_I2C_Read_Byte(I2C_IQS62x_ADDR, IQS269_MM_SYSTEM_FLAGS, &read_data[0], 8, false);

		memcpy(&iqs_data.SysFlags, &read_data[0], 4);
		memset(&app_iqs269_touch_data[0], 0x00, 8);

		//Check if SHOWRESET bit is set
		if(iqs_data.SysFlags.SHOW_RESET)
		{
			TRACE(0,"SHOW_RESET");
			init_IQS269();
			IQS269_init_step_index = APP_INIT_ATI_LOOP;
			//app_iqs269a_init_index = APP_INIT_TOUCH;
			app_iqs269_rdy_irq_set_enable_disenable(false);
		}
		else
		{
			memcpy(&app_iqs269_touch_data[0], &read_data[0], 8);
			DUMP8(" %02x  ", &read_data[0], 8);
			//res = IQS269_I2C_Read_Byte(I2C_IQS62x_ADDR, IQS269_MM_CH2_DTA_L, &read_data[0], 2, true);
			//DUMP8("CH2 %02x  ", &read_data[0], 2);
			res = IQS269_I2C_Read_Byte(I2C_IQS62x_ADDR, 0x18, &read_data[0], 2, true);
			//DUMP8("CH6 %02x  ", &read_data[0], 2);

			if((app_iqs269_touch_data[0] == 0x08)&&(app_iqs269_touch_data[1] == 0x80))
			{
				return;
			}
			//TRACE("putin_touch = %d, putout_touch = %d,",app_putin_touch_send_flag, app_putout_touch_send_flag);
			//if((app_iqs269_touch_data[0] & 0x18) == 0x08)
			{
				if((app_iqs269_touch_data[6] & 0x44) == 0x00)
				{
					//app_putout_touch_send_flag = true;
					//app_putin_touch_send_flag = false;
					#if 1
					//faraway ear
					if(app_putout_touch_send_flag)
					{
						app_putout_touch_send_flag = false;
						app_putin_touch_send_flag = true;
						app_send_flag = true;
						//send_enter_ear_key_event(HAL_KEY_CODE_FN10, HAL_KEY_EVENT_UP);
					}
					#endif
				}
				else if((app_iqs269_touch_data[6] & 0x44) == 0x44)
				{
					//app_putin_touch_send_flag = true;
					//app_putout_touch_send_flag = false;
					#if 1
					//nearly ear
					if(app_putin_touch_send_flag)
					{
						app_putout_touch_send_flag = true;
						app_putin_touch_send_flag = false;
						app_send_flag = true;
						//send_enter_ear_key_event(HAL_KEY_CODE_FN10, HAL_KEY_EVENT_DOWN);
					}
					#endif
				}

				//TRACE("putin_touch = %d, putout_touch = %d, app_send_flag = %d",app_putin_touch_send_flag, app_putout_touch_send_flag,app_send_flag);
			}

			if((app_iqs269_touch_data[6] & 0x01) == 0x00)
			{
				//no press
				if(app_powerkey_up_flag)
				{
					powerkey_press_flag = false;
					app_powerkey_up_flag = false;
					app_powerkey_down_flag = true;
				}
			}
			else if((app_iqs269_touch_data[6] & 0x01) == 0x01)
			{
				//press
				if(app_powerkey_down_flag)
				{
					app_powerkey_down_flag = false;
					powerkey_press_flag = true;
					app_powerkey_up_flag = true;
					hal_key_scan_timer_restart();
				}
			}
		}
	}
	#endif
}




void IQS269_rdy_irq_handler(enum HAL_GPIO_PIN_T pin)
{
	hal_gpio_enter_ear_disable_irq((enum HAL_GPIO_PIN_T)iqs269_i2c_rdy_cfg.pin);
	TRACE(1,"IQS269_rdy_irq_handler %d",IQS269_init_step_index);
	if(!IQS269_init_success_flag)
	{
		if(IQS269_init_step_index == APP_INIT_RESET)
		{
			app_touch_init_reset();
		}
		else if(IQS269_init_step_index == APP_INIT_TOUCH)
		{
			if(app_rdy_wait_turn_low_level())
			{
				init_IQS269();
				//osDelay(1);
				IQS269_init_step_index = APP_INIT_ATI_LOOP;
			}

		}
		else if(IQS269_init_step_index == APP_INIT_ATI_LOOP)
		{
			if(app_rdy_wait_turn_low_level())
			{
				ati_loop();
			}
		}

		while(1)
		{
			if(hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)iqs269_i2c_rdy_cfg.pin))
			{
				break;
			}
		}
		//hal_sys_timer_delay_us(300);
		app_iqs269_rdy_irq_set_enable_disenable(true);
	}
	else if(IQS269_init_success_flag)
	{
		if(app_rdy_wait_turn_low_level())
		{
			app_get_iqs269_touch_status();
		}
		
		//hal_sys_timer_delay_us(300);
		while(1)
		{
			if(hal_gpio_pin_get_val((enum HAL_GPIO_PIN_T)iqs269_i2c_rdy_cfg.pin))
			{
				break;
			}
		}
		app_iqs269_rdy_irq_set_enable_disenable(true);
	}
}


void poweroff_init_IQS269(void)
{
	uint8_t transferBytes[2];

	transferBytes[0] = 0x43;
	IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, 0x80 , &transferBytes[0], 1, false);
	transferBytes[0] = LTA_CHB_RESEED_ENABLED;
	transferBytes[1] = 0xFF;
	
	IQS269_I2C_Write_Byte(I2C_IQS62x_ADDR, 0x82 , &transferBytes[0], 2, true);
}


void app_poweroff_set_touch_channel(void)
{	
	uint8_t ret = 0;
	app_iqs269_rdy_irq_set_enable_disenable(false); 

	while(1)
	{
		ret = poweroff_send_device_addr();

		if(ret == 1)
			break;
	}
	
	if(app_rdy_wait_turn_low_level())
	{
		TRACE(0,"poweroff_init_IQS269");
		poweroff_init_IQS269();
	}
}
#endif // end __BES2500_IQS269_TOUCH__
