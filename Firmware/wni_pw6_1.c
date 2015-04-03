// (C) 1996-2015 Weathernews Inc. All Rights Reserved.

#define BLE_TRACE_DISABLE
#define DEBUG 0
//#define DEBUG 1

////////////////////////////////////////

#include "bleapp.h"
#include "bleprofile.h"

#include "platform.h"
#include "gpiodriver.h"
#include "rtc.h"
#include "devicelpm.h"
#include "miadriver.h"
#include "ws_upgrade_ota.h"

#include "wni_pw6_1.h"

////////////////////////////////////////

#define WNI_PW6_DATA_LENGTH	0x03

#define WNI_PW6_NVRAM_PAGE_SIZE	0x1c	// Changed from 0x1d
#define WNI_PW6_NVRAM_PAGE_NUMBER	0x1c
#define WNI_PW6_NVRAM_PAGE_TOP	0x10
#define WNI_PW6_NVRAM_PAGE_END	(WNI_PW6_NVRAM_PAGE_TOP + WNI_PW6_NVRAM_PAGE_NUMBER - 0x01)

#define WNI_PW6_NVRAM_PAGE_COUNT	(WNI_PW6_NVRAM_PAGE_TOP)
#define WNI_PW6_NVRAM_PAGE_DATA_START	(WNI_PW6_NVRAM_PAGE_COUNT + 0x01)
#define WNI_PW6_NVRAM_PAGE_DATA_END	(WNI_PW6_NVRAM_PAGE_END)
#define WNI_PW6_NVRAM_PAGE_DATA_NUMBER	(WNI_PW6_NVRAM_PAGE_DATA_END - WNI_PW6_NVRAM_PAGE_DATA_START + 0x01)

#if 1	// 2015/Mar/03
#define WNI_PW6_NVRAM_NUMBER_OF_DATA_IN_PAGE	(0x06)
#else	// 2015/Mar/03
#define WNI_PW6_NVRAM_NUMBER_OF_DATA_IN_PAGE	(WNI_PW6_NVRAM_PAGE_SIZE / WNI_PW6_DATA_LENGTH)
#endif	// 2015/Mar/03
#define WNI_PW6_NUMBER_OF_DATA	(WNI_PW6_NVRAM_PAGE_DATA_NUMBER * WNI_PW6_NVRAM_NUMBER_OF_DATA_IN_PAGE)

#define WNI_PW6_NVRAM_COUNT_LENGTH	0x07

#define WNI_PW6_PACKET_LENGTH	0x14
#define WNI_PW6_PACKET_COUNT_LENGTH	(WNI_PW6_NVRAM_COUNT_LENGTH)
#define	WNI_PW6_PACKET_START	0x00
#define WNI_PW6_PACKET_END		0x01
#define	WNI_PW6_PACKET_DATA_START	0x02
#define WNI_PW6_PACKET_DATA_END	(WNI_PW6_PACKET_LENGTH - 0x01)
#define WNI_PW6_PACKET_DATA_SIZE	(WNI_PW6_PACKET_DATA_END - WNI_PW6_PACKET_DATA_START + 0x01)
#define WNI_PW6_PACKET_NUMBER_OF_DATA	(WNI_PW6_PACKET_DATA_SIZE / WNI_PW6_DATA_LENGTH)
#define WNI_PW6_NUMBER_OF_DATA_PACKET	(((WNI_PW6_NUMBER_OF_DATA - 0x01) / WNI_PW6_PACKET_NUMBER_OF_DATA) + 0x01)

#define WNI_PW6_COUNT	0x04
#define WNI_PW6_DS_TO	0x05
#define WNI_PW6_DS_FROM	0x06

#define WNI_PW6_RTC_START_TIME	0x4b3d3b00	// 2010/Jan/01 00:00:00

////////////////////////////////////////

const BLE_PROFILE_CFG wni_pw6_profile_config = {
	100											// fine_timer_interval 100[ms]
	, HIGH_UNDIRECTED_DISCOVERABLE				// default_adv
	, 0x00										// button_adv_toggle
	, 32										// high_undirect_adv_interval 20[ms]
	, 1024										// low_undirect_adv_interval 640[ms]
	, 15										// high_undirect_adv_duration 15[s]
	, 1											// low_undirect_adv_duration 1[s]
	, 0											// high_direct_adv_interval
	, 0											// low_direct_adv_interval
	, 0											// high_direct_adv_duration
	, 0											// low_direct_adv_duration
	, "wni_pw"									// local_name
	, BIT16_TO_8(APPEARANCE_GENERIC_TAG), 0x00	// cod
	, "1.00"									// ver
	, 0											// encr_required
	, 0											// disc_required
	, 0x00										// test_enable
	, 0x00										// tx_power_level
	, 3											// con_idle_timeout
	, 0											// powersave_timeout
	, {0x00, 0x00, 0x00, 0x00, 0x00}			// hdl
	, {0x00, 0x00, 0x00, 0x00, 0x00}			// serv
	, {0x00, 0x00, 0x00, 0x00, 0x00}			// cha
	, 0											// findme_locator_enable
	, 0											// findme_alert_level
	, 0											// client_grouptype_enable
	, 0											// linkloss_button_enable
	, 0											// pathloss_check_interval
	, 0											// alert_interval
	, 0											// high_alert_num
	, 0											// mild_alert_num
	, 0											// status_led_enable
	, 0											// status_led_interval
	, 0											// status_led_con_blink
	, 0											// status_led_dir_adv_blink
	, 0											// status_led_un_adv_blink
	, 0											// led_on_ms
	, 0											// led_off_ms
	, 0											// buz_on_ms
	, 0											// button_power_timeout
	, 0											// button_client_timeout
	, 0											// button_discover_timeout
	, 0											// button_filter_timeout
};

const UINT8 wni_pw6_gatt_database[] = {
	PRIMARY_SERVICE_UUID16(0x0001, UUID_SERVICE_GATT)

	, PRIMARY_SERVICE_UUID16(0x0021, UUID_SERVICE_GAP)
	, CHARACTERISTIC_UUID16(0x0022, 0x0023, UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 16)
		, 'w', 'n', 'i', '_', 'p', 'w', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
	, CHARACTERISTIC_UUID16(0x0024, 0x0025, UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 2)
		, BIT16_TO_8(APPEARANCE_GENERIC_TAG)

	, PRIMARY_SERVICE_UUID16(0x0101, UUID_SERVICE_DEVICE_INFORMATION)
	, CHARACTERISTIC_UUID16(0x0102, 0x0103, UUID_CHARACTERISTIC_SYSTEM_ID, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 12)
		, 'P', 'O', 'L', 'L', 'E', 'N', 'W', 'A', 'T', 'C', 'H', 0x00	// "POLLENWATCH"
	, CHARACTERISTIC_UUID16(0x0104, 0x0105, UUID_CHARACTERISTIC_MODEL_NUMBER_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 12)
		, 'P', 'O', 'L', 'L', 'E', 'N', 'W', 'A', 'T', 'C', 'H', 0x00	// "POLLENWATCH"
	, CHARACTERISTIC_UUID16(0x0106, 0x0107, UUID_CHARACTERISTIC_FIRMWARE_REVISION_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 6)
		, '2', '.', '1', '.', '1', 0x00	// "2.1.1"
	, CHARACTERISTIC_UUID16(0x0108, 0x0109, UUID_CHARACTERISTIC_HARDWARE_REVISION_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 6)
		, '1', '.', '0', '.', '0', 0x00	// "1.0.0"
	, CHARACTERISTIC_UUID16(0x010a, 0x010b, UUID_CHARACTERISTIC_SOFTWARE_REVISION_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 6)
		, '1', '.', '0', '.', '0', 0x00	// "1.0.0"
	, CHARACTERISTIC_UUID16(0x010c, 0x010d, UUID_CHARACTERISTIC_MANUFACTURER_NAME_STRING, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 12)
		, 'W', 'e', 'a', 't', 'h', 'e', 'r', 'n', 'e', 'w', 's', 0x00	// "Weathernews"

	, PRIMARY_SERVICE_UUID16(0x0201, UUID_SERVICE_BATTERY)
	, CHARACTERISTIC_UUID16(0x0202, 0x0203, UUID_CHARACTERISTIC_BATTERY_LEVEL, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 1)
		, 0x64
	, CHARACTERISTIC_UUID16(0x0204, 0x0205, UUID_CHARACTERISTIC_BATTERY_POWER_STATE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 1)
		, 0x64
	, CHARACTERISTIC_UUID16(0x0206, 0x0207, UUID_CHARACTERISTIC_REMOVABLE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 2)
		, 0x01, 0x00


	, PRIMARY_SERVICE_UUID128(WNI_PW6_SERVICE_HANDLE, WNI_PW6_SERVICE_UUID)

	, CHARACTERISTIC_UUID128(WNI_PW6_COUNT_HANDLE, WNI_PW6_COUNT_HANDLE_VALUE, WNI_PW6_COUNT_UUID
		, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY
		, LEGATTDB_PERM_READABLE, 0x07)
		, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

	, CHARACTERISTIC_UUID128(WNI_PW6_DATA_HANDLE, WNI_PW6_DATA_HANDLE_VALUE, WNI_PW6_DATA_UUID
		, LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY
		, LEGATTDB_PERM_READABLE, 0x14)
		, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		, 0x00, 0x00, 0x00, 0x00

	, CHARACTERISTIC_UUID128_WRITABLE(WNI_PW6_COMMAND_HANDLE, WNI_PW6_COMMAND_HANDLE_VALUE, WNI_PW6_COMMAND_UUID
		, LEGATTDB_CHAR_PROP_WRITE
		, LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ, 0x07)
		, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

#if 1	// 2015/Mar/10
	,

#define WNI_PW6_APP_ID	0x105f
#define WNI_PW6_APP_VERSION_MAJOR	0x01
#define WNI_PW6_APP_VERSION_MINOR	0x01

	// Handle 0xff00: Broadcom vendor specific WICED Smart Upgrade Service.
	// The service has 2 characteristics.  The first is the control point.  Client
	// sends commands, and sensor sends status notifications. Note that
	// UUID of the vendor specific service is 16 bytes, unlike standard Bluetooth
	// UUIDs which are 2 bytes.  _UUID128 version of the macro should be used.
	PRIMARY_SERVICE_UUID128 (HANDLE_WS_UPGRADE_SERVICE, UUID_WS_UPGRADE_SERVICE),

	// Handle 0xff01: characteristic WS Control Point, handle 0xff02 characteristic value.
	// This characteristic can be used by the client to send commands to this device
	// and to send status notifications back to the client.  Client has to enable
	// notifications by updating Characteristic Client Configuration Descriptor
	// (see handle ff03 below).  Note that UUID of the vendor specific characteristic is
	// 16 bytes, unlike standard Bluetooth UUIDs which are 2 bytes.  _UUID128 version
	// of the macro should be used.  Also note that characteristic has to be _WRITABLE
	// to correctly enable writes from the client.
	CHARACTERISTIC_UUID128_WRITABLE (HANDLE_WS_UPGRADE_CHARACTERISTIC_CONTROL_POINT,
									 HANDLE_WS_UPGRADE_CONTROL_POINT, UUID_WS_UPGRADE_CHARACTERISTIC_CONTROL_POINT,
									 LEGATTDB_CHAR_PROP_WRITE | LEGATTDB_CHAR_PROP_NOTIFY | LEGATTDB_CHAR_PROP_INDICATE,
									 LEGATTDB_PERM_WRITE_REQ, 5),
		0x00,0x00,0x00,0x00,0x00,

	// Handle 0xff03: Characteristic Client Configuration Descriptor.
	// This is a standard GATT characteristic descriptor.  2 byte value 0 means that
	// message to the client is disabled.  Peer can write value 1 to enable
	// notifications or respectively.  Note _WRITABLE in the macro.  This
	// means that attribute can be written by the peer.
	CHAR_DESCRIPTOR_UUID16_WRITABLE (HANDLE_WS_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR,
									 UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
									 LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ, 2),
		0x00,0x00,

	// Handle 0xff04: characteristic WS Data, handle 0xff05 characteristic value
	// This characteristic is used to send next portion of the FW.  Similar to the
	// control point, characteristic should be _WRITABLE and 128bit version of UUID is used.
	CHARACTERISTIC_UUID128_WRITABLE (HANDLE_WS_UPGRADE_CHARACTERISTIC_DATA,
									 HANDLE_WS_UPGRADE_DATA, UUID_WS_UPGRADE_CHARACTERISTIC_DATA,
									 LEGATTDB_CHAR_PROP_WRITE,
									 LEGATTDB_PERM_VARIABLE_LENGTH | LEGATTDB_PERM_WRITE_REQ,  20),
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,

	// Handle 0xff06: characteristic Application Info, handle 0xff07 characteristic value
	// Client can read value of this characteristic to figure out which application id is
	// running as well as version information.  Characteristic UUID is 128 bits.
	CHARACTERISTIC_UUID128 (HANDLE_WS_UPGRADE_CHARACTERISTIC_APP_INFO,
							HANDLE_WS_UPGRADE_APP_INFO, UUID_WS_UPGRADE_CHARACTERISTIC_APP_INFO,
							LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE,  4),
		 WNI_PW6_APP_ID & 0xff, (WNI_PW6_APP_ID >> 8) & 0xff, WNI_PW6_APP_VERSION_MAJOR, WNI_PW6_APP_VERSION_MINOR,
#endif	// 2015/Mar/10
};

const BLE_PROFILE_PUART_CFG wni_pw6_puart_config = {
	115200
	, PUARTDISABLE | GPIO_PIN_UART_TX	// Disable
	, PUARTDISABLE | GPIO_PIN_UART_RX	// Disable
};

const BLE_PROFILE_GPIO_CFG wni_pw6_gpio_config = {
	{
		GPIO_PIN_WP
		, GPIO_PIN_BATTERY
		, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1
	}, {
		GPIO_SETTINGS_WP
		, GPIO_SETTINGS_BATTERY
		, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	}
};

UINT16 wni_pw6_connection_handle = 0x00;
BD_ADDR wni_pw6_peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

#pragma pack(1)
typedef PACKED struct {
	BD_ADDR bd_addr;	// Bonded peer Bluetooth Device Address
} HOSTINFO;
#pragma pack()

HOSTINFO wni_pw6_hostinfo = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

const UINT8 service_uuid[0x10] = {WNI_PW6_SERVICE_UUID};

struct {
	UINT8 number_of_sneeze;
	UINT8 transmitted;
	UINT8 continued;
	UINT8 completed;
} pw_t;

enum PW_STATE {
	STATE_IDLE
	, STATE_ERROR
	, STATE_SUCCESS
	, STATE_ADV
	, STATE_UPDATE
	, STATE_INIT
	, STATE_BLUE
	, STATE_YELLOW
	, STATE_RED
	, STATE_PURPLE
	, STATE_FULL
};
struct {
	enum PW_STATE state;
	UINT16 timer;
	UINT8 led_counter;
	UINT16 time_to_deepsleep;
	UINT8 wakeup;
} pw_s;

#define PW_TIMEOUT_1SEC	0x0a
#define PW_TIMEOUT_2SEC	(PW_TIMEOUT_1SEC * 2)
#define PW_TIMEOUT_15SEC	(PW_TIMEOUT_1SEC * 15)
#define PW_TIMEOUT_60SEC	(PW_TIMEOUT_1SEC * 60)

#define	PW_LED_COLOR_OFF	0x00
#define	PW_LED_COLOR_RED	0x04
#define	PW_LED_COLOR_GREEN	0x02
#define	PW_LED_COLOR_BLUE	0x01

// LED timer starts from ON, counted down to 0x00 and turned LED off at OFF count
#define	PW_LED_PATTERN_ON_MASK	0xf0
#define	PW_LED_PATTERN_OFF_MASK	0x0f
#define PW_LED_PATTERN_ON	0x1f	// 0x1f means always ON since it's never turned off
#define PW_LED_PATTERN_BLINK_FAST	0x10	// Blinkiing every timeout
#define	PW_LED_PATTERN_BLINK_SLOW	0x94	// 5 times slower
#define	PW_LED_PATTERN_BLINK_ECO	0x98	// Only once ON in 10 counts

const struct {
	UINT16 timeout;	// in fine_timer_interval
	enum PW_STATE next_state;
	UINT8 led_color;
	UINT8 led_pattern;
} pw_table[] = {
	{0x00, 0x00, 0x00, 0x00}	// STATE_IDLE
	, {PW_TIMEOUT_1SEC, STATE_IDLE, PW_LED_COLOR_RED, PW_LED_PATTERN_BLINK_FAST}	// STATE_ERROR
	, {PW_TIMEOUT_2SEC, STATE_IDLE, PW_LED_COLOR_GREEN, PW_LED_PATTERN_ON}	// STATE_SUCCESS
	, {PW_TIMEOUT_15SEC, STATE_ERROR, PW_LED_COLOR_GREEN, PW_LED_PATTERN_BLINK_ECO}	// STATE_ADV
	, {PW_TIMEOUT_60SEC, STATE_ERROR, PW_LED_COLOR_BLUE, PW_LED_PATTERN_BLINK_ECO}	// STATE_UPDATE
	, {PW_TIMEOUT_1SEC, STATE_IDLE, PW_LED_COLOR_BLUE, PW_LED_PATTERN_BLINK_FAST}	// STATE_INIT
	, {PW_TIMEOUT_1SEC, STATE_IDLE, PW_LED_COLOR_BLUE, PW_LED_PATTERN_ON}	// STATE_BLUE
	, {PW_TIMEOUT_1SEC, STATE_IDLE, PW_LED_COLOR_RED | PW_LED_COLOR_GREEN, PW_LED_PATTERN_ON}	// STATE_YELLOW
	, {PW_TIMEOUT_1SEC, STATE_IDLE, PW_LED_COLOR_RED, PW_LED_PATTERN_ON}	// STATE_RED
	, {PW_TIMEOUT_1SEC, STATE_IDLE, PW_LED_COLOR_RED | PW_LED_COLOR_BLUE, PW_LED_PATTERN_ON}	// STATE_PURPLE
	, {PW_TIMEOUT_1SEC, STATE_IDLE, PW_LED_COLOR_RED | PW_LED_COLOR_BLUE, PW_LED_PATTERN_BLINK_FAST}	// STATE_FULL
};

////////////////////////////////////////

static void wni_pw6_create(void);
static void wni_pw6_database_init(void);
static void wni_pw6_gpio_interrupt_handler(void *parameter, UINT8 arg);
static void wni_pw6_advertisement_stopped(void);
static void wni_pw6_connection_up(void);
static void wni_pw6_encryption_changed(HCI_EVT_HDR *evt);
static void wni_pw6_smp_bond_result(int result);
static void wni_pw6_connection_down(void);
static void wni_pw6_timeout(UINT32 count);
static void wni_pw6_fine_timeout(UINT32 finecount);
static INT32 wni_pw6_write_handler(void *p);

extern UINT8 blecm_set_static_random_bd_addr(UINT8* bd_addr);
static void wni_pw6_start_advertisement(void);
static void wni_pw6_stop_advertisement(void);
static wni_pw6_manage_state(void);
static wni_pw6_change_state(enum PW_STATE new_state);
static wni_pw6_control_led(void);
static void wni_pw6_count_sneeze(void);
static void wni_pw6_sneeze_data_transmitter(void);
static void wni_pw6_calculate_time_to_deepsleep(UINT8 to, UINT8 from, RtcTime current_time);
static void wni_pw6_entering_deep_sleep(void);


////////////////////////////////////////

APPLICATION_INIT() {
	bleapp_set_cfg((UINT8 *)wni_pw6_gatt_database, sizeof(wni_pw6_gatt_database)
		, (void *)&wni_pw6_profile_config, (void *)&wni_pw6_puart_config, (void *)&wni_pw6_gpio_config
		, wni_pw6_create);
}

static void wni_pw6_create(void) {
	ble_trace0("\n+wni_pw6_create()\n");

	blecm_StackCheckInit();

	bleprofile_Init(bleprofile_p_cfg);
	bleprofile_GPIOInit(bleprofile_gpio_p_cfg);

	gpio_configurePin(GPIO_PIN_P8 >> 0x04, GPIO_PIN_P8 & 0x0f, GPIO_INPUT_DISABLE, GPIO_PIN_OUTPUT_LOW);
	gpio_configurePin(GPIO_PIN_P11 >> 0x04, GPIO_PIN_P11 & 0x0f, GPIO_INPUT_DISABLE, GPIO_PIN_OUTPUT_LOW);
	gpio_configurePin(GPIO_PIN_P12 >> 0x04, GPIO_PIN_P12 & 0x0f, GPIO_INPUT_DISABLE, GPIO_PIN_OUTPUT_LOW);
	gpio_configurePin(GPIO_PIN_P13 >> 0x04, GPIO_PIN_P13 & 0x0f, GPIO_INPUT_DISABLE, GPIO_PIN_OUTPUT_LOW);
	gpio_configurePin(GPIO_PIN_P14 >> 0x04, GPIO_PIN_P14 & 0x0f, GPIO_INPUT_DISABLE, GPIO_PIN_OUTPUT_LOW);

	devlpm_init();
	devlpm_enableWakeFrom(DEV_LPM_WAKE_SOURCE_GPIO);

	// Turn LEDs off
	gpio_configurePin(GPIO_PIN_P25 >> 0x04, GPIO_PIN_P25 & 0x0f, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
	gpio_configurePin(GPIO_PIN_P4 >> 0x04, GPIO_PIN_P4 & 0x0f, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
	gpio_configurePin(GPIO_PIN_P24 >> 0x04, GPIO_PIN_P24 & 0x0f, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);

	// Enable interrupt by button
	UINT16 interrupt_handler_mask[0x03] = {0x00, 0x00, 0x00};
	interrupt_handler_mask[GPIO_PIN_P27 >> 0x04] |= 1 << (GPIO_PIN_P27 & 0x0f);
	gpio_registerForInterrupt(interrupt_handler_mask, wni_pw6_gpio_interrupt_handler, (void *)(UINT32)GPIO_PIN_P27);
	gpio_configurePin(GPIO_PIN_P27 >> 0x04, GPIO_PIN_P27 & 0x0f, GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE, GPIO_PIN_OUTPUT_LOW);
	gpio_clearPinInterruptStatus(GPIO_PIN_P27 >> 0x04, GPIO_PIN_P27 & 0x0f);
	gpio_configurePin(GPIO_PIN_P0 >> 0x04, GPIO_PIN_P0 & 0x0f, GPIO_INPUT_ENABLE | GPIO_PULL_UP, GPIO_PIN_OUTPUT_LOW);
	gpio_clearPinInterruptStatus(GPIO_PIN_P0 >> 0x04, GPIO_PIN_P0 & 0x0f);

	pw_s.wakeup = 0x00;

	// Configure RTC
	UINT8 count[WNI_PW6_NVRAM_COUNT_LENGTH];
	rtc_init();
	RtcTime reference_time;
	UINT32 reference_second;
	if (mia_isResetReasonPor()) {
		reference_second = 0x00;
		rtc_sec2RtcTime(reference_second, &reference_time);
		if (!rtc_setRTCTime(&reference_time)) {
			ble_trace0("Fail to rtc_setRTCTime()\n");
		}

		memset(&count[0x00], 0x00, sizeof(count));
		UINT8 writtenbyte = bleprofile_WriteNVRAM(WNI_PW6_NVRAM_PAGE_COUNT, sizeof(count), &count[0x00]);
		if (WNI_PW6_NVRAM_COUNT_LENGTH != writtenbyte) {
			ble_trace0("Fail to bleprofile_WriteNVRAM(WNI_PW6_NVRAM_PAGE_COUNT)\n");
		}
		UINT8 nvram[WNI_PW6_NVRAM_PAGE_SIZE];
		memset(&nvram[0x00], 0x00, sizeof(nvram));
		UINT8 page = WNI_PW6_NVRAM_PAGE_DATA_START;
		for (page = WNI_PW6_NVRAM_PAGE_DATA_START; WNI_PW6_NVRAM_PAGE_DATA_END >= page; page++) {
			writtenbyte = bleprofile_WriteNVRAM(page, sizeof(nvram), &nvram[0x00]);
			if (WNI_PW6_NVRAM_PAGE_SIZE != writtenbyte) {
				ble_trace1("Fail to bleprofile_WriteNVRAM(%d)\n", page);
			}
		}
	} else {
		UINT8 readbyte = bleprofile_ReadNVRAM(WNI_PW6_NVRAM_PAGE_COUNT, sizeof(count), &count[0x00]);
		if (WNI_PW6_NVRAM_COUNT_LENGTH != readbyte) {
			ble_trace0("Fail to bleprofile_ReadNVRAM(WNI_PW6_NVRAM_PAGE_COUNT)\n");
		}
		reference_second = *(UINT32 *)(&count[0x00]);
		rtc_sec2RtcTime(reference_second, &reference_time);
		// Set up the original reference time instead of using 01/01/2010, 00:00:00 as the reference
		// because this is a wake from deep sleep. The HW clock keeps running in deep sleep so when
		// we wake up, the FW needs to know what was used as the original reference time.
		rtc_setReferenceTime(&reference_time);
		pw_s.wakeup = 0x01;
	}


	pw_t.number_of_sneeze = 0x00;
	pw_t.transmitted = 0x00;
	pw_t.continued = 0x00;
	pw_t.completed = 0x00;

	wni_pw6_database_init();

	bleprofile_regAppEvtHandler(BLECM_APP_EVT_ADV_TIMEOUT, wni_pw6_advertisement_stopped);
	bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_UP, wni_pw6_connection_up);
	bleprofile_regAppEvtHandler(BLECM_APP_EVT_LINK_DOWN, wni_pw6_connection_down);
	blecm_regEncryptionChangedHandler(wni_pw6_encryption_changed);
	lesmp_regSMPResultCb(wni_pw6_smp_bond_result);

	legattdb_regWriteHandleCb(wni_pw6_write_handler);

	bleprofile_regTimerCb(wni_pw6_fine_timeout, wni_pw6_timeout);

	UINT8 ownaddr[BD_ADDR_LEN];
	if (!blecm_set_static_random_bd_addr(&ownaddr[0x00])) {
		memcpy(&ownaddr[0x00], emconinfo_getAddr(), sizeof(ownaddr));
	}
	sprintf(bleprofile_p_cfg->local_name, "wni_pw-%02x%02x", ownaddr[0x01], ownaddr[0x00]);
	ble_trace1("local_name: %s\n", (UINT32)bleprofile_p_cfg->local_name);
	BLEPROFILE_DB_PDU p_pdu;
	p_pdu.len = 0x10;
	memcpy(&p_pdu.pdu[0x00], bleprofile_p_cfg->local_name, strlen(bleprofile_p_cfg->local_name));
	bleprofile_WriteHandle(0x0023, &p_pdu);

	bleprofile_StartTimer();

	if (!pw_s.wakeup) {
		wni_pw6_start_advertisement();
	} else {
		enum PW_STATE new_state = STATE_IDLE;
		if (!reference_second) {
			new_state = STATE_INIT;
		} else if (0x05 > count[WNI_PW6_COUNT]) {
			new_state = STATE_BLUE;
		} else if (0x0a > count[WNI_PW6_COUNT]) {
			new_state = STATE_YELLOW;
		} else if (0x14 > count[WNI_PW6_COUNT]) {
			new_state = STATE_RED;
		} else if (0x96 > count[WNI_PW6_COUNT]) {
			new_state = STATE_PURPLE;
		} else {
			new_state = STATE_FULL;
		}
		wni_pw6_change_state(new_state);
		wni_pw6_manage_state();
		wni_pw6_control_led();
	}
	pw_s.time_to_deepsleep = 0x00;

	ble_trace0("-wni_pw6_create()\n");
}

static void wni_pw6_database_init(void) {
	blebat_Init();
}

static void wni_pw6_gpio_interrupt_handler(void *parameter, UINT8 arg) {
	ble_trace0("*wni_pw6_gpio_interrupt_handler()\n");
	UINT8 p = (UINT8)(UINT32)parameter;
	gpio_clearPinInterruptStatus(p >> 0x04, p & 0x0f);

	if ((STATE_IDLE == pw_s.state) && (!pw_s.wakeup)) {
		pw_s.wakeup = 0x01;
		UINT8 count[WNI_PW6_NVRAM_COUNT_LENGTH];
		UINT8 readbyte = bleprofile_ReadNVRAM(WNI_PW6_NVRAM_PAGE_COUNT, sizeof(count), &count[0x00]);
		if (WNI_PW6_NVRAM_COUNT_LENGTH != readbyte) {
			ble_trace0("Fail to bleprofile_ReadNVRAM(WNI_PW6_NVRAM_PAGE_COUNT)\n");
			wni_pw6_change_state(STATE_ERROR);
		}
		UINT32 reference_second = *(UINT32 *)(&count[0x00]);
		enum PW_STATE new_state = STATE_IDLE;
		if (!reference_second) {
			new_state = STATE_INIT;
		} else if (0x05 > count[WNI_PW6_COUNT]) {
			new_state = STATE_BLUE;
		} else if (0x0a > count[WNI_PW6_COUNT]) {
			new_state = STATE_YELLOW;
		} else if (0x14 > count[WNI_PW6_COUNT]) {
			new_state = STATE_RED;
		} else if (0x96 > count[WNI_PW6_COUNT]) {
			new_state = STATE_PURPLE;
		} else {
			new_state = STATE_FULL;
		}
		wni_pw6_change_state(new_state);
		return;
	}
	if ((STATE_INIT > pw_s.state) || (STATE_FULL < pw_s.state)) {
		return;
	}
	if ((PW_TIMEOUT_1SEC - 0x02) <= pw_s.timer) {
		return;
	}
	wni_pw6_start_advertisement();
}

static void wni_pw6_advertisement_stopped(void) {
	ble_trace0("*wni_pw6_advertisement_stopped()\n");
}

static void wni_pw6_connection_up(void) {
	ble_trace0("*wni_pw6_connection_up()\n");
	memcpy(wni_pw6_peer_addr, (UINT8 *)emconninfo_getPeerAddr(), sizeof(wni_pw6_peer_addr));
	wni_pw6_connection_handle = (UINT16)emconinfo_getConnHandle();

	pw_t.number_of_sneeze = 0x00;
	pw_t.transmitted = 0x00;
	pw_t.continued = 0x00;
	pw_t.completed = 0x00;

	wni_pw6_stop_advertisement();

	// No encryption requred
	memcpy(wni_pw6_hostinfo.bd_addr, wni_pw6_peer_addr, sizeof(BD_ADDR));
	UINT8 writtenbyte = bleprofile_WriteNVRAM(VS_BLE_HOST_LIST, sizeof(wni_pw6_hostinfo), (UINT8 *)&wni_pw6_hostinfo);
}

static void wni_pw6_encryption_changed(HCI_EVT_HDR *evt) {
	if (NULL == evt) {	// Fail safe
		return;
	}

    bleprofile_ReadNVRAM(VS_BLE_HOST_LIST, sizeof(wni_pw6_hostinfo), (UINT8 *)&wni_pw6_hostinfo);
}

static void wni_pw6_smp_bond_result(int result) {
	if (LESMP_PAIRING_RESULT_BONDED == (LESMP_PARING_RESULT)result) {
		memcpy(wni_pw6_hostinfo.bd_addr, (UINT8 *)emconninfo_getPeerAddr(), sizeof(BD_ADDR));
		UINT8 writtenbyte = bleprofile_WriteNVRAM(VS_BLE_HOST_LIST, sizeof(wni_pw6_hostinfo), (UINT8 *)&wni_pw6_hostinfo);
	}
}

static void wni_pw6_connection_down(void) {
	ble_trace0("*wni_pw6_connection_down()\n");
	memset(wni_pw6_peer_addr, 0x00, 0x06);
	wni_pw6_connection_handle = 0x00;

	if (pw_t.completed) {
		wni_pw6_change_state(STATE_SUCCESS);
	} else {
		wni_pw6_change_state(STATE_ERROR);
	}

	pw_t.number_of_sneeze = 0x00;
	pw_t.transmitted = 0x00;
	pw_t.continued = 0x00;
	pw_t.completed = 0x00;
}

static void wni_pw6_timeout(UINT32 arg) {
	if (BLEPROFILE_GENERIC_APP_TIMER != arg) {
		return;
	}
}

static void wni_pw6_fine_timeout(UINT32 arg) {
	wni_pw6_sneeze_data_transmitter();

	wni_pw6_manage_state();
	wni_pw6_control_led();
}

static INT32 wni_pw6_write_handler(void *p) {
	ble_trace0("*wni_pw6_write_handler()\n");
	UINT16 handle = legattdb_getHandle((LEGATTDB_ENTRY_HDR *)p);
	int length = legattdb_getAttrValueLen((LEGATTDB_ENTRY_HDR *)p);
	UINT8 *attrPtr = legattdb_getAttrValue((LEGATTDB_ENTRY_HDR *)p);
	BLEPROFILE_DB_PDU p_pdu;
	UINT32 reference_second;
	UINT8 count[WNI_PW6_NVRAM_COUNT_LENGTH];
	UINT8 nvram[WNI_PW6_NVRAM_PAGE_SIZE];
	UINT8 readbyte;
	UINT8 writtenbyte;

	ble_trace3("Handle: 0x%04x, Length: 0x%02x, Data: 0x%02x\n", handle, (UINT8)length, *attrPtr);

	switch (handle) {
		case WNI_PW6_COMMAND_HANDLE_VALUE:
			if ((0x00 >= length) || (WNI_PW6_PACKET_COUNT_LENGTH < length)) {
				break;
			}
			switch (*attrPtr) {
				case WNI_PW6_COMMAND_REQUEST:
					readbyte = bleprofile_ReadNVRAM(WNI_PW6_NVRAM_PAGE_COUNT, sizeof(count), &count[0x00]);
					if (WNI_PW6_NVRAM_COUNT_LENGTH != readbyte) {
						ble_trace0("Fail to bleprofile_ReadNVRAM(WNI_PW6_NVRAM_PAGE_COUNT)\n");
						blecm_disconnect(0x00);
						break;
					}
					reference_second = *(UINT32 *)&count[0x00];
					if (reference_second) {
						reference_second += WNI_PW6_RTC_START_TIME;
					}
					pw_t.number_of_sneeze = count[WNI_PW6_COUNT];

					// Transmit number of sneeze
					p_pdu.pdu[0x00] = (reference_second >> 0x18) & 0xff;
					p_pdu.pdu[0x01] = (reference_second >> 0x10) & 0xff;
					p_pdu.pdu[0x02] = (reference_second >> 0x08) & 0xff;
					p_pdu.pdu[0x03] = reference_second & 0xff;
					p_pdu.pdu[WNI_PW6_COUNT] = pw_t.number_of_sneeze;
					p_pdu.pdu[WNI_PW6_DS_TO] = count[WNI_PW6_DS_TO];
					p_pdu.pdu[WNI_PW6_DS_FROM] = count[WNI_PW6_DS_FROM];
					p_pdu.len = WNI_PW6_PACKET_COUNT_LENGTH;
					bleprofile_WriteHandle(WNI_PW6_COUNT_HANDLE_VALUE, &p_pdu);
					bleprofile_sendNotification(WNI_PW6_COUNT_HANDLE_VALUE, (UINT8 *)p_pdu.pdu, p_pdu.len);

					// Start transmitting data
					if (pw_t.number_of_sneeze) {
						pw_t.transmitted = 0x00;
						pw_t.continued = 0x01;
						wni_pw6_sneeze_data_transmitter();
					}
					break;
				case WNI_PW6_COMMAND_ERROR:
					pw_t.number_of_sneeze = 0x00;
					pw_t.transmitted = 0x00;
					pw_t.continued = 0x00;
					pw_t.completed = 0x00;
					wni_pw6_change_state(STATE_ERROR);
					blecm_disconnect(0x00);
					break;
				case WNI_PW6_COMMAND_RESET:
					pw_t.number_of_sneeze = 0x00;
					pw_t.transmitted = 0x00;
					pw_t.continued = 0x00;
					reference_second = (attrPtr[0x01] << 0x18) + (attrPtr[0x02] << 0x10)
						 + (attrPtr[0x03] << 0x08) + attrPtr[0x04];
					if (WNI_PW6_RTC_START_TIME < reference_second) {
						reference_second -= WNI_PW6_RTC_START_TIME;
					} else {
						reference_second = 0x00;
					}
					RtcTime reference_time;
					rtc_sec2RtcTime(reference_second, &reference_time);
					if (!rtc_setRTCTime(&reference_time)) {
						ble_trace0("Fail to rtc_setRTCTime()\n");
						// LED?ÅöÅö
						wni_pw6_change_state(STATE_ERROR);
						break;
					}

					*(UINT32 *)&count[0x00] = reference_second;
					count[WNI_PW6_COUNT] = 0x00;
					count[WNI_PW6_DS_TO] = attrPtr[WNI_PW6_DS_TO];
					count[WNI_PW6_DS_FROM] = attrPtr[WNI_PW6_DS_FROM];
					writtenbyte = bleprofile_WriteNVRAM(WNI_PW6_NVRAM_PAGE_COUNT, sizeof(count), &count[0x00]);
					if (WNI_PW6_NVRAM_COUNT_LENGTH != writtenbyte) {
						ble_trace0("Fail to bleprofile_WriteNVRAM(WNI_PW6_NVRAM_PAGE_COUNT)\n");
						// LED?ÅöÅö
						wni_pw6_change_state(STATE_ERROR);
						break;
					}
					pw_t.completed = 0x01;
			}
			break;
		case HANDLE_WS_UPGRADE_CONTROL_POINT:
			if ((0x00 >= length) || (0x05 < length)) {
				break;
			}
			ws_upgrade_ota_handle_command(attrPtr, length);

			extern UINT8 ws_upgrade_state;
			if (ws_upgrade_state) {
				if (STATE_UPDATE != pw_s.state) {
					wni_pw6_change_state(STATE_UPDATE);
				}
			}
			break;
		case HANDLE_WS_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR:
			if (0x02 != length) {
				break;
			}
			ws_upgrade_ota_handle_configuration(attrPtr, length);
			break;
		case HANDLE_WS_UPGRADE_DATA:
			if ((0x00 >= length) || (WS_UPGRADE_MAX_DATA_LEN < length)) {
				break;
			}
			ws_upgrade_ota_handle_data(attrPtr, length);
			break;
		default:
			ble_trace2("wrong handle:0x04%x, length: 0x%02x\n", length, handle);
			return 0x80;
//			break;
	}
	return 0;
}

static void wni_pw6_start_advertisement(void) {
	BLE_ADV_FIELD adv[0x02];
	adv[0x00].len = 0x01 + 0x01;
	adv[0x00].val = ADV_FLAGS;
	adv[0x00].data[0x00] = LE_LIMITED_DISCOVERABLE | BR_EDR_NOT_SUPPORTED;
	adv[0x01].len = 0x10 + 0x01;
	adv[0x01].val = ADV_SERVICE_UUID128_COMP;
	memcpy(adv[0x01].data, service_uuid, 0x10);
	BLE_ADV_FIELD scr[0x01];
	scr[0x00].len = strlen(bleprofile_p_cfg->local_name) + 0x01;
	scr[0x00].val = ADV_LOCAL_NAME_COMP;
	memcpy(scr[0x00].data, bleprofile_p_cfg->local_name, scr[0x00].len - 0x01);
	bleprofile_GenerateADVData(adv, 0x02);
	bleprofile_GenerateScanRspData(scr, 0x01);
	blecm_setTxPowerInADV(0x00);
	bleprofile_Discoverable(HIGH_UNDIRECTED_DISCOVERABLE, NULL);

	wni_pw6_change_state(STATE_ADV);
}

static void wni_pw6_stop_advertisement(void) {
	bleprofile_Discoverable(NO_DISCOVERABLE, NULL);
}

static wni_pw6_manage_state(void) {
	if (!pw_s.timer) {
		return;
	}
	if (!--pw_s.timer) {
		if ((STATE_BLUE <= pw_s.state) && (STATE_FULL >= pw_s.state)) {
			wni_pw6_count_sneeze();
		}
		wni_pw6_change_state(pw_table[pw_s.state].next_state);
		pw_s.wakeup = 0x00;
	}
}

static wni_pw6_change_state(enum PW_STATE new_state) {
	ble_trace2("[STATE] %d -> %d\n", pw_s.state, new_state);
	pw_s.state = new_state;
	pw_s.timer = pw_table[pw_s.state].timeout;
	pw_s.led_counter = 0x00;
}

static wni_pw6_control_led(void) {
	if (!pw_table[pw_s.state].led_color) {
		gpio_configurePin(GPIO_PIN_P25 >> 0x04, GPIO_PIN_P25 & 0x0f, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
		gpio_configurePin(GPIO_PIN_P4 >> 0x04, GPIO_PIN_P4 & 0x0f, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
		gpio_configurePin(GPIO_PIN_P24 >> 0x04, GPIO_PIN_P24 & 0x0f, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
		return;
	}
	if (!pw_s.led_counter) {
		pw_s.led_counter = pw_table[pw_s.state].led_pattern >> 0x04;
		if (pw_table[pw_s.state].led_color & PW_LED_COLOR_RED) {
			gpio_configurePin(GPIO_PIN_P25 >> 0x04, GPIO_PIN_P25 & 0x0f, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
		}
		if (pw_table[pw_s.state].led_color & PW_LED_COLOR_GREEN) {
			gpio_configurePin(GPIO_PIN_P4 >> 0x04, GPIO_PIN_P4 & 0x0f, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
		}
		if (pw_table[pw_s.state].led_color & PW_LED_COLOR_BLUE) {
			gpio_configurePin(GPIO_PIN_P24 >> 0x04, GPIO_PIN_P24 & 0x0f, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_HIGH);
		}
		return;
	}
	if (--pw_s.led_counter == (pw_table[pw_s.state].led_pattern & PW_LED_PATTERN_OFF_MASK)) {
		gpio_configurePin(GPIO_PIN_P25 >> 0x04, GPIO_PIN_P25 & 0x0f, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
		gpio_configurePin(GPIO_PIN_P4 >> 0x04, GPIO_PIN_P4 & 0x0f, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
		gpio_configurePin(GPIO_PIN_P24 >> 0x04, GPIO_PIN_P24 & 0x0f, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
	}
}

static void wni_pw6_count_sneeze(void) {
	UINT8 count[WNI_PW6_NVRAM_COUNT_LENGTH];

	UINT8 readbyte = bleprofile_ReadNVRAM(WNI_PW6_NVRAM_PAGE_COUNT, sizeof(count), &count[0x00]);
	if (WNI_PW6_NVRAM_COUNT_LENGTH != readbyte) {
		ble_trace0("Fail to bleprofile_ReadNVRAM(WNI_PW6_NVRAM_PAGE_COUNT)\n");
		// LED?ÅöÅö
		wni_pw6_change_state(STATE_ERROR);
		return;
	}
	UINT32 reference_second = *(UINT32 *)(&count[0x00]);
	if (!reference_second) {
		ble_trace0("Reference time has not been set yet!\n");
		// LED??Åö
		wni_pw6_change_state(STATE_ERROR);
		return;
	}
	if (WNI_PW6_NUMBER_OF_DATA <= count[WNI_PW6_COUNT]) {
		ble_trace0("NVRAM has been FULL!\n");
		// LEDÅö
		wni_pw6_change_state(STATE_ERROR);
		return;
	}
	count[WNI_PW6_COUNT]++;

	RtcTime current_time;
	UINT32 current_second;
	rtc_getRTCTime(&current_time);
	rtc_RtcTime2Sec(&current_time, &current_second);
	UINT32 sneeze_second = current_second - reference_second;

	UINT8 nvram[WNI_PW6_NVRAM_PAGE_SIZE];
	UINT8 page = ((count[WNI_PW6_COUNT] - 0x01) / WNI_PW6_NVRAM_NUMBER_OF_DATA_IN_PAGE) + WNI_PW6_NVRAM_PAGE_DATA_START;
	readbyte = bleprofile_ReadNVRAM(page, sizeof(nvram), &nvram[0x00]);
	if (WNI_PW6_NVRAM_PAGE_SIZE != readbyte) {
		ble_trace1("Fail to bleprofile_ReadNVRAM(%d)\n", page);
		// LED?ÅöÅö
		wni_pw6_change_state(STATE_ERROR);
		return;
	}
	UINT8 base = ((count[WNI_PW6_COUNT] - 0x01) % WNI_PW6_NVRAM_NUMBER_OF_DATA_IN_PAGE) * WNI_PW6_DATA_LENGTH;
	nvram[base] = (UINT8)(sneeze_second >> 0x10);
	nvram[base + 0x01] = (UINT8)((sneeze_second >> 0x08) & 0xff);
	nvram[base + 0x02] = (UINT8)(sneeze_second & 0xff);
	UINT8 writtenbyte = bleprofile_WriteNVRAM(page, sizeof(nvram), &nvram[0x00]);
	if (WNI_PW6_NVRAM_PAGE_SIZE != writtenbyte) {
		ble_trace1("Fail to bleprofile_WriteNVRAM(%d)\n", page);
		// LED?ÅöÅö
		wni_pw6_change_state(STATE_ERROR);
		return;
	}

	writtenbyte = bleprofile_WriteNVRAM(WNI_PW6_NVRAM_PAGE_COUNT, sizeof(count), &count[0x00]);
	if (WNI_PW6_NVRAM_COUNT_LENGTH != writtenbyte) {
		ble_trace0("Fail to bleprofile_WriteNVRAM(WNI_PW6_NVRAM_PAGE_COUNT)\n");
		// LED?ÅöÅö
		wni_pw6_change_state(STATE_ERROR);
		return;
	}
}

static void wni_pw6_sneeze_data_transmitter(void) {
	BLEPROFILE_DB_PDU p_pdu;

	if (!pw_t.continued) {
		return;
	}

	while ((pw_t.number_of_sneeze - pw_t.transmitted) && (blecm_getAvailableTxBuffers())) {
		UINT8 nvram[WNI_PW6_NVRAM_PAGE_SIZE];
		UINT8 page = (pw_t.transmitted / WNI_PW6_NVRAM_NUMBER_OF_DATA_IN_PAGE) + WNI_PW6_NVRAM_PAGE_DATA_START;
		UINT8 readbyte = bleprofile_ReadNVRAM(page, sizeof(nvram), &nvram[0x00]);
		if (WNI_PW6_NVRAM_PAGE_SIZE != readbyte) {
			ble_trace1("Fail to bleprofile_ReadNVRAM(%d)\n", page);
			break;
		}

		p_pdu.pdu[0x00] = pw_t.transmitted + 0x01;
		memcpy(&p_pdu.pdu[WNI_PW6_PACKET_DATA_START], &nvram[0x00], WNI_PW6_PACKET_DATA_SIZE);
		if (pw_t.number_of_sneeze > pw_t.transmitted + WNI_PW6_PACKET_NUMBER_OF_DATA) {
			pw_t.transmitted += WNI_PW6_PACKET_NUMBER_OF_DATA;
		} else {
			pw_t.transmitted = pw_t.number_of_sneeze;
			pw_t.continued = 0x00;
		}
		p_pdu.pdu[0x01] = pw_t.transmitted;
		p_pdu.len = WNI_PW6_PACKET_LENGTH;
		bleprofile_WriteHandle(WNI_PW6_DATA_HANDLE_VALUE, &p_pdu);
		bleprofile_sendNotification(WNI_PW6_DATA_HANDLE_VALUE, (UINT8 *)p_pdu.pdu, p_pdu.len);
	}
}

static void wni_pw6_calculate_time_to_deepsleep(UINT8 to, UINT8 from, RtcTime current_time) {
	UINT16 time_to_deepsleep = 0x00;
	if (to != from) {
		if (to < from) {
			if ((current_time.hour >= to) && (current_time.hour < from)) {
				time_to_deepsleep = (from - current_time.hour - 0x01) * 60 * 60;
				time_to_deepsleep += (60 - current_time.minute) * 60;
			}
		} else {
			if (current_time.hour >= to) {
				time_to_deepsleep = (from + 24 - current_time.hour - 0x01) * 60 * 60;
				time_to_deepsleep += (60 - current_time.minute) * 60;
			} else if (current_time.hour < from) {
				time_to_deepsleep = (from - current_time.hour - 0x01) * 60 * 60;
				time_to_deepsleep += (60 - current_time.minute) * 60;
			}
		}
	}
	pw_s.time_to_deepsleep = time_to_deepsleep;
}

static void wni_pw6_entering_deep_sleep(void) {
	ble_trace0("*wni_pw6_entering_deep_sleep()\n");
	devLpmConfig.disconnectedLowPowerMode = DEV_LPM_DISC_LOW_POWER_MODES_HID_OFF;
	devLpmConfig.wakeFromHidoffInMs = 36*60*60*1000;	// 36[h]@ms
	devLpmConfig.wakeFromHidoffRefClk = HID_OFF_TIMED_WAKE_CLK_SRC_128KHZ;
	devlpm_enterLowPowerMode();
}

