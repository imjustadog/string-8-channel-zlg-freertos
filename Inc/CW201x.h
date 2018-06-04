#ifndef _cw2015_battery_H_
#define _cw2015_battery_H_

#define	READ_CW2015				0xc5
#define	WRITE_CW2015			0xc4

#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_BATINFO             0x10

#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)

#define CONFIG_UPDATE_FLG       (0x1<<1)

#define ATHD                    (0x0<<3)        //ATHD = 0%

#define SIZE_BATINFO        64

#define BATTERY_UP_MAX_CHANGE   720             // the max time allow battery change quantity
#define BATTERY_DOWN_MIN_CHANGE 60             // the min time allow battery change quantity when run
#define BATTERY_DOWN_MIN_CHANGE_SLEEP 1800      // the min time allow battery change quantity when run 30min
//****************************struct*********************************/
typedef struct tagSTRUCT_CW_BATTERY {
	unsigned char usb_online;
	unsigned int capacity;
	unsigned int voltage;
	unsigned char alt;
}STRUCT_CW_BATTERY;

static unsigned char cw_bat_config_info[SIZE_BATINFO] = {
	0x15,0x8A,0x77,0x73,0x71,0x6E,0x6B,
	0x67,0x63,0x60,0x5C,0x59,0x51,0x4A,
	0x46,0x3E,0x28,0x26,0x20,0x25,0x28,
	0x32,0x4A,0x67,0x62,0x46,0x08,0xF6,
	0x04,0x07,0x31,0x4C,0x54,0x4D,0x54,
	0x59,0x3A,0x19,0x75,0xAE,0x00,0x13,
	0x52,0x87,0x8F,0x91,0x94,0x52,0x82,
	0x8C,0x92,0x96,0x92,0x63,0x9B,0xCB,
	0x2F,0x7D,0x72,0xA5,0xB5,0x0E,0x98,
	0x11};

unsigned char cw_bat_init(void);
void cw_bat_work(void);

#endif
