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
//#define BAT_LOW_INTERRUPT    1
//#define CW2015_GET_RRT

/*电池建模信息，客户拿到自己电池匹配的建模信息后请替换*/
static unsigned char cw_bat_config_info[SIZE_BATINFO] = {
    0x14,0xA6,0x70,0x6C,0x6C,0x67,0x64,
    0x62,0x60,0x5C,0x59,0x4E,0x50,0x42,
    0x2E,0x26,0x25,0x24,0x27,0x2E,0x41,
    0x5A,0x6D,0x4C,0x39,0xD7,0x06,0x66,
    0x00,0x01,0x05,0x34,0x61,0x76,0x79,
    0x7D,0x42,0x1E,0xA4,0x00,0x27,0x5A,
    0x52,0x87,0x8F,0x91,0x94,0x52,0x82,
    0x8C,0x92,0x96,0x2B,0x59,0x6E,0xCB,
    0x2F,0x7D,0x72,0xA5,0xB5,0xC1,0x94,
    0x99
};

//****************************struct*********************************/
typedef struct tagSTRUCT_CW_BATTERY {
	unsigned int capacity;
	unsigned int voltage;
	unsigned char alt;
}STRUCT_CW_BATTERY;

int cw_bat_init(void);
void cw_bat_work(void);

#endif




