#include "CW201x.h"
#include "IIC.h"
#include "stm32f1xx_hal.h"

unsigned char CHARGE = 0;
unsigned char reset_loop =0;

STRUCT_CW_BATTERY cw_bat;

void CW_Delay10ms(unsigned int c) 
{
  for (;c>0;c--)
	{
     HAL_Delay(10);
	}       
}

void CW_Delay100us()
{
	unsigned char a,b;
	for(b=1;b>0;b--)
		for(a=97;a>0;a--);
}

unsigned char cw_update_config_info(void)
{
	unsigned char ret = 0;
	unsigned char i;
	unsigned char reset_val;
	unsigned char reg_val;
    
	/* make sure no in sleep mode */
	ret = cw_read(REG_MODE, &reg_val);
	if(ret)
	{
		return 1;
	}
	if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP)
	{
		return 2;
	}
	/* update new battery info */
	for(i = 0; i < SIZE_BATINFO; i++)
	{
		reg_val = cw_bat_config_info[i];
		ret = cw_write(REG_BATINFO+i, &reg_val);
		if(ret)
		{
			return 1;
		}
	}

	/* readback & check */
	for(i = 0; i < SIZE_BATINFO; i++)
	{
		ret = cw_read(REG_BATINFO+i, &reg_val);
		if(ret)
		{
			return 1;
		}
		if(reg_val != cw_bat_config_info[i])
		{
			return 3;
		}
	}
	/* set cw2015/cw2013 to use new battery info */
	ret = cw_read(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	reg_val |= CONFIG_UPDATE_FLG;   /* set UPDATE_FLAG */
	reg_val &= 0x07;                /* clear ATHD */
	reg_val |= ATHD;                /* set ATHD */
	ret = cw_write(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	/* reset */
	reset_val = MODE_NORMAL;
	reg_val = MODE_RESTART;
	ret = cw_write(REG_MODE, &reg_val);
	if(ret)
	{
		return 1;
	}
	CW_Delay100us();  //delay 100us      
	ret = cw_write(REG_MODE, &reset_val);
	if(ret)
	{
		return 1;
	}   
	return 0;
}

unsigned char cw_init(void)
{
	unsigned char ret;
	unsigned char i;
	unsigned char reg_val = MODE_NORMAL;

	/* wake up cw2015/13 from sleep mode */
	ret = cw_write(REG_MODE, &reg_val);
	if(ret)
	{
		return 1;
	}

	/* check ATHD if not right */
	ret = cw_read(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	if((reg_val & 0xf8) != ATHD)
	{
		//"the new ATHD need set"
		reg_val &= 0x07;    /* clear ATHD */
		reg_val |= ATHD;    /* set ATHD */
		ret = cw_write(REG_CONFIG, &reg_val);
		if(ret)
		{
			return 1;
		}
	}
	
	/* check config_update_flag if not right */
	ret = cw_read(REG_CONFIG, &reg_val);
	if(ret)
	{
		return 1;
	}
	if(!(reg_val & CONFIG_UPDATE_FLG))
	{
		//"update flag for new battery info need set"
		ret = cw_update_config_info();
		if(ret)
		{
			return ret;
		}
	}
	else
	{
		for(i = 0; i < SIZE_BATINFO; i++)
		{ 
			ret = cw_read(REG_BATINFO +i, &reg_val);
			if(ret)
			{
				return 1;
			}
			if(cw_bat_config_info[i] != reg_val)
			{
				break;
			}
		}
		if(i != SIZE_BATINFO)
		{
			//"update flag for new battery info need set"
			ret = cw_update_config_info();
			if(ret)
			{
				return ret;
			}
		}
	}
	/* check SOC if not eqaul 255 */
	for (i = 0; i < 30; i++) {
		CW_Delay10ms(10);//delay 100ms
		ret = cw_read(REG_SOC, &reg_val);
		if (ret)
			return 1;
		else if (reg_val <= 100) 
			break;		
    }
	
    if (i >=30){
        reg_val = MODE_SLEEP;
        ret = cw_write(REG_MODE, &reg_val);
        // "cw2015/cw2013 input unvalid power error_2\n";
        return 4;
    } 
	return 0;
}

int cw_por(void)
{
	int ret = 0;
	unsigned char reset_val = 0;
	reset_val = MODE_SLEEP;             
	ret = cw_write(REG_MODE, &reset_val);
	if (ret)
		return -1;
	CW_Delay100us(); //delay 100us
	
	reset_val = MODE_NORMAL;
	ret = cw_write(REG_MODE, &reset_val);
	if (ret)
		return -1;
	CW_Delay100us(); //delay 100us
	
	ret = cw_init();
	if (ret) 
		return ret;
	return 0;
}

int cw_get_capacity(void)
{
	int ret = 0;
	unsigned char reg_val;
	unsigned char cw_capacity;

	ret = cw_read(REG_SOC, &reg_val);
	if(ret)
	{
		return -1;
	}
        
	cw_capacity = reg_val;
	if ((cw_capacity < 0) || (cw_capacity > 100)) 
	{
    reset_loop++;
		if (reset_loop >5) 
		{ 
			ret = cw_por(); //por ic
			if(ret)
				return -1;
			reset_loop =0;               
		}                   
    return cw_bat.capacity;
  }
	else
	{
		reset_loop =0;
  }
	return(cw_capacity);
}

unsigned int cw_get_vol(void)
{
	unsigned char ret = 0;
	unsigned char get_ad_times = 0;
	unsigned char reg_val[2] = {0 , 0};
	unsigned long ad_value = 0;
	unsigned int ad_buff = 0;
	unsigned int ad_value_min = 0;
	unsigned int ad_value_max = 0;

	for(get_ad_times = 0; get_ad_times < 3; get_ad_times++)
	{
		ret = cw_read_word(REG_VCELL, &reg_val[0],2);
		if(ret)
		{
			return 1;
		}
		ad_buff = (reg_val[0] << 8) + reg_val[1];

		if(get_ad_times == 0)
		{
			ad_value_min = ad_buff;
			ad_value_max = ad_buff;
		}
		if(ad_buff < ad_value_min)
		{
			ad_value_min = ad_buff;
		}
		if(ad_buff > ad_value_max)
		{
			ad_value_max = ad_buff;
		}
		ad_value += ad_buff;
	}
	ad_value -= ad_value_min;
	ad_value -= ad_value_max;
	ad_value = (ad_value  * 305 + 50000) / 100000;
	return(ad_value); 
}

void update_capacity(void)
{
	int cw_capacity;
	cw_capacity = cw_get_capacity();
	if((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat.capacity != cw_capacity))
	{       
		cw_bat.capacity = cw_capacity;
	}
}


void update_vol(void)
{
	unsigned int cw_voltage;
	cw_voltage = cw_get_vol();
	if(cw_voltage == 1)
	{
		cw_bat.voltage = cw_bat.voltage;
	}
	else if(cw_bat.voltage != cw_voltage)
	{
		cw_bat.voltage = cw_voltage;
	}
}

void update_usb_online(void)
{
	if(CHARGE == 1) 
	{
		cw_bat.usb_online = 1;
	}
	else
	{
		cw_bat.usb_online = 0;
	}
}

void cw_bat_work(void)
{
	update_usb_online();
	update_capacity();
	update_vol();
}

unsigned char cw_bat_init(void)
{
	unsigned char ret;
	unsigned char loop = 0;
	
	ret = cw_init();
	while((loop++ < 200) && (ret != 0))
	{
		ret = cw_init();
	}
	
	cw_bat.usb_online = 0;
	cw_bat.capacity = 2;
	cw_bat.voltage = 0;
	cw_bat.alt = 0;
	
	return ret;	
}
