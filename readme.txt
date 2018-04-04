1、 输入捕捉用的是计数器，因此分频还有计数值还是要设置的
2、 adc设置的时候如果 ADC_Regular_ConversionMode那个rank突然多出来一大坨，把num of conversion减到1， 如果还是不行，把enable regular conversions弄成disable，再enable，再从头来设置。
3、 激频后的延时是20ms
4、计时器重装值一定是65535，不管up还是down
5、串口中断给信号量一定是在cpltcallback，因为一个字符进一次irq
6、协调器不能用ffff当地址
