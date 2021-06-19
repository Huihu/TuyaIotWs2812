#include "wifi.h"
#include <SoftwareSerial.h>
#include <FastLED.h>
//
//https://www.mydigit.cn/forum.php?mod=viewthread&tid=112534
//https://developer.tuya.com/cn/docs/iot/device-development/tuya-development-board-kit/tuya-sandwich-evaluation-kits/development-guide/arduino-implements-simple-demo?id=K96yyxgetltzj
SoftwareSerial softSerial(14, 12); // RX, TX
#define _SS_MAX_RX_BUFF 300
#define ledpin 13  //开关电源引脚，高电平为开，低电平为关
#define PWM_DIN 4  //esp8266 din引脚，控制色彩
#define NUM_LEDS    30    //我的灯带一共级联了12颗LED
#define reset_btn 5  //配网按钮，在mcu启动时，低电平触发配网。ESP8266 GPIO 16不支持外中断（后续更新PCB时可以考虑换个GPIO）
#define wifi_led 2  //wifi指示灯

// int NUM_LEDS = 30;
CRGB leds[NUM_LEDS];  

int time_cnt = 0, cnt = 0, init_flag = 0;

void setup() {
  pinMode(ledpin, OUTPUT);   // led I/O初始化
  pinMode(reset_btn, INPUT_PULLUP); // 重置Wi-Fi按键初始化
  pinMode(wifi_led, OUTPUT);      // Wi-Fi状态指示灯使用ESP8266板载灯
  digitalWrite(ledpin, HIGH);   //上电亮灯
  FastLED.addLeds<WS2812, PWM_DIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);  //可以设置全局亮度，调低亮度不刺眼
  
  softSerial.begin(115200);     // 软件串口初始化
  Serial.println("myserial init successful!");
  Serial.begin(9600);
  Serial.println("serial init successful!");

  wifi_protocol_init();
}

void loop() {
  if (init_flag == 0) {
    time_cnt++;
    if (time_cnt % 6000 == 0) {
      time_cnt = 0;
      cnt ++;
    }
   wifi_stat_led(&cnt);   // Wi-Fi状态处理
  }
  wifi_uart_service();
  serialEvent();      // 串口接收处理
  key_scan();           // 重置配网按键检测
}

void serialEvent() {
  if (softSerial.available()) {
    while(softSerial.available()>0){
    unsigned char ch = (unsigned char)softSerial.read();
    uart_receive_input(ch);
//    delay(10);
    }
  }
}

void key_scan(void)
{
  static char ap_ez_change = 0;
  unsigned char buttonState  = HIGH;
  buttonState = digitalRead(reset_btn);  
  if (buttonState == LOW) {
    delay(3000);
    int wifiState = mcu_get_wifi_work_state();
    printf("==wifi stat==%d\r\n ",wifiState);
    buttonState = digitalRead(reset_btn);
//    printf("----%d",buttonState);
    if (buttonState == LOW) {
      printf("===buttonState:LOW====\r\n");
      init_flag = 0;
      switch (ap_ez_change) {
        case 0 :
          printf("SMART_CONFIG\r\n");
          mcu_set_wifi_mode(SMART_CONFIG);
          break;
        case 1 :
          printf("AP_CONFIG\r\n");
          mcu_set_wifi_mode(AP_CONFIG);
          break;
        default:
          break;
      }
      ap_ez_change = !ap_ez_change;
    }
  }
}

void wifi_stat_led(int *cnt)
{
  switch (mcu_get_wifi_work_state())
  {
    case SMART_CONFIG_STATE:  //0x00
      init_flag = 0;
      if (*cnt == 2) {
        *cnt = 0;
      }
      if (*cnt % 2 == 0)  //LED快闪
      {
        digitalWrite(wifi_led, LOW);
      }
      else
      {
        digitalWrite(wifi_led, HIGH);
      }
      break;
    case AP_STATE:  //0x01
      init_flag = 0;
      if (*cnt >= 30) {
        *cnt = 0;
      }
      if (*cnt  == 0)      // LED 慢闪
      {
        digitalWrite(wifi_led, LOW);
      }
      else if (*cnt == 15)
      {
        digitalWrite(wifi_led, HIGH);
      }
      break;

    case WIFI_NOT_CONNECTED:  // 0x02
      digitalWrite(wifi_led, HIGH); // LED 熄灭
      break;
    case WIFI_CONNECTED:  // 0x03
      break;
    case WIFI_CONN_CLOUD:  // 0x04
      if ( 0 == init_flag )
      {
        digitalWrite(wifi_led, LOW);// LED 常亮
        init_flag = 1;                  // Wi-Fi 连接上后该灯可控
        *cnt = 0;
      }

      break;

    default:
//      printf("==wifi_state:==%d\r\n 请检查1.是否配网。2.MCU与WIFI模组通信是否正常\r\n",mcu_get_wifi_work_state());//灯不亮有两种情况：1.未配网。2.MCU与WIFI模组通信异常
      digitalWrite(wifi_led, HIGH);
      break;
  }
}

void serialWrite(unsigned char value){
  softSerial.write(value);
}
//开电源引脚
void LED_UP()
{
  digitalWrite(ledpin, HIGH);
  ShowClolor(100,255,255);
}
//关电源引脚
void LED_DOWN()
{
  digitalWrite(ledpin, LOW);
  setBrightness(0);

}

//HSV:[0,255]；H（色度：0-360，0X0000-0X0168）;S (饱和：0-1000, 0X0000-0X03E8);V (明度：0-1000，0X0000-0X03E8)
void ShowClolor(unsigned long h,unsigned long s,unsigned long v) {
  int H = map(h,0,360,0,255);
  int S = map(s,0,1000,0,255);
  int V = map(v,0,1000,0,255);
  Serial.println("ccc");
  ShowClolorRaw(H,S,V);
}

void ShowClolorRaw(unsigned long H,unsigned long S,unsigned long V){
  for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CHSV(H,S,V); //用HSV色彩空间，色调h，饱和度S，亮度V
      FastLED.show();
    }
}

//设置亮度，fastLED范围[0,255],涂鸦范围[10,1000]
void setBrightness(unsigned long bright_value){
  FastLED.setBrightness(map(bright_value,10,1000,1,255));
  FastLED.show();
}
//白光模式
void setWhite(){
  ShowClolorRaw(128,128,255);
}

//渐变彩虹色
void setRainbow(){
  //debug
  mcu_dp_value_update(DPID_COUNTDOWN,1);
  fill_gradient(leds, 0, CHSV(50, 255,255) , NUM_LEDS, CHSV(150,255,255), LONGEST_HUES);
  FastLED.show();
}

//闪烁渐变
void loopColor(){
  //debug
  //mcu_dp_value_update(DPID_COUNTDOWN,2);
  
  int h = 0;
  while(1){
    for (int i = 0; i < NUM_LEDS; i++) {
          leds[i] = CHSV( h, 255, 255); //用HSV色彩空间，不断改变H即可
          FastLED.show();
      }
    delay(1000);
    h = (h + 1) % 255;
  }
}

int string_int(unsigned char temp_char){
  if(temp_char>='0' and temp_char<='9'){
    return temp_char-'0';
  }
  if(temp_char>='A' and temp_char<='Z'){
    return temp_char - 'A' + 10;
  }
  if(temp_char>='a' and temp_char<='z'){
    return temp_char - 'a' + 10;
  }
}
