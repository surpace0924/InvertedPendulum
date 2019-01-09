
#include <16f1827.h>
#fuses HS, NOWDT, NOPROTECT, PUT, BROWNOUT, NOLVP, MCLR
#device ADC = 8
#use delay(CLOCK = 16000000)
#use rs232(BAUD = 57600, XMIT = PIN_B2, RCV = PIN_B1)
#byte port_a = 0x00c
#byte port_b = 0x00d
#use fast_io(a)
#use fast_io(b)

const unsigned int CW = 0;
const unsigned int CCW = 1;
const unsigned int BRK = 2;
const unsigned int MT_FREQ = 7.4;
const unsigned int PWM_CNT = (int)(1000.0 / MT_FREQ) - 1;

int dir = 0;
int pwm = 0;

//プロトタイプ宣言
void initSystem(void);
void driveMotor(void);

// シリアル受信割り込み
// 受信後，ビットマスクしてデータをデコード
#INT_RDA
void rda(void)
{
   unsigned char rcv_data;
   rcv_data = getc();
   dir = rcv_data >> 7;
   pwm = (int)((float)(rcv_data & 0b01111111) / 100 * PWM_CNT) + 1;
}

void initSystem(void)
{
   setup_oscillator(OSC_NORMAL | OSC_16MHZ | OSC_PLL_ON);

   // ピン関係
   set_tris_a(0);
   set_tris_b(0b00000010);
   output_a(0);
   output_b(0);

   // シリアル通信関係
   setup_uart(true);
   set_uart_speed(57600);
   enable_interrupts(INT_RDA);
   enable_interrupts(GLOBAL);

   // CCP(PWM)関係
   setup_ccp1(CCP_PWM);
   setup_ccp2(CCP_PWM);
   setup_timer_2(T2_DIV_BY_4, PWM_CNT, 1);
   set_pwm1_duty(0);
   set_pwm2_duty(0);

   delay_ms(500);
}

void driveMotor(void)
{
   switch (dir)
   {
   case CW:
      set_pwm1_duty(0);
      set_pwm2_duty(pwm);
      break;

   case CCW:
      set_pwm1_duty(pwm);
      set_pwm2_duty(0);
      break;

   case BRK:
   default:
      set_pwm1_duty(PWM_CNT);
      set_pwm2_duty(PWM_CNT);
      break;
   }
}

void main(void)
{
   initSystem();

   while (true)
   {
      driveMotor();
   }
}
