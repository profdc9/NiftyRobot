
#include <Arduino.h>
#include <wiring_private.h>
#include <SPI.h>

#include "consoleio.h"
#include "tinycl.h"

#define CURSEN1 A0
#define CURSEN2 A1
#define PHOTLEVELOUT A2
#define ULTRASOUNDOUT A3
#define LIGHTLEVEL A4
#define MICOUT A5

#define ADCCURSEN1 0
#define ADCCURSEN2 1
#define ADCPHOTLEVELOUT 2
#define ADCULTRASOUNDOUT 3
#define ADCLIGHTLEVEL 4
#define ADCMICOUT 5

#define HALLSENS1_1 2
#define HALLSENS1_2 3
#define ULTRASOUNDTHR 4
#define MOTORPWM1 5
#define MOTORPWM2 6
#define ULTRATRANS 7
#define HALLSENS2_1 8
#define HALLSENS2_2 9
#define LATCH 10
#define MOSI 11
#define MICTHR 12
#define SCK 13

uint16_t quickADC(uint8_t ch)
{
  uint8_t low, high;
  ADMUX = (1 << REFS0) | ch;
  sbi(ADCSRA, ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  low = ADCL;
  high = ADCH;
  return (((uint16_t)high) << 8) | low;
}

uint16_t quickADCmult(uint8_t ch, uint8_t rep)
{
  uint16_t ct = 0;
  while (rep > 0)
  {
    rep--;
    ct += quickADC(ch);
  }
  return ct;
}

void fastADC() {
  ADMUX = (1 << REFS0);
  ADCSRB = 0;
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS0);
}


uint8_t spi_outputs[2];

void transfer_spi_outputs(void)
{
  SPI.transfer(spi_outputs[1]);
  SPI.transfer(spi_outputs[0]);
  digitalWrite(LATCH, HIGH);
  digitalWrite(LATCH, LOW);
}

void motor_1_dir(uint8_t dir)
{
  spi_outputs[0] = (spi_outputs[0] & 0xE7) | (dir << 3);
}

void motor_2_dir(uint8_t dir)
{
  spi_outputs[0] = (spi_outputs[0] & 0xF9) | (dir << 1);
}

void select_light_detector(uint8_t det)
{
  const uint8_t det_ary[8] = { 0x00, 0x08, 0x04, 0x0C, 0x02, 0x0A, 0x06, 0x0E };
  spi_outputs[1] = (spi_outputs[1] & 0xF1) | (det << 1);
}

void select_ambient_detector(uint8_t amb, uint8_t on)
{
  const uint8_t amb_ary[8] = { 0x00, 0x40, 0x20, 0x60, 0x10, 0x50, 0x30, 0x70 };
  spi_outputs[1] = (spi_outputs[1] & 0x0F) | (amb << 4) | (on ? 0x80 : 0x00);
}

uint16_t read_light_detector(uint8_t det)
{
  select_ambient_detector(det, 1);
  transfer_spi_outputs();
  delay(100);
  return quickADCmult(ADCPHOTLEVELOUT,50);
}

void ultrasonic_range(uint16_t *range, uint16_t *maxmag)
{
  uint8_t i;
  uint8_t j;
  uint8_t p0 = PORTD & 0x7F;
  uint8_t p1 = p0 | 0x80;
  noInterrupts();
  for (i=10;i>0;i--)
  {
    PORTD = p1;
    for (j=49;j>0;j--)
       __asm__ __volatile__ ("nop\n\t");
    PORTD = p0;
    for (j=49;j>0;j--)
       __asm__ __volatile__ ("nop\n\t");
  }
  interrupts();
  delayMicroseconds(2000);
  ADMUX = (1 << REFS0) | ADCULTRASOUNDOUT;
  
  uint16_t maxval = 0;
  uint32_t starttime = micros();
  uint32_t maxtime = starttime;
  for (uint16_t j=1000;j>0;j--)
  {  
    uint8_t low, high;
    uint16_t val;
    sbi(ADCSRA, ADSC);
    while (bit_is_set(ADCSRA, ADSC));
    low = ADCL;
    high = ADCH;
    val = (((uint16_t)high) << 8) | low;
    if (val > maxval)
    {
      maxval = val;
      maxtime = micros();
    }
  }
  *range = (maxtime - starttime) + 2000;
  *maxmag = maxval;
  return;
}

void setup() {
  Serial.begin(115200);
  console_setMainSerial(&Serial);
  SPI.begin();
  fastADC();
  pinMode(CURSEN1, INPUT);
  pinMode(CURSEN2, INPUT);
  pinMode(PHOTLEVELOUT, INPUT);
  pinMode(ULTRASOUNDOUT, INPUT);
  pinMode(LIGHTLEVEL, INPUT);
  pinMode(MICOUT, INPUT);
  pinMode(HALLSENS1_1, INPUT);
  pinMode(HALLSENS1_2, INPUT);
  pinMode(ULTRASOUNDTHR, INPUT);
  pinMode(MOTORPWM1, OUTPUT);
  pinMode(MOTORPWM2, OUTPUT);
  pinMode(ULTRATRANS, OUTPUT);
  pinMode(HALLSENS2_1, INPUT);
  pinMode(HALLSENS2_2, INPUT);
  pinMode(LATCH, OUTPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(MICTHR, INPUT);
  pinMode(SCK, OUTPUT);
  digitalWrite(MOTORPWM1, LOW);
  digitalWrite(MOTORPWM2, LOW);
}

void print_vals(uint8_t c)
{
  uint16_t range,maxval;
  ultrasonic_range(&range, &maxval);
  Serial.print("range: ");
  Serial.print(range);
  Serial.print(" maxval: ");
  Serial.println(maxval);
  while (c > 0)
  {
    c--;
    delay(250);
    uint16_t d1 = read_light_detector(4);
    uint16_t d2 = read_light_detector(5);
    uint16_t d3 = read_light_detector(6);
    uint16_t d4 = read_light_detector(7);
    Serial.print("detectors: ");
    Serial.print(d1);
    Serial.print(" ");
    Serial.print(d2);
    Serial.print(" ");
    Serial.print(d3);
    Serial.print(" ");
    Serial.println(d4);
  }
}

int print_cmd(int args, tinycl_parameter* tp, void *v)
{
  int n = tp[0].ti.i;

  console_print("Number: ");
  console_println(n);
  return 1;
}

const tinycl_command tcmds[] =
{
  { "PRINT", "Print Number", print_cmd, TINYCL_PARM_INT, TINYCL_PARM_END },
  { "HELP", "Display This Help", help_cmd, {TINYCL_PARM_END } },
};

int help_cmd(int args, tinycl_parameter *tp, void *v)
{
  tinycl_print_commands(sizeof(tcmds) / sizeof(tinycl_command), tcmds);
  return 1;
}

void loop() {

  if (tinycl_task(sizeof(tcmds) / sizeof(tinycl_command), tcmds, NULL))
  {
    tinycl_do_echo = 1;
    console_print("> ");
  }

  print_vals(1);
  motor_1_dir(0);
  motor_2_dir(0);
  transfer_spi_outputs();
  Serial.print("dir1 ");
  Serial.println(spi_outputs[0], HEX);
  analogWrite(MOTORPWM1, 128);
  analogWrite(MOTORPWM2, 128);

  print_vals(1);
  motor_1_dir(0);
  motor_2_dir(0);
  transfer_spi_outputs();
  Serial.print("dir2 ");
  Serial.println(spi_outputs[0], HEX);
  analogWrite(MOTORPWM1, 128);
  analogWrite(MOTORPWM2, 128);
}
