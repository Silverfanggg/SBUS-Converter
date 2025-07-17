#include <Arduino.h>
#include "driver/uart.h"

#define SBUS_RX_PIN 16
#define SBUS_TX_PIN 17
#define SBUS_UART_RX_NUM UART_NUM_1
#define SBUS_UART_TX_NUM UART_NUM_2
#define SBUS_BAUD 100000
#define SBUS_FRAME_LEN 25

#define SBUS_MIN 173
#define SBUS_MID 992
#define SBUS_MAX 1811

#define MODE_NORMAL 0
#define MODE_START 1
#define MODE_STOP 2
#define MODE_FORWARD 3
#define MODE_BACKWARD 4
#define MODE_LEFT 5
#define MODE_RIGHT 6
#define MODE_UP 7
#define MODE_DOWN 8

HardwareSerial sbusIn(SBUS_UART_RX_NUM);
HardwareSerial sbusOut(SBUS_UART_TX_NUM);

void decodeSBUSFrame(const uint8_t *buf, uint16_t *ch)
{
  ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
  ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
  ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
  ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);
  ch[4] = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
  ch[5] = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);
  ch[6] = ((buf[9] >> 2 | buf[10] << 6) & 0x07FF);
  ch[7] = ((buf[10] >> 5 | buf[11] << 3) & 0x07FF);
  ch[8] = ((buf[12] | buf[13] << 8) & 0x07FF);
  ch[9] = ((buf[13] >> 3 | buf[14] << 5) & 0x07FF);
  ch[10] = ((buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF);
  ch[11] = ((buf[16] >> 1 | buf[17] << 7) & 0x07FF);
  ch[12] = ((buf[17] >> 4 | buf[18] << 4) & 0x07FF);
  ch[13] = ((buf[18] >> 7 | buf[19] << 1 | buf[20] << 9) & 0x07FF);
  ch[14] = ((buf[20] >> 2 | buf[21] << 6) & 0x07FF);
  ch[15] = ((buf[21] >> 5 | buf[22] << 3) & 0x07FF);
}

void buildSBUSFrame(const uint16_t *ch, uint8_t *buf)
{
  buf[0] = 0x0F;
  buf[1] = ch[0] & 0xFF;
  buf[2] = ((ch[0] >> 8) & 0x07) | ((ch[1] & 0x07FF) << 3);
  buf[3] = ((ch[1] >> 5) & 0x3F) | ((ch[2] & 0x07FF) << 6);
  buf[4] = (ch[2] >> 2) & 0xFF;
  buf[5] = ((ch[2] >> 10) & 0x01) | ((ch[3] & 0x07FF) << 1);
  buf[6] = ((ch[3] >> 7) & 0x0F) | ((ch[4] & 0x07FF) << 4);
  buf[7] = ((ch[4] >> 4) & 0x7F) | ((ch[5] & 0x07FF) << 7);
  buf[8] = (ch[5] >> 1) & 0xFF;
  buf[9] = ((ch[5] >> 9) & 0x03) | ((ch[6] & 0x07FF) << 2);
  buf[10] = ((ch[6] >> 6) & 0x1F) | ((ch[7] & 0x07FF) << 5);
  buf[11] = (ch[7] >> 3) & 0xFF;
  buf[12] = ch[8] & 0xFF;
  buf[13] = ((ch[8] >> 8) & 0x07) | ((ch[9] & 0x07FF) << 3);
  buf[14] = ((ch[9] >> 5) & 0x3F) | ((ch[10] & 0x07FF) << 6);
  buf[15] = (ch[10] >> 2) & 0xFF;
  buf[16] = ((ch[10] >> 10) & 0x01) | ((ch[11] & 0x07FF) << 1);
  buf[17] = ((ch[11] >> 7) & 0x0F) | ((ch[12] & 0x07FF) << 4);
  buf[18] = ((ch[12] >> 4) & 0x7F) | ((ch[13] & 0x07FF) << 7);
  buf[19] = (ch[13] >> 1) & 0xFF;
  buf[20] = ((ch[13] >> 9) & 0x03) | ((ch[14] & 0x07FF) << 2);
  buf[21] = ((ch[14] >> 6) & 0x1F) | ((ch[15] & 0x07FF) << 5);
  buf[22] = (ch[15] >> 3) & 0xFF;
  buf[23] = 0x00;
  buf[24] = 0x00;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    delay(10);
  }
  Serial.println("SBUS Command Listener (start/stop/dir)");
  sbusIn.begin(SBUS_BAUD, SERIAL_8E2, SBUS_RX_PIN, -1);
  uart_set_line_inverse(SBUS_UART_RX_NUM, UART_SIGNAL_RXD_INV);
  sbusOut.begin(SBUS_BAUD, SERIAL_8E2, -1, SBUS_TX_PIN);
  uart_set_line_inverse(SBUS_UART_TX_NUM, UART_SIGNAL_TXD_INV);
}

void loop()
{
  static uint8_t sbusBuf[SBUS_FRAME_LEN];
  static uint16_t channels[16];
  static uint8_t idx = 0;
  static uint8_t mode = MODE_NORMAL;

  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("start"))
      mode = MODE_START;
    else if (cmd.equalsIgnoreCase("stop"))
      mode = MODE_STOP;
    else if (cmd.equalsIgnoreCase("forward"))
      mode = MODE_FORWARD;
    else if (cmd.equalsIgnoreCase("backward"))
      mode = MODE_BACKWARD;
    else if (cmd.equalsIgnoreCase("left"))
      mode = MODE_LEFT;
    else if (cmd.equalsIgnoreCase("right"))
      mode = MODE_RIGHT;
    else if (cmd.equalsIgnoreCase("up"))
      mode = MODE_UP;
    else if (cmd.equalsIgnoreCase("down"))
      mode = MODE_DOWN;
    else
      mode = MODE_NORMAL;
    Serial.print("MODE = ");
    Serial.println(cmd);
  }

  while (sbusIn.available())
  {
    uint8_t b = sbusIn.read();
    if (idx == 0 && b != 0x0F)
      continue;
    sbusBuf[idx++] = b;
    if (idx < SBUS_FRAME_LEN)
      continue;

    decodeSBUSFrame(sbusBuf, channels);
    channels[4] = SBUS_MAX;

    switch (mode)
    {
    case MODE_START:
      channels[0] = 272;
      channels[1] = 1712;
      channels[2] = 272;
      channels[3] = 1712;
      break;
    case MODE_STOP:
      channels[0] = 1712;
      channels[1] = 1712;
      channels[2] = 272;
      channels[3] = 272;
      break;
    case MODE_FORWARD:
      channels[0] = SBUS_MID;
      channels[1] = SBUS_MAX;
      channels[2] = SBUS_MID;
      channels[3] = SBUS_MID;
      break;
    case MODE_BACKWARD:
      channels[0] = SBUS_MID;
      channels[1] = SBUS_MIN;
      channels[2] = SBUS_MID;
      channels[3] = SBUS_MID;
      break;
    case MODE_LEFT:
      channels[0] = SBUS_MIN;
      channels[1] = SBUS_MID;
      channels[2] = SBUS_MID;
      channels[3] = SBUS_MID;
      break;
    case MODE_RIGHT:
      channels[0] = SBUS_MAX;
      channels[1] = SBUS_MID;
      channels[2] = SBUS_MID;
      channels[3] = SBUS_MID;
      break;
    case MODE_UP:
      channels[0] = SBUS_MID;
      channels[1] = SBUS_MID;
      channels[2] = SBUS_MAX;
      channels[3] = SBUS_MID;
      break;
    case MODE_DOWN:
      channels[0] = SBUS_MID;
      channels[1] = SBUS_MID;
      channels[2] = SBUS_MIN;
      channels[3] = SBUS_MID;
      break;
    default:
      break;
    }

    uint8_t txPkt[SBUS_FRAME_LEN];
    buildSBUSFrame(channels, txPkt);
    sbusOut.write(txPkt, SBUS_FRAME_LEN);

    Serial.print("Out CH1-4: ");
    for (int i = 0; i < 4; i++)
    {
      Serial.print(channels[i]);
      Serial.print(i < 3 ? "  " : "\n");
    }
    idx = 0;
  }
}
