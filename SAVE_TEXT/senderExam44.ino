#include <HardwareSerial.h>

#define MAX_SERIAL_BUF 1024
#define PACKET_SIZE (9*1024)+20 // 9Kbyte + 20byte
#define STX 0x02
#define ETX 0x03
//#define ACK 0x06
#define NCK 0x15

/*  Transmit Protocol Packet Define */
#define SIZE_STX        1
#define SIZE_LEN1       2
#define SIZE_LEN2       2
#define SIZE_CMD                1
#define SIZE_CRC                2
#define SIZE_ETX                1
#define SIZE_FIELD              5
#define SIZE_PAGE               1
#define SIZE_SET_DRAW_FIELD     SIZE_FIELD + SIZE_PAGE + SIZE_LEN1
#define SIZE_WIFI_SSID          1
#define SIZE_WIFI_PW            1
#define SIZE_WIFI_STATUS        1
#define SIZE_PKT_DATA     SIZE_STX + SIZE_LEN1 + SIZE_LEN2 + SIZE_CMD + SIZE_CRC + SIZE_ETX;

#define IDX_STX         0
#define IDX_LEN         IDX_STX + SIZE_STX
#define IDX_CMD         IDX_LEN + SIZE_LEN1 + SIZE_LEN2
#define IDX_CRC         IDX_CMD + SIZE_CMD
#define IDX_ETX         IDX_CRC + SIZE_CRC

/* Byte Size Define */
#define INIT_VALUE            0
#define SIZE_TEXT_LEN         1
#define SIZE_FONT_SIZE            1
#define SIZE_FONT_COLOR         3
#define SIZE_BG_COLOR         3
#define SIZE_WORD_LEN         1
#define SIZE_ACTION_CMD         1
#define SIZE_ACTION_TIME        2
#define SIZE_TEXT_GROUP_COUNT   1
#define SIZE_EFFECT_PACKET   9

#define SET_TEXT                    0x21
#define SET_COLOR_TEXT              0x51
#define SAVE_TEXT                   0xA1

static const unsigned short crc16tab[256]= {
  0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
  0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
  0x1231,0x0210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
  0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
  0x2462,0x3443,0x0420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
  0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
  0x3653,0x2672,0x1611,0x0630,0x76d7,0x66f6,0x5695,0x46b4,
  0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
  0x48c4,0x58e5,0x6886,0x78a7,0x0840,0x1861,0x2802,0x3823,
  0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
  0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0x0a50,0x3a33,0x2a12,
  0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
  0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0x0c60,0x1c41,
  0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
  0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0x0e70,
  0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
  0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
  0x1080,0x00a1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
  0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
  0x02b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
  0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
  0x34e2,0x24c3,0x14a0,0x0481,0x7466,0x6447,0x5424,0x4405,
  0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
  0x26d3,0x36f2,0x0691,0x16b0,0x6657,0x7676,0x4615,0x5634,
  0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
  0x5844,0x4865,0x7806,0x6827,0x18c0,0x08e1,0x3882,0x28a3,
  0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
  0x4a75,0x5a54,0x6a37,0x7a16,0x0af1,0x1ad0,0x2ab3,0x3a92,
  0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
  0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0x0cc1,
  0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
  0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0x0ed1,0x1ef0
};

void transmit_data(char cmd, char* data, uint32_t len);
uint16_t crc16_ccitt(uint8_t *buf, int len);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200,SERIAL_8N1, 18, 19);
  // put your main code here, to run repeatedly:
  char * text = "날씨\n맑음"; //  디스플레이 출력 텍스트 : "미세\n먼지\n좋음"
  int textColor = 0x00ff0000; //  폰트 컬러 : red
  int bgColor = 0x00000000;   //  배경 색 : black
  uint8_t fontSize = 1;       //  폰트 크기 : 1
  uint8_t actionCmd = 0x00;   //  액션 : 디폴트 
  uint8_t pageNum = 1;        //  페이지 : 1
  short actionTime = 0x00;    //  액션 이동 시간 : 00
  uint8_t effectCmd = 0x00; //  효과 : 없음
  short effectBorderTime = 0x00;   //  Effect Border Blink 주기
  int effectBorderColor  = 0;      //  Effect Border RGB 색상
  uint8_t effectColorSpeed= 0x00;  // Effect Color effect 주기
  short effectStarRainTime= 0x00; //Effect  Star 속도

  char * sendBuf;
  uint8_t textLen;
  int idx= 0;
  int sendBufLen;
  uint8_t cmd;
  uint8_t fontColor[3];
  uint8_t scrColor[3];
  textLen = strlen(text);
  sendBufLen = SIZE_PAGE + SIZE_TEXT_LEN + textLen + SIZE_FONT_SIZE + SIZE_FONT_COLOR + SIZE_BG_COLOR + SIZE_ACTION_CMD + SIZE_ACTION_TIME +SIZE_EFFECT_PACKET;
  sendBuf = (char*)malloc(sendBufLen); 

  fontColor[0] = (uint8_t) (((textColor & 0x00ff0000) >> 16) / 8);  // font color - red
  fontColor[1] = (uint8_t) (((textColor & 0x0000ff00) >> 8) / 8);   // font color - blue
  fontColor[2] = (uint8_t) (((textColor & 0x000000ff) >> 0) / 8);   // font color - blue
  scrColor[0] = (uint8_t) (((bgColor & 0x00ff0000) >> 16) / 8);     // screen color - red
  scrColor[1] = (uint8_t) (((bgColor & 0x0000ff00) >> 8) / 8);      // screen color - blue
  scrColor[2] = (uint8_t) (((bgColor & 0x000000ff) >> 0) / 8);      // screen color - blue

  sendBuf[idx++] = pageNum;            
  sendBuf[idx++] = textLen;            

  for (int i = 0; i <textLen; i++) {
    sendBuf[idx++] = text[i];
  }

  sendBuf[idx++] = fontSize;           
  sendBuf[idx++] = fontColor[0];       
  sendBuf[idx++] = fontColor[1];       
  sendBuf[idx++] = fontColor[2];       
  sendBuf[idx++] = scrColor[0];      
  sendBuf[idx++] = scrColor[1];       
  sendBuf[idx++] = scrColor[2];       
  sendBuf[idx++] = actionCmd;
  sendBuf[idx++] = (actionTime & 0x0000ff00 >> 8);
  sendBuf[idx++] = (actionTime & 0x000000ff); 

  sendBuf[idx++] = (effectCmd& 0x000000ff);
  sendBuf[idx++] = (effectBorderTime & 0x0000ff00 >> 8);
  sendBuf[idx++] = (effectBorderTime & 0x000000ff);
  sendBuf[idx++] = (((effectBorderColor & 0x00ff0000) >> 16) / 8);
  sendBuf[idx++] = (((effectBorderColor & 0x0000ff00) >> 8) / 8);
  sendBuf[idx++] = (((effectBorderColor & 0x000000ff) >> 0) / 8);
  sendBuf[idx++] = effectColorSpeed;
  sendBuf[idx++] = ((effectStarRainTime & 0x0000ff00) >> 8);
  sendBuf[idx++] = (effectStarRainTime & 0x000000ff);  

  cmd = (uint8_t)SAVE_TEXT;
  transmit_data(cmd, (char*)sendBuf, sendBufLen);
  free(sendBuf);
}

void loop() {

}

void transmit_data(char cmd, char* data, uint32_t len)
{
  uint16_t crc = 0;
  uint32_t inx = 0;
  uint8_t* sendData;
  uint8_t* cmdData;
  uint16_t dataLen;
  uint16_t sendLen;
  uint16_t sendDataLen;

  dataLen = (uint16_t)(len + SIZE_CMD);
  sendLen = len + SIZE_PKT_DATA; 

  sendDataLen = sendLen + 2; // +2 is newline Word 
  sendData = (uint8_t*)malloc(sendDataLen);
  memset(sendData, 0, sendLen);
  sendData[inx++] = STX;
  sendData[inx++] = (uint8_t)(dataLen);
  sendData[inx++] = (uint8_t)(dataLen >> 8);
  sendData[inx++] = (uint8_t)(dataLen);
  sendData[inx++] = (uint8_t)(dataLen >> 8);
  sendData[inx++] = cmd;
    
  for (int i=0; i<len; i++) {
    sendData[inx++] = data[i];
  }
  cmdData = (uint8_t*)malloc(dataLen);
  memset(cmdData, 0, dataLen);
  memcpy(cmdData, sendData + IDX_CMD, dataLen);
  crc = crc16_ccitt(cmdData, dataLen);
  sendData[inx++] = (crc & 0xFF00) >> 8;
  sendData[inx++] = (crc & 0x00FF);
  
  sendData[inx++] = ETX;
  sendData[inx++] = 0x0A;
  sendData[inx++] = 0x0D;  

  for (int i=0; i < sendDataLen; i++) {
    Serial2.write(sendData[i]);
      Serial.printf("%02X ", sendData[i]);
  }

  free(cmdData);
  free(sendData);
}

uint16_t crc16_ccitt(uint8_t *buf, int len)
{
  int counter;
  unsigned short crc = 0;
  for( counter = 0; counter < len; counter++)
    //crc = (crc<<8) ^ crc16tab[((crc>>8) ^ buf[counter]) &0x00FF];
    crc = (crc<<8) ^ crc16tab[((crc>>8) ^ *(uint8_t *)buf++)&0x00FF];
  return crc;
}
