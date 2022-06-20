// https://www.electroniclinic.com/soil-npk-sensor-with-arduino-and-android-cell-phone-application-for-monitoring-soil-nutrient
// https://www.circuitschools.com/measure-soil-npk-values-using-soil-npk-sensor-with-arduino/
#define USE_MALLOC

#include <SoftwareSerial.h>
#include <Wire.h>

/*
  黄线 A
  蓝线 B

  RO  D2
  DI  D3
  DE  D7
  RE  D8

  RO is the receiver output and it should be connected with the RX pin of the Arduino.

  RE is the Receiver Enable. This is active low.
  This pin should be connected with the Arduino’s digital output pin.
  Drive LOW to enable receiver, HIGH to enable Driver.

  DE is the Driver enable pin. This is Active High and is typically jumpered to the RE Pin.

  DI is the Driver Input and it should be connected with the TX pin of the Arduino.

*/

#define RE 8
#define DE 7

SoftwareSerial modbus(2, 3);

void setup()
{
  Serial.begin(9600);
  modbus.begin(9600);

  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);

  // put RS-485 into receive mode
  digitalWrite(DE, LOW);
  digitalWrite(RE, LOW);
}

word BytesToWord(byte high, byte low)
{
  return high << 8 | low;
}

short BytesToShort(byte high, byte low)
{
  return high << 8 | low;
}

const int return_size = 23; // in bytes

byte return_buffer[return_size];

const size_t inquery_frame_size = 8; //查询帧8个字节
//有9个
//含水率 温度 电导率 PH 氮含量 磷含量 钾含量 盐度 总溶解固体TDS
// const byte all_inquiry_frame[] = {0x03, 0x03, 0x00, 0x00, 0x00, 0x09, 0x84, 0x2E};

byte all_inquiry_command[] = {0x03, 0x03, 0x00, 0x00, 0x00, 0x09};

void read_all(double *water, 
              double *temperature, 
              short *ec,
              short *ph,
              short *nitrogen, 
              short *phosphorus,
              short *potassium, 
              short *salt, 
              short *tds,
              byte deviceAddress = 0x03)
{

  Serial.print("device address 地址 : ");
  Serial.println(deviceAddress);

  modbus.flush();

  digitalWrite(DE, HIGH);
  digitalWrite(RE, HIGH);
  delay(10);

  all_inquiry_command[0] = deviceAddress;

  byte *inquiry_frame = CRC16(all_inquiry_command, 6);

  // if (modbus.write(all_inquiry_frame, sizeof(all_inquiry_frame)) == 8)
  if (modbus.write(inquiry_frame, inquery_frame_size) == inquery_frame_size)
  {
    modbus.flush();

    digitalWrite(DE, LOW);
    digitalWrite(RE, LOW);

    delay(200);

    size_t read_size = modbus.readBytes(return_buffer, static_cast<size_t>(return_size));
    Serial.print("read_size : ");
    Serial.println(read_size);

    if (read_size == 0)
      return;
    if (!CheckCRC16(return_buffer, return_size))
      return;

    *water = BytesToShort(return_buffer[3], return_buffer[4]) / 10.0;

    *temperature = BytesToShort(return_buffer[5], return_buffer[6]) / 10.0;

    *ec = BytesToShort(return_buffer[7], return_buffer[8]);

    *ph = BytesToShort(return_buffer[9], return_buffer[10]) / 10.0;

    *nitrogen = BytesToShort(return_buffer[11], return_buffer[12]);

    *phosphorus = BytesToShort(return_buffer[13], return_buffer[14]);

    *potassium = BytesToShort(return_buffer[15], return_buffer[16]);

    *salt = BytesToShort(return_buffer[17], return_buffer[18]);

    *tds = BytesToShort(return_buffer[19], return_buffer[20]);

    modbus.flush();

//释放查询帧
#ifdef USE_MALLOC
    free(inquiry_frame);
#else
    delete[] inquiry_frame;
#endif
    inquiry_frame = nullptr;
  }
}

void printValues(
  double water, double temperature, short ec, short ph, short nitrogen, short phosphorus, short potassium, short salt, short tds)
{
  Serial.print("water : ");
  Serial.println(water);

  Serial.print("temperature : ");
  Serial.println(temperature);

  Serial.print("ec : ");
  Serial.println(ec);

  Serial.print("ph : ");
  Serial.println(ph);

  Serial.print("nitrogen : ");
  Serial.println(nitrogen);

  Serial.print("phosphorus : ");
  Serial.println(phosphorus);

  Serial.print("potassium : ");
  Serial.println(potassium);

  Serial.print("salt : ");
  Serial.println(salt);

  Serial.print("TDS : ");
  Serial.println(tds);

  Serial.println("--------------");
}

void loop()
{
  double water, temperature;
  short ec, ph, nitrogen, phosphorus, potassium, salt, tds;

  read_all(&water, &temperature, &ec, &ph, &nitrogen, &phosphorus, &potassium, &salt, &tds, 0x02);
  printValues(water, temperature, ec, ph, nitrogen, phosphorus, potassium, salt, tds);

  read_all(&water, &temperature, &ec, &ph, &nitrogen, &phosphorus, &potassium, &salt, &tds, 0x03);
  printValues(water, temperature, ec, ph, nitrogen, phosphorus, potassium, salt, tds);

  delay(2000);
}

/*
  CRC16 MODBUS生成
  生成后会多出两个字节
*/

byte *CRC16_2(const byte *bytes, int len, byte CH, byte CL, byte preH = 255, byte preL = 255)
{
  int resultLen = len + 2;

#ifdef USE_MALLOC
  byte *result = (byte *)malloc(static_cast<size_t>(resultLen));
#else
  byte *result = new byte[resultLen];
#endif

  memcpy(result, bytes, len);

  byte cL = preL;
  byte cH = preH;

  for (int i = 0; i < len; i++)
  {
    cL = (byte)(cL ^ bytes[i]);

    for (int j = 0; j <= 7; j++)
    {

      byte num = cH;
      byte num1 = cL;
      cH = (byte)(cH >> 1);
      cL = (byte)(cL >> 1);

      if ((num & 1) == 1)
      {
        cL = (byte)(cL | 128);
      }

      if ((num1 & 1) == 1)
      {
        cH = (byte)(cH ^ CH);
        cL = (byte)(cL ^ CL);
      }
    }
  }

  result[resultLen - 2] = cL;
  result[resultLen - 1] = cH;

  return result;
}

// 直接计算 CRC
// 其结果会比输入的字节长度多出两个字节
// 用完记得free()
// 示例：
// byte* data = new byte[]{ 0x03, 0x03, 0x00, 0x01,0x00,0x01 };
// auto t = CRC16(data, 6);
// for (size_t i = 0; i < 8; i++)
// {
//   std::cout << (int) t[i] << std::endl;
// }
// free(t);
byte *CRC16(const byte *bytes, int len)
{
  return CRC16_2(bytes, len, 160, 1, 255, 255);
}

/*
  CRC16 MODBUS 校验
*/

bool CheckCRC16_2(const byte *value, int len, byte CH, byte CL)
{
  bool flag;

  if (value == nullptr)
  {
    flag = false;
  }
  else if (len >= 2)
  {

#ifdef USE_MALLOC
    byte *numArray = (byte *)malloc(len - 2);
#else
    byte *numArray = new byte[len - 2];
#endif

    memcpy(numArray, value, static_cast<size_t>(len) - 2);

    byte *numArray1 = CRC16_2(numArray, len - 2, CH, CL, 255, 255);

    flag = ((numArray1[len - 2] != value[len - 2] ? true : numArray1[len - 1] != value[len - 1]) ? false : true);

// release memory
#ifdef USE_MALLOC
    free(numArray);
    free(numArray1);
#else
    delete[] numArray;
    delete[] numArray1;
#endif
    numArray = nullptr;
    numArray1 = nullptr;
  }
  else
  {
    flag = false;
  }
  return flag;
}

bool CheckCRC16(byte *value, int len)
{
  return CheckCRC16_2(value, len, 160, 1);
}
