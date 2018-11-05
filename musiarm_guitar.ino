#include <SPI.h>
#include <Wire.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SPI1_CLK  14
#define SPI1_MISO 12
#define SPI1_MOSI 13
#define SPI1_SS   27

// set SPI freqency 1MHz
#define SPI_CLK 1000000

#define MPU6050_ADDR_DEF 0x68
byte MPU6050_ADDR = MPU6050_ADDR_DEF;
#define MPU6050_AX  0x3B
#define MPU6050_AY  0x3D
#define MPU6050_AZ  0x3F
#define MPU6050_TP  0x41    //  data not used
#define MPU6050_GX  0x43
#define MPU6050_GY  0x45
#define MPU6050_GZ  0x47
#define MPU6050_WHO_AM_I     0x75

const byte note_satrt = 40;

#define SERVICE_UUID        "03b80e5a-ede8-4b33-a751-6ce34ec4c700"
#define CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"

BLECharacteristic * pCharacteristic;
bool deviceConnected = false;

short int AccX, AccY, AccZ;
float ax = 0, ay = 0, az = 0;
short int Temp;
short int GyroX, GyroY, GyroZ;


#define EM  {40,44,47,52}
#define FM  {41,45,48,53}
#define FsM {42,46,49,54}
#define GM  {43,47,50,55}
#define GsM {44,48,51,56}
#define AM  {45,49,52,57}
#define AsM {46,50,53,58}
#define BM  {47,51,54,59}
#define CM  {48,52,55,60}
#define CsM {49,53,56,61}
#define DM  {50,54,57,62}
#define DsM {51,55,58,63}

#define E7  {40,44,47,50}
#define F7  {41,45,48,51}
#define Fs7 {42,46,49,52}
#define G7  {43,47,50,53}
#define Gs7 {44,48,51,54}
#define A7  {45,49,52,55}
#define As7 {46,50,53,56}
#define B7  {47,51,54,57}
#define C7  {48,52,55,58}
#define Cs7 {49,53,56,59}
#define D7  {50,54,57,60}
#define Ds7 {51,55,58,61}

#define E6  {40,44,47,49}
#define F6  {41,45,48,50}
#define Fs6 {42,46,49,51}
#define G6  {43,47,50,52}
#define Gs6 {44,48,51,53}
#define A6  {45,49,52,54}
#define As6 {46,50,53,55}
#define B6  {47,51,54,56}
#define C6  {48,52,55,57}
#define Cs6 {49,53,56,58}
#define D6  {50,54,57,59}
#define Ds6 {51,55,58,60}

#define Eadd9  {40,44,47,54}
#define Fadd9  {41,45,48,55}
#define Fsadd9 {42,46,49,56}
#define Gadd9  {43,47,50,57}
#define Gsadd9 {44,48,51,58}
#define Aadd9  {45,49,52,59}
#define Asadd9 {46,50,53,60}
#define Badd9  {47,51,54,61}
#define Cadd9  {48,52,55,62}
#define Csadd9 {49,53,56,63}
#define Ddda9  {50,54,57,64}
#define Dsadd9 {51,55,58,65}

#define Esus4  {40,45,47,52}
#define Fsus4  {41,46,48,53}
#define Fssus4 {42,47,49,54}
#define Gsus4  {43,48,50,55}
#define Gssus4 {44,49,51,56}
#define Asus4  {45,50,52,57}
#define Assus4 {46,51,53,58}
#define Bsus4  {47,52,54,59}
#define Csus4  {48,53,55,60}
#define Cssus4 {49,54,56,61}
#define Dsus4  {50,55,57,62}
#define Dssus4 {51,56,58,63}

#define Em  {40,43,47,52}
#define Fm  {41,44,48,53}
#define Fsm {42,45,49,54}
#define Gm  {43,46,50,55}
#define Gsm {44,47,51,56}
#define Am  {45,48,52,57}
#define Asm {46,49,53,58}
#define Bm  {47,50,54,59}
#define Cm  {48,51,55,60}
#define Csm {49,52,56,61}
#define Dm  {50,53,57,62}
#define Dsm {51,54,58,63}

#define Em7  {40,43,47,50}
#define Fm7  {41,44,48,51}
#define Fsm7 {42,45,49,52}
#define Gm7  {43,46,50,53}
#define Gsm7 {44,47,51,54}
#define Am7  {45,48,52,55}
#define Asm7 {46,49,53,56}
#define Bm7  {47,50,54,57}
#define Cm7  {48,51,55,58}
#define Csm7 {49,52,56,59}
#define Dm7  {50,53,57,60}
#define Dsm7 {51,54,58,61}

#define E   40
#define F   41
#define Fs  42
#define G   43
#define Gs  44
#define A   45
#define As  46
#define B   47
#define C   48
#define Cs  49
#define D   50
#define Ds  51

byte mapping_byte[16][4] {C7, Cs7, DM, EM, DsM, Dsadd9, F7, Fs7, AM, GsM, Gs6, Gsm7, BM, Bsus4, AsM, Assus4} ;
//byte a[2][2]={{1,2},{1,2}};

uint8_t midiPacket[] = {
  0x80,  // header
  0x80,  // timestamp, not implemented
  0x00,  // status
  0x3c,  // 0x3c == 60 == middle c
  0x00   // velocity
};
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

//uninitalised pointers to SPI objects
SPIClass SPI1(HSPI);
//SPIClass SPI2(VSPI);
SPISettings spiSettings = SPISettings(SPI_CLK, SPI_MSBFIRST, SPI_MODE1);



void setup() {

  Serial.begin(115200);

  BLEDevice::init("GTR");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));
  pCharacteristic = pService->createCharacteristic(
                      BLEUUID(CHARACTERISTIC_UUID),
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_WRITE_NR
                    );

  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  SPI1.begin(SPI1_CLK, SPI1_MISO, SPI1_MOSI, SPI1_SS);

  pinMode(SPI1_SS, OUTPUT);
  pinMode(SPI1_CLK, OUTPUT);
  pinMode(SPI1_MISO, INPUT);
  pinMode(SPI1_MOSI, OUTPUT);
  digitalWrite(SPI1_SS, HIGH) ;

  create_task();
  //  while (true);
}
uint8_t oldnote1 = 0;
uint8_t oldnote2 = 0;
uint8_t oldnote3 = 0;
uint8_t oldnote4 = 0;
uint8_t oldnote5 = 0;
uint8_t oldnote6 = 0;
uint8_t oldnote7 = 0;
uint8_t oldnote8 = 0;
int old_bend = 0;
bool d_status = false;
float filterd_a0, filterd_a1, filterd_a2, filterd_a3;
int a0_old = 0, a1_old = 0, a2_old = 0, a3_old = 0, a4_old = 0, a5_old = 0, a6_old = 0, a7_old = 0;
void loop() {

  int a0 = adc_read(0);
  int a1 = adc_read(1);
  int a2 = adc_read(2);
  int a3 = adc_read(3);
  int a4 = adc_read(4);
  int a5 = adc_read(5);
  int a6 = adc_read(6);
  int a7 = adc_read(7);


  int bend = constrain(map(abs(ax), 10000, 5000, 0, 3), 0, 3);

  int velo = constrain(map(az, 2000, 5000, 127, 0), 0, 127);

  //  int bend = 0;
  //  int velo = 100;

  oldnote1 = play_code(oldnote1, a0, a0_old, 0, bend, velo);
  oldnote2 = play_code(oldnote2, a1, a1_old, 1, bend, velo);
  oldnote3 = play_code(oldnote3, a2, a2_old, 2, bend, velo);
  oldnote4 = play_code(oldnote4, a3, a3_old, 3, bend, velo);

  oldnote5 = play(oldnote5, a4, a4_old, 3, bend, velo);
  oldnote6 = play(oldnote6, a5, a5_old, 2, bend, velo);
  oldnote7 = play(oldnote7, a6, a6_old, 1, bend, velo);
  oldnote8 = play(oldnote8, a7, a7_old, 0, bend, velo);

  Serial.print('\t');

  Serial.print(bend);
  Serial.print('\t');
  Serial.print(velo);
  Serial.print('\t');
  Serial.print(a0);
  Serial.print('\t');
  Serial.print(a1);
  Serial.print('\t');
  Serial.print(a2);
  Serial.print('\t');
  Serial.print(a3);
  Serial.print('\t');
  Serial.print(a4);
  Serial.print('\t');
  Serial.print(a5);
  Serial.print('\t');
  Serial.print(a6);
  Serial.print('\t');
  Serial.print(a7);
  Serial.print('\t');
  Serial.print(oldnote1);
  Serial.print('\t');
  Serial.print(oldnote2);
  Serial.print('\t');
  Serial.print(oldnote3);
  Serial.print('\t');
  Serial.print(oldnote4);
  Serial.print('\t');
  Serial.print(oldnote5);
  Serial.print('\t');
  Serial.print(oldnote6);
  Serial.print('\t');
  Serial.print(oldnote7);
  Serial.print('\t');
  Serial.print(oldnote8);
  Serial.print('\t');
  Serial.print(ax);
  Serial.print('\t');
  Serial.print(ay);
  Serial.print('\t');
  Serial.print(az);
  Serial.print('\n');

  old_bend = bend;
}

void loop2(void *pvParameters) {
  delay(1000);
  //  i2c as a master
  Wire.begin(22, 21);

  //  wake it up
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x18);  //  0x00:2g, 0x08:4g, 0x10:8g, 0x18:16g
  Wire.endTransmission();

  //  range of gyro
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x18);  //  0x00:250, 0x08:500, 0x10:1000, 0x18:2000deg/s
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("\nWHO_AM_I error.");
    while (true) ;
  }
  //  mpu6050_start();
  //  if (is_mpu6050_connected()) {
  //    Serial.println("MPU6050_68 connected!!!!");
  //  } else {
  //    Serial.println("MPU6050_68 Not connect");
  //    MPU6050_ADDR += 1;
  //    mpu6050_start();
  //    if (! is_mpu6050_connected()) {
  //      Serial.println("MPU6050_69 Not connect");
  //      while (true);
  //    } else {
  //      Serial.println("MPU6050_69 connected!!!!");
  //    }
  //  }
  float ax_ = 0;
  float ay_ = 0;
  float az_ = 0;
  while (true) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_AX);
    Wire.endTransmission();
    //  request 14bytes (int16 x 7)
    Wire.requestFrom(MPU6050_ADDR, 14);
    //  get 14bytes
    short int AccX = Wire.read() << 8;  AccX |= Wire.read();
    short int AccY = Wire.read() << 8;  AccY |= Wire.read();
    short int AccZ = Wire.read() << 8;  AccZ |= Wire.read();
    short int Temp = Wire.read() << 8;  Temp |= Wire.read();  //  (Temp-12421)/340.0 [degC]
    short int GyroX = Wire.read() << 8; GyroX |= Wire.read();
    short int GyroY = Wire.read() << 8; GyroY |= Wire.read();
    short int GyroZ = Wire.read() << 8; GyroZ |= Wire.read();


    float p = 0.9;
    ax_ = p * ax_ + (1 - p) * AccX;
    ay_ = p * ay_ + (1 - p) * AccY;
    az_ = p * az_ + (1 - p) * AccZ;
    ax = ax_;
    ay = ay_;
    az = az_;
    //  debug monitor
    //    Serial.println("");
    //    Serial.print("  "); Serial.print(ax_);
    //    Serial.print("  "); Serial.print(ay_);
    //    Serial.print("  "); Serial.print(az_);
    //    Serial.print("  "); Serial.print(AccX);
    //    Serial.print("  "); Serial.print(AccY);
    //    Serial.print("  "); Serial.print(AccZ);
    //    Serial.println("");
    delay(1);
  }
}
uint8_t play_code(uint8_t oldnote, int analog, int &analog_old, byte note_init, byte bend, byte velo) {

  if (abs(analog_old - analog) < 100 && (bend == old_bend)) {
    return oldnote;
  }
  analog_old = analog;
  uint8_t note = map(analog, 0, 8192, -1, 10);
  if (!(note < 11)) {
    releasenote(oldnote);
    return 0;
  }
  if (note < 11) {
    note = mapping_byte[note][note_init] + bend;
  } else {
    note = note + note_init * 5 + note_satrt + bend;
  }
  if (oldnote != note ) {
    releasenote(oldnote);
  }
  if (oldnote != note && note >= note_satrt + note_init + bend) {
    playnote(note, velo);
    analog_old = analog;
    return note;
  }
  if (note < note_satrt + note_init + bend) {
    return note;
  }
  return oldnote;
}
uint8_t play(uint8_t oldnote, int analog, int &analog_old, byte note_init, byte bend, byte velo) {

  if (abs(analog_old - analog) < 100 && (bend == old_bend)) {
    return oldnote;
  }
  analog_old = analog;
  uint8_t note = map(analog, 0, 8192, -1, 10);
  if (!(note < 24)) {
    releasenote(oldnote);
    return 0;
  }
  if (note < 0) {
    releasenote(oldnote);
    return 0;
  }
  note=10-note;
  note = note + note_init * 5 + note_satrt + bend;
  if (oldnote != note ) {
    releasenote(oldnote);
  }
  if (oldnote != note && note >= note_satrt + note_init + bend) {
    playnote(note, velo);
    analog_old = analog;
    return note;
  }
  if (note < note_satrt + note_init + bend) {
    return note;
  }
  return oldnote;
}

//uint8_t play(uint8_t oldnote, int analog, int &analog_old, byte note_init, byte bend, byte velo) {
//
//  if (abs(analog_old - analog) < 100 && (bend == old_bend)) {
//    return oldnote;
//  }
//  uint8_t note = map(analog, 1000, 8192, note_satrt + note_init, note_satrt + 24 + note_init)+bend;
//  if (oldnote != note && oldnote >= note_satrt + note_init) {
//    releasenote(oldnote);
//  }
//  if (oldnote != note && note >= note_satrt + note_init + bend) {
//    playnote(note, velo);
//    analog_old = analog;
//    return note;
//  }
//  if (oldnote != note) {
//    analog_old = analog;
//  }
//  if (note < note_satrt + note_init + bend) {
//    return note;
//  }
//  return oldnote;
//}

int create_task() {
  xTaskCreate(loop2, "loop2", 1024, NULL, 1, NULL);
}

int adc_read(uint8_t channel) {
  SPI1.beginTransaction(spiSettings);
  digitalWrite(SPI1_SS, LOW);
  SPI1.transfer(0x06 | (channel >> 2));
  int d1 = SPI1.transfer(channel << 6) ;
  int d2 = SPI1.transfer(0x00) ;
  digitalWrite(SPI1_SS, HIGH);
  SPI1.endTransaction();
  return ((d1 & 0x1F) * 256 + d2);
}

void releasenote(int index) {
  if (!deviceConnected)
    return ;
  midiPacket[2] = 0x80; // note up, channel 0
  midiPacket[3] = index; // note up, channel 0
  midiPacket[4] = 0;    // velocity
  pCharacteristic->setValue(midiPacket, 5); // packet, length in bytes)
  pCharacteristic->notify();
}
void playnote(int index, int power) {
  if (!deviceConnected)
    return ;
  midiPacket[2] = 0x90; // note up, channel 0
  midiPacket[3] = index; // note up, channel 0
  midiPacket[4] = power; // velocity
  pCharacteristic->setValue(midiPacket, 5); // packet, length in bytes)
  pCharacteristic->notify();
}


byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/);
  byte data =  Wire.read();
  return data;
}
bool is_mpu6050_connected() {
  byte a = readMPU6050(MPU6050_WHO_AM_I);
  if (a != 0x68) {
    Serial.println();
    Serial.print("test_connection : WHO_AM_I :");
    Serial.print(a);
    Serial.println();
    return false;
  }
  return true;
}
void mpu6050_start() {
  Wire.begin(21, 22);
  //  wake it up
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);  //  0x00:2g, 0x08:4g, 0x10:8g, 0x18:16g
  Wire.endTransmission();

  //  range of gyro
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x18);  //  0x00:250, 0x08:500, 0x10:1000, 0x18:2000deg/s
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
}

