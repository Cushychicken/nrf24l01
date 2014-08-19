#include <SPI.h>

// Command set 
#define R_REGISTER         0x00
#define W_REGISTER         0x20
#define R_RX_PAYLOAD       0x61
#define W_TX_PAYLOAD       0xA0
#define FLUSH_TX           0xE1
#define FLUSH_RX           0xE2
#define REUSE_TX_PL        0xE3
#define ACTIVATE           0X50
#define R_RX_PL_WID        0x60
#define W_ACK_PAYLOAD      0xA8
#define W_TX_PAYLOAD_NOACK 0xB0
#define NOP                0xFF

// Address Locations
#define ADDR_NRF_CONFIG         0x00
#define ADDR_NRF_EN_AA          0x01
#define ADDR_NRF_EN_RXADDR      0x02
#define ADDR_NRF_SETUP_AW	0x03
#define ADDR_NRF_SETUP_RETR     0x04
#define ADDR_NRF_RF_CH          0x05
#define ADDR_NRF_RF_SETUP       0x06
#define ADDR_NRF_STATUS         0x07
#define ADDR_NRF_OBSERVE_TX     0x08
#define ADDR_NRF_CD             0x09
#define ADDR_NRF_RX_ADDR_P0     0x0A
#define ADDR_NRF_RX_ADDR_P1     0x0B
#define ADDR_NRF_RX_ADDR_P2     0x0C
#define ADDR_NRF_RX_ADDR_P3     0x0D
#define ADDR_NRF_RX_ADDR_P4     0x0E
#define ADDR_NRF_RX_ADDR_P5     0x0F
#define ADDR_NRF_TX_ADDR        0x10
#define ADDR_NRF_RX_PW_P0       0x11
#define ADDR_NRF_RX_PW_P1       0x12
#define ADDR_NRF_RX_PW_P2       0x13
#define ADDR_NRF_RX_PW_P3       0x14
#define ADDR_NRF_RX_PW_P4       0x15
#define ADDR_NRF_RX_PW_P5       0x16
#define ADDR_NRF_FIFO_STATUS    0x17

// Bitmasks and shifts
#define NRF_CONFIG_SET_PWR_UP        0x1
#define NRF_CONFIG_SET_PRIM_RX       0x1
#define NRF_CONFIG_MASK_RX_DR_SHIFT    6
#define NRF_CONFIG_MASK_TX_DS_SHIFT    5
#define NRF_CONFIG_MASK_MAX_RT_SHIFT   4
#define NRF_CONFIG_MASK_EN_CRC_SHIFT   3
#define NRF_CONFIG_MASK_CRCO_SHIFT     2
#define NRF_CONFIG_MASK_PWR_UP_SHIFT   1
#define NRF_CONFIG_MASK_PRIM_RX_SHIFT  0

const int MOD1_RF_EN = 9;
const int MOD1_CSN =  10;

const int MOD2_RF_EN = 6;
const int MOD2_CSN =   7;

void setup()
{
  Serial.begin(9600);

  // start SPI library
  setup_SPI();
  
  // configure output pins
  pinMode(MOD1_RF_EN, OUTPUT);
  pinMode(MOD2_RF_EN, OUTPUT);
  pinMode(MOD1_CSN, OUTPUT);
  pinMode(MOD2_CSN, OUTPUT);
  digitalWrite(MOD1_CSN, HIGH);
  digitalWrite(MOD2_CSN, HIGH);
  

  delay(300);
  Serial.println("Initializing RF modules...");

  byte rf1_status = nrf_status(MOD1_CSN);
  delay(100);
  byte rf2_status = nrf_status(MOD2_CSN);

  Serial.print("Mod1 Status: ");
  Serial.println(rf1_status, BIN);
  Serial.print("Mod2 Status: ");
  Serial.println(rf2_status, BIN);
  
  Serial.println("Module 1 registry");
  nrf_register_dump(MOD1_CSN);
  Serial.println();
  
  Serial.println("Module 2 registry");
  nrf_register_dump(MOD2_CSN);
  Serial.println();
  
  
  /*
  if ((rf1_status & rf2_status) == 0x0E) {
      Serial.println("NRF modules ready");
  } else {
      Serial.println("ERROR: Bad Status");
      Serial.print("Mod1 Status: ");
      Serial.println(rf1_status, BIN);
      Serial.print("Mod2 Status: ");
      Serial.println(rf2_status, BIN);
  }
  */


/*
  // Read contents of config register  
  byte cmd = R_REGISTER | ADDR_NRF_CONFIG;
  digitalWrite(chipSelectPin, LOW);
  rf_status = SPI.transfer(cmd);
  byte cfg_data = SPI.transfer(NOP);
  digitalWrite(chipSelectPin, HIGH);  
  Serial.println("Config data");
  Serial.println(cfg_data);

  // Write config register
  cmd = W_REGISTER | ADDR_NRF_CONFIG;
  byte data[32] = {};
  data[0] = cfg_data | (NRF_CONFIG_SET_PWR_UP << NRF_CONFIG_MASK_PWR_UP_SHIFT);
  Serial.println(data[0]);
  digitalWrite(chipSelectPin, LOW);
  rf_status = SPI.transfer(cmd);
  SPI.transfer(data[0]);
  digitalWrite(chipSelectPin, HIGH);
 
  // Reading back config register 
  cmd = R_REGISTER | ADDR_NRF_CONFIG;
  digitalWrite(chipSelectPin, LOW);
  rf_status = SPI.transfer(cmd);
  cfg_data = SPI.transfer(NOP);
  digitalWrite(chipSelectPin, HIGH);
  Serial.println("Config data");
  Serial.println(cfg_data);
  
  char buf[20];
  for (int i = 0 ; i < 0xA; i++) {
    cmd = R_REGISTER | i;
    digitalWrite(chipSelectPin, LOW);
    rf_status = SPI.transfer(cmd);
    cfg_data = SPI.transfer(NOP);
    digitalWrite(chipSelectPin, HIGH);
    Serial.print("Register ");
    Serial.print(i);
    Serial.print(" data : ");
    Serial.println(cfg_data, BIN); 
  }
*/
}

void loop()
{

}

// Starts the SPI hardware and configures it properly
void setup_SPI() {
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
}

// Gets the status of RF device for pin_CSN
byte nrf_status(int pin_CSN) {
  Serial.println(pin_CSN);
 
  digitalWrite(pin_CSN, LOW);
  byte rf_status = SPI.transfer(0xff);
  digitalWrite(pin_CSN, HIGH);
  return rf_status;
}



void nrf_register_dump(int pin_CSN) {
  for (int i = 0 ; i < 0xA; i++) {
    byte cmd = R_REGISTER | i;
    digitalWrite(pin_CSN, LOW);
    byte rf_status = SPI.transfer(cmd);
    byte cfg_data = SPI.transfer(NOP);
    digitalWrite(pin_CSN, HIGH);
    Serial.print("Register ");
    Serial.print(i);
    Serial.print(" data : ");
    Serial.println(cfg_data, BIN); 
  }
}  

