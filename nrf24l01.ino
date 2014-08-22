#include <SPI.h>

#define DEBUG 1

// Command set 
#define R_REGISTER         0x00
#define W_REGISTER         0x20
#define R_RX_PAYLOAD       0x61
#define W_TX_PAYLOAD       0xA0
#define FLUSH_TX           0xE1
#define FLUSH_RX           0xE2
#define REUSE_TX_PL        0xE3
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
#define ADDR_NRF_DYNPD          0x1C
#define ADDR_NRF_FEATURE        0x1D

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

// Pin Controls for NRF chip
// Chip 1 pins
const int MOD1_RF_EN = 9;
const int MOD1_CSN =  10;

// Chip 2 pins
const int MOD2_RF_EN = 6;
const int MOD2_CSN =   7;

// Struct for one SPI read/write operation from NRF24L01+
typedef struct nrf_cmd_t {
  byte stat;      // Status code of NRF24L01+
  byte opcode;    // Opcode being sent to NRF
  byte buflen;    // Length of data variable; not relevant 
  byte data[32];  // Byte data transferred to/from NRF chip
};

void setup()
{
  // Buffer structs for each RF module
  nrf_cmd_t rf1_cmd = { 
    0, FLUSH_TX, 2, {
      0xAA, 0x55    } 
  };
  nrf_cmd_t rf2_cmd = { 
    0, FLUSH_TX, 2, {
      0xAA, 0x55    } 
  };

  Serial.begin(9600);
  if (DEBUG) {
    Serial.println("Contents of rf_cmd");  
    nrf_command_debug(&rf1_cmd);
  }
  
  // start SPI library
  setup_SPI();

  // configure output pins
  configure_Pins();

  Serial.println("Initializing RF modules...");
  
  if (DEBUG) {
    byte rf1_status = nrf_status(MOD1_CSN);
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
  }

  nrf_powerup(MOD1_CSN);
}

void loop()
{

}


/*******************************
******* BASIC FUNCTIONS  *******
*******************************/

// Gets the status of RF device for pin_CSN
byte nrf_status(int pin_CSN) { 
  digitalWrite(pin_CSN, LOW);
  byte rf_status = SPI.transfer(NOP);
  digitalWrite(pin_CSN, HIGH);
  return rf_status;
}

void read_nrf(struct nrf_cmd_t *cmd, int pin_CSN) {
  cmd->stat = 0;
  digitalWrite(pin_CSN, LOW);
  cmd->stat = SPI.transfer(cmd->opcode);
  for (int i = 0; i < cmd->buflen; i++) {
    cmd->data[i] = SPI.transfer(NOP);
  }
  digitalWrite(pin_CSN, HIGH);
  if (cmd->buflen < 32) {
    cmd->data[cmd->buflen] = '\0';
  }
}

void write_nrf(struct nrf_cmd_t *cmd, int pin_CSN) {
  cmd->stat = 0;
  digitalWrite(pin_CSN, LOW);
  cmd->stat = SPI.transfer(cmd->opcode);
  for (int i = 0; i < cmd->buflen; i++) {
    SPI.transfer(cmd->data[i]);
  }
  digitalWrite(pin_CSN, HIGH);
}


/*******************************
******* SETUP FUNCTIONS  *******
*******************************/

// Starts the SPI hardware and configures it properly
void setup_SPI() {
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
}

// Sets all of the non-SPI-controlled pins
void configure_Pins() {
  pinMode(MOD1_RF_EN, OUTPUT);
  pinMode(MOD2_RF_EN, OUTPUT);
  pinMode(MOD1_CSN, OUTPUT);
  pinMode(MOD2_CSN, OUTPUT);
  digitalWrite(MOD1_CSN, HIGH);
  digitalWrite(MOD2_CSN, HIGH);  
  delay(300);
}

// Brings NRF into powerup state
int nrf_powerup(int chipSelectPin) {
  nrf_cmd_t startup = { 
    0,                               //Status
    (R_REGISTER | ADDR_NRF_CONFIG),  //Read Config Register
    1,                               //Buffer length
    { 0 , 0 , 0 , 0 , 0 , NULL }     //Junk data in buffer
  };
  if (DEBUG) {
    Serial.println("Cmd Struct-PWR_UP, before read");
    nrf_command_debug(&startup);
  }
  read_nrf(&startup, chipSelectPin);
  if (DEBUG) {
    Serial.println("Cmd Struct-PWR_UP, before read");
    nrf_command_debug(&startup);
  }

  byte up_config = startup.data[0] | (NRF_CONFIG_SET_PWR_UP << NRF_CONFIG_MASK_PWR_UP_SHIFT);
  startup.opcode = (W_REGISTER | ADDR_NRF_CONFIG);
  startup.data[0] = up_config;
  if (DEBUG) {
    Serial.println("Cmd Struct-PWR_UP, before write");
    nrf_command_debug(&startup);
  }
  write_nrf(&startup, chipSelectPin);
  read_nrf(&startup, chipSelectPin);
  
  if (DEBUG) {
    Serial.println("Config values");
    Serial.print("Written: ");
    Serial.println(up_config);
    Serial.print("Read   : ");
    Serial.println(startup.data[0]);
  }

  if (up_config == startup.data[0]) {
    return 0;
  } else {
    return -1;
  }
}


/***********************************
******* AVAST! HERE BE DEBUG *******
***********************************/

// Dumps contents of a nrf_cmd_t struct to serial console
// DEBUG FUNCTION
void nrf_command_debug(struct nrf_cmd_t *cmd) {
  Serial.println("Register dump");
  Serial.println(cmd->stat, HEX);
  Serial.println(cmd->opcode, HEX);
  Serial.println(cmd->buflen);
  byte i=0;
  while ( i < cmd->buflen ) {
    if (cmd->data[i] == '\0') {
      break;
    } 
    else {
      Serial.println(cmd->data[i], HEX);
      i++;
    }
  };
};

// Dumps all the usable registers of a given NRF24L01+
// DEBUG FUNCTION
void nrf_register_dump(int pin_CSN) {
  
  // All registers on an NRF24L01+
  byte registry[26] = { ADDR_NRF_CONFIG,
                        ADDR_NRF_EN_AA,
                        ADDR_NRF_EN_RXADDR,
                        ADDR_NRF_SETUP_AW,
                        ADDR_NRF_SETUP_RETR,
                        ADDR_NRF_RF_CH,
                        ADDR_NRF_RF_SETUP,
                        ADDR_NRF_STATUS,
                        ADDR_NRF_OBSERVE_TX,
                        ADDR_NRF_CD,
                        ADDR_NRF_RX_ADDR_P0,
                        ADDR_NRF_RX_ADDR_P1,
                        ADDR_NRF_RX_ADDR_P2,
                        ADDR_NRF_RX_ADDR_P3,
                        ADDR_NRF_RX_ADDR_P4,
                        ADDR_NRF_RX_ADDR_P5,
                        ADDR_NRF_TX_ADDR,
                        ADDR_NRF_RX_PW_P0,
                        ADDR_NRF_RX_PW_P1,
                        ADDR_NRF_RX_PW_P2,
                        ADDR_NRF_RX_PW_P3,
                        ADDR_NRF_RX_PW_P4,
                        ADDR_NRF_RX_PW_P5,
                        ADDR_NRF_FIFO_STATUS,
                        ADDR_NRF_DYNPD,
                        ADDR_NRF_FEATURE };
                    
  // Loops through all the values in 
  for (int i = 0 ; i < 26; i++) {
    byte cmd = R_REGISTER | registry[i];
    digitalWrite(pin_CSN, LOW);
    byte rf_status = SPI.transfer(cmd);
    byte cfg_data = SPI.transfer(NOP);
    digitalWrite(pin_CSN, HIGH);
    Serial.print("Register ");
    Serial.print(registry[i], HEX);
    Serial.print(" data : ");
    Serial.println(cfg_data, BIN); 
  }
}  

