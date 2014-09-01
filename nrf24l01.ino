/* File: nrf24l01.ino
 * Description: This library interacts with the nRF wifi chip.
 * TODO: More descriptive stuff here for silly 'duino n00bs like Josh.
 * #codereview
 */

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
  byte opcode;    // Opcode being sent to NRF
  byte buflen;    // Length of data variable; not relevant 
  byte data[32];  // Byte data transferred to/from NRF chip
};

void setup()
{
  // Buffer structs for each RF module
  nrf_cmd_t rf1_cmd = { 
    FLUSH_TX, 
    2, 
    { 
      0xAA, 
      0x55
    } 
  };
  nrf_cmd_t rf2_cmd = { 
    FLUSH_TX, 
    2, 
    { 
      0xAA, 
      0x55
    } 
  };

  Serial.begin(9600);

  setup_SPI();

  configure_Pins();

  Serial.println("Initializing RF modules...");

  nrf_powerup(MOD1_CSN);

  /*
  rf1_cmd.opcode = W_TX_PAYLOAD;
  rf1_cmd.buflen = 2;
  rf1_cmd.data[0] = 0xAA;
  rf1_cmd.data[1] = 0x55;
  rf1_cmd.data[2] = '\0';

  rf2_cmd.opcode = W_TX_PAYLOAD;
  rf2_cmd.buflen = 2;
  rf2_cmd.data[0] = 0xAA;
  rf2_cmd.data[1] = 0x55;
  rf2_cmd.data[2] = '\0';

  write_nrf(&rf1_cmd, MOD1_CSN);
  digitalWrite(MOD1_RF_EN, HIGH);
  delay(2);
  digitalWrite(MOD1_RF_EN, LOW);
  clear_status(MOD1_CSN); // Note - you must drive the RF_EN low to clear interrupts in Status  
  Serial.print("Status register reads: ");
  Serial.println(nrf_status(MOD1_CSN), HEX);


  Serial.println("Trying to clear register");
  Serial.print("Status register reads: ");
  Serial.println(nrf_status(MOD1_CSN), HEX);
  */

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

byte clear_status(int pin_CSN) {
  byte clearmask = (nrf_status(pin_CSN) & 0xF0);
  Serial.print("Clearmask:");
  Serial.println(clearmask, HEX);
  nrf_cmd_t stat = { 
    (W_REGISTER | ADDR_NRF_STATUS), 
    1, 
    { 
      clearmask    
    } 
  };
  write_nrf(&stat, pin_CSN);
  return nrf_status(pin_CSN); 
}

// Reads a register from the NRF chip
//   Note: all multi-byte reads come back backwards (LS Byte first) 
byte read_nrf(struct nrf_cmd_t *cmd, int pin_CSN) {
  digitalWrite(pin_CSN, LOW);
  byte rf_status = SPI.transfer(cmd->opcode);
  for (int i = (cmd->buflen - 1); i > 0; i--) {
    cmd->data[i] = SPI.transfer(NOP);
  }
  digitalWrite(pin_CSN, HIGH);
  if (cmd->buflen < 32) {
    cmd->data[cmd->buflen] = '\0';
  }
  return rf_status;
}

// Writes to a register on the NRF chip
//   Note: Bytes are written LSByte first
byte write_nrf(struct nrf_cmd_t *cmd, int pin_CSN) {
  digitalWrite(pin_CSN, LOW);
  byte rf_status = SPI.transfer(cmd->opcode);
  for (int i = 0; i < cmd->buflen; i++) {
    SPI.transfer(cmd->data[i]);
  }
  digitalWrite(pin_CSN, HIGH);
  return rf_status;
}


/*******************************
 ******* SETUP FUNCTIONS  *******
 *******************************/

// Starts the SPI hardware and configures it properly
//   Note: setting Datamode might be unnecessary as that is default 
//         for the Arduino SPI registers
void setup_SPI() {
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
}

// Set up all of the control lines for the NRF chips that aren't managed
//   by the internal SPI 
void configure_Pins() {
  delay(120);
  pinMode(MOD1_RF_EN, OUTPUT);
  pinMode(MOD2_RF_EN, OUTPUT);
  pinMode(MOD1_CSN, OUTPUT);
  pinMode(MOD2_CSN, OUTPUT);
  digitalWrite(MOD1_CSN, HIGH);
  digitalWrite(MOD2_CSN, HIGH);  
  delay(300);
}

// Brings NRF into powerup state
//   End state of NRF is Standby-1; it is ready to transmit.
//   Pack a FIFO and go!
int nrf_powerup(int chipSelectPin) {
  nrf_cmd_t startup = { 
    (R_REGISTER | ADDR_NRF_CONFIG),  //Read Config Register
    1,                               //Buffer length
    {                                //Junk data in buffer
      0, 
      0, 
      0,
      0,
      0, 
      NULL     
    }     
  };

  // Dumps command struct before reading from the chip
  if (DEBUG) {
    Serial.println("Cmd Struct-PWR_UP, before read");
    nrf_command_debug(&startup);
  }
 
  // Dumps command struct after reading from the chip
  if (DEBUG) {
    byte rf_stat = read_nrf(&startup, chipSelectPin);
    Serial.print("NRF Status from read: ");
    Serial.println(rf_stat);
    Serial.println("Cmd Struct-PWR_UP, after read");
    nrf_command_debug(&startup);
  } else {
    read_nrf(&startup, chipSelectPin);
  }

  byte up_config = startup.data[0] | (NRF_CONFIG_SET_PWR_UP << NRF_CONFIG_MASK_PWR_UP_SHIFT);
  startup.opcode = (W_REGISTER | ADDR_NRF_CONFIG);
  startup.data[0] = up_config;
  if (DEBUG) {
    Serial.println("Cmd Struct-PWR_UP, before write");
    nrf_command_debug(&startup);
  }
  write_nrf(&startup, chipSelectPin);
  
  startup.opcode = (R_REGISTER | ADDR_NRF_CONFIG);
  startup.buflen = 1;
  
  read_nrf(&startup, chipSelectPin);

  if (DEBUG) {
    Serial.println("Cmd Struct-PWR_UP, after write");
    nrf_command_debug(&startup);
  }

  if (DEBUG) {
    Serial.println("Config values");
    Serial.print("Written: ");
    Serial.println(up_config);
    Serial.print("Read   : ");
    Serial.println(startup.data[0]);
  }

  if (up_config == startup.data[0]) {
    return 0;
  } 
  else {
    return -1;
  }
}


/***********************************
 ******* AVAST! HERE BE DEBUG *******
 ***********************************/

// Dumps contents of a nrf_cmd_t struct to serial console
// DEBUG FUNCTION
void nrf_command_debug(struct nrf_cmd_t *cmd) {

  Serial.println("nrf_cmd dump");
  Serial.println(cmd->opcode, HEX);
  Serial.println(cmd->buflen);
  byte i = 0;
  while ( i < cmd->buflen ) {
    Serial.println(cmd->data[i], HEX);
    i++;
  };
};

// Dumps all the usable registers of a given NRF24L01+
// DEBUG FUNCTION
void nrf_register_dump(int pin_CSN) {

  // All registers on an NRF24L01+
  byte registry[26] = { 
    ADDR_NRF_CONFIG,
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
    ADDR_NRF_FEATURE 
  };

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


