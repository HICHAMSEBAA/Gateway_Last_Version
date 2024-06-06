/*
 Library:				NRF24L01/NRF24L01+
 Written by:			abdelmalek Belloula / Hicham Sebaa
 Date written:				04/04/2024
 Last modified:	                           -/-
 Description:			This is an ESP32 device driver library for the NRF24L01 Nordic Radio transceiver

References:				This library was written based on the Arduino NRF24 Open-Source library by J. Coliz and the NRF24 datasheet
										- https://github.com/maniacbug/RF24
										- https://www.sparkfun.com/datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf
 */
#ifndef NRF24L01_H_
#define NRF24L01_H_

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <driver/spi_master.h>
#include <driver/gpio.h>

/* Memory Map */
#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_EN_RXADDR   0x02
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_OBSERVE_TX  0x08
#define REG_CD          0x09
#define REG_RX_ADDR_P0  0x0A
#define REG_RX_ADDR_P1  0x0B
#define REG_RX_ADDR_P2  0x0C
#define REG_RX_ADDR_P3  0x0D
#define REG_RX_ADDR_P4  0x0E
#define REG_RX_ADDR_P5  0x0F
#define REG_TX_ADDR     0x10
#define REG_RX_PW_P0    0x11
#define REG_RX_PW_P1    0x12
#define REG_RX_PW_P2    0x13
#define REG_RX_PW_P3    0x14
#define REG_RX_PW_P4    0x15
#define REG_RX_PW_P5    0x16
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD	    0x1C
#define REG_FEATURE	    0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define BIT_EN_CRC      3
#define BIT_CRCO        2
#define BIT_PWR_UP      1
#define BIT_PRIM_RX     0
#define BIT_ENAA_P5     5
#define BIT_ENAA_P4     4
#define BIT_ENAA_P3     3
#define BIT_ENAA_P2     2
#define BIT_ENAA_P1     1
#define BIT_ENAA_P0     0
#define BIT_ERX_P5      5
#define BIT_ERX_P4      4
#define BIT_ERX_P3      3
#define BIT_ERX_P2      2
#define BIT_ERX_P1      1
#define BIT_ERX_P0      0
#define BIT_AW          0
#define BIT_ARD         4
#define BIT_ARC         0
#define BIT_PLL_LOCK    4
#define BIT_RF_DR       3
#define BIT_RF_PWR      6
#define BIT_RX_DR       6
#define BIT_TX_DS       5
#define BIT_MAX_RT      4
#define BIT_RX_P_NO     1
#define BIT_TX_FULL     0
#define BIT_PLOS_CNT    4
#define BIT_ARC_CNT     0
#define BIT_TX_REUSE    6
#define BIT_FIFO_FULL   5
#define BIT_TX_EMPTY    4
#define BIT_RX_FULL     1
#define BIT_RX_EMPTY    0
#define BIT_DPL_P5	    5
#define BIT_DPL_P4	    4
#define BIT_DPL_P3	    3
#define BIT_DPL_P2	    2
#define BIT_DPL_P1	    1
#define BIT_DPL_P0	    0
#define BIT_EN_DPL	    2
#define BIT_EN_ACK_PAY  1
#define BIT_EN_DYN_ACK  0

/* Instruction Mnemonics */
#define CMD_R_REGISTER    0x00
#define CMD_W_REGISTER    0x20
#define CMD_REGISTER_MASK 0x1F
#define CMD_ACTIVATE      0x50
#define CMD_R_RX_PL_WID   0x60
#define CMD_R_RX_PAYLOAD  0x61
#define CMD_W_TX_PAYLOAD  0xA0
#define CMD_W_ACK_PAYLOAD 0xA8
#define CMD_FLUSH_TX      0xE1
#define CMD_FLUSH_RX      0xE2
#define CMD_REUSE_TX_PL   0xE3
#define CMD_NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define REG_RPD         0x09

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2

//**** TypeDefs ****//
//1. Power Amplifier function, NRF24_setPALevel()
typedef enum {
	RF24_PA_m18dB = 0, RF24_PA_m12dB, RF24_PA_m6dB, RF24_PA_0dB, RF24_PA_ERROR
} NRF24_pa_dbm;
//2. NRF24_setDataRate() input
typedef enum {
	RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS
} NRF24_datarate;
//3. NRF24_setCRCLength() input
typedef enum {
	RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16
} NRF24_CRC_length;
//4. Pipe address registers
static const uint8_t NRF24_ADDR_REGS[7] = {
REG_RX_ADDR_P0,
REG_RX_ADDR_P1,
REG_RX_ADDR_P2,
REG_RX_ADDR_P3,
REG_RX_ADDR_P4,
REG_RX_ADDR_P5,
REG_TX_ADDR };
//5. RX_PW_Px registers addresses
static const uint8_t RF24_RX_PW_PIPE[6] = {
REG_RX_PW_P0,
REG_RX_PW_P1,
REG_RX_PW_P2,
REG_RX_PW_P3,
REG_RX_PW_P4,
REG_RX_PW_P5 };
//**** Functions prototypes ****//

uint32_t microsDelay(uint32_t delay_us);

//1: Config CE end CSN GPIO pin mode and default state
void gpio_Config(void);

//2:  Configure SPI
esp_err_t SPI_Config(void);

//3: Chip Select function
void NRF24_CSN(int state);

//4: Chip Enable
void NRF24_CE(int state);

//5:Configure NRF24 Bus
void NRF24_Bus_Config(void);

//6: Write single byte register
void NRF24_write_Register(uint8_t reg, uint8_t value);

//7: Read single byte from a register
uint8_t NRF24_read_Register(uint8_t reg);

//8: Write multipl bytes register
void NRF24_write_RegisterN(uint8_t reg, const uint8_t *buf, uint8_t len);

//9: Read multiple bytes register
void NRF24_read_RegisterN(uint8_t reg, uint8_t *buf, uint8_t len);

//10: power up
void NRF24_powerUp(void);

//11: power down
void NRF24_powerDown(void);

//12: Flush Tx buffer
void NRF24_flush_Tx(void);

//13: Flush Rx buffer
void NRF24_flush_Rx(void);

//14: Set RF channel frequency
void NRF24_set_Channel(uint8_t channel);

//15: Set transmit power level
void NRF24_set_PALevel(NRF24_pa_dbm level);

//16 Set data rate (250 Kbps, 1Mbps, 2Mbps)
bool NRF24_set_DataRate(NRF24_datarate speed);

//17: Set crc length (disable, 8-bits or 16-bits)
void NRF24_set_CRCLength(NRF24_CRC_length length);

//18: Microsecond delay function
void NRF24_setRetransmitDelay(uint8_t val); //void NRF24_DelayMicroSeconds(uint32_t uSec);

//19: Set payload size
void NRF24_set_PayloadSize(uint8_t size);

//20:
NRF24_pa_dbm NRF24_get_PALevel(void);

//21: Get data rate
NRF24_datarate NRF24_get_DataRate(void);

//22: Get payload size
uint8_t NRF24_get_PayloadSize(void);

//23: Get AckPayload Size
uint8_t NRF24_get_AckPayloadSize(void);

//24: Get CRC length
NRF24_CRC_length NRF24_get_CRCLength(void);

//25: Get status register value
uint8_t NRF24_get_Status(void);

//26: Write acknowledge payload
void NRF24_write_AckPayload(uint8_t pipe, const void *buf, uint8_t len);

//27: Read receive payload
void NRF24_read_Payload(void *buf, uint8_t len);

//28: Check if data are available and on which pipe (Use this for multiple rx pipes)
bool NRF24_availablePipe(uint8_t *pipe_num);

//29: Write transmit payload
void NRF24_write_Payload(uint8_t *buf, uint8_t len);

//30. Start write (for IRQ mode)
void NRF24_startWrite(const void *buf, uint8_t len);

//31: ACTIVATE cmd
void NRF24_ACTIVATE_cmd(void);

//32: Disable CRC
void NRF24_disable_CRC(void);

//33: Check interrupt flags
void NRF24_whatHappened(bool *tx_ok, bool *tx_fail, bool *rx_ready);

//34: Test if there is a carrier on the previous listenning period (useful to check for intereference)
bool NRF24_test_Carrier(void);

//35: Test if a signal carrier exists (=> -64dB), only for NRF24L01+
bool NRF24_test_RPD(void);

//36: Reset Status
void NRF24_reset_Status(void);

//37: Check if an Ack payload is available
bool NRF24_isAckPayloadAvailable(void);

//38: Set Auto Ack for all
void NRF24_set_AutoAck(bool enable);

//39: Set Auto Ack for certain pipe
void NRF24_set_AutoAckPipe(uint8_t pipe, bool enable);

//40: Enable dynamic payloads
void NRF24_enableDynamicPayloads(void);

//41: Disable dynamic payloads
void NRF24_disable_DynamicPayloads(void);

//42: Enable payload on Ackknowledge packet
void NRF24_enable_AckPayload(void);

//43: Get dynamic payload size, of latest packet received
uint8_t NRF24_get_DynamicPayloadSize(void);

//44: Check for available data to read
bool NRF24_available(void);

//45: set transmit retries (NRF24_Retries_e) and delay
void NRF24_set_Retries(uint8_t delay, uint8_t count);

//46: Write(Transmit data), returns true if successfully sent
bool NRF24_write(const void *buf, uint8_t len);

//47: Read received data
bool NRF24_read(void *buf, uint8_t len);

//48: Open Tx pipe for writing (Cannot perform this while Listenning, has to call NRF24_stopListening)
void NRF24_openWritingPipe(uint64_t address);

//49: Open reading pipe
void NRF24_openReadingPipe(uint8_t number, uint64_t address);

//50: Listen on open pipes for reading (Must call NRF24_openReadingPipe() first)
void NRF24_startListening(void);

//51: Stop listening (essential before any write operation)
void NRF24_stopListening(void);

//52: NRF24 Software Reset
void NRF24_SoftwareReset(void);

//53: Radio NRF24 Radio setting
void NRF24_Radio_Config(void);

//54: NRF24 initialization
void NRF24_Init(NRF24_datarate data_reat, NRF24_pa_dbm power_level, uint8_t delay);

//55: Check if module is NRF24L01+ or normal module
bool NRF24_isNRF_Plus(void);

//56: FIFO Status
void printFIFOstatus(void);

//57: Print radio settings
void printRadioSettings(void);

//58: Print Status
void printStatusReg(void);

//59: Print register configuration
void printConfigReg(void);

//60:Set Auto Retransmit delay to  0=250us, 1=500us, ... 15=4000us
void NRF24_set_RetransmitDelay(uint8_t val);


void NRF24_REST(NRF24_datarate data_reat, NRF24_pa_dbm power_level, uint8_t delay);




















//**********  DEBUG Functions **********//

#endif /* NRF24L01_H_ */