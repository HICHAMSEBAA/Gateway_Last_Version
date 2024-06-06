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

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "esp_log.h"
#include "NRF.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define _BOOL(x) (((x) > 0) ? 1 : 0)

static bool wide_band;                /* 2Mbs data rate in use? */
static uint8_t ack_payload_length;    /**< Dynamic size of pending ack payload. */
static uint8_t payload_size;          /**< Fixed size of payloads */
static bool ack_payload_available;    /**< Whether there is an ack payload waiting */
static bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
static uint64_t pipe0_reading_address;
static bool p_variant; /* False for RF24L01 and true for RF24L01P */

#define PIN_NUM_SCLK 12 // 18
#define PIN_NUM_MOSI 13 // 19
#define PIN_NUM_MISO 11 // 21
#define PIN_NUM_CSN  9  // 4
#define PIN_NUM_CE 46    // 4
#define _BV(x) (1 << (x))

#define RESET 0
#define SET 1

#define TAG "NRF24"
#define HOST_ID SPI2_HOST
static const int SPI_Frequency = 4000000; // Stable even with a long jumper cable
uint8_t Payload[32];
spi_device_handle_t SPIHandle;


// Define CPU frequency in MHz (modify this value based on your actual CPU frequency)
#define CPU_FREQ_MHZ 240

/**
 * This function creates a busy wait delay in microseconds on the target MCU.
 *
 * The function calculates the number of CPU cycles required to achieve the desired delay
 * and then loops for that amount of time. This is a simple but not very power-efficient
 * way to create delays. Be cautious when using busy wait delays in your code as they
 * can consume more power and block other processes from running.
 *
 * Args:
 *   delay_us (uint32_t): The desired delay in microseconds.
 *
 * Returns:
 *   uint32_t: Always returns 0.
 */
uint32_t microsDelay(uint32_t delay_us) {
    // Calculate the number of CPU cycles required for the delay
    uint32_t cycles = (delay_us * CPU_FREQ_MHZ) / 1000000;

    // Busy wait loop for the specified number of cycles
    for (uint32_t i = 0; i < cycles; i++) {
        // Do nothing (busy waiting)
    }

    return 0;
}


/**
 * @brief Configures the GPIO pins used for communication with the NRF24L01 module.
 *
 * This function initializes the Chip Enable (CE) and Chip Select (CSN) pins for the NRF24L01 radio module.
 * - CE: Controls the start and end of data transmission/reception.
 * - CSN: Selects the NRF24L01 module on the SPI bus.
 */
void gpio_Config(void)
{
    // Reset CE pin to ensure initial low state (inactive)
    gpio_reset_pin(PIN_NUM_CE);

    // Set CE pin as output for controlling NRF24L01 operations
    gpio_set_direction(PIN_NUM_CE, GPIO_MODE_OUTPUT);

    // Initially disable NRF24L01 (CE low)
    gpio_set_level(PIN_NUM_CE, 0);

    // Reset CSN pin to ensure initial high state (not selected)
    gpio_reset_pin(PIN_NUM_CSN);

    // Set CSN pin as output for SPI device selection
    gpio_set_direction(PIN_NUM_CSN, GPIO_MODE_OUTPUT);

    // Deselect NRF24L01 by setting CSN high
    gpio_set_level(PIN_NUM_CSN, 1);
}

/**
 * @brief Configures the SPI bus for communication with the NRF24L01 module.
 *
 * This function initializes the SPI bus (SPI2_HOST by default) with the following settings:
 *  - SCLK (Serial Clock): Pin specified by PIN_NUM_SCLK
 *  - MOSI (Master Out Slave In): Pin specified by PIN_NUM_MOSI
 *  - MISO (Master In Slave Out): Pin specified by PIN_NUM_MISO
 *  - Maximum transfer size: 100 bytes (adjust as needed)
 *  - SPI mode: 0 (CPOL=0, CPHA=0 - common for NRF24L01)
 *  - Clock source: Default SPI clock source
 *  - Duty cycle: 50% (128 increments)
 *  - Clock speed: SPI_Frequency (defined elsewhere)
 *  - CS (Chip Select) pin: Not used by this device (-1)
 *  - Transaction queue size: 1
 *
 * The function returns ESP_OK on success or an error code otherwise.
 *
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t SPI_Config(void)
{

    esp_err_t ret;

    // Informational message about SPI bus initialization
    printf("Initialize SPI bus\n");

    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK, // Serial Clock pin
        .mosi_io_num = PIN_NUM_MOSI, // Master Out Slave In pin
        .miso_io_num = PIN_NUM_MISO, // Master In Slave Out pin
        .quadwp_io_num = -1,         // Not used for NRF24L01
        .quadhd_io_num = -1,         // Not used for NRF24L01
        .max_transfer_sz = 100,      // Maximum transfer size (adjust as needed)
    };

    // Initialize the SPI bus
    ret = spi_bus_initialize(HOST_ID, &buscfg, SPI_DMA_CH_AUTO);
    ESP_LOGI(TAG, "spi_bus_initialize=%d", ret);
    assert(ret == ESP_OK); // Assert if initialization fails

    // Configure SPI device interface
    spi_device_interface_config_t devcfg = {};
    memset(&devcfg, 0, sizeof(spi_device_interface_config_t));

    // Specific SPI device interface configuration
    spi_device_interface_config_t device_interface_config = {
        .mode = 0,                           // SPI mode (CPOL=0, CPHA=0)
        .clock_source = SPI_CLK_SRC_DEFAULT, // Default SPI clock source
        .duty_cycle_pos = 128,               // 50% duty cycle
        .clock_speed_hz = SPI_Frequency,     // Clock speed (defined elsewhere)
        .spics_io_num = -1,                  // CS pin not used by NRF24L01
        .queue_size = 1,                     // Transaction queue size (1)
    };

    // Add NRF24L01 device to the SPI bus and store handle
    ret = spi_bus_add_device(SPI2_HOST, &device_interface_config, &SPIHandle);
    ESP_LOGI(TAG, "spi_bus_add_device=%d", ret);
    assert(ret == ESP_OK); // Assert if device addition fails

    return ret;
}

/**
 * @brief Configures the necessary peripherals for communication with the NRF24L01 module.
 *
 * This function initializes the GPIO pins and SPI interface required for communication with the NRF24L01 radio module.
 * The specific configuration details will depend on your hardware platform and chosen GPIO pins.
 *
 * Refer to your microcontroller's datasheet and NRF24L01 documentation for proper pin configuration.
 */
void NRF24_Bus_Config(void) {
    // Configure GPIO pins for NRF24L01 communication (CE, CSN, etc.)
    gpio_Config();

    // Configure SPI interface for communication with NRF24L01
    SPI_Config();
}


/**
 * @brief Controls the Chip Select (CSN) pin of the NRF24L01 module.
 *
 * This function sets the CSN pin HIGH (active) or LOW (inactive) to control communication with the NRF24L01 module.
 *
 * - When CSN is HIGH, the NRF24L01 is not selected and communication cannot occur.
 * - When CSN is LOW, the NRF24L01 is selected and communication can take place.
 *
 * @param state The desired state of the CSN pin:
 *               - 1 (or any non-zero value) for HIGH (active)
 *               - 0 for LOW (inactive)
 */
void NRF24_CSN(int state)
{
    if (state)
    {
        gpio_set_level(PIN_NUM_CSN, SET); // Set CSN high (active)
    }
    else
    {
        gpio_set_level(PIN_NUM_CSN, RESET); // Set CSN low (inactive)
    }
}

/**
 * @brief Controls the Chip Enable (CE) pin of the NRF24L01 module.
 *
 * This function sets the CE pin HIGH (active) or LOW (inactive) to control the start and end of data transmission/reception in the NRF24L01 module.
 *
 * - When CE is pulsed HIGH, the NRF24L01 starts or ends a data transfer depending on its internal state.
 * - When CE is LOW, the NRF24L01 is in standby mode.
 *
 * @param state The desired state of the CE pin:
 *               - 1 (or any non-zero value) for HIGH (active)
 *               - 0 for LOW (inactive)
 */
void NRF24_CE(int state)
{
    if (state)
    {
        gpio_set_level(PIN_NUM_CE, SET); // Set CE high (active)
    }
    else
    {
        gpio_set_level(PIN_NUM_CE, RESET); // Set CE low (inactive)
    }
}

/**
 * @brief Writes a value to a single register in the NRF24L01 module.
 *
 * This function writes a specified value to a register in the NRF24L01 using SPI communication.
 *
 * @param reg The address of the register to write to.
 * @param value The value to write to the register.
 */
void NRF24_write_Register(uint8_t reg, uint8_t value)
{
    spi_transaction_t SPITransaction;
    uint8_t Dataout[2];

    // Construct the register write command
    Dataout[0] = reg | 0x20; //CMD_W_REGISTER | (CMD_REGISTER_MASK & reg);
    Dataout[1] = value;

    // Configure the SPI transaction
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = 2 * 8; // 16 bits (2 bytes) for command and data
    SPITransaction.tx_buffer = Dataout;
    SPITransaction.rx_buffer = NULL; // No data received in write operation

    // Select the NRF24L01 on the SPI bus
    NRF24_CSN(RESET);

    // Perform the SPI write transaction
    spi_device_transmit(SPIHandle, &SPITransaction);

    // Deselect the NRF24L01
    NRF24_CSN(SET);
}

/**
 * @brief Reads a single register from the NRF24L01 module.
 *
 * This function reads the value of a specified register from the NRF24L01 using SPI communication.
 *
 * @param reg The address of the register to read.
 * @return uint8_t The value read from the register.
 */
uint8_t NRF24_read_Register(uint8_t reg)
{
    spi_transaction_t SPITransaction;
    uint8_t Datain[2];
    uint8_t Dataout[2]; // Corrected array size (was 'Dataout2]')

    // Construct the register read command
    Dataout[0] = reg & 0x1F; //CMD_R_REGISTER | (CMD_REGISTER_MASK & reg);
    Dataout[1] = 0; // Dummy byte for register read

    // Configure the SPI transaction
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = 2 * 8; // 16 bits (2 bytes) for command and data
    SPITransaction.tx_buffer = Dataout;
    SPITransaction.rx_buffer = Datain;

    // Select the NRF24L01 on the SPI bus
    NRF24_CSN(RESET);

    // Perform the SPI read transaction
    spi_device_transmit(SPIHandle, &SPITransaction);

    // Deselect the NRF24L01
    NRF24_CSN(SET);

    // Return the register value (second byte)
    // Datain[0] contains the status byte (see README Fig. 26)
    return Datain[1];
}

/**
 * @brief Writes a sequence of bytes to a register in the NRF24L01 module.
 *
 * This function writes a specified number of bytes (`len`) from a buffer (`buf`) to a register (`reg`) in the NRF24L01 using SPI communication.
 *
 * @param reg The address of the register to write to.
 * @param buf A pointer to the buffer containing the data to write.
 * @param len The number of bytes to write from the buffer.
 */
void NRF24_write_RegisterN(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    spi_transaction_t SPITransaction;
    uint8_t Dataout[50]; // Adjust size if needed for larger payloads

    // Construct the register write command
    Dataout[0] = reg | 0x20; //CMD_W_REGISTER | (CMD_REGISTER_MASK & reg);

    // Copy data from buffer to output array
    memcpy(Dataout + 1, buf, len); // Corrected memcpy usage

    // Configure the SPI transaction
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = (len + 1) * 8; // Total data length (command + payload)
    SPITransaction.tx_buffer = Dataout;
    SPITransaction.rx_buffer = NULL; // No data received in write operation

    // Select the NRF24L01 on the SPI bus
    NRF24_CSN(RESET);

    // Perform the SPI write transaction
    spi_device_transmit(SPIHandle, &SPITransaction);

    // Deselect the NRF24L01
    NRF24_CSN(SET);
}

/**
 * @brief Reads a sequence of bytes from a register in the NRF24L01 module.
 *
 * This function reads a specified number of bytes (`len`) from a register (`reg`) in the NRF24L01 and stores them in a buffer (`buf`) using SPI communication.
 *
 * @param reg The address of the register to read from.
 * @param buf A pointer to the buffer where the read data will be stored.
 * @param len The number of bytes to read from the register.
 */
void NRF24_read_RegisterN(uint8_t reg, uint8_t *buf, uint8_t len)
{
    spi_transaction_t SPITransaction;
    uint8_t Dataout[1];
    uint8_t Datain[50]; // Adjust size if needed for larger payloads

    // Construct the register read command
    Dataout[0] = reg & 0x1F; //CMD_R_REGISTER | (CMD_REGISTER_MASK & reg);

    // Select the NRF24L01 on the SPI bus
    NRF24_CSN(RESET);

    // Configure the first SPI transaction (send command byte)
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = (1) * 8; // 8 bits for the command byte
    SPITransaction.tx_buffer = Dataout;
    SPITransaction.rx_buffer = NULL; // No data received in first transaction
    spi_device_transmit(SPIHandle, &SPITransaction);

    // Configure the second SPI transaction (receive data bytes)
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = (len) * 8; // Total data length to receive (in bits)
    SPITransaction.tx_buffer = NULL;   // No data transmitted in second transaction
    SPITransaction.rx_buffer = Datain;
    spi_device_transmit(SPIHandle, &SPITransaction);

    // Deselect the NRF24L01
    NRF24_CSN(SET);

    // Copy received data to the buffer
    memcpy(buf, Datain, len);
}

/**
 * @brief Powers up the NRF24L01 module.
 *
 * This function attempts to power up the NRF24L01 module by setting the Power Up (PWR_UP) bit in the configuration register.
 *
 * The function performs the following steps:
 * 1. Reads the current value of the configuration register using `NRF24_read_Register(REG_CONFIG)`.
 * 2. Uses the bitwise OR operator (`|`) with the bit mask `_BV(BIT_PWR_UP)` to set the PWR_UP bit in the read value.
 * 3. Writes the modified value back to the configuration register using `NRF24_write_Register(REG_CONFIG, new_value)`.
 *
 */
void NRF24_powerUp(void)
{
    uint8_t reg_val;

    // Read the current configuration register value
    reg_val = NRF24_read_Register(REG_CONFIG);

    // Set the Power Up (PWR_UP) bit using bitwise OR with mask
    reg_val |= _BV(BIT_PWR_UP);

    // Write the modified value back to the configuration register
    NRF24_write_Register(REG_CONFIG, reg_val);
}

/**
 * @brief Powers down the NRF24L01 module.
 *
 * This function attempts to power down the NRF24L01 module by clearing the Power Up (PWR_UP) bit in the configuration register.
 *
 * The function performs the following steps:
 * 1. Reads the current value of the configuration register using `NRF24_read_Register(REG_CONFIG)`.
 * 2. Uses the bitwise AND operator (`&`) with the inverted bit mask `~_BV(BIT_PWR_UP)` to clear the PWR_UP bit in the read value. The inverted bit mask ensures only the PWR_UP bit is affected.
 * 3. Writes the modified value back to the configuration register using `NRF24_write_Register(REG_CONFIG, new_value)`.
 *
 * **Note:** This function does not implement error handling for SPI communication. Consider adding checks for successful transactions or including error handling mechanisms in your application.
 */
void NRF24_powerDown(void)
{
    uint8_t reg_val;

    // Read the current configuration register value
    reg_val = NRF24_read_Register(REG_CONFIG);

    // Clear the Power Up (PWR_UP) bit using bitwise AND with inverted mask
    reg_val &= ~_BV(BIT_PWR_UP);

    // Write the modified value back to the configuration register
    NRF24_write_Register(REG_CONFIG, reg_val);
}

/**
 * @brief Flushes the NRF24L01 transmit (TX) FIFO buffer.
 *
 * This function transmits a dummy byte (0xFF) to the NRF24L01's CMD_FLUSH_TX register, which initiates flushing the transmit FIFO buffer.
 */
void NRF24_flush_Tx(void)
{
    NRF24_write_Register(CMD_FLUSH_TX, 0xFF);
}

/**
 * @brief Flushes the NRF24L01 receive (RX) FIFO buffer.
 *
 * This function transmits a dummy byte (0xFF) to the NRF24L01's CMD_FLUSH_RX register, which initiates flushing the receive FIFO buffer.
 */
void NRF24_flush_Rx(void)
{
    NRF24_write_Register(CMD_FLUSH_RX, 0xFF);
}

/**
 * @brief Sets the RF channel for the NRF24L01 module.
 *
 * This function sets the RF channel used for communication by the NRF24L01. It ensures that the channel value stays within the valid range (0-125) to prevent potential errors.
 *
 * @param channel The desired RF channel (0-125).
 */
void NRF24_set_Channel(uint8_t channel)
{
    const uint8_t max_channel = 125; // Adjust based on your NRF24L01 variant

    // Clamp the channel value to the valid range (0-125)
    channel = MIN(channel, max_channel);

    // Write the confirmed valid channel to the RF_CH register
    NRF24_write_Register(REG_RF_CH, channel);
}

/**
 * @brief Sets the output power level for the NRF24L01 module.
 *
 * This function sets the RF output power level (PA Level) for the NRF24L01 module based on the provided `level` parameter.
 * It reads the current value of the RF_SETUP register, clears the existing power level bits, and then sets the appropriate bits based on the desired level.
 *
 * @param level The desired power level (RF24_PA_0dB, RF24_PA_m6dB, RF24_PA_m12dB, RF24_PA_m18dB, RF24_PA_ERROR).
 */
void NRF24_set_PALevel(NRF24_pa_dbm level)
{
    uint8_t setup = NRF24_read_Register(REG_RF_SETUP);
    setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)); // Clear existing power level bits

    // Use switch statement to set power level bits based on level
    switch (level)
    {
    case RF24_PA_0dB:
        setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));
        break;
    case RF24_PA_m6dB:
        setup |= _BV(RF_PWR_HIGH);
        break;
    case RF24_PA_m12dB:
        setup |= _BV(RF_PWR_LOW);
        break;
    case RF24_PA_m18dB:
        // nothing (already low power)
        break;
    case RF24_PA_ERROR:
        // On error, go to maximum PA
        setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));
        break;
    default:
        // Handle invalid level (optional)
        break;
    }

    NRF24_write_Register(REG_RF_SETUP, setup);
}

/**
 * @brief Sets the data rate for the NRF24L01 module.
 *
 * This function attempts to set the data rate for the NRF24L01 module based on the provided `speed` parameter. It reads the current value of the RF_SETUP register, clears the existing data rate bits, and then sets the appropriate bits based on the desired speed.
 * The function also includes a verification step to check if the data rate was successfully written.
 *
 * @param speed The desired data rate (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS).
 * @return bool True if the data rate was set successfully, false otherwise.
 */
bool NRF24_set_DataRate(NRF24_datarate speed)
{
    bool result = false;
    uint8_t setup = NRF24_read_Register(REG_RF_SETUP);

    // Clear existing data rate bits (RF_DR_LOW and RF_DR_HIGH)
    setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    // Set data rate based on speed
    switch (speed)
    {
    case RF24_250KBPS:
        // Set RF_DR_LOW for 250kbps (wide band is already false)
        setup |= _BV(RF_DR_LOW);
        break;
    case RF24_2MBPS:
        // Set RF_DR_HIGH for 2Mbps (wide band is true)
        setup |= _BV(RF_DR_HIGH);
        wide_band = true;
        break;
    case RF24_1MBPS: // Default case (no change needed, 1Mbps is default)
    default:
        wide_band = false;
        break;
    }

    // Write the modified setup value back to the RF_SETUP register
    NRF24_write_Register(REG_RF_SETUP, setup);

    // Verify data rate setting (optional)
    if (NRF24_read_Register(REG_RF_SETUP) == setup)
    {
        result = true; // Data rate set successfully
    }

    return result;
}

/**
 * @brief Sets the CRC length for the NRF24L01 module.
 *
 * This function sets the CRC length (Cyclic Redundancy Check) for the NRF24L01 module based on the provided `length` parameter. It reads the current value of the CONFIG register, clears the existing CRC bits, and then sets the appropriate bits based on the desired length.
 *
 * @param length The desired CRC length (RF24_CRC_DISABLED, RF24_CRC_8, RF24_CRC_16).
 */
void NRF24_set_CRCLength(NRF24_CRC_length length)
{
    uint8_t config = NRF24_read_Register(REG_CONFIG) & ~(_BV(BIT_CRCO) | _BV(BIT_EN_CRC)); // Clear existing CRC bits

    // Use switch statement to set CRC bits based on length
    switch (length)
    {
    case RF24_CRC_DISABLED:
        // Do nothing, already disabled above
        break;
    case RF24_CRC_8:
        config |= _BV(BIT_EN_CRC);
        break;
    case RF24_CRC_16:
        config |= _BV(BIT_EN_CRC);
        config |= _BV(BIT_CRCO);
        break;
    default:
        // Handle invalid length (optional)
        break;
    }

    NRF24_write_Register(REG_CONFIG, config);
}

/**
 * @brief Sets the automatic retransmission delay for the NRF24L01 module.
 *
 * This function sets the delay between automatic retransmission attempts for the NRF24L01 module. The delay is specified in multiples of 250µs (microseconds).
 *
 * The NRF24L01 allows setting the delay value in the range of 0 (no retransmissions) to 15 (maximum delay).
 *
 * @param val The desired retransmission delay in units of 250µs (0-15).
 */
void NRF24_set_RetransmitDelay(uint8_t val)
{
    // Read the current value of the SETUP_RETR register
    uint8_t value = NRF24_read_Register(REG_SETUP_RETR);

    // Clear the existing retransmission delay bits (ARD field)
    value &= 0x0F; // Mask with 0000 1111 to keep only the first 4 bits

    // Set the new retransmission delay value
    value |= (val << BIT_ARD); // Shift the desired value (val) and OR it with the masked register value

    // Write the modified value back to the SETUP_RETR register
    NRF24_write_Register(REG_SETUP_RETR, value);
}

/**
 * @brief Sets the payload size for the NRF24L01 module.
 *
 * This function sets the size of the data payload that will be transmitted or received by the NRF24L01 module. The payload size is the actual data portion of a packet excluding headers and footers.
 *
 * The NRF24L01 supports a maximum payload size of 32 bytes. This function ensures that the provided size does not exceed this limit.
 *
 * @param size The desired payload size in bytes (1-32).
 */
void NRF24_set_PayloadSize(uint8_t size)
{
    const uint8_t max_payload_size = 32; // Maximum payload size supported by NRF24L01

    // Clamp the payload size to the valid range (1-32)
    payload_size = MIN(size, max_payload_size);
}

/**
 * @brief Gets the currently set output power level (PA Level) of the NRF24L01 module.
 *
 * This function reads the RF_SETUP register of the NRF24L01 module and determines the currently set output power level (PA Level) based on the state of the RF_PWR_LOW and RF_PWR_HIGH bits.
 * It then returns the corresponding NRF24_pa_dbm enum value.
 *
 * @return NRF24_pa_dbm The currently set output power level (RF24_PA_0dB, RF24_PA_m6dB, RF24_PA_m12dB, RF24_PA_m18dB, RF24_PA_ERROR on invalid configuration).
 */
NRF24_pa_dbm NRF24_get_PALevel(void)
{
    NRF24_pa_dbm result = RF24_PA_ERROR; // Default to error
    uint8_t power = NRF24_read_Register(REG_RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));

    // Use switch statement to determine power level based on register value
    switch (power)
    {
    case (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)):
        result = RF24_PA_0dB;
        break;
    case _BV(RF_PWR_HIGH):
        result = RF24_PA_m6dB;
        break;
    case _BV(RF_PWR_LOW):
        result = RF24_PA_m12dB;
        break;
    default:
        result = RF24_PA_m18dB; // Or handle differently (e.g., return error)
        break;
    }

    return result;
}

/**
 * 21:
 * @brief  Get data rate
 * @param:
 * @retval: Data Rate value
 */
NRF24_datarate NRF24_get_DataRate(void)
{
    NRF24_datarate result;
    uint8_t dr = NRF24_read_Register(REG_RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

    // switch uses RAM (evil!)
    // Order matters in our case below
    if (dr == _BV(RF_DR_LOW))
    {
        // '10' = 250KBPS
        result = RF24_250KBPS;
    }
    else if (dr == _BV(RF_DR_HIGH))
    {
        // '01' = 2MBPS
        result = RF24_2MBPS;
    }
    else
    {
        // '00' = 1MBPS
        result = RF24_1MBPS;
    }
    return result;
}

/**
 * 22:
 * @brief  Get data rate
 * @param:
 * @retval: value of payload size
 */
uint8_t NRF24_get_PayloadSize(void)
{
    return payload_size;
}

/**
 * 23:
 * @brief  Get GetAckPayloadSize
 * @param:
 * @retval:
 */
uint8_t NRF24_get_AckPayloadSize(void)
{
    return ack_payload_length;
}

/**
 * 24:
 * @brief  Get Get CRC length
 * @param:
 * @retval:
 */
NRF24_CRC_length NRF24_get_CRCLength(void)
{
    NRF24_CRC_length result = RF24_CRC_DISABLED;
    uint8_t config = NRF24_read_Register(REG_CONFIG) & (_BV(BIT_CRCO) | _BV(BIT_EN_CRC));
    if (config & _BV(BIT_EN_CRC))
    {
        if (config & _BV(BIT_CRCO))
            result = RF24_CRC_16;
        else
            result = RF24_CRC_8;
    }
    return result;
}

/**
 * 25:
 * @brief   Get status register value
 * @param:
 * @retval:
 */
uint8_t NRF24_get_Status(void)
{
    uint8_t statReg;
    statReg = NRF24_read_Register(REG_STATUS);
    return statReg;
}

/**
 * @brief Writes an acknowledgement payload to the specified data pipe on the NRF24L01 module.
 *
 * This function transmits data as an acknowledgement payload through a specific data pipe
 * on the NRF24L01 radio module. It's typically used after receiving a data packet
 * and enabling acknowledgements (ACKs).
 *
 * Args:
 *   pipe (uint8_t): The data pipe number (0-5) to use for transmitting the acknowledgement.
 *   buf (const void*): A pointer to the data buffer containing the acknowledgement payload.
 *   len (uint8_t): The length of the acknowledgement payload data in bytes.
 *
 * Returns:
 *   void: This function does not return a value.
 */
void NRF24_write_AckPayload(uint8_t pipe, const void *buf, uint8_t len) {
    // Cast the data buffer pointer to a constant byte pointer
    const uint8_t *current = (uint8_t *)buf;

    // Define the maximum payload size for the NRF24L01
    const uint8_t max_payload_size = 32;

    // Determine the actual data length to transmit (limited by max payload size)
    uint8_t data_len = MIN(len, max_payload_size);

    // Send the data using NRF24_write_RegisterN with appropriate command and data
    NRF24_write_RegisterN(CMD_W_ACK_PAYLOAD | (pipe & 0x7), current, data_len);
}

/**
 * 27:
 * @brief    Read receive payload
 * @param:
 * @retval: None
 */
void NRF24_read_Payload(void *buf, uint8_t len)
{
    spi_transaction_t SPITransaction;
    uint8_t Dataout[1];
    uint8_t Datain[32]; // Max payload lenth
    Dataout[0] = CMD_R_RX_PAYLOAD;
    // Get data length using payload size
    uint8_t data_len = MIN(len, NRF24_get_PayloadSize());
    NRF24_CSN(RESET);
    // Configure the first SPI transaction (send command byte)
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = (1) * 8; // (CMD_W_TX_PAYLOAD) + payload
    SPITransaction.tx_buffer = Dataout;
    SPITransaction.rx_buffer = NULL;
    spi_device_transmit(SPIHandle, &SPITransaction);
    // Configure the second SPI transaction (receive data bytes)
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    // Read data from Rx payload buffer
    SPITransaction.length = (data_len) * 8; // (CMD_W_TX_PAYLOAD) + payload
    SPITransaction.tx_buffer = NULL;
    SPITransaction.rx_buffer = Datain;
    spi_device_transmit(SPIHandle, &SPITransaction);
    // Deselect the NRF24L01
    NRF24_CSN(SET);

    // Copy received data to the buffer
    memcpy(buf, Datain, len);
}

/**
 * @brief Checks if data is available on one of the NRF24L01 data pipes.
 *
 * This function checks the STATUS register of the NRF24L01 to determine if 
 * there is data waiting to be received on any of the data pipes (0-5).
 *
 * Args:
 *   pipe_num (uint8_t*): An optional pointer to store the data pipe number 
 *                         where data is available. If NULL, the pipe number 
 *                         is not retrieved.
 *
 * Returns:
 *   bool: True if data is available on a pipe, False otherwise.
 */
bool NRF24_availablePipe(uint8_t *pipe_num) {
    // Read the STATUS register to check for data received (RX_DR flag)
    uint8_t status = NRF24_get_Status();
    bool result = (status & _BV(BIT_RX_DR));

    if (result) {
        // If data is available and the caller wants the pipe number...
        if (pipe_num) {
            // Extract the data pipe number from the STATUS register
            *pipe_num = (status >> BIT_RX_P_NO) & 0x7;
        }

        // Clear the RX_DR flag in the STATUS register
        NRF24_write_Register(REG_STATUS, _BV(BIT_RX_DR));

        // Handle acknowledgement payload receipt (optional)
        if (status & _BV(BIT_TX_DS)) {
            // Clear the TX_DS flag if necessary (implementation detail)
            NRF24_write_Register(REG_STATUS, _BV(BIT_TX_DS));
        }
    }

    return result;
}


/**
 * 29:
 * @brief Write transmit payload
 * @param:
 * @retval: None
 */
void NRF24_write_Payload(uint8_t *buf, uint8_t len)
{
    spi_transaction_t SPITransaction;
    uint8_t Dataout[33]; // 32+1
    Dataout[0] = CMD_W_TX_PAYLOAD;
    // Bring CSN low
    NRF24_CSN(RESET);
    // Send Write Tx payload command followed by pbuf data
    memcpy(Dataout + 1, buf, len);
    memset(&SPITransaction, 0, sizeof(spi_transaction_t));
    SPITransaction.length = (len + 1) * 8; // (CMD_W_TX_PAYLOAD) + payload
    SPITransaction.tx_buffer = Dataout;
    SPITransaction.rx_buffer = NULL;
    spi_device_transmit(SPIHandle, &SPITransaction);
    // Bring CSN high
    NRF24_CSN(SET);
}

/**
 * 30:
 * @brief  Start write (for IRQ mode)
 * @param:
 * @retval: None
 */
void NRF24_startWrite(const void *buf, uint8_t len)
{
    // Transmitter power-up
    NRF24_CE(RESET);
    NRF24_write_Register(REG_CONFIG,
                         (NRF24_read_Register(REG_CONFIG) | _BV(BIT_PWR_UP)) & ~_BV(BIT_PRIM_RX));
    NRF24_CE(SET);
    //	NRF24_DelayMicroSeconds(150);
    // Send the payload
    NRF24_write_Payload(buf, len);
    // Enable Tx for 15usec
    NRF24_CE(SET);
    //	NRF24_DelayMicroSeconds(15);
    NRF24_CE(RESET);
}

/**
 * 31:
 * @brief ACTIVATE cmd
 * @param:
 * @retval:None
 */
void NRF24_ACTIVATE_cmd(void)
{
    // Read data from Rx payload buffer
    NRF24_write_Register(CMD_ACTIVATE, 0x73);
}

/**
 * 32:
 * @brief Disable CRC
 * @param:
 * @retval: None
 */
void NRF24_disable_CRC(void)
{
    uint8_t disable = NRF24_read_Register(REG_CONFIG) & ~_BV(BIT_EN_CRC);
    NRF24_write_Register(REG_CONFIG, disable);
}

/**
 * 33:
 * @brief Check interrupt flags
 * @param:
 * @retval:
 */
void NRF24_whatHappened(bool *tx_ok, bool *tx_fail, bool *rx_ready)
{
    uint8_t status = NRF24_get_Status();
    *tx_ok = 0;
    NRF24_write_Register(REG_STATUS,
                         _BV(BIT_RX_DR) | _BV(BIT_TX_DS) | _BV(BIT_MAX_RT));
    // Report to the user what happened
    *tx_ok = status & _BV(BIT_TX_DS);
    *tx_fail = status & _BV(BIT_MAX_RT);
    *rx_ready = status & _BV(BIT_RX_DR);
}

/**
 * 34:
 * @brief Test if there is a carrier on the previous listenning period (useful to check for intereference)
 * @param:
 * @retval:
 */
bool NRF24_test_Carrier(void)
{
    return NRF24_read_Register(REG_CD) & 1;
}

/**
 * 35:
 * @brief Test if a signal carrier exists (=> -64dB), only for NRF24L01+
 * @param:
 * @retval:
 */
bool NRF24_test_RPD(void)
{
    return NRF24_read_Register(REG_RPD) & 1;
}

/**
 * 36:
 * @brief  Reset Status
 * @param:
 * @retval:
 */
void NRF24_reset_Status(void)
{
    NRF24_write_Register(REG_STATUS,
                         _BV(BIT_RX_DR) | _BV(BIT_TX_DS) | _BV(BIT_MAX_RT));
}

/**
 * 37:
 * @brief  Check if an Ack payload is available
 * @param:
 * @retval:
 */
bool NRF24_isAckPayloadAvailable(void)
{
    bool result = ack_payload_available;
    ack_payload_available = false;
    return result;
}

/**
 * 38:
 * @brief  Check if an Ack payload is available
 * @param:
 * @retval:
 */
void NRF24_set_AutoAck(bool enable)
{
    if (enable)
        NRF24_write_Register(REG_EN_AA, 0x3F);
    else
        NRF24_write_Register(REG_EN_AA, 0x00);
}

/**
 * 39:
 * @brief  Set Auto Ack for certain pipe
 * @param:
 * @retval:
 */
void NRF24_set_AutoAckPipe(uint8_t pipe, bool enable)
{
    if (pipe <= 6)
    {
        uint8_t en_aa = NRF24_read_Register(REG_EN_AA);
        if (enable)
        {
            en_aa |= _BV(pipe);
        }
        else
        {
            en_aa &= ~_BV(pipe);
        }
        NRF24_write_Register(REG_EN_AA, en_aa);
    }
}

/**
 * 40:
 * @brief  Enable dynamic payloads
 * @param:
 * @retval:
 */
void NRF24_enableDynamicPayloads(void)
{
    // Enable dynamic payload through FEATURE register
    NRF24_write_Register(REG_FEATURE,
                         NRF24_read_Register(REG_FEATURE) | _BV(BIT_EN_DPL));
    if (!NRF24_read_Register(REG_FEATURE))
    {
        NRF24_ACTIVATE_cmd();
        NRF24_write_Register(REG_FEATURE,
                             NRF24_read_Register(REG_FEATURE) | _BV(BIT_EN_DPL));
    }
    // Enable Dynamic payload on all pipes
    NRF24_write_Register(REG_DYNPD,
                         NRF24_read_Register(
                             REG_DYNPD) |
                             _BV(BIT_DPL_P5) | _BV(BIT_DPL_P4) | _BV(BIT_DPL_P3) | _BV(BIT_DPL_P2) | _BV(BIT_DPL_P1) | _BV(BIT_DPL_P0));
    dynamic_payloads_enabled = true;
}

/**
 * 41:
 * @brief  Disable dynamic payloads
 * @param:
 * @retval:
 */
void NRF24_disable_DynamicPayloads(void)
{
    NRF24_write_Register(REG_FEATURE, NRF24_read_Register(REG_FEATURE) & ~(_BV(BIT_EN_DPL)));
    // Disable for all pipes
    NRF24_write_Register(REG_DYNPD, 0);
    dynamic_payloads_enabled = false;
}

/**
 * 42:
 * @brief  Enable payload on Ackknowledge packet
 * @param:
 * @retval:
 */
void NRF24_enable_AckPayload(void)
{
    // Need to enable dynamic payload and Ack payload together
    NRF24_write_Register(REG_FEATURE, NRF24_read_Register(
                                          REG_FEATURE) |
                                          _BV(BIT_EN_ACK_PAY) | _BV(BIT_EN_DPL));
    if (!NRF24_read_Register(REG_FEATURE))
    {
        NRF24_ACTIVATE_cmd();
        NRF24_write_Register(REG_FEATURE, NRF24_read_Register(
                                              REG_FEATURE) |
                                              _BV(BIT_EN_ACK_PAY) | _BV(BIT_EN_DPL));
    }
    // Enable dynamic payload on pipes 0 & 1
    NRF24_write_Register(REG_DYNPD,
                         NRF24_read_Register(REG_DYNPD) | _BV(BIT_DPL_P1) | _BV(BIT_DPL_P0));
}

/**
 * 43:
 * @brief   Get dynamic payload size, of latest packet received
 * @param:
 * @retval:
 */
uint8_t NRF24_get_DynamicPayloadSize(void)
{
    return NRF24_read_Register(CMD_R_RX_PL_WID);
}

/**
 * 44:
 * @brief   Check for available data to read
 * @param:
 * @retval:
 */
bool NRF24_available(void)
{
    return NRF24_availablePipe(NULL);
}

/**
 * 45:
 * @brief   set transmit retries (NR24_Retries) and delay
 * @param:
 * @retval:
 */
void NRF24_setRetries(uint8_t delay, uint8_t count)
{
    NRF24_write_Register(REG_SETUP_RETR,
                         (delay & 0xf) << BIT_ARD | (count & 0xf) << BIT_ARC);
}

/**
 * 46:
 * @brief   Write(Transmit data), returns true if successfully sent
 * @param:
 * @retval:
 */
bool NRF24_write(const void *buf, uint8_t len)
{
    bool retStatus;
    // Start writing
    NRF24_reset_Status();
    NRF24_startWrite(buf, len);
    //ms to wait for timeout
    vTaskDelay(10 / portTICK_PERIOD_MS);
    //    Wait until complete or failed
    bool tx_ok, tx_fail;
    NRF24_whatHappened(&tx_ok, &tx_fail, &ack_payload_available);
    retStatus = tx_ok;
    if (ack_payload_available)
    {
        ack_payload_length = NRF24_get_DynamicPayloadSize();
    }
    // Power down
    NRF24_available();
    NRF24_flush_Tx();
    // printf("NRF24_write Status %d \n\r", retStatus);
    return retStatus;
}

/**
 * 47:
 * @brief   Read received data
 * @param:
 * @retval:
 */
bool NRF24_read(void *buf, uint8_t len)
{
    NRF24_read_Payload(buf, len);
    uint8_t rxStatus = NRF24_read_Register(REG_FIFO_STATUS) & _BV(BIT_RX_EMPTY);
    NRF24_flush_Rx();
    NRF24_get_DynamicPayloadSize();
    return rxStatus;
}

/**
 * 48:
 * @brief  Open Tx pipe for writing (Cannot perform this while Listenning, has to call NRF24_stopListening)
 * @param:
 * @retval:
 */
void NRF24_openWritingPipe(uint64_t address)
{
    NRF24_write_RegisterN(REG_RX_ADDR_P0, (uint8_t *)(&address), 5);
    NRF24_write_RegisterN(REG_TX_ADDR, (uint8_t *)(&address), 5);

    const uint8_t max_payload_size = 32;
    NRF24_write_Register(REG_RX_PW_P0, MIN(payload_size, max_payload_size));
}
/**
 * 49:
 * @brief   Open reading pipe
 * @param:
 * @retval:
 */
void NRF24_openReadingPipe(uint8_t number, uint64_t address)
{
    if (number == 0)
        pipe0_reading_address = address;

    if (number <= 6)
    {
        if (number < 2)
        {
            // Address width is 5 bytes
            NRF24_write_RegisterN(NRF24_ADDR_REGS[number],
                                  (uint8_t *)(&address), 5);
        }
        else
        {
            NRF24_write_RegisterN(NRF24_ADDR_REGS[number],
                                  (uint8_t *)(&address), 1);
        }
        // Write payload size
        NRF24_write_Register(RF24_RX_PW_PIPE[number], payload_size);
        // Enable pipe
        NRF24_write_Register(REG_EN_RXADDR,
                             NRF24_read_Register(REG_EN_RXADDR) | _BV(number));
    }
}

/**
 * 50:
 * @brief   Listen on open pipes for reading (Must call NRF24_openReadingPipe() first)
 * @param:
 * @retval:
 */
void NRF24_startListening(void)
{
    // Power up and set to RX mode
    NRF24_write_Register(REG_CONFIG,
                         NRF24_read_Register(REG_CONFIG) | (1UL << 1) | (1UL << 0));
    // Restore pipe 0 address if exists
    if (pipe0_reading_address)
        NRF24_write_RegisterN(REG_RX_ADDR_P0,
                              (uint8_t *)(&pipe0_reading_address), 5);

    // Flush buffers
    NRF24_flush_Tx();
    NRF24_flush_Rx();
    // Set CE HIGH to start listenning
    NRF24_CE(SET);
    // Wait for 130 uSec for the radio to come on
    microsDelay(150);
}

/**
 * 51:
 * @brief   Stop listening (essential before any write operation)
 * @param:
 * @retval:
 */
void NRF24_stopListening(void)
{
    NRF24_CE(RESET);
    NRF24_flush_Tx();
    NRF24_flush_Rx();
}

/**
 * 52:
 * @brief   NRF24 Software Reset
 * @param:
 * @retval:
 */
void NRF24_SoftwareReset(void)
{
    // Put pins to idle state
    NRF24_CSN(SET);
    NRF24_CE(RESET);
    // 5 ms initial delay
    vTaskDelay(5 / portTICK_PERIOD_MS);
    // HAL_Delay(5);

    //**** Soft Reset Registers default values ****//
    NRF24_write_Register(0x00, 0x08);
    NRF24_write_Register(0x01, 0x3f);
    NRF24_write_Register(0x02, 0x03);
    NRF24_write_Register(0x03, 0x03);
    NRF24_write_Register(0x04, 0x03);
    NRF24_write_Register(0x05, 0x02);
    NRF24_write_Register(0x06, 0x0F);
    NRF24_write_Register(0x07, 0x0e);
    NRF24_write_Register(0x08, 0x00);
    NRF24_write_Register(0x09, 0x00);
    uint8_t pipeAddrVar[5];
    pipeAddrVar[4] = 0xE7;
    pipeAddrVar[3] = 0xE7;
    pipeAddrVar[2] = 0xE7;
    pipeAddrVar[1] = 0xE7;
    pipeAddrVar[0] = 0xE7;
    NRF24_write_RegisterN(0x0A, pipeAddrVar, 5);
    pipeAddrVar[4] = 0xC2;
    pipeAddrVar[3] = 0xC2;
    pipeAddrVar[2] = 0xC2;
    pipeAddrVar[1] = 0xC2;
    pipeAddrVar[0] = 0xC2;
    NRF24_write_RegisterN(0x0B, pipeAddrVar, 5);
    NRF24_write_Register(0x0C, 0xC3);
    NRF24_write_Register(0x0D, 0xC4);
    NRF24_write_Register(0x0E, 0xC5);
    NRF24_write_Register(0x0F, 0xC6);
    pipeAddrVar[4] = 0xE7;
    pipeAddrVar[3] = 0xE7;
    pipeAddrVar[2] = 0xE7;
    pipeAddrVar[1] = 0xE7;
    pipeAddrVar[0] = 0xE7;
    NRF24_write_RegisterN(0x10, pipeAddrVar, 5);
    NRF24_write_Register(0x11, 0);
    NRF24_write_Register(0x12, 0);
    NRF24_write_Register(0x13, 0);
    NRF24_write_Register(0x14, 0);
    NRF24_write_Register(0x15, 0);
    NRF24_write_Register(0x16, 0);
    NRF24_ACTIVATE_cmd();
    NRF24_write_Register(0x1c, 0);
    NRF24_write_Register(0x1d, 0);
    // printRadioSettings();
}

/**
 * 53:
 * @brief  NRF24 Radio setting
 * @param:
 * @retval:
 */
void NRF24_Radio_Config(void)
{
    printf("\tConfig  NRF24 Radio module \n\r");
    NRF24_powerDown(); // Enabling change of configuration and the uploading/downloading of data registers
    NRF24_set_Channel(25);
    NRF24_set_DataRate(RF24_250KBPS); // RF24_250KBPS RF24_2MBPS RF24_1MBPS
    NRF24_set_PALevel(RF24_PA_m12dB); // RF24_PA_m12dB, RF24_PA_m6dB, RF24_PA_0dB
    // Initialize CRC lenth
    NRF24_set_CRCLength(RF24_CRC_16); // RF24_CRC_8, RF24_CRC_16
    // Set payload size
    NRF24_set_PayloadSize(32);
    // Initialise retries 15 and delay 1250 usec
    NRF24_set_Retries(15, 15);
    // Disable dynamic payload
    NRF24_disable_DynamicPayloads();
    // Reset status register
    NRF24_reset_Status();
    // Flush buffers
    NRF24_flush_Tx();
    NRF24_flush_Rx();
}

void NRF24_set_Retries(uint8_t delay, uint8_t count)
{
    NRF24_write_Register(REG_SETUP_RETR,
                         (delay & 0xf) << BIT_ARD | (count & 0xf) << BIT_ARC);
}


void NRF24_REST(NRF24_datarate data_reat, NRF24_pa_dbm power_level, uint8_t delay){

    // 2. Software Reset
    NRF24_SoftwareReset();

    // 3. Automatic Retransmission Settings
    NRF24_set_Retries(15, delay); // Set retries (15) and delay (1250us)

    // 4. Data Rate
    NRF24_set_DataRate(data_reat); // Choose desired data rate

    // 5. Power Level
    NRF24_set_PALevel(power_level); // Choose desired power level

    // 6. CRC Length
    NRF24_set_CRCLength(RF24_CRC_16); // Set CRC length

    // 7. Disable Dynamic Payload
    NRF24_disable_DynamicPayloads();

    // 8. Payload Size
    NRF24_set_PayloadSize(32); // Set fixed payload size (32 bytes)

    // 9. Reset Status Register
    NRF24_reset_Status();

    // 10. Set Channel
    NRF24_set_Channel(52); // Set communication channel

    // 11. Flush FIFOs
    NRF24_flush_Tx();
    NRF24_flush_Rx();

    // 12. Power Down (can be powered up later)
    NRF24_powerDown();


}

/**
 * @brief Initializes the NRF24L01 module for communication.
 *
 * This function performs the following initialization steps for the NRF24L01 module:
 * - Configures GPIO and SPI for communication with the NRF24L01 module (refer to NRF24_Bus_Config() documentation).
 * - Performs a software reset on the NRF24L01 module.
 * - Sets the automatic retransmission attempts (retries) and delay between retries.
 * - Sets the data rate for communication (RF24_250KBPS, RF24_1MBPS, RF24_2MBPS).
 * - Sets the output power level (PA Level) for the NRF24L01 module (RF24_PA_m12dB, RF24_PA_m6dB, RF24_PA_0dB).
 * - Initializes the CRC length (Cyclic Redundancy Check).
 * - Disables dynamic payload length for fixed-size payloads.
 * - Sets the desired payload size in bytes (here, 32 bytes).
 * - Resets the status register of the NRF24L01 module.
 * - Sets the RF channel for communication (here, channel 76).
 * - Flushes the transmit (TX) and receive (RX) FIFOs of the NRF24L01 module.
 * - Powers down the NRF24L01 module (can be powered up later using NRF24_powerUp()).
 * - Optionally, calls a function `printRadioSettings()` to display the configuration details (implementation not provided).
 */
void NRF24_Init(NRF24_datarate data_reat, NRF24_pa_dbm power_level, uint8_t delay)
{

    // 1. GPIO and SPI Configuration
    NRF24_Bus_Config(); // Call user-defined function for GPIO/SPI setup

    // 2. Software Reset
    NRF24_SoftwareReset();

    // 3. Automatic Retransmission Settings
    NRF24_set_Retries(15, delay); // Set retries (15) and delay (1250us)

    // 4. Data Rate
    NRF24_set_DataRate(data_reat); // Choose desired data rate

    // 5. Power Level
    NRF24_set_PALevel(power_level); // Choose desired power level

    // 6. CRC Length
    NRF24_set_CRCLength(RF24_CRC_16); // Set CRC length

    // 7. Disable Dynamic Payload
    NRF24_disable_DynamicPayloads();

    // 8. Payload Size
    NRF24_set_PayloadSize(32); // Set fixed payload size (32 bytes)

    // 9. Reset Status Register
    NRF24_reset_Status();

    // 10. Set Channel
    NRF24_set_Channel(52); // Set communication channel

    // 11. Flush FIFOs
    NRF24_flush_Tx();
    NRF24_flush_Rx();

    // 12. Power Down (can be powered up later)
    NRF24_powerDown();

    // 13. Print Settings (Optional)
    // printRadioSettings(); // Optional function to display settings (not provided)
}

/**
 * 55:
 * @brief  Check if module is NRF24L01+ or normal module
 * @param: Non
 * @retval:Non
 */
bool NRF24_isNRF_Plus(void)
{
    return p_variant;
}

/**
 * 56:
 * @brief  Check if an Ack payload is available
 * @param:
 * @retval: None
 */
// 5. FIFO Status
void printFIFOstatus(void)
{
    uint8_t reg8Val;
    printf("\t\r\n-------------------------\r\n");
    reg8Val = NRF24_read_Register(0x17);
    printf(
        "\tFIFO Status reg:\t\r\n		TX_FULL:		%d\r\n		TX_EMPTY:		%d\r\n		RX_FULL:		%d\r\n		RX_EMPTY:		%d\r\n",
        _BOOL(reg8Val & (1 << 5)), _BOOL(reg8Val & (1 << 4)),
        _BOOL(reg8Val & (1 << 1)), _BOOL(reg8Val & (1 << 0)));
    printf("\t\r\n-------------------------\r\n");
}

/**
 * 57:
 * @brief  Print radio setting
 * @param:
 * @retval:
 */
// 1.
void printRadioSettings(void)
{
    uint8_t reg8Val;
    printf("\t****************************************************\r\n");
    // a) Get CRC settings - Config Register
    reg8Val = NRF24_read_Register(0x00);
    if (reg8Val & (1 << 3))
    {
        if (reg8Val & (1 << 2))
            printf("\tCRC:\tEnabled, 2 Bytes\r\n");
        else
            printf("\tCRC:\tEnabled, 1 Byte\r\n");
    }
    else
    {
        printf("\tCRC:\tDisabled\r\n");
    }

    // b) AutoAck on pipes
    reg8Val = NRF24_read_Register(0x01);
    printf("\tENAA:\tP0: %d\tP1 :%d\tP2 :%d\tP3 :%d\tP4 :%d\tP5 :%d\r\n",
           _BOOL(reg8Val & (1 << 0)), _BOOL(reg8Val & (1 << 1)),
           _BOOL(reg8Val & (1 << 2)), _BOOL(reg8Val & (1 << 3)),
           _BOOL(reg8Val & (1 << 4)), _BOOL(reg8Val & (1 << 5)));

    // c) Enabled Rx addresses
    reg8Val = NRF24_read_Register(0x02);
    printf("\tEN_RXADDR:\tP0 :%d\tP1 :%d\tP2 :%d\tP3 :%d\tP4 :%d\tP5 :%d\r\n",
           _BOOL(reg8Val & (1 << 0)), _BOOL(reg8Val & (1 << 1)),
           _BOOL(reg8Val & (1 << 2)), _BOOL(reg8Val & (1 << 3)),
           _BOOL(reg8Val & (1 << 4)), _BOOL(reg8Val & (1 << 5)));

    // d) Address width
    reg8Val = NRF24_read_Register(0x03) & 0x03;
    reg8Val += 2;
    printf("\tSETUP_AW:\t%d bytes \r\n", reg8Val);

    // e) RF channel
    reg8Val = NRF24_read_Register(0x05);
    printf("\tRF_CH:\t%d CH\r\n", reg8Val & 0x7F);

    // f1) Data rate
    reg8Val = NRF24_read_Register(0x06) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));
    if (reg8Val == _BV(RF_DR_LOW))
        printf("\tData Rate:\t250Kbps\r\n"); // '10' = 250KBPS
    else if (reg8Val & _BV(RF_DR_HIGH))
        printf("\tData Rate:\t2Mbps\r\n");
    else
    {
        printf("\tData Rate:\t1Mbps\r\n");
    }

    // f2) Data rate
    reg8Val = NRF24_read_Register(0x06);

    reg8Val &= (3 << 1);
    reg8Val = (reg8Val >> 1);
    if (reg8Val == 0)
        printf("\tRF_PWR:\t-18dB\r\n");
    else if (reg8Val == 1)
        printf("\tRF_PWR:\t-12dB\r\n");
    else if (reg8Val == 2)
        printf("\tRF_PWR:\t-6dB\r\n");
    else if (reg8Val == 3)
        printf("\tRF_PWR:\t0dB\r\n");

    // g) RX pipes addresses
    uint8_t pipeAddrs[6];
    NRF24_read_RegisterN(0x0A, pipeAddrs, 5);
    printf("\tRX_Pipe0 Addrs:\t%02X,%02X,%02X,%02X,%02X\r\n", pipeAddrs[4],
           pipeAddrs[3], pipeAddrs[2], pipeAddrs[1], pipeAddrs[0]);

    NRF24_read_RegisterN(0x0A + 1, pipeAddrs, 5);
    printf("\tRX_Pipe1 Addrs:\t%02X,%02X,%02X,%02X,%02X\r\n", pipeAddrs[4],
           pipeAddrs[3], pipeAddrs[2], pipeAddrs[1], pipeAddrs[0]);

    NRF24_read_RegisterN(0x0A + 2, pipeAddrs, 1);
    printf("\tRX_Pipe2 Addrs:\txx,xx,xx,xx,%02X\r\n", pipeAddrs[0]);

    NRF24_read_RegisterN(0x0A + 3, pipeAddrs, 1);
    printf("\tRX_Pipe3 Addrs:\txx,xx,xx,xx,%02X\r\n", pipeAddrs[0]);

    NRF24_read_RegisterN(0x0A + 4, pipeAddrs, 1);
    printf("\tRX_Pipe4 Addrs:\txx,xx,xx,xx,%02X\r\n", pipeAddrs[0]);

    NRF24_read_RegisterN(0x0A + 5, pipeAddrs, 1);
    printf("\tRX_Pipe5 Addrs:\txx,xx,xx,xx,%02X\r\n", pipeAddrs[0]);

    NRF24_read_RegisterN(0x0A + 6, pipeAddrs, 5);
    printf("\tTX Addrs:\t%02X,%02X,%02X,%02X,%02X\r\n", pipeAddrs[4],
           pipeAddrs[3], pipeAddrs[2], pipeAddrs[1], pipeAddrs[0]);

    // h) RX PW (Payload Width 0 - 32)
    reg8Val = NRF24_read_Register(0x11);
    printf("\tRX_PW_P0:\t%d bytes \r\n", reg8Val & 0x3F);

    reg8Val = NRF24_read_Register(0x11 + 1);
    printf("\tRX_PW_P1:\t%d bytes \r\n", reg8Val & 0x3F);

    reg8Val = NRF24_read_Register(0x11 + 2);
    printf("\tRX_PW_P2:\t%d bytes \r\n", reg8Val & 0x3F);

    //(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

    reg8Val = NRF24_read_Register(0x11 + 3);
    printf("\tRX_PW_P3:\t%d bytes \r\n", reg8Val & 0x3F);

    reg8Val = NRF24_read_Register(0x11 + 4);
    printf("\tRX_PW_P4:\t%d bytes \r\n", reg8Val & 0x3F);

    reg8Val = NRF24_read_Register(0x11 + 5);
    printf("\tRX_PW_P5:\t%d bytes \r\n", reg8Val & 0x3F);

    // i) Dynamic payload enable for each pipe
    reg8Val = NRF24_read_Register(0x1c);
    printf("\tDYNPD_pipe:\tP0 :%d\tP1 :%d\tP2 :%d\tP3 :%d\tP4 :%d\tP5 :%d\r\n",
           _BOOL(reg8Val & (1 << 0)), _BOOL(reg8Val & (1 << 1)),
           _BOOL(reg8Val & (1 << 2)), _BOOL(reg8Val & (1 << 3)),
           _BOOL(reg8Val & (1 << 4)), _BOOL(reg8Val & (1 << 5)));

    // j) EN_DPL (is Dynamic payload feature enabled ?)
    reg8Val = NRF24_read_Register(0x1d);
    if (reg8Val & (1 << 2))
        printf("\tEN_DPL:\tEnabled \r\n");
    else
        printf("\tEN_DPL:\tDisabled \r\n");

    // k) EN_ACK_PAY
    if (reg8Val & (1 << 1))
        printf("\tEN_ACK_PAY:\tEnabled \r\n");
    else
        printf("\tEN_ACK_PAY:\tDisabled \r\n");
    printf("\t****************************************************\r\n");
}

/**
 * 58:
 * @brief Print Status
 * @param:
 * @retval:
 */
void printStatusReg(void)
{
    uint8_t reg8Val;
    printf("\t\r\n-------------------------\r\n");
    reg8Val = NRF24_read_Register(0x07);
    printf(
        "\tSTATUS reg:\tRX_DR :%d\tTX_DS :%d\tMAX_RT :%d\tRX_P_NO :%d\tTX_FULL :%d\r\n",
        _BOOL(reg8Val & (1 << 6)), _BOOL(reg8Val & (1 << 5)),
        _BOOL(reg8Val & (1 << 4)), _BOOL(reg8Val & (3 << 1)),
        _BOOL(reg8Val & (1 << 0)));
    printf("\t-------------------------\r\n");
}

/**
 * 59:
 * @brief  Print register configuration
 * @param None
 * @retval None
 */
void printConfigReg(void)
{
    uint8_t reg8Val;
    printf("\t-------------------------\r\n");
    reg8Val = NRF24_read_Register(0x00);
    printf("\tCONFIG reg:\tPWR_UP :%d\tPRIM_RX :%d\r\n",
           _BOOL(reg8Val & (1 << 1)), _BOOL(reg8Val & (1 << 0)));
    printf("\t-------------------------\r\n");
}