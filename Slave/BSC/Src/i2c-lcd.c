/**
 * Copyright Nikita Bulaev 2017
 *
 * STM32 HAL libriary for LCD display based on HITACHI HD44780U chip.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "stdlib.h"
#include "string.h"
#include "i2c-lcd.h"

uint8_t lcdCommandBuffer[6] = { 0x00 };

static LCDParams lcdParams;

/**
 * @brief  Turn display on and init it params
 * @note   We gonna make init steps according to datasheep page 46.
 *         There are 4 steps to turn 4-bits mode on,
 *         then we send initial params.
 * @param  hi2c    I2C struct to which display is connected
 * @param  address Display I2C 7-bit address
 * @param  lines   Number of lines of display
 * @param  columns Number of colums
 * @return         true if success
 */
bool lcdInit(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t lines,
		uint8_t columns) {

	uint8_t lcdData = LCD_BIT_5x8DOTS;

	lcdParams.hi2c = hi2c;
	lcdParams.address = address; //address << 1;
	lcdParams.lines = lines;
	lcdParams.columns = columns;
	lcdParams.backlight = LCD_BIT_BACKLIGHT_ON;
	lcdCommandBuffer[0] = (0x03 << 4);
	lcdCommandBuffer[1] = (0x03 << 4) | LCD_BIT_E;
	lcdCommandBuffer[2] = (0x03 << 4);

	/* First 3 steps of init cycles. They are the same. */

	for (uint8_t i = 0; i < 3; ++i) {
		if (HAL_I2C_Master_Transmit(lcdParams.hi2c, lcdParams.address,
				(uint8_t*) lcdCommandBuffer, 3, 100) != HAL_OK) {
			return false;
		}
		while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
			HAL_Delay(10);
		}

		if (i == 2) {
			// For the last cycle delay is less then 1 ms (100us by datasheet)
			HAL_Delay(5);
		} else {
			// For first 2 cycles delay is less then 5ms (4100us by datasheet)
			HAL_Delay(5);
		}
	}

	/* Lets turn to 4-bit at least */
	lcdCommandBuffer[0] = LCD_BIT_BACKLIGHT_ON | LCD_BIT_FUNCTION_SET;
	lcdCommandBuffer[1] = LCD_BIT_BACKLIGHT_ON | LCD_BIT_FUNCTION_SET
			| LCD_BIT_E;
	lcdCommandBuffer[2] = LCD_BIT_BACKLIGHT_ON | LCD_BIT_FUNCTION_SET;

	if (HAL_I2C_Master_Transmit(lcdParams.hi2c, lcdParams.address,
			(uint8_t*) lcdCommandBuffer, 3, 100) != HAL_OK) {
		return false;
	}

	while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
		HAL_Delay(1);
	}

	/* Lets set display params */
	/* First of all lets set display size */

	lcdData |= LCD_BIT_FUNCTION_SET;

	if (lcdParams.lines > 1) {
		lcdData |= LCD_BIT_2LINE;
	}

	lcdWriteByte(Byte_CMD, &lcdData); // TODO: Make 5x10 dots font usable for some 1-line display

	/* Now lets set display, cursor and blink all on */
	lcdDisplayOn();
	HAL_Delay(5);
	/* Set cursor moving to the right */
	lcdCursorDirToRight();
	HAL_Delay(5);
	/* Clear display and Set cursor at Home */
	lcdDisplayClear();
	HAL_Delay(5);
	lcdCursorHome();
	HAL_Delay(5);
	return true;
}

/**
 * @brief  Send command to display
 * @param  command  One of listed in LCDCommands enum
 * @param  action   LCD_PARAM_SET or LCD_PARAM_UNSET
 * @return          true if success
 */
bool lcdCommand(LCDCommands command, LCDParamsActions action) {
	uint8_t lcdData = 0x00;

	/* First of all lest store the command */
	switch (action) {
	case LCD_PARAM_SET:
		switch (command) {
		case LCD_DISPLAY:
			lcdParams.modeBits |= LCD_BIT_DISPLAY_ON;
			break;

		case LCD_CURSOR:
			lcdParams.modeBits |= LCD_BIT_CURSOR_ON;
			break;

		case LCD_CURSOR_BLINK:
			lcdParams.modeBits |= LCD_BIT_BLINK_ON;
			break;

		case LCD_CLEAR:
			lcdData = LCD_BIT_DISP_CLEAR;

			if (lcdWriteByte((uint8_t) 0x00, &lcdData) == false) {
				return false;
			} else {
				HAL_Delay(1);
				return true;
			}

		case LCD_CURSOR_HOME:
			lcdData = LCD_BIT_CURSOR_HOME;

			if (lcdWriteByte((uint8_t) 0x00, &lcdData) == false) {
				return false;
			} else {
				HAL_Delay(2);
				return true;
			}

		case LCD_CURSOR_DIR_RIGHT:
			lcdParams.entryBits |= LCD_BIT_CURSOR_DIR_RIGHT;
			break;

		case LCD_CURSOR_DIR_LEFT:
			lcdParams.entryBits |= LCD_BIT_CURSOR_DIR_LEFT;
			break;

		case LCD_DISPLAY_SHIFT:
			lcdParams.entryBits |= LCD_BIT_DISPLAY_SHIFT;
			break;

		default:
			return false;
		}

		break;

	case LCD_PARAM_UNSET:
		switch (command) {
		case LCD_DISPLAY:
			lcdParams.modeBits &= ~LCD_BIT_DISPLAY_ON;
			break;

		case LCD_CURSOR:
			lcdParams.modeBits &= ~LCD_BIT_CURSOR_ON;
			break;

		case LCD_CURSOR_BLINK:
			lcdParams.modeBits &= ~LCD_BIT_BLINK_ON;
			break;

		case LCD_CURSOR_DIR_RIGHT:
			lcdParams.entryBits &= ~LCD_BIT_CURSOR_DIR_RIGHT;
			break;

		case LCD_CURSOR_DIR_LEFT:
			lcdParams.entryBits &= ~LCD_BIT_CURSOR_DIR_LEFT;
			break;

		case LCD_DISPLAY_SHIFT:
			lcdParams.entryBits &= ~LCD_BIT_DISPLAY_SHIFT;
			break;

		default:
			return false;
		}

		break;

	default:
		return false;
	}

	/* Now lets send the command */
	switch (command) {
	case LCD_DISPLAY:
	case LCD_CURSOR:
	case LCD_CURSOR_BLINK:
		lcdData = LCD_BIT_DISPLAY_CONTROL | lcdParams.modeBits;
		break;

	case LCD_CURSOR_DIR_RIGHT:
	case LCD_CURSOR_DIR_LEFT:
	case LCD_DISPLAY_SHIFT:
		lcdData = LCD_BIT_ENTRY_MODE | lcdParams.entryBits;
		break;

	default:
		break;
	}

	return lcdWriteByte(Byte_CMD, &lcdData);
}

/**
 * @brief  Turn display's Backlight On or Off
 * @param  command LCD_BIT_BACKIGHT_ON to turn display On
 *                 LCD_BIT_BACKIGHT_OFF (or 0x00) to turn display Off
 * @return         true if success
 */
bool lcdBacklight(uint8_t command) {
	lcdParams.backlight = command;

	if (HAL_I2C_Master_Transmit(lcdParams.hi2c, lcdParams.address,
			&lcdParams.backlight, 1, 100) != HAL_OK) {
		return false;
	}

	while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
		HAL_Delay(1);
	}
	return true;
}

/**
 * @brief  Set cursor position on the display
 * @param  column counting from 0
 * @param  line   counting from 0
 * @return        true if success
 */
bool lcdSetCursorPosition(uint8_t column, uint8_t line) {
	// We will setup offsets for 4 lines maximum
	static const uint8_t lineOffsets[4] = { 0x00, 0x40, 0x14, 0x54 };

	if (line >= lcdParams.lines) {
		line = lcdParams.lines - 1;
	}

	uint8_t lcdCommand = LCD_BIT_SETDDRAMADDR | (column + lineOffsets[line]);

	return lcdWriteByte(Byte_CMD, &lcdCommand);
}

/**
 * @brief  Print string from cursor position
 * @param  data   Pointer to string
 * @param  length Number of symbols to print
 * @return        true if success
 */
bool lcdPrintStr(uint8_t *data, uint8_t length) {
	for (uint8_t i = 0; i < length; ++i) {
		if (lcdWriteByte(Byte_DATA, &data[i]) == false) {
			return false;
		}
	}

	return true;
}

/**
 * @brief  Print single char at cursor position
 * @param  data Symbol to print
 * @return      true if success
 */
bool lcdPrintChar(uint8_t data) {
	return lcdWriteByte(LCD_BIT_RS, &data);
}

/**
 * @brief Loading custom Chars to one of the 8 cells in CGRAM
 * @note  You can create your custom chars according to
 *        documentation page 15.
 *        It consists of array of 8 bytes.
 *        Each byte is line of dots. Lower bits are dots.
 * @param  cell     Number of cell from 0 to 7 where to upload
 * @param  charMap  Pointer to Array of dots
 *                  Example: { 0x07, 0x09, 0x09, 0x09, 0x09, 0x1F, 0x11 }
 * @return          true if success
 */
bool lcdLoadCustomChar(uint8_t cell, uint8_t *charMap) {

	// Stop, if trying to load to incorrect cell
	if (cell > 7) {
		return false;
	}

	uint8_t lcdCommand = LCD_BIT_SETCGRAMADDR | (cell << 3);

	if (lcdWriteByte((uint8_t) 0x00, &lcdCommand) == false) {
		return false;
	}

	for (uint8_t i = 0; i < 8; ++i) {
		if (lcdWriteByte(LCD_BIT_RS, &charMap[i]) == false) {
			return false;
		}
	}

	return true;
}

/**
 * @brief  Local function to send data to display
 * @param  rsRwBits State of RS and R/W bits
 * @param  data     Pointer to byte to send
 * @return          true if success
 */
bool lcdWriteByte(uint8_t RS_Bits, uint8_t *data) {

	/* Higher 4 bits*/
	lcdCommandBuffer[0] = RS_Bits | lcdParams.backlight | (*data & 0xF0); // Send data and set strobe
	lcdCommandBuffer[1] = RS_Bits | lcdParams.backlight | (*data & 0xF0)
			| LCD_BIT_E; // Strobe turned on E = 1
	lcdCommandBuffer[2] = RS_Bits | lcdParams.backlight | (*data & 0xF0); // Turning strobe off E = 0

	/* Lower 4 bits*/
	lcdCommandBuffer[3] = RS_Bits | lcdParams.backlight | ((*data << 4) & 0xF0); // Send data and set strobe
	lcdCommandBuffer[4] = RS_Bits | lcdParams.backlight | ((*data << 4) & 0xF0)
			| LCD_BIT_E; // Strobe turned on E = 1                                             // Strobe turned on
	lcdCommandBuffer[5] = RS_Bits | lcdParams.backlight | ((*data << 4) & 0xF0); // Turning strobe off E = 0

	if (HAL_I2C_Master_Transmit(lcdParams.hi2c, lcdParams.address,
			(uint8_t*) lcdCommandBuffer, 6, 100) != HAL_OK) {
		return false;
	}

	while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
		HAL_Delay(1);
	}
	return true;
}
