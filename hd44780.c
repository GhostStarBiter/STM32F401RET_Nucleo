#include "hd44780.h"

/* Private HD44780 structure */
typedef struct {
	int DisplayControl;
	int DisplayFunction;
	int DisplayMode;
	int Rows;
	int Cols;
    int currentX;
	int currentY;
} HD44780_Options_t;

/* Private functions */

static void HD44780_Cmd(int cmd);
static void HD44780_Cmd4bit(int cmd);
static void HD44780_Data(int data);
static void HD44780_CursorSet(int col, int row);

/* Private variable */
static HD44780_Options_t HD44780_Opts;


/* Commands*/
#define HD44780_CLEARDISPLAY        0x01
#define HD44780_RETURNHOME          0x02
#define HD44780_ENTRYMODESET        0x04
#define HD44780_DISPLAYCONTROL      0x08
#define HD44780_CURSORSHIFT         0x10
#define HD44780_FUNCTIONSET         0x20
#define HD44780_SETCGRAMADDR        0x40
#define HD44780_SETDDRAMADDR        0x80

/* Flags for display entry mode */
#define HD44780_ENTRYRIGHT          0x00
#define HD44780_ENTRYLEFT           0x02
#define HD44780_ENTRYSHIFTINCREMENT 0x01
#define HD44780_ENTRYSHIFTDECREMENT 0x00

/* Flags for display on/off control */
#define HD44780_DISPLAY_OFF			0x00
#define HD44780_DISPLAYON           0x04
#define HD44780_CURSORON            0x02
#define HD44780_BLINKON             0x01

/* Flags for display/cursor shift */
#define HD44780_DISPLAYMOVE         0x08
#define HD44780_CURSORMOVE          0x00
#define HD44780_MOVERIGHT           0x04
#define HD44780_MOVELEFT            0x00

/* Flags for function set */
#define HD44780_8BITMODE            0x10
#define HD44780_4BITMODE            0x00
#define HD44780_2LINE               0x08
#define HD44780_1LINE               0x00
#define HD44780_5x10DOTS            0x04
#define HD44780_5x8DOTS             0x00

/* Pin definitions */
#define HD44780_RS_LOW              HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN, GPIO_PIN_RESET)
#define HD44780_RS_HIGH             HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN, GPIO_PIN_SET)
#define HD44780_E_LOW               HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN, GPIO_PIN_RESET)
#define HD44780_E_HIGH              HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN, GPIO_PIN_SET)
#define HD44780_Delay(x)            HAL_Delay(x)

#define HD44780_E_BLINK             HD44780_E_HIGH; HD44780_Delay(40); HD44780_E_LOW; HD44780_Delay(40)



static void reset_lcd_cntrl_Pins(void){
    HAL_GPIO_WritePin(HD44780_RS_PORT, HD44780_RS_PIN,GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN,GPIO_PIN_RESET);
}
static void reset_lcd_data_pins(void){
	HAL_GPIO_WritePin(HD44780_D4_PORT, HD44780_D4_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HD44780_D5_PORT, HD44780_D5_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HD44780_D6_PORT, HD44780_D6_PIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(HD44780_D7_PORT, HD44780_D7_PIN,GPIO_PIN_RESET);
}

void HD44780_Init(int cols, int rows) {
    /* Set LCD width and height */
    HD44780_Opts.Rows = rows;
    HD44780_Opts.Cols = cols;
    /* Set cursor pointer to beginning for LCD */
    HD44780_Opts.currentX = 0;
    HD44780_Opts.currentY = 0;

	reset_lcd_cntrl_Pins();
	reset_lcd_data_pins();
	HD44780_Delay(50);

	/* set 4bit mode */
	HD44780_Cmd4bit(0x03);
	HD44780_Delay(40);
	HD44780_Cmd4bit(0x03);
	HD44780_Delay(40);
	HD44780_Cmd4bit(0x03);
	HD44780_Delay(40);
	HD44780_Cmd4bit(0x02);
	HD44780_Delay(50);
	//Function Set: 2-line, 5x8 matrix
	HD44780_Cmd(HD44780_FUNCTIONSET|HD44780_2LINE|HD44780_5x8DOTS);
    HD44780_Delay(50);
	//Display Off, Cursor Off, Blink Off
	HD44780_Cmd(HD44780_DISPLAYCONTROL|HD44780_DISPLAY_OFF);
    HD44780_Delay(50);
	//Clear Screen & Return Cursor Home
	HD44780_Cmd(HD44780_CLEARDISPLAY);
    HD44780_Delay(50);
	//Inc cursor to right then writing & don't shift screen
    HD44780_Cmd(HD44780_ENTRYMODESET|HD44780_ENTRYLEFT|HD44780_ENTRYSHIFTDECREMENT);
	HD44780_Delay(50);
	//Display On, Cursor On, Blink Off
	HD44780_Cmd(HD44780_DISPLAYCONTROL|HD44780_DISPLAYON|HD44780_CURSORON);
    HD44780_Delay(50);
    HD44780_Clear();
}

void HD44780_Clear(void) {
	HD44780_Cmd(HD44780_CLEARDISPLAY);
	//HD44780_Delay(10);
}

void HD44780_Puts(int x, int y, char* str) {
	HD44780_CursorSet(x, y);
	while (*str) {
		if (HD44780_Opts.currentX >= HD44780_Opts.Cols) {
			HD44780_Opts.currentX = 0;
			HD44780_Opts.currentY++;
			HD44780_CursorSet(HD44780_Opts.currentX, HD44780_Opts.currentY);
		}
		if (*str == '\n') {
			HD44780_Opts.currentY++;
			HD44780_CursorSet(HD44780_Opts.currentX, HD44780_Opts.currentY);
		} else if (*str == '\r') {
			HD44780_CursorSet(0, HD44780_Opts.currentY);
		} else if (*str == '\0')
			return;
		else {
			HD44780_Data(*str);
			HD44780_Opts.currentX++;
		}
		str++;
	}
}

void HD44780_DisplayOn(void) {
	HD44780_Opts.DisplayControl |= HD44780_DISPLAYON;
	HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void HD44780_DisplayOff(void) {
	HD44780_Opts.DisplayControl &= ~HD44780_DISPLAYON;
	HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void HD44780_BlinkOn(void) {
	HD44780_Opts.DisplayControl |= HD44780_BLINKON;
	HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void HD44780_BlinkOff(void) {
	HD44780_Opts.DisplayControl &= ~HD44780_BLINKON;
	HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void HD44780_CursorOn(void) {
	HD44780_Opts.DisplayControl |= HD44780_CURSORON;
	HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void HD44780_CursorOff(void) {
	HD44780_Opts.DisplayControl &= ~HD44780_CURSORON;
	HD44780_Cmd(HD44780_DISPLAYCONTROL | HD44780_Opts.DisplayControl);
}

void HD44780_ScrollLeft(void) {
	HD44780_Cmd(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVELEFT);
}

void HD44780_ScrollRight(void) {
	HD44780_Cmd(HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVERIGHT);
}

void HD44780_CreateChar(int location, int *data) {
	short i;
	/* We have 8 locations available for custom characters */
	location &= 0x07;
	HD44780_Cmd(HD44780_SETCGRAMADDR | (location << 3));
	
	for (i = 0; i < 8; i++) {
		HD44780_Data(data[i]);
	}
}

void HD44780_PutCustom(int x, int y, int location) {
	HD44780_CursorSet(x, y);
	HD44780_Data(location);
}

/* Private functions */
static void HD44780_Cmd(int cmd) {
	/* Command mode */
	HD44780_RS_LOW;
	/* High nibble */
	HD44780_Cmd4bit(cmd >> 4);
	/* Low nibble */
	HD44780_Cmd4bit(cmd & 0x0F);
	HD44780_RS_HIGH;
}

static void HD44780_Data(int data) {
	/* Data mode */
	HD44780_RS_HIGH;
	/* High nibble */
    int high_nibble = data >> 4;
	HD44780_Cmd4bit(high_nibble);
	/* Low nibble */
    int low_nibble = data & 0x0F;
	HD44780_Cmd4bit(low_nibble);
}

static void HD44780_Cmd4bit(int cmd) {
	/* Set output port */
	if((cmd & 0x08) == 8)
		HAL_GPIO_WritePin(HD44780_D7_PORT, HD44780_D7_PIN,GPIO_PIN_SET);
	if((cmd & 0x04) == 4)
		HAL_GPIO_WritePin(HD44780_D6_PORT, HD44780_D6_PIN,GPIO_PIN_SET);
	if((cmd & 0x02) == 2)
		HAL_GPIO_WritePin(HD44780_D5_PORT, HD44780_D5_PIN,GPIO_PIN_SET);
	if((cmd & 0x01) == 1)
		HAL_GPIO_WritePin(HD44780_D4_PORT, HD44780_D4_PIN,GPIO_PIN_SET);
    //LCD_EN pin strobe
    HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(HD44780_E_PORT, HD44780_E_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
	reset_lcd_data_pins();
}

static void HD44780_CursorSet(int col, int row) {
	short row_offsets[] = {0x00, 0x40, 0x14, 0x54};
	
	/* Go to beginning */
	if (row >= HD44780_Opts.Rows) {
		row = 0;
	}
	
	/* Set current column and row */
	HD44780_Opts.currentX = col;
	HD44780_Opts.currentY = row;
	
	/* Set location address */
	HD44780_Cmd(HD44780_SETDDRAMADDR | (col + row_offsets[row]));
}
