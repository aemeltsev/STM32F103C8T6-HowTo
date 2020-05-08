
#define SSD1306_ADDRESS                0x78
#define SSD1306_WIDTH                  128
#define SSD1306_HEIGHT                 64
#define SSD1306_COMMAND_MODE           0x00
#define SSD1306_DATA_MODE              0x40

/**
 ***************************************************************
 *
 * Setting Commands
 *
 ***************************************************************
 */
#define SSD1306_SET_CONTRAST           0x81
#define SSD1306_ENTIRE_DISPLAY_RESUME  0xA4
#define SSD1306_ENTIRE_DISPLAY_ON      0xA5
#define SSD1306_NORMAL_DISPLAY         0xA6      /*sets the display to be either normal*/
#define SSD1306_INVERSE_DISPLAY        0xA7      /*sets the display to be either inverse*/
#define SSD1306_DISPLAY_OFF            0xAE      /*off*/
#define SSD1306_DISPLAY_ON             0xAF      /*on*/
#define SSD1306_SET_LCOL_START_ADDRESS 0x00
#define SSD1306_SET_HCOL_START_ADDRESS 0x10
#define SSD1306_MEMORY_ADDRESS_MODE    0x20
#define SSD1306_SET_COLUMN_ADDRESS     0x21      /**/
#define SSD1306_SET_PAGE_ADDRESS       0x22      /**/
#define SSD1306_SET_PAGE_START_ADDRESS 0xB0      /**/
#define SSD1306_SET_START_LINE         0x40      /**/
#define SSD1306_SEGMENT_REMAP          0xA0      /**/
#define SSD1306_SET_MULTIPLEX_RATIO    0xA8      /*switches the default 63 multiplex mode to any multiplex ratio*/
#define SSD1306_COM_SCAN_NORMAL        0xC0      /**/
#define SSD1306_COM_SCAN_INVERSE       0xC8      /**/
#define SSD1306_SET_DISPLAY_OFFSET     0xD3      /**/
#define SSD1306_SET_COM_PINS_CONFIG    0xDA      /*sets the COM signals pin configuration*/
#define SSD1306_SET_DISPLAY_CLOCK_DIV  0xD5      /**/
#define SSD1306_SET_PRECHARGE_PERIOD   0xD9      /*set  the  duration  of  the  pre-charge  period*/
#define SSD1306_SET_VCOM_DESELECT_LVL  0xDB      /*adjusts the VCOMH regulator output*/
#define SSD1306_NOP                    0xE3      /*no operation command*/
#define SSD1306_SET_CHARGE_PUMP        0x8D      /*charge pump setting*/
 
 /**
 ***************************************************************
 *
 * Scrolling Command
 *
 ***************************************************************
 */
#define SSD1306_RIGHT_SCROLL_SETUP      0x26     /**/
#define SSD1306_LEFT_SCROLL_SETUP       0x27     /**/
#define SSD1306_VERT_RIGHT_SCROLL_SETUP 0x29     /**/
#define SSD1306_VERT_LEFT_SCROLL_SETUP  0x2A     /**/
#define SSD1306_ACTIVATE_SCROLL         0x2F     /*starts the motion of scrolling*/
#define SSD1306_DEACTIVATE_SCROLL       0x2E    /*stop the motion of scrolling*/
#define SSD1306_SET_VERT_SCROLL_AREA    0xA3    /*consists of 3 consecutive bytes to set up the vertical scroll area.*/
