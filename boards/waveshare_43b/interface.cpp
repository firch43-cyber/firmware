#include "core/powerSave.h"
#include "core/utils.h"
#include <Arduino.h>
#include <Wire.h>
#include <interface.h>

#if defined(HAS_CAPACITIVE_TOUCH) && defined(TOUCH_GT911_I2C)
#include "TouchDrvGT911.hpp"
TouchDrvGT911 touch;
struct TouchPointPro {
    int16_t x = 0;
    int16_t y = 0;
};
#endif

// =============================================================================
//  CH422G IO-expander helpers
//  The 4.3B routes LCD backlight (EXIO2/DISP), touch reset (EXIO1) and SD_CS
//  (EXIO4) through a CH422G on the main I2C bus. Nothing on the panel is
//  visible until EXIO2 is driven high, so this must run before anything else.
// =============================================================================
static uint8_t ch422g_state = 0x00;

static void ch422g_commit() {
    Wire.beginTransmission(CH422G_OUT_ADDR);
    Wire.write(ch422g_state);
    Wire.endTransmission();
}

static void ch422g_set(uint8_t bit, bool level) {
    if (level) ch422g_state |= (1 << bit);
    else ch422g_state &= ~(1 << bit);
    ch422g_commit();
}

static void ch422g_begin() {
    // Set IO0-7 as push-pull outputs
    Wire.beginTransmission(CH422G_MODE_ADDR);
    Wire.write(0x01);
    Wire.endTransmission();

    // Safe defaults: backlight off (for now), touch held in reset, SD deselected
    ch422g_state = 0;
    ch422g_state |= (1 << CH422G_BIT_SD_CS); // SD_CS high = deselected
    ch422g_commit();
}

/***************************************************************************************
** Function name: _setup_gpio()
** Location: main.cpp
** Description:   initial setup for the device
***************************************************************************************/
void _setup_gpio() {
    // Bring up the I2C bus the expander + touch live on
    Wire.begin(GROVE_SDA, GROVE_SCL);
    ch422g_begin();

    // ---- LCD panel reset (EXIO3). The RGB panel stays blank until released. ----
    ch422g_set(CH422G_BIT_LCD_RST, LOW);
    delay(20);
    ch422g_set(CH422G_BIT_LCD_RST, HIGH);
    delay(120); // panel needs time to come out of reset before RGB scanout

    // Turn the LCD backlight on (EXIO2/LCD_BL).
    ch422g_set(CH422G_BIT_DISP, HIGH);

#if defined(HAS_CAPACITIVE_TOUCH) && defined(TOUCH_GT911_I2C)
    // GT911 reset sequence. TP_RST is on the expander (EXIO1); INT is a real GPIO.
    // Holding INT low across the RST rising edge selects I2C address 0x5D.
    pinMode(TOUCH_INT, OUTPUT);
    digitalWrite(TOUCH_INT, LOW);
    ch422g_set(CH422G_BIT_TP_RST, LOW);
    delay(10);
    ch422g_set(CH422G_BIT_TP_RST, HIGH);
    delay(10);
    delay(50);
    pinMode(TOUCH_INT, INPUT);

    // RST is handled above via the expander, so pass -1 for it.
    touch.setPins(-1, TOUCH_INT);
    if (!touch.begin(Wire, GT911_SLAVE_ADDRESS_L, GROVE_SDA, GROVE_SCL)) {
        Serial.println("Failed to find GT911 touch - check wiring / expander reset!");
    } else {
        Serial.println("GT911 touch started");
    }
#endif
}

/***************************************************************************************
** Function name: _post_setup_gpio()
** Location: main.cpp
** Description:   second stage gpio setup to make a few functions work
***************************************************************************************/
void _post_setup_gpio() {
    // Ensure backlight is on after the display has been initialized
    ch422g_set(CH422G_BIT_DISP, HIGH);
}

/***************************************************************************************
** Function name: getBattery()
** location: display.cpp
** Description:   Delivers the battery value from 1-100
***************************************************************************************/
int getBattery() {
    // USB / externally powered; no fuel gauge wired by default.
    return 100;
}

/*********************************************************************
** Function: setBrightness
** location: settings.cpp
** Backlight is a simple on/off via the CH422G expander (no PWM line).
**********************************************************************/
void _setBrightness(uint8_t brightval) { ch422g_set(CH422G_BIT_DISP, brightval > 0 ? HIGH : LOW); }

/*********************************************************************
** Function: InputHandler
** Handles PrevPress, NextPress, SelPress, AnyKeyPress and EscPress
**********************************************************************/
void InputHandler(void) {
#if defined(HAS_CAPACITIVE_TOUCH) && defined(TOUCH_GT911_I2C)
    static long d_tmp = 0;
    if (millis() - d_tmp > 200 || LongPress) {
        static unsigned long tm = millis();
        TouchPointPro t;
        uint8_t touched = 0;
        static uint8_t rot = 5;

        if (rot != bruceConfigPins.rotation) {
            if (bruceConfigPins.rotation == 1) {
                touch.setMaxCoordinates(TFT_HEIGHT, TFT_WIDTH);
                touch.setSwapXY(true);
                touch.setMirrorXY(false, true);
            }
            if (bruceConfigPins.rotation == 3) {
                touch.setMaxCoordinates(TFT_HEIGHT, TFT_WIDTH);
                touch.setSwapXY(true);
                touch.setMirrorXY(true, false);
            }
            if (bruceConfigPins.rotation == 0) {
                touch.setMaxCoordinates(TFT_WIDTH, TFT_HEIGHT);
                touch.setSwapXY(false);
                touch.setMirrorXY(false, false);
            }
            if (bruceConfigPins.rotation == 2) {
                touch.setMaxCoordinates(TFT_WIDTH, TFT_HEIGHT);
                touch.setSwapXY(false);
                touch.setMirrorXY(true, true);
            }
            rot = bruceConfigPins.rotation;
        }

        static bool lastTouchState = false;
        static unsigned long lastTouchTime = 0;

        touched = touch.getPoint(&t.x, &t.y);
        bool currentTouchState = touched > 0;

        if (currentTouchState && !lastTouchState && (millis() - lastTouchTime) > 100) {
            lastTouchTime = millis();
        } else if (!currentTouchState || lastTouchState) {
            touched = 0;
        }
        lastTouchState = currentTouchState;

        if (((millis() - tm) > 190 || LongPress) && touched) {
            tm = millis();
            if (!wakeUpScreen()) AnyKeyPress = true;
            else goto END;

            touchPoint.x = t.x;
            touchPoint.y = t.y;
            touchPoint.pressed = true;
            touchHeatMap(touchPoint);
        END:
            d_tmp = millis();
        }
    }
#else
    checkPowerSaveTime();
    PrevPress = false;
    NextPress = false;
    SelPress = false;
    AnyKeyPress = false;
    EscPress = false;
#endif
}

/*********************************************************************
** Function: keyboard
** location: mykeyboard.cpp
** Starts keyboard to type data
**********************************************************************/
String keyboard(String mytext, int maxSize, String msg) { return mytext; }

/*********************************************************************
** Function: powerOff
** location: mykeyboard.cpp
** Turns off the device (or try to)
**********************************************************************/
void powerOff() {}

/*********************************************************************
** Function: checkReboot
** location: mykeyboard.cpp
** Btn logic to turn off the device (name is odd btw)
**********************************************************************/
void checkReboot() {}
