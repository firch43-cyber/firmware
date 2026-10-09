#include "core/bus_HAL.h"
#include "core/powerSave.h"
#include "core/utils.h"
#include "hal/device.h"
#include "hal/inputs/touch.h"
#include <Arduino.h>
#include <Wire.h>
#include <interface.h>

#define BOARD_TOUCH_INT 4
#define GT911_SLAVE_ADDRESS_L 0x5D

// SD SPI pins. CS is routed through the CH422G expander (EXIO4, active low) —
// there is no direct GPIO CS line. GPIO6 is an unused, unconnected pin used
// purely to satisfy the Arduino SPI/SD library, which requires a real GPIO
// number for CS bookkeeping. The real hardware CS is held LOW permanently by
// the expander in _setup_gpio(); the SD SPI bus is dedicated (no other
// device shares it), so keeping CS asserted throughout is safe.
#define SDCARD_SCK 12
#define SDCARD_MISO 13
#define SDCARD_MOSI 11
#define SDCARD_CS 6 // dummy GPIO, not connected to the SD card

// =============================================================================
//  Waveshare ESP32-S3-Touch-LCD-4.3B interface
//  - Display: 800x480 16-bit RGB parallel panel via Arduino_GFX (USE_ARDUINO_GFX)
//  - Touch:   GT911 capacitive on I2C (SDA=8, SCL=9, INT=4); RST is on the
//             CH422G expander (EXIO1), driven through DeviceTouch.reset_cb.
//  - The panel's backlight (EXIO2), LCD reset (EXIO3) and SD CS (EXIO4) also
//             sit behind the CH422G. Nothing is visible until EXIO2/EXIO3 are
//             driven, so the expander must come up before anything else.
// =============================================================================

// ---------------------------------------------------------------------------
// CH422G IO-expander helpers
// ---------------------------------------------------------------------------
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

    // Safe defaults: everything off, SD deselected (CS high = deselected)
    ch422g_state = 0;
    ch422g_state |= (1 << CH422G_BIT_SD_CS);
    ch422g_commit();
}

// GT911 RST lives on the expander (EXIO1), not on a raw GPIO. The touch HAL
// pulses reset through this callback (see DeviceTouch.reset_cb).
static void ch422g_touch_reset(bool level) { ch422g_set(CH422G_BIT_TP_RST, level); }

static DeviceTouch touchCfg() {
    DeviceTouch cfg;
    cfg.pin_sda = 8;
    cfg.pin_scl = 9;
    cfg.pin_irq = BOARD_TOUCH_INT;
    cfg.pin_rst = -1; // RST via the expander, see reset_cb below
    cfg.reset_cb = ch422g_touch_reset;
    cfg.i2c_bus = &Wire;
    // rotation:        0      1      2      3
    const bool swapXY[4] = {false, true, false, true};
    const bool mirrorX[4] = {false, false, true, true};
    const bool mirrorY[4] = {false, true, true, false};
    for (int i = 0; i < 4; i++) {
        cfg.SwapXY[i] = swapXY[i];
        cfg.MirrorX[i] = mirrorX[i];
        cfg.MirrorY[i] = mirrorY[i];
    }
    return cfg;
}

/***************************************************************************************
** Function name: _setup_gpio()
***************************************************************************************/
void _setup_gpio() {
    // Bring up the I2C bus the expander + touch live on, then the expander
    // itself. Everything else on this board hangs off the expander.
    Wire.begin(GROVE_SDA, GROVE_SCL);
    ch422g_begin();

    // ---- LCD panel reset (EXIO3). The RGB panel stays blank until released.
    ch422g_set(CH422G_BIT_LCD_RST, LOW);
    delay(20);
    ch422g_set(CH422G_BIT_LCD_RST, HIGH);
    delay(120); // panel needs time to come out of reset before RGB scanout

    // Turn the LCD backlight on (EXIO2/LCD_BL).
    ch422g_set(CH422G_BIT_DISP, HIGH);

    // Assert SD card CS via the expander (EXIO4, active low). Held permanently
    // LOW; see SDCARD_CS note above.
    ch422g_set(CH422G_BIT_SD_CS, LOW);

    // ---- Bruce pin map ----
    bruceConfigPins.sys_i2c = {(gpio_num_t)GROVE_SDA, (gpio_num_t)GROVE_SCL};
    bruceConfigPins.rotation = ROTATION;
    bruceConfigPins.uart_bus = {(gpio_num_t)44, (gpio_num_t)43};   // rx, tx
    bruceConfigPins.gps_bus = {(gpio_num_t)44, (gpio_num_t)43};     // rx, tx
    bruceConfigPins.badusb_bus = {(gpio_num_t)44, (gpio_num_t)43}; // rx, tx
    bruceConfigPins.SDCARD_bus = {
        (gpio_num_t)SDCARD_SCK, (gpio_num_t)SDCARD_MISO, (gpio_num_t)SDCARD_MOSI, (gpio_num_t)SDCARD_CS
    }; // sck, miso, mosi, cs
    // No radios populated on a bare 4.3B; leave IR/RF unassigned so the RF
    // tools don't touch the touch/expander I2C pins (the defaults are
    // GROVE_SDA/GROVE_SCL, which here ARE that bus).
    bruceConfigPins.irTx = -1;
    bruceConfigPins.irRx = -1;
    bruceConfigPins.rfTx = -1;
    bruceConfigPins.rfRx = -1;

    bruceConfig.colorInverted = 0;

    // Route the shared sys_i2c HAL at the already-started bus.
    setSysI2CBus(&Wire);

    // GT911. RST is pulsed through ch422g_touch_reset() by the HAL; holding
    // INT low across the RST rising edge selects I2C address 0x5D.
    if (!hal_touch_init(touchCfg(), GT911_SLAVE_ADDRESS_L)) {
        Serial.println("Failed to find GT911 touch - check wiring / expander reset!");
    } else {
        Serial.println("GT911 touch started");
    }
}

/***************************************************************************************
** Function name: _post_setup_gpio()
***************************************************************************************/
void _post_setup_gpio() {
    // Ensure backlight is on after the display has been initialized
    ch422g_set(CH422G_BIT_DISP, HIGH);
}

/***************************************************************************************
** Function name: getBattery()
** Description:   Delivers the battery value from 1-100
***************************************************************************************/
int getBattery() {
    // USB / externally powered; no fuel gauge wired by default.
    return 100;
}

/*********************************************************************
** Function: _setBrightness
** Backlight is a simple on/off via the CH422G expander (no PWM line).
** Waking it back from off proved unreliable, so keep it on while running
** rather than risk an unrecoverable dark screen.
**********************************************************************/
void _setBrightness(uint8_t brightval) {
    (void)brightval;
    ch422g_set(CH422G_BIT_DISP, HIGH);
}

/*********************************************************************
** Function: InputHandler (GT911 capacitive)
** Tap handling is done centrally by hal_touch_apply(); swipe gestures for
** next/prev/back are handled by the big-screen UI layer.
**********************************************************************/
void InputHandler(void) {
    static long d_tmp = 0;
    if (millis() - d_tmp > 200 || LongPress) {
        BruceTouchPoint t;
        if (hal_touch_read(touchCfg(), t)) {
            d_tmp = millis();
            hal_touch_apply(t);
        }
    }
}

/*********************************************************************
** Function: powerOff
**********************************************************************/
void powerOff() {}

/*********************************************************************
** Function: checkReboot
**********************************************************************/
void checkReboot() {}
