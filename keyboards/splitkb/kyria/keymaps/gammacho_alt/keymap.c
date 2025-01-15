/* Copyright 2019 Thomas Baart <thomas@splitkb.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include QMK_KEYBOARD_H
#ifdef OS_DETECTION_ENABLE
  #include "os_detection.h"
#endif

enum layers {
    _WINDOWS = 0,
    _MAC = 1,
    _NAV = 2,
    _MOUSE = 3,
    _M_FNC = 4,
    _W_FNC = 5,
    _M_NUM = 6,
    _W_NUM = 7,
};

// Tap Dance declarations
enum tap_dances {
    TD_BASE,
    TD_SP,
    TD_LBK,
    TD_RBK,
    TD_LPR,
    TD_RPR,
    TD_LYR,
};

// Aliases for readability
#define MAC      DF(_MAC)
#define WINDOWS  DF(_WINDOWS)
#define NAV      DF(_NAV)
#define MOUSE    DF(_MOUSE)
#define MAC_FNC  DF(_M_FNC)
#define WIN_FNC  DF(_W_FNC)
#define MAC_NUM  DF(_M_NUM)
#define WIN_NUM  DF(_W_NUM)

//#define CTL_ESC  MT(MOD_LCTL, KC_ESC)
//#define CTL_QUOT MT(MOD_RCTL, KC_QUOTE)
//#define CTL_MINS MT(MOD_RCTL, KC_MINUS)
//#define ALT_ENT  MT(MOD_LALT, KC_ENT)

//#define KCMJ KC_J
//#define KCWJ KC_J

#define MOU_A   LT(_MOUSE, KC_A)
#define NAV_QUO LT(_NAV, KC_QUOTE)
#define MFNC_Q  LT(_M_FNC, KC_Q)
#define WFNC_Q  LT(_W_FNC, KC_Q)
#define MFNC_P  LT(_M_FNC, KC_P)
#define WFNC_P  LT(_W_FNC, KC_P)
#define MNUM_SP LT(_M_NUM, KC_SPC)
#define WNUM_SP LT(_W_NUM, KC_SPC)
#define HO_LSFT LSFT_T(KC_Z)
#define HO_RSFT RSFT_T(KC_SLSH)
#define LSFT_F  LSFT_T(KC_F)
#define RSFT_J  RSFT_T(KC_J)
#define NV_LSFT LSFT_T(CC_HASH)

enum custom_keycodes {
    DRAG_SCROLL = SAFE_RANGE,
    CC_EXLM,
    CC_AT,
    CC_HASH,
    CC_DLR,
    CC_PERC,
    CC_CIRC,
    CC_AMPR,
    CC_ASTR,
    CC_LPRN,
    CC_RPRN,
    CC_PLUS,
    CC_PIPE,
    CC_UNDS,
    CC_LT,
    CC_GT,
    CC_LCBR,
    CC_RCBR,
    CC_TILD
};

int shifted_key_delay = 30;
bool drag_scrolling = false;
bool ring_scrolling = false;

// Modify these values to adjust the scrolling speed
#define SCROLL_DIVISOR_V 180.0

// Variables to store accumulated scroll values
float scroll_accumulated_v = 0;

// Function to handle mouse reports and perform drag scrolling
report_mouse_t pointing_device_task_user(report_mouse_t mouse_report) {
    // Check if drag scrolling is active
    if (drag_scrolling) {
        // Calculate and accumulate scroll values based on mouse movement and divisors
        scroll_accumulated_v -= (float)mouse_report.y / SCROLL_DIVISOR_V;

        // Assign integer parts of accumulated scroll values to the mouse report
        mouse_report.v = (int8_t)scroll_accumulated_v;
        //int8_t
        // Update accumulated scroll values by subtracting the integer parts
        scroll_accumulated_v -= (int8_t)scroll_accumulated_v;

        // Clear the X and Y values of the mouse report
        mouse_report.x = 0;
        mouse_report.y = 0;
    }
  return mouse_report;
}

// Function to handle key events and enable/disable drag scrolling
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    /*
    switch (keycode) {
        case DRAG_SCROLL:
            // Toggle set_scrolling when DRAG_SCROLL key is pressed or released
            set_scrolling = record->event.pressed;
            break;
        default:
            break;
    }
    */

    // When keys A-Z and 0-9 are held, send a tap on the press event.
    // if (IS_QK_BASIC(keycode)) {
    //     if (record->event.pressed) {
    //         tap_code16(keycode);  // Send tap.
    //     }
    //     return false;  // Skip default handling.
    // }

    switch (keycode) {
        case DRAG_SCROLL:
            if (record->event.pressed) {
                // this toggles the state each time you tap it
                drag_scrolling ^= 1;
            }
            break;
        case KC_BTN1:
            if (record->event.pressed) {
                // this toggles the state each time you tap it
                drag_scrolling = false;
            }
            break;
        case KC_BTN2:
            if (record->event.pressed) {
                // this toggles the state each time you tap it
                drag_scrolling = false;
            }
            break;
        case NAV_QUO:
            if (record->event.pressed) {
                //led_t led_usb_state = host_keyboard_led_state();
                rgblight_setrgb_at(20, 10, 10, 0);
                rgblight_setrgb_at(20, 10, 10, 1);
                //tap_code16(NAV_QUO);  // Send tap.
            }
            break;
        case CC_EXLM:
            if (record->event.pressed) {
                //KC_EXLM : !
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_1);
                unregister_code(KC_LSFT);
                //SEND_STRING("!");

            }
            break;
        case CC_AT:
            if (record->event.pressed) {
                //KC_AT : @
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_2);
                unregister_code(KC_LSFT);
                //SEND_STRING("@");
            }
            break;
        case CC_HASH:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_3);
                unregister_code(KC_LSFT);
                //SEND_STRING("#");
            }
            break;
        case CC_DLR:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_4);
                unregister_code(KC_LSFT);
                //SEND_STRING("$");
            }
            break;
        case CC_PERC:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_5);
                unregister_code(KC_LSFT);
                //SEND_STRING("%");
            }
            break;
        case CC_CIRC:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_6);
                unregister_code(KC_LSFT);
                //SEND_STRING("^");
            }
            break;
        case CC_AMPR:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_7);
                unregister_code(KC_LSFT);
                //SEND_STRING("&");
            }
            break;
        case CC_ASTR:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_8);
                unregister_code(KC_LSFT);
                //SEND_STRING("*");
            }
            break;
        case CC_LPRN:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_9);
                unregister_code(KC_LSFT);
                //SEND_STRING("(");
            }
            break;
        case CC_RPRN:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_0);
                unregister_code(KC_LSFT);
                //SEND_STRING(")");
            }
            break;
        case CC_PLUS:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_EQL);
                unregister_code(KC_LSFT);
                //SEND_STRING("+");
            }
            break;
        case CC_PIPE:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_BSLS);
                unregister_code(KC_LSFT);
                //SEND_STRING("|");
            }
            break;
        case CC_UNDS:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_MINS);
                unregister_code(KC_LSFT);
                //SEND_STRING("_");
            }
            break;
        case CC_LT:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_COMM);
                unregister_code(KC_LSFT);
                //SEND_STRING("<");
            }
            break;
        case CC_GT:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_DOT);
                unregister_code(KC_LSFT);
                //SEND_STRING(">");
            }
            break;
        case CC_LCBR:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_LBRC);
                unregister_code(KC_LSFT);
                //SEND_STRING("{");
            }
            break;
        case CC_RCBR:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_RBRC);
                unregister_code(KC_LSFT);
                //SEND_STRING("}");
            }
            break;
        case CC_TILD:
            if (record->event.pressed) {
                register_code(KC_LSFT);
                wait_ms(shifted_key_delay);
                tap_code(KC_GRV);
                unregister_code(KC_LSFT);
                //SEND_STRING("}");
            }
            break;
    }
    return true;
}

// Function to handle layer changes and disable drag scrolling when not in AUTO_MOUSE_DEFAULT_LAYER
layer_state_t layer_state_set_user(layer_state_t state) {
    // Disable set_scrolling if the current layer is not the AUTO_MOUSE_DEFAULT_LAYER
    if ((get_highest_layer(state) != _MAC) ||
        (get_highest_layer(state) != _WINDOWS) ||
        (get_highest_layer(state) != _MOUSE)) {
        drag_scrolling = false;
    }
    return state;
}

// Tap Dance definitions
tap_dance_action_t tap_dance_actions[] = {
    // Tap once for space, twice for Underscore
    [TD_BASE] = ACTION_TAP_DANCE_DOUBLE(WINDOWS, MAC),
    [TD_SP]   = ACTION_TAP_DANCE_DOUBLE(KC_SPC, LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(KC_MINS))))))))))),
    [TD_LBK]  = ACTION_TAP_DANCE_DOUBLE(KC_LBRC, LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(KC_LBRC))))))))))),
    [TD_RBK]  = ACTION_TAP_DANCE_DOUBLE(KC_RBRC, LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(KC_RBRC))))))))))),
    [TD_LPR]  = ACTION_TAP_DANCE_DOUBLE(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(KC_9)))))))))), LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(KC_COMM))))))))))),
    [TD_RPR]  = ACTION_TAP_DANCE_DOUBLE(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(KC_0)))))))))), LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(LSFT(KC_DOT))))))))))),
    [TD_LYR]  = ACTION_TAP_DANCE_DOUBLE(TO(WINDOWS), TO(MAC)),
   // [MJ]      = ACTION_TAP_DANCE_DOUBLE(KC_J, KC_J),
   // [WJ]      = ACTION_TAP_DANCE_DOUBLE(KC_J, KC_J),
};

//void SPC_UNDS_test(tap_dance_state_t *state, void *user_data) {
//	if (state->count == 1) {
//		tap_code(KC_LPRN);
//	} else if (state->count == 2) {
//		tap_code(KC_LBRC);
//	} else if (state->count == 3) {
//		tap_code16(S(KC_LBRC));
//	}
//}


enum combos {
    CO_MESC,
    CO_WESC,
    CO_CAPS,
    CO_SEMI,
    CO_MDEL,
    CO_WDEL,
    CO_LANG,
    CO_MIDCLK,
    CO_SCROLL,
    CO_TO_MAC,
    CO_TO_WIN,
    NOLOCK,
};

const uint16_t PROGMEM COM_MESC[] = {KC_TAB, MFNC_Q, COMBO_END};
const uint16_t PROGMEM COM_WESC[] = {KC_TAB, WFNC_Q, COMBO_END};
const uint16_t PROGMEM COM_CAPS[] = {KC_D, LSFT_F, COMBO_END};
const uint16_t PROGMEM COM_SEMI[] = {KC_COMM, KC_DOT, COMBO_END};
const uint16_t PROGMEM COM_MDEL[] = {MFNC_P, KC_BSPC, COMBO_END};
const uint16_t PROGMEM COM_WDEL[] = {WFNC_P, KC_BSPC, COMBO_END};
const uint16_t PROGMEM COM_LANG[] = {RSFT_J, KC_K, COMBO_END};
const uint16_t PROGMEM COM_MIDCLK[] = {KC_BTN1, KC_BTN2, COMBO_END};
const uint16_t PROGMEM COM_SCROLL[] = {KC_C, KC_V, COMBO_END};
const uint16_t PROGMEM COM_TO_MAC[] = {KC_U, KC_I, KC_O, WFNC_P, COMBO_END};
const uint16_t PROGMEM COM_TO_WIN[] = {KC_U, KC_I, KC_O, MFNC_P, COMBO_END};
const uint16_t PROGMEM COM_NOLOCK[] = {KC_M, KC_COMM, COMBO_END};

combo_t key_combos[] = {
  [CO_MESC] = COMBO(COM_MESC, KC_ESC),
  [CO_WESC] = COMBO(COM_WESC, KC_ESC),
  [CO_CAPS] = COMBO(COM_CAPS, KC_CAPS),
  [CO_SEMI] = COMBO(COM_SEMI, KC_SCLN),
  [CO_MDEL] = COMBO(COM_MDEL, KC_DEL),
  [CO_WDEL] = COMBO(COM_WDEL, KC_DEL),
  [CO_LANG] = COMBO_ACTION(COM_LANG),
  [CO_MIDCLK] = COMBO(COM_MIDCLK, KC_BTN3),
  [CO_SCROLL] = COMBO(COM_SCROLL, DRAG_SCROLL),
  [CO_TO_MAC] = COMBO(COM_TO_MAC, DF(MAC)),
  [CO_TO_WIN] = COMBO(COM_TO_WIN, DF(WINDOWS)),
  [NOLOCK] = COMBO_ACTION(COM_NOLOCK),
};

void process_combo_event(uint16_t combo_index, bool pressed) {
    led_t led_usb_state = host_keyboard_led_state();
    uint8_t state = get_highest_layer(layer_state | default_layer_state);
    switch(combo_index) {
        case NOLOCK:
            if (pressed) {
                if (led_usb_state.caps_lock ) {
                    SEND_STRING(" with(nolock)");
                }
                else {
                    SEND_STRING(" WITH(NOLOCK)");
                }
            }
            break;

        case CO_LANG:
            if (pressed){
                if (state == _MAC) {
                    tap_code16_delay(LCTL(KC_SPC), 5);
                }
                else if (state == _WINDOWS) {
                    tap_code16(KC_RALT);
                }
            }
            break;
    }
};


void keyboard_pre_init_user(void) {
  // Set our LED pin as output
  setPinOutput(24);
  // Turn the LED off
  // (Due to technical reasons, high is off and low is on)
  writePinHigh(24);
}

void keyboard_post_init_user(void) {
    // Initialize RGB to static black
    rgblight_enable_noeeprom();
    rgblight_sethsv_noeeprom(HSV_BLACK);
    rgblight_mode_noeeprom(RGBLIGHT_MODE_STATIC_LIGHT);

    // Set default layer based on the detected OS after a 500 ms delay.
    uint32_t get_host_os(uint32_t trigger_time, void* cb_arg) {
        switch (detected_host_os()) {
        case OS_UNSURE:  // Don't change default layer if unsure.
            break;
        case OS_MACOS:
            set_single_persistent_default_layer(_MAC);
            break;
        case OS_IOS:
            set_single_persistent_default_layer(_MAC);
            break;
        case OS_LINUX:
            set_single_persistent_default_layer(_WINDOWS);
            break;
        case OS_WINDOWS:
            set_single_persistent_default_layer(_WINDOWS);
            break;
        default:
            break;
        }
        return 0;
  }
  defer_exec(1000, get_host_os, NULL);
}

void housekeeping_task_user(void) {
    led_t led_usb_state = host_keyboard_led_state();
    //rgblight_setrgb_at(RGB_RED, 0);
    if (led_usb_state.caps_lock) {
        rgblight_setrgb_at(0, 20, 0, 0);
        rgblight_setrgb_at(0, 20, 0, 1);
    } else {
        rgblight_setrgb_at(0, 0, 0, 0);
        rgblight_setrgb_at(0, 0, 0, 1);
    }

    if (drag_scrolling) {
        rgblight_setrgb_at(20, 0, 0, 0);
        rgblight_setrgb_at(20, 0, 0, 1);
    } else {
        rgblight_setrgb_at(0, 0, 0, 0);
        rgblight_setrgb_at(0, 0, 0, 1);
    }
}

// Note: LAlt/Enter (ALT_ENT) is not the same thing as the keyboard shortcut Alt+Enter.
// The notation `mod/tap` denotes a key that activates the modifier `mod` when held down, and
// produces the key `tap` when tapped (i.e. pressed and released).

// clang-format off
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    [_WINDOWS] = LAYOUT(
     KC_TAB,  WFNC_Q,  KC_W, KC_E,    KC_R,    KC_T,                                            KC_Y,      KC_U,    KC_I,    KC_O,   WFNC_P,  KC_BSPC,
     KC_LCTL, MOU_A,   KC_S, KC_D,    LSFT_F,  KC_G,                                            KC_H,      RSFT_J,  KC_K,    KC_L,   NAV_QUO, KC_ENT,
     KC_LSFT, HO_LSFT, KC_X, KC_C,    KC_V,    KC_B,    XXXXXXX, KC_WH_U,     KC_BTN1, KC_BTN2, KC_N,      KC_M,    KC_COMM, KC_DOT, HO_RSFT, KC_RSFT,
                             XXXXXXX, KC_LALT, KC_LCTL, WNUM_SP, KC_WH_D,     KC_BTN2, WNUM_SP, TD(TD_SP), KC_LGUI, XXXXXXX
    ),

    [_MAC] = LAYOUT(
     KC_TAB,  MFNC_Q,  KC_W, KC_E,    KC_R,    KC_T,                                            KC_Y,      KC_U,    KC_I,    KC_O,   MFNC_P,  KC_BSPC,
     KC_LCTL, MOU_A,   KC_S, KC_D,    LSFT_F,  KC_G,                                            KC_H,      RSFT_J,  KC_K,    KC_L,   NAV_QUO, KC_ENT,
     KC_LSFT, HO_LSFT, KC_X, KC_C,    KC_V,    KC_B,    XXXXXXX, KC_WH_U,     KC_BTN1, KC_BTN2, KC_N,      KC_M,    KC_COMM, KC_DOT, HO_RSFT, KC_RSFT,
                             XXXXXXX, KC_LALT, KC_LGUI, MNUM_SP, KC_WH_D,     KC_BTN2, WNUM_SP, TD(TD_SP), KC_LCTL, XXXXXXX
    ),

    [_NAV] = LAYOUT(
     KC_TAB,  CC_EXLM, CC_AT,   CC_HASH, KC_EQL,  TD(TD_LBK),                                         TD(TD_RBK), XXXXXXX, KC_UP,   XXXXXXX, XXXXXXX, KC_BSPC,
     _______, CC_PERC, CC_AMPR, CC_ASTR, KC_SLSH, TD(TD_LPR),                                         TD(TD_RPR), KC_LEFT, KC_DOWN, KC_RGHT, XXXXXXX, _______,
     KC_LSFT, NV_LSFT, CC_DLR,  CC_PLUS, KC_MINS, CC_PIPE,    _______, _______,     _______, _______, KC_BSLS,    XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_RSFT,
                                _______, _______, _______,    _______, _______,     _______, _______, _______,    _______, XXXXXXX
    ),

    [_MOUSE] = LAYOUT(
     KC_TAB,  XXXXXXX, XXXXXXX, KC_WH_U, XXXXXXX, KC_PGUP,                                            KC_HOME, XXXXXXX, KC_MS_U, XXXXXXX, XXXXXXX, KC_BSPC,
     _______, XXXXXXX, KC_WH_L, KC_WH_D, KC_WH_R, KC_PGDN,                                            KC_END,  KC_MS_L, KC_MS_D, KC_MS_R, XXXXXXX, _______,
     KC_LSFT, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX,    _______, _______,     _______, _______, KC_BSLS, XXXXXXX, XXXXXXX, XXXXXXX, XXXXXXX, KC_RSFT,
                                _______, _______, _______,    _______, _______,     _______, _______, _______,    _______, XXXXXXX
    ),

    [_M_FNC] = LAYOUT(
     KC_TAB,  XXXXXXX, KC_VOLU, XXXXXXX, LGUI(KC_LBRC), LGUI(KC_RBRC),                                              LCTL(KC_UP),  KC_F9,   KC_F10, KC_F11, KC_F12, KC_BSPC,
     _______, XXXXXXX, KC_VOLD, XXXXXXX, XXXXXXX,       LGUI(KC_GRV),                                               LGUI(KC_GRV), KC_F5,   KC_F6,  KC_F7,  KC_F8,  _______,
     KC_LSFT, XXXXXXX, KC_MUTE, XXXXXXX, XXXXXXX,       LGUI(LALT(KC_ESC)), _______, _______,     _______, _______, XXXXXXX,      KC_F1,   KC_F2,  KC_F3,  KC_F4,  KC_RSFT,
                                _______, _______,       _______,            _______, _______,     _______, _______, _______,      _______, XXXXXXX
    ),

    [_W_FNC] = LAYOUT(
     KC_TAB,  XXXXXXX, KC_VOLU, XXXXXXX, LALT(KC_LEFT), LALT(KC_RGHT),                                              LGUI(KC_TAB), KC_F9,   KC_F10, KC_F11, KC_F12, KC_BSPC,
     _______, XXXXXXX, KC_VOLD, XXXXXXX, XXXXXXX,       LALT(KC_TAB),                                               LALT(KC_TAB), KC_F5,   KC_F6,  KC_F7,  KC_F8,  _______,
     KC_LSFT, XXXXXXX, KC_MUTE, XXXXXXX, XXXXXXX,       LCTL(LSFT(KC_ESC)), _______, _______,     _______, _______, XXXXXXX,      KC_F1,   KC_F2,  KC_F3,  KC_F4,  KC_RSFT,
                                _______, _______,       _______,            _______, _______,     _______, _______, _______,      _______, XXXXXXX
    ),

    [_M_NUM] = LAYOUT(
     KC_TAB,  CC_EXLM, CC_AT,   CC_HASH, CC_DLR,  CC_PERC,                                         CC_CIRC, CC_AMPR, CC_ASTR, CC_LPRN, CC_RPRN, KC_BSPC,
     _______, KC_1,    KC_2,    KC_3 ,   KC_4,    KC_5,                                            KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    _______ ,
     KC_LSFT, KC_GRV,  CC_TILD, CC_PLUS, KC_MINS, CC_ASTR, KC_SLSH, _______,     _______, _______, KC_BSLS, KC_EQL,  KC_COMM, KC_DOT,  KC_SLSH, KC_RSFT,
                                _______, _______, _______, _______, _______,     _______, _______, _______, _______, XXXXXXX
    ),

    [_W_NUM] = LAYOUT(
     KC_TAB,  CC_EXLM, CC_AT,   CC_HASH, CC_DLR,  CC_PERC,                                         CC_CIRC, CC_AMPR, CC_ASTR, CC_LPRN, CC_RPRN, KC_BSPC,
     _______, KC_1,    KC_2,    KC_3 ,   KC_4,    KC_5,                                            KC_6,    KC_7,    KC_8,    KC_9,    KC_0,    _______ ,
     KC_LSFT, KC_GRV,  CC_TILD, CC_PLUS, KC_MINS, CC_ASTR, KC_SLSH, _______,     _______, _______, KC_BSLS, KC_EQL,  KC_COMM, KC_DOT,  KC_SLSH, KC_RSFT,
                                _______, _______, _______, _______, _______,     _______, _______, _______, _______, XXXXXXX
    ),

// /*
//  * Layer template
//  *
//  * ,-------------------------------------------.                              ,-------------------------------------------.
//  * |        |      |      |      |      |      |                              |      |      |      |      |      |        |
//  * |--------+------+------+------+------+------|                              |------+------+------+------+------+--------|
//  * |        |      |      |      |      |      |                              |      |      |      |      |      |        |
//  * |--------+------+------+------+------+------+-------------.  ,-------------+------+------+------+------+------+--------|
//  * |        |      |      |      |      |      |      |      |  |      |      |      |      |      |      |      |        |
//  * `----------------------+------+------+------+------+------|  |------+------+------+------+------+----------------------'
//  *                        |      |      |      |      |      |  |      |      |      |      |      |
//  *                        |      |      |      |      |      |  |      |      |      |      |      |
//  *                        `----------------------------------'  `----------------------------------'
//  */
//     [_LAYERINDEX] = LAYOUT(
//       _______, _______, _______, _______, _______, _______,                                     _______, _______, _______, _______, _______, _______,
//       _______, _______, _______, _______, _______, _______,                                     _______, _______, _______, _______, _______, _______,
//       _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______,
//                                  _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
//     ),
};

/*
#ifdef POINTING_DEVICE_ENABLE
void pointing_device_init_user(void) {
    pointing_device_set_cpi(512);
}
#endif
*/

/* The default OLED and rotary encoder code can be found at the bottom of qmk_firmware/keyboards/splitkb/kyria/rev1/rev1.c
 * These default settings can be overriden by your own settings in your keymap.c
 * For your convenience, here's a copy of those settings so that you can uncomment them if you wish to apply your own modifications.
 * DO NOT edit the rev1.c file; instead override the weakly defined default functions by your own.
 */

/* DELETE THIS LINE TO UNCOMMENT (1/2)*/
//#ifdef OLED_ENABLE
oled_rotation_t oled_init_user(oled_rotation_t rotation) { return OLED_ROTATION_180; }

bool oled_task_user(void) {
    if (is_keyboard_master()) {
        // QMK Logo and version information
        // clang-format off
        static const char PROGMEM qmk_logo[] = {
            0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x94,
            0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,0xb0,0xb1,0xb2,0xb3,0xb4,
            0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xce,0xcf,0xd0,0xd1,0xd2,0xd3,0xd4,0};
        // clang-format on

        oled_write_P(qmk_logo, false);
        oled_write_P(PSTR("\n "), false);

        // Host Keyboard Layer Status
        #ifdef OS_DETECTION_ENABLE
            switch (detected_host_os()) {
                case OS_UNSURE:
                    oled_write_P(PSTR("UNK "), false);
                    break;
                case OS_LINUX:
                    oled_write_P(PSTR("LINUX "), false);
                    break;
                case OS_WINDOWS:
                    oled_write_P(PSTR("WINDOWS "), false);
                    break;
                case OS_MACOS:
                    oled_write_P(PSTR("MAC/iOS "), false);
                    break;
                case OS_IOS:
                    oled_write_P(PSTR("MAC/iOS "), false);
                    break;
                default:
                    oled_write_P(PSTR("UNK "), false);
            }
            oled_write_P(PSTR("OS Device\n\n"), false);
        #endif

        oled_write_P(PSTR(" Layer: "), false);
        switch (get_highest_layer(layer_state|default_layer_state)) {
            case _MAC:
                oled_write_P(PSTR("MAC\n"), false);
                break;
            case _WINDOWS:
                oled_write_P(PSTR("WINDOWS\n"), false);
                break;
            case _NAV:
                oled_write_P(PSTR("NAV\n"), false);
                break;
            case _MOUSE:
                oled_write_P(PSTR("MOUSE\n"), false);
                break;
            case _M_NUM:
                oled_write_P(PSTR("MAC_NUM\n"), false);
                break;
            case _W_NUM:
                oled_write_P(PSTR("WIN_NUM\n"), false);
                break;
            case _M_FNC:
                oled_write_P(PSTR("MAC_FNC\n"), false);
                break;
            case _W_FNC:
                oled_write_P(PSTR("WIN_FNC\n"), false);
                break;
            default:
                oled_write_P(PSTR("Undefined\n"), false);
        }

        // Write host Keyboard LED Status to OLEDs
        led_t led_usb_state = host_keyboard_led_state();
        oled_write_P(drag_scrolling ? PSTR(" SCROLL") : PSTR("       "), false);
        //oled_write_P(led_usb_state.num_lock    ? PSTR(" NUMLCK ") : PSTR("        "), false);
        oled_write_P(led_usb_state.caps_lock   ? PSTR(" CAPLCK") : PSTR("       "), false);
        //oled_write_P(led_usb_state.scroll_lock ? PSTR(" SCRLCK ") : PSTR("        "), false);
    }/* else {
        // clang-format off
        static const char PROGMEM kyria_logo[] = {
            0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,128,128,192,224,240,112,120, 56, 60, 28, 30, 14, 14, 14,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7,  7, 14, 14, 14, 30, 28, 60, 56,120,112,240,224,192,128,128,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  0,  0,  0,192,224,240,124, 62, 31, 15,  7,  3,  1,128,192,224,240,120, 56, 60, 28, 30, 14, 14,  7,  7,135,231,127, 31,255,255, 31,127,231,135,  7,  7, 14, 14, 30, 28, 60, 56,120,240,224,192,128,  1,  3,  7, 15, 31, 62,124,240,224,192,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
            0,  0,  0,  0,240,252,255, 31,  7,  1,  0,  0,192,240,252,254,255,247,243,177,176, 48, 48, 48, 48, 48, 48, 48,120,254,135,  1,  0,  0,255,255,  0,  0,  1,135,254,120, 48, 48, 48, 48, 48, 48, 48,176,177,243,247,255,254,252,240,192,  0,  0,  1,  7, 31,255,252,240,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
            0,  0,  0,255,255,255,  0,  0,  0,  0,  0,254,255,255,  1,  1,  7, 30,120,225,129,131,131,134,134,140,140,152,152,177,183,254,248,224,255,255,224,248,254,183,177,152,152,140,140,134,134,131,131,129,225,120, 30,  7,  1,  1,255,255,254,  0,  0,  0,  0,  0,255,255,255,  0,  0,  0,  0,255,255,  0,  0,192,192, 48, 48,  0,  0,240,240,  0,  0,  0,  0,  0,  0,240,240,  0,  0,240,240,192,192, 48, 48, 48, 48,192,192,  0,  0, 48, 48,243,243,  0,  0,  0,  0,  0,  0, 48, 48, 48, 48, 48, 48,192,192,  0,  0,  0,  0,  0,
            0,  0,  0,255,255,255,  0,  0,  0,  0,  0,127,255,255,128,128,224,120, 30,135,129,193,193, 97, 97, 49, 49, 25, 25,141,237,127, 31,  7,255,255,  7, 31,127,237,141, 25, 25, 49, 49, 97, 97,193,193,129,135, 30,120,224,128,128,255,255,127,  0,  0,  0,  0,  0,255,255,255,  0,  0,  0,  0, 63, 63,  3,  3, 12, 12, 48, 48,  0,  0,  0,  0, 51, 51, 51, 51, 51, 51, 15, 15,  0,  0, 63, 63,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 48, 48, 63, 63, 48, 48,  0,  0, 12, 12, 51, 51, 51, 51, 51, 51, 63, 63,  0,  0,  0,  0,  0,
            0,  0,  0,  0, 15, 63,255,248,224,128,  0,  0,  3, 15, 63,127,255,239,207,141, 13, 12, 12, 12, 12, 12, 12, 12, 30,127,225,128,  0,  0,255,255,  0,  0,128,225,127, 30, 12, 12, 12, 12, 12, 12, 12, 13,141,207,239,255,127, 63, 15,  3,  0,  0,128,224,248,255, 63, 15,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  0,  0,  0,  3,  7, 15, 62,124,248,240,224,192,128,  1,  3,  7, 15, 30, 28, 60, 56,120,112,112,224,224,225,231,254,248,255,255,248,254,231,225,224,224,112,112,120, 56, 60, 28, 30, 15,  7,  3,  1,128,192,224,240,248,124, 62, 15,  7,  3,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
            0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  3,  7, 15, 14, 30, 28, 60, 56,120,112,112,112,224,224,224,224,224,224,224,224,224,224,224,224,224,224,224,224,112,112,112,120, 56, 60, 28, 30, 14, 15,  7,  3,  1,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
        };
        // clang-format on
        oled_write_raw_P(kyria_logo, sizeof(kyria_logo));
    }*/
    return false;
}
//#endif
/*
#ifdef ENCODER_ENABLE
bool encoder_update_user(uint8_t index, bool clockwise) {

    if (index == 0) {
        // Volume control
        if (clockwise) {
            tap_code(KC_VOLU);
        } else {
            tap_code(KC_VOLD);
        }
    } else if (index == 1) {
        // Page up/Page down
        if (clockwise) {
            tap_code(KC_PGDN);
        } else {
            tap_code(KC_PGUP);
        }
    }
    return false;
}
#endif
DELETE THIS LINE TO UNCOMMENT (2/2) */
