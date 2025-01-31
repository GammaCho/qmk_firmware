/* Copyright 2022 Thomas Baart <thomas@splitkb.com>
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

#pragma once
/*
#ifdef RGBLIGHT_ENABLE
#    define RGBLIGHT_ANIMATIONS
#    define RGBLIGHT_HUE_STEP  8
#    define RGBLIGHT_SAT_STEP  8
#    define RGBLIGHT_VAL_STEP  8
#    define RGBLIGHT_LIMIT_VAL 150
#endif
*/
// Lets you roll mod-tap keys : NO LONGER USED
//#define IGNORE_MOD_TAP_INTERRUPT

#define SPLIT_POINTING_ENABLE
#define POINTING_DEVICE_ROTATION_270
#define POINTING_DEVICE_GESTURES_CURSOR_GLIDE_ENABLE
#define POINTING_DEVICE_GESTURES_SCROLL_ENABLE
/*
 * For some reason yet to be debugged, POINTING_DEVICE_RIGHT gets sporadic false movements when the two halves are connected.
 * Right half works fine on its own with POINTING_DEVICE_RIGHT.
 * Current workaround is to turn on POINTING_DEVICE_COMBINED and build a separate firmware with POINTING_DEVICE_DRIVER = custom for the left half.
 * In this configuration, the sporadic false movements show up when using the left half on its own, but works fine when the two are connected.
 */
//#define POINTING_DEVICE_RIGHT
//qmk compile -e CONVERT_TO=liatris -kb splitkb/kyria/rev3
//#define POINTING_DEVICE_COMBINED
#define CIRQUE_PINNACLE_ADDR 0x2A
#define CIRQUE_PINNACLE_CURVED_OVERLAY
//#define CIRQUE_PINNACLE_DISABLE_SMOOTHING
#define CIRQUE_PINNACLE_DISABLE_TAP
#define CIRQUE_PINNACLE_POSITION_MODE  CIRQUE_PINNACLE_ABSOLUTE_MODE
#define CIRQUE_PINNACLE_ATTENUATION EXTREG__TRACK_ADCCONFIG__ADC_ATTENUATE_2X
//#define CIRQUE_PINNACLE_TAP_ENABLE
//#define CIRQUE_PINNACLE_ENABLE_CURSOR_GLIDE
//#define CIRQUE_PINNACLE_ENABLE_CIRCULAR_SCROLL
// Pointing device is on the right split.
#define POINTING_DEVICE_RIGHT
// Limits the frequency that the sensor is polled for motion.
#define POINTING_DEVICE_TASK_THROTTLE_MS 10
// Adjust trackpad rotation.
//#define POINTING_DEVICE_ROTATION_90
//#define TAPPING_FORCE_HOLD
#define TAPPING_TERM 200
#define PERMISSIVE_HOLD
#define TAPPING_TERM_PER_KEY
//#define TAP_CODE_DELAY


// Split OnBoard Controller LED Enable
//#define MOD_TAP_INTERUPT
//#define IGNORE_MOD_TAP_INTERRUPT
/*
#define TAPPING_FORCE_HOLD
#define SPLIT_TRANSPORT_MIRROR
#define SPLIT_LAYER_STATE_ENABLE
*/
//#define LED_CAPS_LOCK_PIN 24
//#define LED_PIN_ON_STATE 0

#define SPLIT_LED_STATE_ENABLE
#undef WS2812_DI_PIN
#define WS2812_DI_PIN 25
//#undef WS2812_LED_COUNT
//#define WS2812_LED_COUNT 2
//#undef RGBLIGHT_LED_COUNT
//#define RGBLIGHT_LED_COUNT 62
//#undef RGBLED_NUM
//#define RGBLED_NUM 2
#undef RGBLED_SPLIT
#define RGBLED_SPLIT {0, 1}
#define SPLIT_TRANSPORT_MIRROR
#define SPLIT_LAYER_STATE_ENABLE

#define COMBO_ONLY_FROM_LAYER 0
