/**
 * @file RGBLed.h
 * @author Robson Costa (robson.costa@ifsc.edu.br)
 * @brief RGB LED library.
 * @version 0.1.0
 * @since 2019-01-14 
 * @date 2020-01-14
 * 
 * @copyright Copyright (c) 2019 - 2021 Robson Costa\n
 * Licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Unported License (the <em>"License"</em>). 
 * You may not use this file except in compliance with the License. You may obtain a copy of the License at 
 * \url{https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode}. Unless required by applicable law or agreed to in writing,  
 * software distributed under the License is distributed on an <em>"as is" basis, without warranties or 
 * conditions of any kind</em>, either express or implied. See the License for the specific language governing 
 * permissions and limitations under the License.
 */
#ifndef __RGB_LED_H__
#define __RGB_LED_H__

#include <Arduino.h>

/**
 * @struct Color 
 * @brief Color struct for RGB LED.
 */
struct Color {
    const uint8_t red;      /**< Red pin value (0 - 255). */
    const uint8_t green;    /**< Green pin value (0 - 255). */
    const uint8_t blue;     /**< Blue pin value (0 - 255). */

    Color(uint8_t red, uint8_t green, uint8_t blue);
};

/**
 * @enum LEDType
 * @brief Define the LED type.
 * @var CATHODE
 * for cathode (-) LED type.
 * @var ANODE
 * for anode (+) LED type.
 */
enum LEDType {
    CATHODE,
    ANODE
};

class RGBLed {
    private:
        const uint8_t RED;
        const uint8_t GREEN;
        const uint8_t BLUE;
        const uint8_t OFFSET;

    public:
        RGBLed(LEDType type, uint8_t redPin, uint8_t greenPin, uint8_t bluePin);

        void setup() const;
        void on(Color color) const;
        void off();
};

Color::Color(uint8_t red, uint8_t green, uint8_t blue) :
    red(red), green(green), blue(blue) { }

/**
 * @fn RGBLed::RGBLed(LEDType type, uint8_t redPin, uint8_t greenPin, uint8_t bluePin)
 * @brief Constructor of RGBLed class.
 * @param[in] type - LED type (see \ref LEDType).
 * @param[in] redPin - GPIO of red pin.
 * @param[in] greenPin - GPIO of green pin.
 * @param[in] bluePin - GPIO of blue pin.
 */
RGBLed::RGBLed(LEDType type, uint8_t redPin, uint8_t greenPin, uint8_t bluePin) :
    RED(redPin), GREEN(greenPin), BLUE(bluePin), OFFSET(type == ANODE ? 255 : 0) { }

/**
 * @fn RGBLed::setup()
 * @brief Setup RGB LED pins.
 */
void RGBLed::setup() const {
    pinMode(RED, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);
}

/**
 * @fn RGBLed::on(Color color) const
 * @brief Power on RGB LED with a specific color.
 * @param[in] color - RGB configuration of a color.
 */
void RGBLed::on(Color color) const {
    analogWrite(RED, abs(color.red - OFFSET));
    analogWrite(GREEN, abs(color.green - OFFSET));
    analogWrite(BLUE, abs(color.blue - OFFSET));
}

/**
 * @fn RGBLed::off()
 * @brief Power off RGB LED.
 */
void RGBLed::off() {
    on(Color(0,0,0));
}

#endif // __RGB_LED_H__