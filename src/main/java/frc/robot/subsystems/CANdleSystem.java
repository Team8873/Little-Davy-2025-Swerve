/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The CANdle MultiAnimation example demonstrates using multiple animations with CANdle.
 * This example has the robot using a Command Based template to control the CANdle.
 * 
 * This example uses:
 * - A CANdle wired on the CAN Bus, with a 5m led strip attached for the extra animatinos.
 * 
 * Controls (with Xbox controller):
 * Right Bumper: Increment animation
 * Left Bumper: Decrement animation
 * Start Button: Switch to setting the first 8 LEDs a unique combination of colors
 * POV Right: Configure maximum brightness for the CANdle
 * POV Down: Configure medium brightness for the CANdle
 * POV Left: Configure brightness to 0 for the CANdle
 * POV Up: Change the direction of Rainbow and Fire, must re-select the animation to take affect
 * A: Print the VBat voltage in Volts
 * B: Print the 5V voltage in Volts
 * X: Print the current in amps
 * Y: Print the temperature in degrees C
 * 
 * Supported Version:
 * 	- CANdle: 22.1.1.0
 */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class CANdleSystem extends SubsystemBase {
    private final int LEDS_PER_ANIMATION = 158;
    private final CANdle m_candle = new CANdle(Constants.CANdleConstants.CANdleID, "rio");
    private CommandXboxController joystick;
    private boolean m_clearAllAnims = false;
    private boolean m_last5V = false;
    private boolean m_animDirection = false;
    private boolean m_setAnim = false;
    private double speed;

    public enum Color {
        Pink,
        Green,
        None
    }

    private Animation m_toAnimate = null;
    private Animation m_toAnimate2 = null;
    private Animation m_toAnimate3 = null;
    private Animation m_toAnimate4 = null;

    private int red = 0;
    private int green = 0;
    private int blue = 0;
    private int white = 0;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,
        Empty
    }

    private AnimationTypes m_currentAnimation;

    public CANdleSystem(CommandXboxController joy) {
        this.joystick = joy;
        changeAnimation(AnimationTypes.SetAll);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = Double.MAX_VALUE;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    public void toggle5VOverride() {
        System.out.println("State is: " + m_last5V);
        m_candle.configV5Enabled(m_last5V);
        m_last5V = !m_last5V;
    }

    public void toggleAnimDirection() {
        m_animDirection = !m_animDirection;
    }

    public int getMaximumAnimationCount() {
        return m_candle.getMaxSimultaneousAnimationCount();
    }

    public void incrementAnimation() {
        switch (m_currentAnimation) {
            case ColorFlow:
                changeAnimation(AnimationTypes.Fire);
                break;
            case Fire:
                changeAnimation(AnimationTypes.Larson);
                break;
            case Larson:
                changeAnimation(AnimationTypes.Rainbow);
                break;
            case Rainbow:
                changeAnimation(AnimationTypes.RgbFade);
                break;
            case RgbFade:
                changeAnimation(AnimationTypes.SingleFade);
                break;
            case SingleFade:
                changeAnimation(AnimationTypes.Strobe);
                break;
            case Strobe:
                changeAnimation(AnimationTypes.Twinkle);
                break;
            case Twinkle:
                changeAnimation(AnimationTypes.TwinkleOff);
                break;
            case TwinkleOff:
                changeAnimation(AnimationTypes.Empty);
                break;
            case Empty:
                changeAnimation(AnimationTypes.ColorFlow);
                break;
            case SetAll:
                changeAnimation(AnimationTypes.ColorFlow);
                break;
        }
    }

    public void decrementAnimation() {
        switch (m_currentAnimation) {
            case ColorFlow:
                changeAnimation(AnimationTypes.Empty);
                break;
            case Fire:
                changeAnimation(AnimationTypes.ColorFlow);
                break;
            case Larson:
                changeAnimation(AnimationTypes.Fire);
                break;
            case Rainbow:
                changeAnimation(AnimationTypes.Larson);
                break;
            case RgbFade:
                changeAnimation(AnimationTypes.Rainbow);
                break;
            case SingleFade:
                changeAnimation(AnimationTypes.RgbFade);
                break;
            case Strobe:
                changeAnimation(AnimationTypes.SingleFade);
                break;
            case Twinkle:
                changeAnimation(AnimationTypes.Strobe);
                break;
            case TwinkleOff:
                changeAnimation(AnimationTypes.Twinkle);
                break;
            case Empty:
                changeAnimation(AnimationTypes.TwinkleOff);
                break;
            case SetAll:
                changeAnimation(AnimationTypes.ColorFlow);
                break;
        }
    }

    public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() {
        return m_candle.getBusVoltage();
    }

    public double get5V() {
        return m_candle.get5VRailVoltage();
    }

    public double getCurrent() {
        return m_candle.getCurrent();
    }

    public double getTemperature() {

        return m_candle.getTemperature();
    }

    public void configBrightness(double percent) {
        m_candle.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        m_candle.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        m_candle.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        m_candle.configStatusLedState(offWhenActive, 0);
    }

    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;

        switch (toChange) {
            default:
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(0, 255, 119, 100, 0.7, 75, Direction.Forward,
                        8);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(1, 0.7, LEDS_PER_ANIMATION, 0.8, 0.5, m_animDirection, 0);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 119, 100, 0.1, 75, BounceMode.Front, 70,
                        8);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 1, LEDS_PER_ANIMATION, m_animDirection,
                        8);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LEDS_PER_ANIMATION,
                        8);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(0, 255, 119, 100, 0.5, LEDS_PER_ANIMATION,
                        8);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(0, 255, 119, 100, 0.01, LEDS_PER_ANIMATION,
                        8);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(128, 0, 255, 0, 0.4, LEDS_PER_ANIMATION, TwinklePercent.Percent42,
                        8);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, LEDS_PER_ANIMATION,
                        TwinkleOffPercent.Percent76, 8);
                break;
            case Empty:
                m_toAnimate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, m_animDirection,
                        8);
                break;

            case SetAll:
                m_toAnimate = null;
                break;
        }
        System.out.println("Changed to " + m_currentAnimation.toString());
    }

    public void clearAllAnims() {
        m_clearAllAnims = true;
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        if (m_toAnimate == null && m_toAnimate2 == null && m_toAnimate3 == null && m_toAnimate4 == null) {
            if (!m_setAnim) {
                /* Only setLEDs once, because every set will transmit a frame */
                m_candle.setLEDs(255, 255, 255, 0, 0, 1);
                m_candle.setLEDs(255, 255, 0, 0, 1, 1);
                m_candle.setLEDs(255, 0, 255, 0, 2, 1);
                m_candle.setLEDs(255, 0, 0, 0, 3, 1);
                m_candle.setLEDs(0, 255, 255, 0, 4, 1);
                m_candle.setLEDs(0, 255, 0, 0, 5, 1);
                m_candle.setLEDs(0, 0, 0, 0, 6, 1);
                m_candle.setLEDs(0, 0, 255, 0, 7, 1);
                m_setAnim = true;
            }
        } else {
            m_toAnimate.setSpeed(speed/3);
            m_toAnimate3.setSpeed(speed/3);
            m_toAnimate2.setSpeed(speed*3);
            m_candle.animate(m_toAnimate, 1);
            m_candle.animate(m_toAnimate2, 2);
            m_candle.animate(m_toAnimate3, 3);

            m_setAnim = false;
        }
        m_candle.modulateVBatOutput(joystick.getRightY());

        if (m_clearAllAnims) {
            m_clearAllAnims = false;
            for (int i = 0; i < 10; ++i) {
                m_candle.clearAnimation(i);
            }
        }
    }

    private void colorAnalyzer(Color color) {
        switch (color) {
            case Pink:
                red = 0;
                green = 255;
                blue = 119;
                white = 100;
                break;
            case Green:
                red = 255;
                green = 0;
                blue = 0;
                white = 0;
                break;
            case None:
                break;
        }
    }

    private void redToGreen() {
        int tempRed = red;
        red = green;
        green = tempRed;
    }

    public Command ledAnimation(int whereLED, Color color, AnimationTypes animationType) {
        return this.runOnce(
                () -> {
                    int ledOffset;
                    int maxLed;
                    colorAnalyzer(color);
                    switch (whereLED) {
                        case 0:
                            clearAllAnims();
                            ledOffset = 8;
                            maxLed = 40;
                            m_toAnimate = setAnimation(ledOffset, maxLed, animationType, Direction.Forward);
                            ledOffset += 40;
                            m_toAnimate3 = setAnimation(ledOffset, maxLed, animationType, Direction.Backward);
                            ledOffset += 40;
                            maxLed += 67;
                            m_candle.configBrightnessScalar(.5);
                            m_toAnimate2 = setAnimation(ledOffset, maxLed, animationType, Direction.Forward);
                            break;
                        case 1:
                            m_candle.clearAnimation(2);
                            ledOffset = 88;
                            maxLed = 127;
                            redToGreen();
                            speed = 0.1;
                            m_toAnimate2 = setAnimation(ledOffset, maxLed, animationType, Direction.Forward);
                            break;
                        default:
                            maxLed = 10;
                            ledOffset = 8;
                            break;
                    }

                    // m_candle.setLEDs(red, green, blue, white, ledOffset, maxLed);

                });
    }

    public void startAnimation() {
        int ledOffset;
        int maxLed;
        colorAnalyzer(Color.Pink);
        clearAllAnims();
        int ledArea = 0;
        switch (ledArea) {
            case 0:
                clearAllAnims();
                            ledOffset = 8;
                            maxLed = 39;
                            m_toAnimate = setAnimation(ledOffset, maxLed, AnimationTypes.ColorFlow, Direction.Forward);
                            ledOffset += 39;
                            m_toAnimate3 = setAnimation(ledOffset, maxLed, AnimationTypes.ColorFlow, Direction.Backward);
                            ledOffset += 42;
                            maxLed += 67;
                            colorAnalyzer(Color.Green);
                            redToGreen();
                            m_candle.configBrightnessScalar(.5);
                            m_toAnimate2 = setAnimation(ledOffset, maxLed, AnimationTypes.ColorFlow, Direction.Forward);
                            break;
            default:
                maxLed = 10;
                ledOffset = 8;
                break;
        }
    }

    private Animation setAnimation(int ledOffset, int maxLed, AnimationTypes toChange, Direction d) {
        Animation animate;
        switch (toChange) {
            default:
            case ColorFlow:
                animate = new ColorFlowAnimation(red, green, blue, white, 0.7, maxLed, d,
                        ledOffset);
                break;
            case Fire:
                animate = new FireAnimation(1, 0.7, maxLed, 0.8, 0.5, m_animDirection, ledOffset);
                break;
            case Larson:
                animate = new LarsonAnimation(red, green, blue, white, 0.1, maxLed, BounceMode.Front, 70,
                        ledOffset);
                break;
            case Rainbow:
                animate = new RainbowAnimation(1, 1, maxLed, m_animDirection,
                        ledOffset);
                break;
            case RgbFade:
                animate = new RgbFadeAnimation(0.7, 0.4, maxLed,
                        ledOffset);
                break;
            case SingleFade:
                animate = new SingleFadeAnimation(red, green, blue, white, 1, maxLed,
                        ledOffset);
                        speed = 0.5;

                break;
            case Strobe:
                animate = new StrobeAnimation(red, green, blue, white, 0.1, maxLed,
                        ledOffset);
                break;
            case Twinkle:
                animate = new TwinkleAnimation(red, green, blue, white, 0.4, maxLed, TwinklePercent.Percent42,
                        ledOffset);
                break;
            case TwinkleOff:
                animate = new TwinkleOffAnimation(red, green, blue, white, 0.2, maxLed,
                        TwinkleOffPercent.Percent76, ledOffset);
                break;
            case Empty:
                animate = new RainbowAnimation(1, 0.7, maxLed, m_animDirection,
                        ledOffset);
                break;

            case SetAll:
                animate = null;
                break;

        }
        return animate;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

}