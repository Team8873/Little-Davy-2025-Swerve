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

import edu.wpi.first.wpilibj.util.Color.RGBChannel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.command.CANdleAnimationCommand;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

public class CANdleSystem extends SubsystemBase {
    private int LEDS_PER_ANIMATION = 1580;
    private final CANdle m_candle = new CANdle(Constants.CANdleConstants.CANdleID, "rio");
    private CommandXboxController joystick;
    private int m_candleChannel = 0;
    private boolean m_clearAllAnims = false;
    private boolean m_last5V = false;
    private boolean m_animDirection = false;
    private boolean m_setAnim = false;
    private double speed;

    private int ledOffset = 8;

    private Animation m_toAnimate = null;
    private Animation m_toAnimate2 = null;
    private Animation m_toAnimate3 = null;
    private Animation m_toAnimate4 = null;
    private int m_id;

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

    public enum Color {
        Pink,
        Green,
        None
    }

    private int red = 0;
    private int green = 0;
    private int blue = 0;
    private int white = 0;

    private AnimationTypes m_currentAnimation;

    public CANdleSystem(CommandXboxController joy) {
        this.joystick = joy;
        changeAnimation(AnimationTypes.SetAll, Color.None);
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

    public void setColors() {
        changeAnimation(AnimationTypes.SetAll, Color.None);
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
    
/**
 * sets the animation of the leds to the desired animation
 * @param toChange the desired animation
 * @return the desired animation
 */
    public Animation animationSwitch(AnimationTypes toChange) {
        Animation animate;
        switch (toChange) {
            default:
            case ColorFlow:
                animate = new ColorFlowAnimation(red, green, blue, white, 0.7, LEDS_PER_ANIMATION,
                        Direction.Forward,
                        ledOffset);
                break;
            case Fire:
                animate = new FireAnimation(1, 0.7, LEDS_PER_ANIMATION, 0.8, 0.5, m_animDirection, ledOffset);
                break;
            case Larson:
                animate = new LarsonAnimation(red, green, blue, white, 0.1, LEDS_PER_ANIMATION,
                        BounceMode.Front, 70,
                        ledOffset);
                break;
            case Rainbow:
                animate = new RainbowAnimation(1, 1, LEDS_PER_ANIMATION, m_animDirection,
                        ledOffset);
                break;
            case RgbFade:
                animate = new RgbFadeAnimation(0.7, 0.4, LEDS_PER_ANIMATION,
                        ledOffset);
                break;
            case SingleFade:
                animate = new SingleFadeAnimation(red, green, blue, white, 0.5, LEDS_PER_ANIMATION,
                        ledOffset);
                break;
            case Strobe:
                animate = new StrobeAnimation(red, green, blue, white, 0.01, LEDS_PER_ANIMATION,
                        ledOffset);
                break;
            case Twinkle:
                animate = new TwinkleAnimation(red, green, blue, white, 0.4, LEDS_PER_ANIMATION,
                        TwinklePercent.Percent42,
                        ledOffset);
                break;
            case TwinkleOff:
                animate = new TwinkleOffAnimation(red, green, blue, white, 0.2, LEDS_PER_ANIMATION,
                        TwinkleOffPercent.Percent76, ledOffset);
                break;
            case Empty:
                animate = new RainbowAnimation(1, 0.7, LEDS_PER_ANIMATION, m_animDirection,
                        ledOffset);
                break;

            case SetAll:
                animate = null;
                break;
        }
        return animate;
    }

    public void clearAllAnims() {
        m_clearAllAnims = true;
    }

    @Override
    public void periodic() {

        // This method will be called once per scheduler run
        if (m_toAnimate == null) {
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
            m_toAnimate.setSpeed(speed);
            m_candle.animate(m_toAnimate, 0);
            m_candle.animate(m_toAnimate2, 1);
            m_candle.animate(m_toAnimate3, 2);
            m_candle.animate(m_toAnimate4, 3);

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
    /**
     * sets red to green and green to red
     */
    private void redToGreen(){
        int tempRed = red;
        red = green;
        green = tempRed;
    }
    /**
     * sets the color of the leds for animation
     * @param color the desired color
     */
    private void colorAnalyzer(Color color){
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

    /**
     * sets the color of the animation and the animation itself
     * @param toChange the animation to change to 
     * @param color the color to change to
     */
    public void changeAnimation(AnimationTypes toChange, Color color) {
        m_currentAnimation = toChange;
       colorAnalyzer(color);

        switch (m_id) {
            case 0:
                redToGreen();
                m_toAnimate = animationSwitch(toChange);
                redToGreen();
                ledOffset += 40;
                m_toAnimate4 = animationSwitch(toChange);
                break;
            case 1:
                m_toAnimate = animationSwitch(toChange);
                break;
            case 2:
                m_toAnimate = animationSwitch(toChange);
                break;

            default:
                break;
        }
        System.out.println("Changed to " + m_currentAnimation.toString());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void setSpeedOfStrobeAnimations(double lightSpeedBasedOnDistance) {
        speed = lightSpeedBasedOnDistance + 0.1;
    }

    /**
     * tells candle how many leds to control
     * @param id the location
     */
    public void numControl(int id) {
        m_id = id;
        switch (id) {
            case 0:
                LEDS_PER_ANIMATION = 50;
                ledOffset = 48;
                break;
            case 1:
                LEDS_PER_ANIMATION = 1580;
                ledOffset = 8;
                break;
            case 2:
                LEDS_PER_ANIMATION = 1580;
                ledOffset = 8;
                break;
            case 10:
            LEDS_PER_ANIMATION = 1580;
            ledOffset = 8;
            break;
        }
    }

    public void clearAnimation(int animationID) {
        m_candle.clearAnimation(animationID);
    }

}