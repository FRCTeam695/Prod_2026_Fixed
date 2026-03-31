package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.DoubleSupplier;


    /*
     * 0 - Red
     * 1 - Orange
     * 2 - Yellow
     * 3 - Green
     * 4 - Blue
     * 5 - Purple
     * 6 - Grey
     * 7 - Black (off)
     * 8 - White
     * Defaults to off
     */
public class LED extends SubsystemBase {
    private AddressableLED realLED;
    private AddressableLEDBuffer realLEDBuffer;
    
    public LED() {

        realLED = new AddressableLED(9);
        realLEDBuffer = new AddressableLEDBuffer(9);
        realLED.setLength(realLEDBuffer.getLength());
        realLED.setData(realLEDBuffer);
        realLED.start();

    }
    //___________________________________________________________________________________________
    //Rainbow Pattern
    public void rainbow() {
        LEDPattern rainbow = LEDPattern.rainbow(255, 255); //The first value is saturation, second value is brightness. Max of 255
        /*
         * Calculate this by finding the length of your LED Strip. 
         * Divide 100 by that number and then multiply by the number of leds on your strip that you measured. 
         * Then divide that number by one. 
         * That number represents the amount of LEDs per meter of strip, 
         * which in this case is rounded to 58.8. 
         * Dividing one by this number is to create a ratio that says "for every one meter, there are 58.8 LEDs"
         */
        Distance kLedSpacing = Meters.of(1 / 58.8); //MAY NEED CHANGE
        LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.1), kLedSpacing); //creates scrolling rainbow effect. Change the MetersPerSecond.of() variable to control speed.

        scrollingRainbow.applyTo(realLEDBuffer);
        realLED.setData(realLEDBuffer);
    }
    //Turns LED Off
    public void setLEDOff() {
        for (int i = 0; i < realLEDBuffer.getLength(); i++) {

            realLEDBuffer.setRGB(i, 0, 0, 0);

        }

        realLED.setData(realLEDBuffer);
    }
    
    //Select a number that will correspond to a color. The LED will be set to that color.
    public LEDPattern colorBase(int colorNumber) {
        switch (colorNumber) {
            case 0:
                return LEDPattern.solid(Color.kRed);
            case 1:
                return LEDPattern.solid(Color.kOrange);
            case 2:
                return LEDPattern.solid(Color.kYellow);
            case 3:
                return LEDPattern.solid(Color.kGreen);
            case 4:
                return LEDPattern.solid(Color.kBlue);
            case 5:
                return LEDPattern.solid(Color.kPurple);
            case 6:
                return LEDPattern.solid(Color.kGray);
            case 7:
                return LEDPattern.solid(Color.kBlack);
            case 8:
                return LEDPattern.solid(Color.kWhite);
            default:
                return LEDPattern.solid(Color.kBlack);
        }
    }
    public void setColor(int colorNumber) {
        LEDPattern color = colorBase(colorNumber);

        color.applyTo(realLEDBuffer);

        realLED.setData(realLEDBuffer);
    }
    public void breathingEffect(int colorNumber, double seconds) {
        LEDPattern base = colorBase(colorNumber);
        LEDPattern pattern = base.breathe(Seconds.of(seconds));

        pattern.applyTo(realLEDBuffer);
        realLED.setData(realLEDBuffer);
    }
    
    //___________________________________________________________________________________________
    //Commands

    //solid color command
    public Command solidColor(int colorNumber) {
        return new FunctionalCommand( 
            () -> {
            }, 
            
            () -> {
                setColor(colorNumber);
            }, 
            
            interrupted -> {
                setLEDOff();
            }, 
            
            () -> false, 
            
            this);
    }

        public Command solidColor(DoubleSupplier colorNumber) {
        return new FunctionalCommand( 
            () -> {
            }, 
            
            () -> {
                setColor((int)colorNumber.getAsDouble());
            }, 
            
            interrupted -> {
                setLEDOff();
            }, 
            
            () -> false, 
            
            this);
    }

    public Command breatheEffect(DoubleSupplier colorNumber, double seconds) {
        return new FunctionalCommand(

                () -> {

                },

                () -> {
                    breathingEffect((int) colorNumber.getAsDouble(), seconds);
                },

                interrupted -> {
                    setLEDOff();
                },

                () -> false,

                this);
    }
    
    //breatheEffect command
    public Command breatheEffect(int colorNumber, double seconds) {
        return new FunctionalCommand(

                () -> {

                },

                () -> {
                    breathingEffect(colorNumber, seconds);
                },

                interrupted -> {
                    setLEDOff();
                },

                () -> false,

                this);
    }    //rainbowLED command
    public Command rainbowLED() {
        return new FunctionalCommand(
            () -> {}, 
            
            () -> {
                rainbow();
            }, 
        
            interrupted -> {
                setLEDOff();
            }, 
            
            () -> false, 
        
        this);
    }

}