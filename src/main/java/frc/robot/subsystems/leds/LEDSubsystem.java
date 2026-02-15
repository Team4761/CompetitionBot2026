package frc.robot.subsystems.leds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Map;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import static frc.robot.subsystems.leds.ImageCompressor.loadImage;
import static frc.robot.subsystems.leds.ImageCompressor.compressImage;


public class LEDSubsystem extends SubsystemBase {
    public static AddressableLED leds;
    public static AddressableLEDBuffer buffer;
    // private AddressableLED rightSide;
    // private AddressableLEDBuffer rightBuffer;

    private LEDPattern previousPattern;
    private LEDPattern currentPattern;
        
    // Also, per LED strip, there are 150 LEDs.
    // Supposedly the LEDs function in GRB not RGB... We'll need to test this though.
    /** Available LED patterns:
     * <p> green-black discontinuous gradient
     * <p> lights that move across the strip, and change to a random color when bounce off the edge
     * <p> LED patterns that aren't finished:
     * <p> lights that blink blanched almond when the robot is perfectly aligned in teleop (the only one with a debugging function)
     */
    
    public LEDSubsystem() {
        // Comment out patterns that aren't being used
        

        leds = new AddressableLED(Constants.LEDS_PORT);
        buffer = new AddressableLEDBuffer(Constants.LEDS_NUMBER_OF_LEDS); // 150 LEDs in a straight line
        leds.setLength(Constants.LEDS_NUMBER_OF_LEDS);
        leds.start();

        // rightSide = new AddressableLED(1);
        // rightBuffer = new AddressableLEDBuffer(Constants.LEDS_NUMBER_OF_LEDS); // 150 LEDs in a straight line
        // rightSide.setLength(Constants.LEDS_NUMBER_OF_LEDS);
        // rightSide.start();
        
        currentPattern = RobocketsLEDPatterns.OFF;
        previousPattern = RobocketsLEDPatterns.OFF;
    }

        // this is my most ambitious pattern yet... A Rick Roll!
        // this might unfourtunately not be run at competitions due to uncertainties regarding processing power
        //pattern.applyTo(buffer);
        //leds.setData(buffer);
        int[][][] testImageAsCompressed3dArrayComingToThearterMarch32_2111 = compressImage(loadImage("/C:/Users/alex/Pictures/wtf.png", false), 4, 5);{
        for (int i = 0; i < testImageAsCompressed3dArrayComingToThearterMarch32_2111.length; i++)
            {
                for (int f = 0; f < testImageAsCompressed3dArrayComingToThearterMarch32_2111[i].length; f++)
                {
                    buffer.setRGB(f + (i * Constants.LEDS_HEIGHT), testImageAsCompressed3dArrayComingToThearterMarch32_2111[i][f][0], testImageAsCompressed3dArrayComingToThearterMarch32_2111[i][f][1], testImageAsCompressed3dArrayComingToThearterMarch32_2111[i][f][2] );
                }
            }
            leds.setData(buffer);
        }


    public void setPattern(LEDPattern pattern) {
        previousPattern = currentPattern;
        currentPattern = pattern;
        pattern.applyTo(buffer);
        leds.setData(buffer);
    }


    /**
     * This gets the last pattern that was used. This starts out as RobocketsLEDPatterns.OFF
     * @return The previously used LED Pattern
     */
    public LEDPattern getPreviousPattern() {
        return previousPattern;
    }


    /**
     * This sets the LEDs to the "black" color (which turns them off)
     */
    public void stopLEDs() {
        LEDPattern off = LEDPattern.solid(Color.kBlack);
        off.applyTo(buffer);
        leds.setData(buffer);
    }
        


    // the command that aplies the pattern to the LEDs
    public Command runPattern(LEDPattern pattern) {
        return run(() -> setPattern(pattern));
    }
}