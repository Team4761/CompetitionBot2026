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
import static frc.robot.subsystems.leds.ImageCompressor.loadVideoAsMultipleImages;
import static frc.robot.subsystems.leds.ImageCompressor.compressImage;
import static frc.robot.subsystems.leds.ImageCompressor.compressVideo;
import frc.robot.subsystems.intake.IntakeCommand;


@SuppressWarnings("unused")
public class LEDSubsystem extends SubsystemBase {
    public static AddressableLED leds;
    public static AddressableLEDBuffer buffer;

    //no, the type "double" is not a mistake
    //note: must make each false condition set it to a UNIQUE out of bounds spot
    public static double intakeTrue = 1.0;
    public static double shootTrue = 1.1;

    private LEDPattern previousPattern;
    private LEDPattern currentPattern;
        
    // We will have a 16x1 strip

    // IMPORTANT: If the LEDs are in GRB instead of RGB again, please infom me (Alex Maniscalco). I already have a fix for it
    // also ask/tell me about any other problems or conflicts this subsystem makes

    /** Available LED patterns:
     * <p> LEDs that have a certain part light up a certain color when one or more functions are being performed. also displays a color on elastic. 
     *     Current functions needed: Shoot, Intake
     * <p> LED patterns that aren't finished:
     * <p> Rickroll (any dimensions)
     */
    
    public LEDSubsystem() {
        // Comment out patterns that aren't being used

        leds = new AddressableLED(Constants.LEDS_PORT);
        buffer = new AddressableLEDBuffer(Constants.LEDS_NUMBER_OF_LEDS); // 16 LEDs in a straight line
        leds.setLength(Constants.LEDS_NUMBER_OF_LEDS);
        leds.start();
        
        currentPattern = RobocketsLEDPatterns.OFF;
        previousPattern = RobocketsLEDPatterns.OFF;

    }

    // This pattern is for debugging. lights up a certain part of the LEDs with a color corresponding with a certain robot function, if said function is running.
    LEDPattern debugFunctions = LEDPattern.steps(Map.of(intakeTrue, Color.kRed, shootTrue, Color.kBlue));
        
        

    //int[][][][] testVideoAsCompressedArrayComingToTheatersMarch32_2111 = compressVideo(loadVideoAsMultipleImages("C:/Users/alex/Documents/CODINF/RickrollFragments/frame.png", 1, 100, true),32,8);
    //public static double freme = 0;
    public void periodic(){

        
        // This pattern emulates a Rickroll, with the height and width of it being adjustable
        // this might unfourtunately not be run at competitions due to uncertainties regarding processing power, and the fact that it is way too slow to load
        // find a way to take images on the fly rather than load them all beforehand
        /**
        for (int y = 0; y < testVideoAsCompressedArrayComingToTheatersMarch32_2111[(int) freme].length; y++)
        {
            for (int x = 0; x < testVideoAsCompressedArrayComingToTheatersMarch32_2111[(int) freme][y].length; x++)
            {
                buffer.setRGB(x + (y * Constants.LEDS_WIDTH), testVideoAsCompressedArrayComingToTheatersMarch32_2111[(int) freme][y][x][0], testVideoAsCompressedArrayComingToTheatersMarch32_2111[(int) freme][y][x][1], testVideoAsCompressedArrayComingToTheatersMarch32_2111[(int) freme][y][x][2]);
            }
        }
        if(freme < testVideoAsCompressedArrayComingToTheatersMarch32_2111.length-1){
            freme += 0.5;
        }
        */
        debugFunctions.applyTo(buffer);
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