// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commandGroups.TurretLockCommand;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    final boolean win = true; //THIS IS ESSENTIAL DO NOT DELETE.

    private final RobotContainer m_robotContainer;

    private Timer matchTimer;

    private enum MatchPhase {
        AUTONOMOUS,
        TRANSITION,
        TELEOP_FIRST_SHIFT,
        TELEOP_SECOND_SHIFT,
        TELEOP_THIRD_SHIFT,
        TELEOP_FOURTH_SHIFT,
        ENDGAME
    }
    private enum TEAM {
        RED,
        BLUE
    }
    private enum POSITION {
        LEFT,
        CENTER,
        RIGHT
    }

    private MatchPhase currentPhase;
    private Timer phaseTimer;
    private double phaseDuration;
    private final VisionSubsystem visionSubsystem;
    private SendableChooser<String> teamChooser = new SendableChooser<>();
    private SendableChooser<String> positionChooser = new SendableChooser<>();
    
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        visionSubsystem = RobotContainer.getVisionSubsystem();
        matchTimer = new Timer();
        currentPhase = MatchPhase.AUTONOMOUS;
        phaseTimer = new Timer();
        phaseDuration = FieldConstants.Match.AUTONOMOUS_DURATION;

    }

    @Override
    public void robotInit() {
        // Set up the SmartDashboard options for the alliance color and starting position
        SmartDashboard.putNumber("TURRET HORIZONTAL ANGLE", 0.0);
        SmartDashboard.putNumber("TURRET VERTICAL ANGLE", 0.0);
        SmartDashboard.putBoolean("Manual Turret Control", false);
        teamChooser.addOption("BLUE", "BLUE");
        teamChooser.addOption("RED", "RED");
        teamChooser.setDefaultOption("BLUE", "BLUE");
        positionChooser.addOption("LEFT", "LEFT");
        positionChooser.addOption("CENTER", "CENTER");
        positionChooser.addOption("RIGHT", "RIGHT");
        positionChooser.setDefaultOption("CENTER", "CENTER");
        SmartDashboard.putData("Team", teamChooser);
        SmartDashboard.putData("Position", positionChooser);
        SmartDashboard.putString("Driver Controller Bindings", 
            "Left Joystick: Move the Robot (field oriented)\n" +
            "Right Joystick: Turn (Right Clockwise, Left Counter-Clockwise)\n" +
            "Right Trigger: Run the Intake\n" +
            "Left Trigger: Run the Outtake\n" +
            "Back Button: Reset Field Orientation"
        );
        SmartDashboard.putString("Operator Controller Bindings", 
            "Right Trigger: Shoot (Hold to Shoot)\n" +
            "Right Bumper: Jostle the Intake\n" +
            "Start Button: Extend the Intake, if not already extended\n" +
            "Left Trigger: Initiate Manual Override (Hold to Override)\n" +
            "Manual Override Controls (While Manual Override is Active):\n" +
            "MO: Left Joystick: Control the Turret's Vertical Aim\n" +
            "MO: Right Joystick: Control the Turret's Horizontal Aim\n" +
            "MO: Right Trigger: Shoot Wihout Safeties (Hold to Shoot)\n" +
            "MO: B Button: Run the Spindexer Backwards\n" +
            "MO: Back Button: Disable Vision Tracking\n"

        );
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        // The Original Alexander Maniscalco helped
        SmartDashboard.putNumber("Match Time Left", Math.round((FieldConstants.Match.MATCH_DURATION-matchTimer.get())*10)/10.0);
        SmartDashboard.putString("Current Match Phase", currentPhase.toString());
        SmartDashboard.putNumber("Phase Time Left", Math.round((phaseDuration-phaseTimer.get())*10)/10.0);
        //below has not been tested please test
        //displays a color showing the apriltag status red is no april tag yellow is apriltag detected green is ready to fire purple is tracking disabled
        //make sure to to right click and click on show as single color veiw
        SmartDashboard.putString("April Tag Status", (visionSubsystem.seesAprilTag()).toHexString());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_robotContainer.init();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        phaseDuration = FieldConstants.Match.AUTONOMOUS_DURATION;
        currentPhase = MatchPhase.AUTONOMOUS;
        Constants.Field.ALLIANCE_COLOR = teamChooser.getSelected();
        Constants.Field.STARTING_POSITION = positionChooser.getSelected();
        System.out.println("Alliance: " + teamChooser.getSelected() + ", Starting Position: " +  positionChooser.getSelected());

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
        matchTimer.reset();
        matchTimer.start();
        phaseTimer.reset();
        phaseTimer.start();
        
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        currentPhase = MatchPhase.TRANSITION;
        phaseTimer.reset();
        phaseDuration = FieldConstants.Match.TRANSITION_DURATION;
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        switch (currentPhase) {
            case TRANSITION:
                if (phaseTimer.get() >= FieldConstants.Match.TRANSITION_DURATION) {
                    currentPhase = MatchPhase.TELEOP_FIRST_SHIFT;
                    phaseTimer.reset();
                    phaseDuration = FieldConstants.Match.ALLIANCE_SHIFT_DURATION;
                }
                break;
            case TELEOP_FIRST_SHIFT:
                if (phaseTimer.get() >= FieldConstants.Match.ALLIANCE_SHIFT_DURATION) {
                    currentPhase = MatchPhase.TELEOP_SECOND_SHIFT;
                    phaseTimer.reset();
                    phaseDuration = FieldConstants.Match.ALLIANCE_SHIFT_DURATION;
                }
                break;
            case TELEOP_SECOND_SHIFT:
                if (phaseTimer.get() >= FieldConstants.Match.ALLIANCE_SHIFT_DURATION) {
                    currentPhase = MatchPhase.TELEOP_THIRD_SHIFT;
                    phaseTimer.reset();
                    phaseDuration = FieldConstants.Match.ALLIANCE_SHIFT_DURATION;
                }
                break;
            case TELEOP_THIRD_SHIFT:
                if (phaseTimer.get() >= FieldConstants.Match.ALLIANCE_SHIFT_DURATION) {
                    currentPhase = MatchPhase.TELEOP_FOURTH_SHIFT;
                    phaseTimer.reset();
                    phaseDuration = FieldConstants.Match.ALLIANCE_SHIFT_DURATION;
                }
                break;
            case TELEOP_FOURTH_SHIFT:
                if (phaseTimer.get() >= FieldConstants.Match.ALLIANCE_SHIFT_DURATION) {
                    currentPhase = MatchPhase.ENDGAME;
                    phaseTimer.reset();
                    phaseDuration = FieldConstants.Match.ENDGAME_DURATION;
                }
                break;
            case ENDGAME:
                // Do nothing, just stay in endgame until the match ends
                break;
        }
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
