// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.vision.VisionSubsystem;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    final boolean win = true;

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

    private MatchPhase currentPhase;
    private Timer phaseTimer;
    private double phaseDuration;
    private VisionSubsystem visionSubsystem = new VisionSubsystem();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        matchTimer = new Timer();
        currentPhase = MatchPhase.AUTONOMOUS;
        phaseTimer = new Timer();
        phaseDuration = FieldConstants.Match.AUTONOMOUS_DURATION;

    }

    @Override
    public void robotInit() {
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        // Alex M. helped
        SmartDashboard.putNumber("Match Time Left", Math.round((FieldConstants.Match.MATCH_DURATION-matchTimer.get())*10)/10.0);
        SmartDashboard.putString("Current Match Phase", currentPhase.toString());
        SmartDashboard.putNumber("Phase Time Left", Math.round((phaseDuration-phaseTimer.get())*10)/10.0);
        //below has not been tested please test
        //displays a color showing the apriltag status red is no april tag yellow is apriltag detected green is ready to fire
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
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        phaseDuration = FieldConstants.Match.AUTONOMOUS_DURATION;
        currentPhase = MatchPhase.AUTONOMOUS;

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
