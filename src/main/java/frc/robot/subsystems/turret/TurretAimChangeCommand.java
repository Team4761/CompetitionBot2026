package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


public class TurretAimChangeCommand extends Command{
    private TurretSubsystem turretSubsystem;
    private DoubleSupplier supplierX;
    private DoubleSupplier supplierY;

    /**
     * X
     * @param sub The turret subsystem
     * @param supplierX A doubleSupplier to get an X change value
     * @param supplierY A doubleSupplier to get a Y change value
     */
    public TurretAimChangeCommand(TurretSubsystem sub, DoubleSupplier supplierX,  DoubleSupplier supplierY) {
        this.turretSubsystem = sub;
        this.supplierX = supplierX;
        this.supplierY = supplierY;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        // This function is always running, mostly because it's related to the joysticks
    }

    @Override
    public void execute() {
        this.turretSubsystem.verticalMotor.turn(supplierY.getAsDouble() * Constants.Turret.ShootConfig.TURRET_VERTICAL_MULTIPLIER);
        this.turretSubsystem.horizontalMotor.turn(supplierX.getAsDouble() * Constants.Turret.ShootConfig.TURRET_HORIZONTAL_MULTIPLIER);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        turretSubsystem.horizontalMotor.stopTurning();
        turretSubsystem.verticalMotor.stopTurning();
    }
}
