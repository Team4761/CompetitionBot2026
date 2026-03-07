package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class TurretAimChangeCommand extends Command{
    private TurretSubsystem turretSubsystem;
    private DoubleSupplier supplierX;
    private DoubleSupplier supplierY;
    public TurretAimChangeCommand(TurretSubsystem sub, DoubleSupplier supplierX,  DoubleSupplier supplierY) {
        this.turretSubsystem = sub;
        this.supplierX = supplierX;
        this.supplierY = supplierY;
        addRequirements(sub);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        this.turretSubsystem.turnVerticalMotor(supplierY.getAsDouble() * Constants.Turret.Vertical.CONVERSION_FACTOR_MtoH);
        //System.out.println("SUPPOSED TO TURN: " + supplierX.getAsDouble() * Constants.Turret.Horizontal.CONVERSION_FACTOR_TtoM);
        this.turretSubsystem.turnHorizontalMotor(supplierX.getAsDouble() * Constants.Turret.Horizontal.CONVERSION_FACTOR_MtoT);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        turretSubsystem.stopHorizontal();
        turretSubsystem.stopVertical();
    }
}
