package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ZeroTheHood extends Command {

    ShooterSubsystem shooterSubsystem;
    private int finalCurrentLimit;

    public ZeroTheHood(ShooterSubsystem shooterSubsystem, int finalCurrentLimit) {
        this.shooterSubsystem = shooterSubsystem;
        this.finalCurrentLimit = finalCurrentLimit;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.isZeroing = true;
    }

    @Override
    public void execute() {
        shooterSubsystem.vantHoodPIDMotor.setStatorCurrentLimit(5);
        shooterSubsystem.vantHoodPIDMotor.setPercentOutput(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.isZeroing = false;
        shooterSubsystem.vantHoodPIDMotor.setPercentOutput(0);
        shooterSubsystem.vantHoodPIDMotor.setStatorCurrentLimit(finalCurrentLimit);
        shooterSubsystem.vantHoodPIDMotor.resetEncoder();
    }
}
