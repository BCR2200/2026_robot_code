package frc.robot;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/**
 * Custom Epilogue logger for PIDMotor.
 * Logs motor telemetry data during Epilogue's logging phase (offset from control loop).
 */
@CustomLoggerFor(PIDMotor.class)
public class PIDMotorLogger extends ClassSpecificLogger<PIDMotor> {

    public PIDMotorLogger() {
        super(PIDMotor.class);
    }

    private String positionName = null;
    private String velocityName = null;
    private String currentName = null;
    private String targetName = null;

    @Override
    public void update(EpilogueBackend backend, PIDMotor motor) {
        // if (positionName == null) {
        //     positionName = motor.name + " Position";
        //     velocityName = motor.name + " Velocity";
        //     currentName = motor.name + " Current";
        //     targetName = motor.name + " Target";
        // }

        backend.log(motor.name + " position", motor.getPosition());
        backend.log(motor.name + " velocity", motor.getVelocity());
        backend.log(motor.name + " current", motor.getCurrent());
        backend.log(motor.name + " target", motor.target);
    }
}
