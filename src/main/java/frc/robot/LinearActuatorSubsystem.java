package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LinearActuatorSubsystem extends SubsystemBase {

    private double target;
    private final String name;
    
    private final Servo servo;

    public LinearActuatorSubsystem(int channel, String name) {

        this.servo = new Servo(channel);
        this.name = name;

        // The example has setBounds(), which takes milliseconds, but this function uses microseconds, so we've converted the values
        servo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);

    }

    /**
     * Set the target position of the linear actuator.
     * @param setpoint the target position of the servo (0.0-1.0, factor of total length)
     */
    public void setPosition(double setpoint) {
        target = ExtraMath.clamp(setpoint, -0.000, 1);
        servo.setPosition(target);
    }
    
    /**
     * Get the current position target.
     * @return current target position
     */
    public double getPosition() {
        return target;
    }
    
    @Override
    public void periodic() {
        this.dashboardPutPosition();
    }

    public void dashboardPutPosition() {
        SmartDashboard.putNumber(this.name + " position", this.servo.getPosition());
    }

}
