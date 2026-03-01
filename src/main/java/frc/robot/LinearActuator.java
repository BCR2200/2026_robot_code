package frc.robot;

import edu.wpi.first.wpilibj.Servo;

public class LinearActuator {

    private double target;
    private final Servo servo;
    private final Servo servo2;

    public LinearActuator(int channel, String name) {

        this.servo = new Servo(channel);
        this.servo2 = new Servo(channel + 3); // adam mode
        // The example has setBounds(), which takes milliseconds, but this function uses microseconds, so we've converted the values
        servo.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        servo2.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    }

    /**
     * Set the target position of the linear actuator.
     * @param setpoint the target position of the servo (0.0-1.0, factor of total length)
     */
    public void setTargetPosition(double setpoint) {
        // clamp to [0.15, 1] because below ~15% the mechanism binds and breaks the servo
        target = ExtraMath.clamp(setpoint, 0.05, 0.95);
        servo.setPosition(target);
        servo2.setPosition(target);
    }
    
    /**
     * Get the current position target.
     * @return current target position
     */
    public double getTargetPosition() {
        return target;
    }
}
