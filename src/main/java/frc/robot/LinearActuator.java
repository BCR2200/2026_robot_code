package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LinearActuator extends Servo {

    private final double m_speed;
    private final double m_length;
    private double setPos;
    private double curPos;
    private double lastTime = 0;

    private final String name;

    /**
     * Parameters for L16-R Actuonix Linear Actuators
     *
     * @param channel PWM channel used to control the servo
     * @param length  max length of the servo [mm]
     * @param speed   max speed of the servo [mm/second]
     */
    public LinearActuator(int channel, int length, int speed, String name) {
        super(channel);
        this.name = name;
        // The example has setBounds(), which takes milliseconds, but this function uses microseconds, so we've converted the values
        setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
        m_length = length;
        m_speed = speed;
    }

    /**
     * Run this method in any periodic function to update the position estimation of your servo
     * @param setpoint the target position of the servo [mm]
     */
    public void setPosition(double setpoint) {
        setPos = MathUtil.clamp(setpoint, 0, m_length);
        setSpeed((setPos / m_length * 2) - 1);
    }

    /**
     * Increment the target position of the servo
     * To decrement, use a negative value
     * @param increment the amount to increment the target position by [mm]
     */
    public void incrementPosition(double increment) {
        setPosition(ExtraMath.clamp(setPos + increment, 0, m_length));
    }

    /**
     * Run this method in any periodic function to update the position estimation of your servo
     */
    public void updateCurPos() {
        double dt = Timer.getFPGATimestamp() - lastTime;
        if (curPos > setPos + m_speed * dt) {
            curPos -= m_speed * dt;
        } else if (curPos < setPos - m_speed * dt) {
            curPos += m_speed * dt;
        } else {
            curPos = setPos;
        }
    }

    /**
     * Current position of the servo, must be calling {@link #updateCurPos() updateCurPos()} periodically
     *
     * @return Servo Position [mm]
     */
    public double getPosition() {
        return curPos;
    }

    /**
     * Checks if the servo is at its target position, must be calling
     * {@link #updateCurPos() updateCurPos()} periodically
     * 
     * @return true when servo is at its target
     */
    public boolean isFinished() {
        return curPos == setPos;
    }

    public void dashboardPutPosition() {
        SmartDashboard.putNumber(this.name + " position", this.curPos);
    }

}
