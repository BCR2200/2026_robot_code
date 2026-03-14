package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.servohub.config.ServoHubConfig;

public class LinearActuator {

    private double target;

    // PWM servos (used on practice bot)
    private Servo servo;
    private Servo servo2;

    // Servo Hub channels (used on comp bot)
    private static ServoHub servoHub;
    private ServoChannel servoChannel;

    // Pulse width bounds in microseconds
    private static final int MIN_PULSE_US = 1000;
    private static final int MAX_PULSE_US = 2000;

    public LinearActuator(int channel, String name) {
        if (Robot.isCompBot) {
            initServoHub(channel);
        } else {
            initPwmServos(channel);
        }
    }

    /**
     * Initialize PWM servos (practice bot).
     */
    private void initPwmServos(int channel) {
        this.servo = new Servo(channel);
        this.servo2 = new Servo(channel + 3); // adam mode
        // The example has setBounds(), which takes milliseconds, but this function uses microseconds
        servo.setBoundsMicroseconds(2000, 1550, 1500, 1450, 1000);
        servo2.setBoundsMicroseconds(2000, 1550, 1500, 1450, 1000);
    }

    /**
     * Initialize Servo Hub channels (comp bot).
     * ServoHub channels are 0-5, while PWM channels are 1-6, so we subtract 1.
     */
    private void initServoHub(int channel) {
        // Create shared ServoHub instance on first use
        if (servoHub == null) {
            servoHub = new ServoHub(Constants.SERVO_HUB_CAN_ID);

            // Configure pulse ranges for all channels
            ServoHubConfig config = new ServoHubConfig();
            config.channel0.pulseRange(MIN_PULSE_US, 1500, MAX_PULSE_US);
            config.channel1.pulseRange(MIN_PULSE_US, 1500, MAX_PULSE_US);
            config.channel2.pulseRange(MIN_PULSE_US, 1500, MAX_PULSE_US);
            config.channel3.pulseRange(MIN_PULSE_US, 1500, MAX_PULSE_US);
            config.channel4.pulseRange(MIN_PULSE_US, 1500, MAX_PULSE_US);
            config.channel5.pulseRange(MIN_PULSE_US, 1500, MAX_PULSE_US);
            servoHub.configure(config, ResetMode.kResetSafeParameters);
        }

        // Map PWM channel (1-6) to ServoHub channel (0-5)
        int hubChannel = channel - 1;

        ServoChannel.ChannelId channelId = getChannelId(hubChannel);

        this.servoChannel = servoHub.getServoChannel(channelId);

        // Enable and power the channels
        servoChannel.setEnabled(true);
        servoChannel.setPowered(true);
    }

    /**
     * Convert channel number (0-5) to ChannelId enum.
     */
    private static ServoChannel.ChannelId getChannelId(int channel) {
        return switch (channel) {
            case 0 -> ServoChannel.ChannelId.kChannelId0;
            case 1 -> ServoChannel.ChannelId.kChannelId1;
            case 2 -> ServoChannel.ChannelId.kChannelId2;
            case 3 -> ServoChannel.ChannelId.kChannelId3;
            case 4 -> ServoChannel.ChannelId.kChannelId4;
            case 5 -> ServoChannel.ChannelId.kChannelId5;
            default -> throw new IllegalArgumentException("Invalid ServoHub channel: " + channel);
        };
    }

    /**
     * Convert position (0.0-1.0) to pulse width in microseconds.
     */
    private int positionToPulseWidth(double position) {
        return (int) (MIN_PULSE_US + (position * (MAX_PULSE_US - MIN_PULSE_US)));
    }

    /**
     * Set the target position of the linear actuator.
     * @param setpoint the target position of the servo (0.0-1.0, factor of total length)
     */
    public void setTargetPosition(double setpoint) {
        // clamp to [0.05, 0.95] because below ~5% the mechanism binds and breaks the servo
        target = ExtraMath.clamp(setpoint, 0.05, 0.95);

        if (Robot.isCompBot) {
            int pulseWidth = positionToPulseWidth(target);
            servoChannel.setPulseWidth(pulseWidth);
        } else {
            servo.setPosition(target);
            servo2.setPosition(target);
        }
    }

    /**
     * Get the current position target.
     * @return current target position
     */
    public double getTargetPosition() {
        return target;
    }
}
