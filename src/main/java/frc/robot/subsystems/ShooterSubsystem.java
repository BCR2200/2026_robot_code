package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Interpolator;
import frc.robot.PIDMotor;
import frc.robot.RobotContainer;

@Logged
public class ShooterSubsystem extends SubsystemBase {

    @NotLogged
    private static final double RPS_STEP = 4.0; // rps
    @NotLogged
    private static final double MAX_RPS = 140.0; // 5000 rpm in rps is 84. Max the motors can go is ~140 rps

    // Logged automatically by Epilogue
    private boolean isShooting = false;
    private double shooterSpeed = 54; // in rps
    private boolean isFeeding = false;

    private boolean isManualMode = true;
    public double manualShooterSpeed = 0;
    public double manualTagetHoodPosition = 0;

    @NotLogged
    private RobotContainer rc;

    // Logged via PIDMotorLogger
    @Logged(name = "JohnShootMotor")
    public PIDMotor johnShootPIDMotor;
    @Logged(name = "JawbreakerShootMotor")
    public PIDMotor jawbreakerShootPIDMotor;
    @Logged(name = "TaylorShootMotor")
    public PIDMotor taylorShootPIDMotor;

    @Logged(name = "ErikFeedMotor")
    public PIDMotor erikFeedPIDMotor;
    @Logged(name = "HoekFeedMotor")
    public PIDMotor hoekFeedPIDMotor;

    @Logged(name = "VantHoodMotor")
    public PIDMotor vantHoodPIDMotor;


    @NotLogged
    private final Interpolator shooterAngleInterpolator;
    @NotLogged
    private final Interpolator shooterVelocityInterpolator;
    @NotLogged
    private final Interpolator timeOfFlightInterpolator;

    public ShooterSubsystem(int shootCurrentLimit, int feedCurrentLimit, int hoodCurrentLimit,
            Interpolator shooterAngleInterpolator, Interpolator shooterVelocityInterpolator,
            Interpolator timeOfFlightInterpolator, RobotContainer rc) {

        // John shooter (top left)
        johnShootPIDMotor = PIDMotor.makeMotor(Constants.JOHN_SHOOTER_MOTOR_ID, "john shooter", 0.1, 0.0, 0.0,
                0.2, 0.0957, 0.0, MAX_RPS, MAX_RPS, 0.00);
        johnShootPIDMotor.setInverted(InvertedValue.Clockwise_Positive);
        johnShootPIDMotor.setStatorCurrentLimit(shootCurrentLimit);
        johnShootPIDMotor.setIdleCoastMode();

        // Jawbreaker shooter (top right)
        jawbreakerShootPIDMotor = PIDMotor.makeMotor(Constants.JAWBREAKER_SHOOTER_MOTOR_ID, "jawbreaker shooter", 0.1, 0.0, 0.0,
                0.2, 0.0957, 0.0, MAX_RPS, MAX_RPS, 0.00);
        jawbreakerShootPIDMotor.setInverted(InvertedValue.CounterClockwise_Positive); // The one on the other side is flipped
        jawbreakerShootPIDMotor.setStatorCurrentLimit(shootCurrentLimit);
        jawbreakerShootPIDMotor.setIdleCoastMode();
        jawbreakerShootPIDMotor.follow(johnShootPIDMotor, true); // Inverted follower

        // Taylor shooter (middle left)
        taylorShootPIDMotor = PIDMotor.makeMotor(Constants.TAYLOR_SHOOTER_MOTOR_ID, "taylor shooter", 0.1, 0.0, 0.0,
                0.2, 0.0957, 0.0, MAX_RPS, MAX_RPS, 0.00);
        taylorShootPIDMotor.setInverted(InvertedValue.Clockwise_Positive);
        taylorShootPIDMotor.setStatorCurrentLimit(shootCurrentLimit);
        taylorShootPIDMotor.setIdleCoastMode();
        taylorShootPIDMotor.follow(johnShootPIDMotor, false); // Not inverted follower

        // Erik feed (bottom left)
        erikFeedPIDMotor = PIDMotor.makeMotor(Constants.ERIK_FEED_MOTOR_ID,"erik feeder", 0.10, 0.0, 0.0,
                0.25, 0.1, 0.0, MAX_RPS, MAX_RPS * 10, 0.00);
        erikFeedPIDMotor.setStatorCurrentLimit(feedCurrentLimit);
        erikFeedPIDMotor.setInverted(InvertedValue.CounterClockwise_Positive);
        erikFeedPIDMotor.setIdleCoastMode();

        // Hoek feed (bottom right)
        hoekFeedPIDMotor = PIDMotor.makeMotor(Constants.HOEK_FEED_MOTOR_ID,"hoek feeder", 0.10, 0.0, 0.0,
                0.25, 0.1, 0.0, MAX_RPS, MAX_RPS * 10, 0.00);
        hoekFeedPIDMotor.setStatorCurrentLimit(feedCurrentLimit);
        hoekFeedPIDMotor.setInverted(InvertedValue.Clockwise_Positive); // The one on the other side is flipped
        hoekFeedPIDMotor.setIdleCoastMode();
        hoekFeedPIDMotor.follow(erikFeedPIDMotor, true); // Inverted follower

        // Vant hood (middle right)
        vantHoodPIDMotor = PIDMotor.makeMotor(Constants.VANT_HOOD_MOTOR_ID, "vant hood", 0.1, 0.0, 0.0,
                0.2, 0.1, 0.0, MAX_RPS, MAX_RPS, 0.00);
        vantHoodPIDMotor.setStatorCurrentLimit(hoodCurrentLimit);
        vantHoodPIDMotor.setIdleBrakeMode();

        this.shooterAngleInterpolator = shooterAngleInterpolator;
        this.shooterVelocityInterpolator = shooterVelocityInterpolator;
        this.timeOfFlightInterpolator = timeOfFlightInterpolator;

        this.rc = rc;
    }

    public boolean getIsShooting() {
        return isShooting;
    }

    public void setIsShooting(boolean shooting) {
        isShooting = shooting;
    }

    public boolean getIsFeeding() {
        return isFeeding;
    }

    public void setIsFeeding(boolean feeding) {
        isFeeding = feeding;
    }

    /**
     * Interpolate the shooter speed given a distance from the target
     * 
     * @param distance in m
     */
    public void setShooterSpeedViaInterpolatedValue(double distance) {
        this.shooterSpeed = shooterVelocityInterpolator.clampedInterpolate(distance, 0, 95); // In rps
    }

     /**
     * Interpolate the shooter hood positon given a distance from the target
     * 
     * @param distance in m
     */
    public void setHoodPositionViaInterpolatedValue(double distance) {
        this.shooterSpeed = shooterAngleInterpolator.clampedInterpolate(distance, 0, 0); // in rotations
    }

    /**
     * use this to determine if the shooter is at speed before spinning up the
     * feeder motor
     *
     * @return if the shooter motor is at or above the expected speed
     */
    public boolean isShooterAtSpeed() { // TODO: MAKE THIS WORK WITH THE OTHER MOTORS???
        // Why >5? Because we only set a velocity target in periodic, but
        // isShooterAtSpeed is called when target is still 0,
        // atVelocity() can return true unexpectedly, because its velocity is really 0.
        return johnShootPIDMotor.getVelocity() > 5 && johnShootPIDMotor.atVelocity(3);
    }

    public boolean needsFloorFeed() {
        return isFeeding;
    }

    public void setManualMode(boolean isManualMode) {
        this.isManualMode = isManualMode;
    }

    @Override
    public void periodic() {
        // Feed at full speed first,
        // then try to preload (until beam break is broken),
        // then stop
        if (isFeeding) {
            erikFeedPIDMotor.setPercentOutput(1);
        } else {
            erikFeedPIDMotor.setPercentOutput(0);
        }

        if (isManualMode) {
            johnShootPIDMotor.setVelocityTarget(manualShooterSpeed);
            vantHoodPIDMotor.setTarget(manualTagetHoodPosition);
            return;
        }

        // Fixed shots
        if (rc.fixedPassingShot) {
            setShooterSpeedViaInterpolatedValue(8.5);
            setHoodPositionViaInterpolatedValue(8.5);
            if (isShooting) {
                johnShootPIDMotor.setVelocityTarget(shooterSpeed);
            }
            else {
                johnShootPIDMotor.setPercentOutput(0);
            }
            return;
        }
        else if (rc.fixedShotFromHub) {
            setShooterSpeedViaInterpolatedValue(1);
            setHoodPositionViaInterpolatedValue(1);
            if (isShooting) {
                johnShootPIDMotor.setVelocityTarget(shooterSpeed);
            }
            else {
                johnShootPIDMotor.setPercentOutput(0);
            }
            return;
        }
        else if (rc.fixedShotFromClimber) {
            setShooterSpeedViaInterpolatedValue(3.5);
            setHoodPositionViaInterpolatedValue(3.5);
            if (isShooting) {
                johnShootPIDMotor.setVelocityTarget(shooterSpeed);
            }
            else {
                johnShootPIDMotor.setPercentOutput(0);
            }
            return;
        }

        // Shoot
        if (isShooting) {
            if (rc.passing) {
                setShooterSpeedViaInterpolatedValue(rc.getDistanceToTarget(rc.passTarget));
            } else {
                setShooterSpeedViaInterpolatedValue(rc.getDistanceToTarget(rc.compensatedTargetHub));
            }
            johnShootPIDMotor.setVelocityTarget(shooterSpeed);
        } else {
            johnShootPIDMotor.setPercentOutput(0);
        }

        // Move the hood
        if (rc.passing) {
            setHoodPositionViaInterpolatedValue(rc.getDistanceToTarget(rc.passTarget));
        } else if (isShooting) {
            setHoodPositionViaInterpolatedValue(rc.getDistanceToTarget(rc.compensatedTargetHub));
        } else if (!rc.isOutsideAllianceZone()) {
            setHoodPositionViaInterpolatedValue(rc.getDistanceToTarget(rc.targetHub));
        }
    }
}
