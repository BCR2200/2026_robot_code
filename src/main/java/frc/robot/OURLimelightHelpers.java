package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;


public class OURLimelightHelpers {

    public record LimelightContour(boolean hasTarget, double degreesX, double degreesY) implements Sendable {
        @Override
        public void initSendable(SendableBuilder builder) {
            builder.addBooleanProperty("hasTarget", () -> hasTarget, null);
            builder.addDoubleProperty("degreesX", () -> degreesX, null);
            builder.addDoubleProperty("degreesY", () -> degreesY, null);
        }
    }

    /**
     * Gets the coordnates of the box of the largest countour
     * @return LimelightContour which hasTarget, offsetX, offsetY
     */
    // public static LimelightContour getContour() {
    //     double[] limelightData = NetworkTableInstance.getDefault().getTable(Constants.FEEDER_LIMELIGHT_NAME).getEntry("llpython").getDoubleArray(new double[8]);
    //     boolean hasTarget = limelightData[0] == 1;
    //     double degreesX = limelightData[1];
    //     double degreesY = limelightData[2];
    //     return new LimelightContour(hasTarget, degreesX, degreesY);
    // }

    public static Pose2d betterGetPose2d(String primaryCam, String fallBackCam) {
        Pose2d primaryBotPose = LimelightHelpers.getBotPose2d(primaryCam);
        Pose2d fallbackBotPose = LimelightHelpers.getBotPose2d(fallBackCam);

        if (!hasAprilTag(primaryCam)) {
            return fallbackBotPose;
        } 
        else {
            return primaryBotPose;
        }
    }

    public static boolean hasAprilTag(String limelightKey) {
        // Get the network table entry for "tv" (target valid)
        // The second argument (0.0 here) is the default value if the key is missing
        long tv = NetworkTableInstance.getDefault().getTable(limelightKey).getEntry("tv").getInteger(0);

        // If 'tv' is 0.0, no valid target is found. Otherwise, one or more are found.
        return tv == 1;
    }

    
}
