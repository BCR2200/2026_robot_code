package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;

public class OURLimelightHelpers {

    public record LimelightContour(boolean hasTarget, double offsetX, double offsetY) {
    }

    /**
     * Gets the coordnates of the box of the largest countour
     * @return LimelightContour which hasTarget, offsetX, offsetY
     */
    public static LimelightContour getContour() {
        double[] limelightData = NetworkTableInstance.getDefault().getTable(Constants.FEEDER_LIMELIGHT_NAME).getEntry("largest_contour").getDoubleArray(new double[5]);
        boolean hasTarget = limelightData[0] == 1;
        double offsetX = limelightData[1] + (limelightData[3] / 2); // Calculating center x by adding half the width to the left x
        double offsetY = limelightData[2] + (limelightData[4] / 2); // Calculating center y by adding half the height to the top y
        return new LimelightContour(hasTarget, offsetX, offsetY);
    }

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
