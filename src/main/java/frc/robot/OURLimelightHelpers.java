package frc.robot;

import java.util.Arrays;

public class OURLimelightHelpers {
    public static double[] getJohnJawbreakerTaylorPercentages() {
        double[] dataFromLimelight = LimelightHelpers.getLimelightNTTableEntry("limelight", "llpython").getDoubleArray(new double[8]);
        return Arrays.copyOfRange(dataFromLimelight, 3, 6);
    }
}
