package frc.robot;

import java.util.TreeMap;

public class Interpolator {
    private TreeMap<Double, Double> points;

    public Interpolator(double[] xPoints, double[] yPoints) {
        points = new TreeMap<>();
        if (xPoints.length != yPoints.length) {
            throw new IllegalArgumentException("xPoints and yPoints must have the same length.");
        }
        for (int i = 0; i < xPoints.length; i++) {
            points.put(xPoints[i], yPoints[i]);
        }
    }

    public double interpolate(double x) {
        if (points.isEmpty()) return 0;
        if (points.size() == 1) return points.firstEntry().getValue();

        Double floorKey = points.floorKey(x);
        Double ceilingKey = points.ceilingKey(x);

        if (floorKey == null){
            floorKey = ceilingKey;
            ceilingKey = points.higherKey(floorKey);
        }
        if (ceilingKey == null){
            ceilingKey = floorKey;
            floorKey = points.lowerKey(ceilingKey);
        }

        double floorValue = points.get(floorKey);
        double ceilingValue = points.get(ceilingKey);

        if (floorKey.equals(ceilingKey)) return floorValue;

        double ratio = (x - floorKey) / (ceilingKey - floorKey);
        return floorValue + ratio * (ceilingValue - floorValue);
    }

    /**
     * Interpolates the value at x and clamps it between min and max.
     * @param x The input value to interpolate.
     * @param min The minimum value to clamp the result to.
     * @param max The maximum value to clamp the result to.
     * @return The interpolated value at x, clamped between min and max.
     */
    public double clampedInterpolate(double x, double min, double max) {
        return ExtraMath.clamp(interpolate(x), min, max);
    }
}