package org.firstinspires.ftc.teamcode.Math;

public class Brute {
    private double gain, tol;
    private double max, min;

    private double err, maxError = 50, minError = -50;

    /**
     * Calculates a Adaptive Brute Force, with initial Strength.
     */
    public Brute(double minStrength, double maxStrength, double tolerance) {
        max = maxStrength;
        min = minStrength;
        tol = tolerance;
    }

    public void setRange(double min, double max) {
        minError = min;
        maxError = max;
    }


    private void adapt() {
        gain = LazyMath.lerp(min, max, LazyMath.simplify(err, minError, maxError));
    }

    public double calculate(double error) {
        err = error;

        adapt();

        if (err > 0) {
            return gain;
        }

        if (err < 0) {
            return -gain;
        }

        return 0;
    }
}
