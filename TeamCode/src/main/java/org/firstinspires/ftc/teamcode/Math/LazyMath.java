package org.firstinspires.ftc.teamcode.Math;

public class LazyMath {
    public static double lerp(double a, double b, double c) {
        return a * (1.0 - c) + (b * c);
    }

    /**
     * Normalize t, that is a value between [rA, rB], to [nA, nB];
     * @param t value to be normalized
     * @param rA min of initial range
     * @param rB max of initial range
     * @param nA min of target range
     * @param nB max of target range
     * @return normalized t between [nA, nB]
     */
    public static double normalize(double t, double rA, double rB, double nA, double nB) {
        return nA + ( ( (t - rA) * (nB - nA) ) / rB - rA );
    }

    /**
     * Normalize t, that is a value between [a, b] into [0, 1];
     * @param t the value to be normalized
     * @param a min of current range
     * @param b max of current range
     * @return
     */
    public static double simplify(double t, double a, double b) {
        return normalize(t, a, b, 0, 1);
    }

    /**
     * Normalize t, that is a value between [a, b] into [-1, 1];
     * @param t the value to be normalized
     * @param a min of current range
     * @param b max of current range
     * @return
     */
    public static double signNormalize(double t, double a, double b) {
        return normalize(t, a, b, -1, 1);
    }

}
