package org.firstinspires.ftc.teamcode.Math;

import java.util.ArrayList;
import java.util.function.Supplier;

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

    public static double average(ArrayList<Double> l) {
        if (l.isEmpty()) return 0.0;

        int avg = 0;

        for (int i = 0; i < (l.size() - 1); i++) {
            avg += l.get(i);
        }

        avg /= l.size();

        return avg;
    }

    public static double biggest(ArrayList<Double> l) {
        double b = 0;

        for (double i : l) {
            if (i > b) b = i;
        }

        return b;
    }

    public static double smallest(ArrayList<Double> l) {
        double b = Double.MAX_VALUE;

        for (double i : l) {
            if (i < b) b = i;
        }

        return b;
    }

    public static double closest(double x, ArrayList<Double> l) {
        double d = Double.MAX_VALUE;
        double v = 0;

        for (Double y : l) {
            double dist = Math.abs(x - y);

            if (dist < d) {
                d = dist;
                v = y;
            }
        }

        return v;
    }

    public static double averageSupplier(ArrayList<Supplier<Double>> l) {
        if (l.isEmpty()) return 0.0;
        int avg = 0;

        for (int i = 0; i < (l.size() - 1); i++) {
            avg += l.get(i).get();
        }

        if (!l.isEmpty()) {
            avg /= l.size();
        }

        return avg;
    }

    public static double abs(double x) {
        return x * Math.signum(x);
    }

    public static double median(double min, double max) {
        return lerp(min, max, 0.5);
    }

    public static double clamp(double x, double min, double max) {
        return Math.max(min, Math.min(x, max));
    }

    public static double variance(ArrayList<Double> values) {
        double mean = values.stream()
                .mapToDouble(Double::doubleValue)
                .average()
                .orElse(0.0);

        double variance = values.stream()
                .mapToDouble(v -> {
                    double diff = v - mean;
                    return diff * diff;
                })
                .average()
                .orElse(0.0);

        return Math.sqrt(variance);
    }

    public static double normalizedVariance(ArrayList<Double> values, double scale) {
        if (values.isEmpty() || scale == 0) return 0;

        return clamp(variance(values) / Math.abs(scale), 0, 1);
    }
}
