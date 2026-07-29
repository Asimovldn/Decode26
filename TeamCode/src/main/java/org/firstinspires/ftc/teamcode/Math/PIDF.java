package org.firstinspires.ftc.teamcode.Math;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class PIDF implements Controller {
    private PIDFCoefficients coefficients;
    private double error, target, current;
    public double lastError = 0, integral = 0, derivative = 0;
    private double correction = 0;
    private ArrayList<Supplier<Double>> errors;
    private int average, memorySize;
    private double kA;
    private double avgCorrection;

    /**
     * PID simples com calculo de erro e reset de Integral.
     * @param PIDFcoefficients coeficientes do PID.
     */
    public PIDF(PIDFCoefficients PIDFcoefficients) {
        coefficients = PIDFcoefficients;
        kA = 0;
        errors = new ArrayList<>();

        memorySize = 10;
    }

    /**
     * Calculador PIDA com reset de Integral e calculo de média, para um PID de reação lenta e rápida.
     * @param PIDFcoefficients coeficientes do PID.
     * @param kAvg gain do average.
     */
    public PIDF(PIDFCoefficients PIDFcoefficients, double kAvg) {
        coefficients = PIDFcoefficients;
        kA = kAvg;
        errors = new ArrayList<>();
    }

    public void setMemorySize(int size) {
        memorySize = size;
    }

    public void useAsDerivative(double x) {
        derivative = x;
    }

    public void useAsIntegral(double x) {
        integral = x;
    }


    public int getAverage() {
        int avg = 0;

        if (errors.size() > memorySize) {
            errors.remove(0);
        }

        for (int i = 0; i < (errors.size() - 1); i++) {
            avg += errors.get(i).get();
        }

        if (!errors.isEmpty()) {
            avg /= errors.size();
        } else {
            avg = 0;
        }

        return avg;
    }

    public double getAccumulative() {
        ArrayList<Double> acm = new ArrayList<>();

        for (Supplier<Double> err : errors) {
            acm.add(err.get() * Math.signum(err.get()));
        }

        return LazyMath.average(acm);
    }

    public ArrayList<Double> getErrors() {
        ArrayList<Double> errs = new ArrayList<>();
        errors.forEach((Supplier<Double> err) -> {
            errs.add(err.get());
        });

        return errs;
    }

    /**
     * Calcula o PIDF
     * @return
     */
    public double calculate(double... args) {
        double currentError = args[0] - args[1];
        target = args[0];
        current = args[1];

        average = getAverage();
        if (average <= 7 && average >= -7) {
            integral = 0;
        }

        avgCorrection = LazyMath.lerp(avgCorrection, (average * kA), 0.8);
        if ((avgCorrection - average * kA) < 0.2) {
            avgCorrection = average * kA;
        }

        integral += currentError;
        derivative = currentError - lastError;
        correction = (currentError * coefficients.P) + (integral * coefficients.I) + (derivative * coefficients.D) + (target * coefficients.F) + avgCorrection;

        return correction;
    }

    public void editCoefficients(PIDFCoefficients newCoefficients) {
        coefficients = newCoefficients;
    }
}
