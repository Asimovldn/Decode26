package org.firstinspires.ftc.teamcode.Math;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

public class PID implements Controller {
    private PIDCoefficients coefficients;
    private Supplier<Double> error;
    public double lastError = 0, integral = 0, derivative = 0;
    private double correction = 0;
    private ArrayList<Supplier<Double>> errors;
    private ArrayList<Double> outputs = new ArrayList<>();
    private int average, memorySize;
    private double kA, maxOutput, minOutput;
    private boolean getNormalized = false;

    /**
     * PID simples com calculo de erro e reset de Integral.
     * @param PIDcoefficients coeficientes do PID.
     */
    public PID(PIDCoefficients PIDcoefficients) {
        coefficients = PIDcoefficients;
        kA = 0;
        errors = new ArrayList<>();

        memorySize = 10;
    }

    public static PID fromPIDF(PIDF pidf) {
        PIDFCoefficients coeff = pidf.getCoefficients();
        return new PID(new PIDCoefficients(
                coeff.P,
                coeff.I,
                coeff.D
        ));
    }

    public void setMemorySize(int size) {
        memorySize = size;
    }

    /**
     * Calculador PIDA com reset de Integral e calculo de média, para um PID de reação lenta e rápida.
     * @param PIDcoefficients coeficientes do PID.
     * @param kAvg gain do average.
     */
    public PID(PIDCoefficients PIDcoefficients, double kAvg) {
        coefficients = PIDcoefficients;
        kA = kAvg;
        errors = new ArrayList<>();
    }

    public void setError(Supplier<Double> newError) {
        error = newError;
    }

    public void setError(double newError) {
        setError(() -> newError);
    }

    public void setError(double t, double curr) {
        setError(() -> t - curr);
    }

    public int getAverage() {
        int avg = 0;

        if (errors.size() > memorySize) {
            errors.remove(0);
        }

        return (int) LazyMath.averageSupplier(errors);
    }

    public void normalize() {
        getNormalized = true;
    }

    public void setRange(double min, double max) {
        minOutput = min;
        maxOutput = max;
    }

    public double getAccumulative() {
        ArrayList<Double> acm = new ArrayList<Double>();

        for (Supplier<Double> err : errors) {
            acm.add(Math.abs(err.get()));
        }

        return LazyMath.average(acm);
    }

    public void useAsDerivative(double x) {
        derivative = x;
    }

    public void useAsIntegral(double x) {
        integral = x;
    }

    /**
     * Do not use ...args; Consider addError() instead.
     * @param args
     * @return
     */
    public double calculate(double... args) {
        double currentError = error.get();
        errors.add(error);

        if (errors.size() > memorySize) {
            errors.remove(0);
        }

        if (outputs.size() > memorySize) {
            outputs.remove(0);
        }

        if (!outputs.isEmpty()) {
            if (maxOutput < outputs.get(outputs.size() - 1)) {
                maxOutput = outputs.get(outputs.size() - 1);
            }

            if (minOutput > outputs.get(outputs.size() - 1)) {
                minOutput = outputs.get(outputs.size() - 1);
            }
        }

        average = getAverage();
        if (average <= 7 && average >= -7) {
            integral = 0;
        }

        integral += currentError;
        derivative = currentError - lastError;
        correction = (currentError * coefficients.p) + (integral * coefficients.i) + (derivative * coefficients.d) + (average * kA);

        lastError = error.get();

        outputs.add(correction);

        if (getNormalized) {
            correction /= maxOutput;
        }

        return correction;
    }

    public void editCoefficients(PIDCoefficients newCoefficients) {
        coefficients = newCoefficients;
    }
}
