package org.firstinspires.ftc.teamcode.Math;

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
    private int average, memorySize;
    private double kA;

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

    public double calculate(double... args) {
        double currentError = error.get();
        errors.add(error);

        average = getAverage();
        if (average <= 7 && average >= -7) {
            integral = 0;
        }

        integral += currentError;
        derivative = currentError - lastError;
        correction = (currentError * coefficients.p) + (integral * coefficients.i) + (derivative * coefficients.d) + (average * kA);

        return correction;
    }

    public void editCoefficients(PIDCoefficients newCoefficients) {
        coefficients = newCoefficients;
    }
}
