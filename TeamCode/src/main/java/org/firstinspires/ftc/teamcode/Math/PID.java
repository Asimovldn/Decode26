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
    private double lastError = 0, integral = 0, derivative = 0;
    private double correction = 0;
    private ArrayList<Supplier<Double>> errors;
    private int average;
    private double kA;

    /**
     * PID simples com calculo de erro e reset de Integral.
     * @param PIDcoefficients coeficientes do PID.
     */
    public PID(PIDCoefficients PIDcoefficients) {
        coefficients = PIDcoefficients;
        kA = 0;
        errors = new ArrayList<>();
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

        if (errors.size() > 9) {
            errors.remove(errors.size() - 1);
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
