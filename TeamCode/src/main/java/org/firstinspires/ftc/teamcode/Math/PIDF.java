package org.firstinspires.ftc.teamcode.Math;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.List;
import java.util.function.Supplier;

public class PIDF {
    private PIDFCoefficients coefficients;
    private double error, target, current;
    private double lastError = 0, integral = 0, derivative = 0;
    private double correction = 0;
    private List<Supplier<Double>> errors;
    private int average;
    private double kA;

    /**
     * PID simples com calculo de erro e reset de Integral.
     * @param PIDFcoefficients coeficientes do PID.
     */
    public PIDF(PIDFCoefficients PIDFcoefficients) {
        coefficients = PIDFcoefficients;
        kA = 0;
    }

    /**
     * Calculador PIDA com reset de Integral e calculo de média, para um PID de reação lenta e rápida.
     * @param PIDFcoefficients coeficientes do PID.
     * @param kAvg gain do average.
     */
    public PIDF(PIDFCoefficients PIDFcoefficients, double kAvg) {
        coefficients = PIDFcoefficients;
        kA = kAvg;
    }

    public int getAverage() {
        int avg = 0;

        if (errors.size() > 9) {
            errors.remove(errors.size() - 1);
        }

        for (int i = 0; i < (errors.size() - 1); i++) {
            avg += errors.get(i).get();
        }

        avg /= errors.size();

        return avg;
    }

    /**
     * Calcula o PIDF
     * @param curr valor atual
     * @param t valor alvo
     * @return
     */
    public double calculate(double curr, double t) {
        double currentError = t - curr;
        target = t;
        current = curr;

        average = getAverage();
        if (average <= 7 && average >= -7) {
            integral = 0;
        }

        integral += currentError;
        derivative = currentError - lastError;
        correction = (currentError * coefficients.P) + (integral * coefficients.I) + (derivative * coefficients.D) + (target * coefficients.F) + (average * kA);

        return correction;
    }
}
