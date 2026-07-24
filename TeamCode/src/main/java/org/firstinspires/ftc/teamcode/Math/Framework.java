package org.firstinspires.ftc.teamcode.Math;


import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class Framework implements Controller {
    PIDFCoefficients coeff;
    PIDCoefficients amp;
    double bruteStrength;
    double target, current, tolerance;
    double ku, kw, kb;
    double max, min;

    PIDF fast;
    PID weighted;

    boolean dangerZone;

    /**
     * Cria um novo sistema de Framework.
     * Basicamente, cria um sistema onde
     * res(t) = ku * u(t) + kw * uw(t) + kb * b(t)
     *
     * Para tentar melhorar o controle de valores para se aproximar ainda mais da posição desejada.
     */
    public Framework(PIDFCoefficients coefficients, PIDCoefficients ampCoefficients, double bruteStrength, double bruteTolerance) {
        ku = 1; kw = 1; kb = 1;

        assignRange(-360, 360);
        fast = new PIDF(coefficients);
        weighted = new PID(ampCoefficients, 0.3);

        dangerZone = true;
    }

    public void assignRange(double minError, double maxError) {
        max = maxError;
        min = minError;
    }

    public boolean useBrute(double err, double tolerance) {
        return !(err < tolerance && err > -tolerance);
    }

    public double brute(double err, double tolerance) {
        if (err > tolerance && err < -tolerance) {
            return bruteStrength * Math.pow(LazyMath.lerp(1, 20, LazyMath.simplify(Math.abs(err), 0, max)), 2) * Math.signum(err);
        }

        return 0;
    }

    public void updateGains(double target, double current, double avg, double accumulative) {

    }

    /**
     * Calculates given [target, current];
     * @param args {target, current};
     * @return
     */
    @Override
    public double calculate(double... args) {
        double output = 0;

        target = args[0];
        current = args[1];
        double err = target - current;

        while (useBrute(err, tolerance)) {
             output = brute(err, tolerance) * kb;
        };

        if (!useBrute(err, tolerance) || kb < 0.8) {
            double u = fast.calculate(target, current);

            weighted.setError(target, fast.getAverage());
            double w = weighted.calculate();

            output = u * ku + w * kw;
        }

        return output;
    }
}