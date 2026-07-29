package org.firstinspires.ftc.teamcode.Math;


import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.util.ArrayList;

import kotlin.Lazy;

public class Framework implements Controller {
    PIDFCoefficients coeff;
    PIDCoefficients amp;
    double bruteStrength, memorySize;
    double target, current, tolerance;
    double ku, kw, kb;
    double max, min, averageError, alpha;
    double maxOutput, minOutput;

    PIDF fast;
    PID weighted;

    boolean dangerZone, tryToNormalize;
    double D, C, dangerGain;

    ArrayList<Double> gainChanges, predictedErrors, absPredicted, setpoints, outputHistory;

    /**
     * Cria um novo sistema de Framework.
     * Basicamente, cria um sistema onde
     * res(t) = ku * u(t) + kw * uw(t) + kb * b(t)
     * sendo {@code res(t)} seu resultado final;<br>
     * {@code u(t)} o PID Fast [primeiro];<br>
     * {@code uw(t)} o PID Weighted [segundo];<br>
     * {@code b(t)} o Brute [bang-bang];<br>
     * ku, kw, kb são valores de ganho (adaptativos, não constantes).<br>
     * Para tentar melhorar o controle de valores para se aproximar ainda mais da posição desejada.
     */
    public Framework(PIDFCoefficients coefficients, PIDCoefficients ampCoefficients, double bruteStrength, double bruteTolerance) {
        ku = 1; kw = 1; kb = 1;

        assignRange(-360, 360);
        fast = new PIDF(coefficients);
        weighted = new PID(ampCoefficients, 0.3);

        dangerZone = false;

        gainChanges = new ArrayList<>();

        setpoints = new ArrayList<>();

        D = 0;
        C = 1;
        max = 100;
        min = -100;

        memorySize = 10;

        dangerGain = LazyMath.clamp(coeff.P * ampCoefficients.p, 0.1, 1);
    }

    public void assignAverageError(double average) {
        averageError = average;
    }

    public void setMemorySize(double size) {
        memorySize = size;
    }

    public void setDangerGain(double x) {
        dangerGain = LazyMath.clamp(x, 0, 1);
    }
    //
    public void addPoint(double p) {
        setpoints.add(p);
    }

    /**
     * Assignes the normal range of the error. For example, {@code [-360, 360]} is a good range for turret modules.<br>
     * This works for the framework to understand what is the extreme error or small error, is a important value.<br>
     * If you are not sure, tend to make the numbers bigger then normal.<br>
     * If you suspect of a {@code [-50, 50]} range, for example, make it {@code [-60, 60]}, to avoid overshooting.<br>
     * @param minError the minimum error in the range.
     * @param maxError the maximum error in the range.
     */
    public void assignRange(double minError, double maxError) {
        max = maxError;
        min = minError;

        assignAverageError(LazyMath.median(min, max));
    }

    public boolean useBrute(double err, double tolerance) {
        return !(err < tolerance && err > -tolerance);
    }

    public double brute(double err, double tolerance) {
        double s = LazyMath.closest(target, setpoints);

        if (s - target < tolerance && C > 0.5 && D <= 0.5) {
            target = s;
            err = target - current;
            return bruteStrength * Math.pow(LazyMath.lerp(1, 20, LazyMath.simplify(Math.abs(err), 0, max)), 2) * Math.signum(err);
        }

        if (Math.abs(err) > tolerance && C > 0.5 && D <= 0.5) {
            return bruteStrength * Math.pow(LazyMath.lerp(1, 20, LazyMath.simplify(Math.abs(err), 0, max)), 2) * Math.signum(err);
        }

        return 0.0;
    }

    public void updateGains(double target, double current, double avg, double accumulative) {
        ku = D * (1 - C);
        kw = C * (1 - LazyMath.simplify(target - current, min, max));
        kb = C * (1 - D);

        if (gainChanges.size() > memorySize) {
            gainChanges.remove(0);
        }

        gainChanges.add(kb);

        if (accumulative > C * (Math.abs(max) + Math.abs(min)) / 2) {
            ku = LazyMath.clamp(ku * 2, 0, 1);
            kb = LazyMath.clamp(kb / 2, 0, 1);
            D = LazyMath.clamp(D * 2, 1, 0);

            double derivateDifference = coeff.P - coeff.D;
            double ampDifference = amp.p - amp.d;

            coeff.P = LazyMath.lerp(coeff.P, coeff.P + (ku * 0.25) * -D, alpha);
            coeff.D = LazyMath.lerp(coeff.D, coeff.P - derivateDifference, alpha);
            amp.p = LazyMath.lerp(amp.p, amp.p * (kw * 0.5), alpha);
            amp.d = LazyMath.lerp(amp.d, amp.p - ampDifference, alpha);
        }
    }

    public void updateConfidence() {
        double errorC = 1 - LazyMath.clamp((fast.getAccumulative() / Math.abs(max)), 0, 1);

        double kbChange = 1 - LazyMath.clamp(LazyMath.variance(gainChanges), 0, 1);

        double stable = 1 - LazyMath.clamp(LazyMath.normalizedVariance(fast.getErrors(), Math.max(Math.abs(max), Math.abs(min))), 0, 1);

        double predictedError =
                Math.abs(predictAccumulative() - fast.getAccumulative()) / Math.max(fast.getAccumulative(), 1);
        double predictConfidence = LazyMath.clamp(1 - predictedError, 0, 1);

        C = LazyMath.lerp(C, errorC * kbChange * stable * predictConfidence, alpha);

        updateGains(target, current, fast.getAverage(), fast.getAccumulative());

        setDangerGain(LazyMath.lerp(dangerGain, LazyMath.clamp(coeff.P * amp.p, 0, 1), alpha));
    }

    public double predictAccumulative() {
        return LazyMath.average(absPredicted);
    }

    public double predictNext() {
        double predict = fast.lastError + slope();

        predictedErrors.add(predict);
        absPredicted.add(Math.abs(predict));

        if (absPredicted.size() > memorySize) {
            absPredicted.remove(0);
        }

        if (predictedErrors.size() > memorySize) {
            predictedErrors.remove(0);
        }

        return predict;
    }

    public void useAsDerivative(double x) {
        fast.useAsDerivative(x);
    }

    public void useAsIntegral(double x) {
        fast.useAsIntegral(x);
    }

    public void normalizeOutput(boolean state) {
        tryToNormalize = true;
    }

    public ArrayList<Double> getErrors() {
        return fast.getErrors();
    }

    public double slope() {
        double rate = 0;

        for (int i = 1; i < fast.getErrors().size(); i++) {
            rate += fast.getErrors().get(i) - fast.getErrors().get(i - 1);
        }

        rate /= fast.getErrors().size() - 1;


        return rate;
    }

    public void attemptToResetRanges() {
        boolean ignoreMax = getErrors().contains(max);
        boolean ignoreMin = getErrors().contains(min);

        boolean s = slope() < 0;
        boolean safety = C > D;
        boolean acm = fast.getAccumulative() > 3 && fast.getAccumulative() < Math.max(Math.abs(max), Math.abs(min)) / 2;;
        boolean stdDev = LazyMath.variance(getErrors()) > 2;

        if (s && safety && acm && stdDev) {
            if (!ignoreMax) {
                max = LazyMath.biggest(getErrors());
            }
            if (!ignoreMin) {
                min = LazyMath.smallest(getErrors());
            }

            maxOutput = LazyMath.biggest(outputHistory);
            minOutput = LazyMath.smallest(outputHistory);
        }

        if (max == 0) max = 1;
        if (min == 0) min = 1;
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

        if (err < min) {
            min = err;
        }

        if (err > max) {
            max = err;
        }

        updateConfidence();

        D = LazyMath.lerp(D, LazyMath.clamp(dangerGain * (ku / Math.min(C, 0.01)), 0, 1), alpha);
        dangerZone = D > 0.8;

        output = 0;

        if (useBrute(err, tolerance)) {
            fast.calculate(target, current);
            output = brute(err, tolerance) * kb;
        };

        double predict = 0;

        if (!useBrute(err, tolerance) || C < 0.5 || D >= 0.5) {
            double u = fast.calculate(target, current);

            weighted.setError(target, fast.getAverage());
            double w = weighted.calculate();

            predict = predictNext();

            output = u * ku + w * kw + ((predict - fast.lastError) * (coeff.D * alpha)) * kw;

            if (setpoints.contains(output)) {
                output = 0;
            }
        }

        attemptToResetRanges();

        if (tryToNormalize) {
            output = LazyMath.simplify(output, minOutput, maxOutput);
        }

        outputHistory.add(output);

        if (outputHistory.size() > memorySize) {
            outputHistory.remove(0);
        }

        return output;
    }
}