package org.firstinspires.ftc.teamcode.Math;


import static org.firstinspires.ftc.teamcode.Math.LazyMath.biggest;
import static org.firstinspires.ftc.teamcode.Math.LazyMath.clamp;
import static org.firstinspires.ftc.teamcode.Math.LazyMath.lerp;
import static org.firstinspires.ftc.teamcode.Math.LazyMath.normalize;
import static org.firstinspires.ftc.teamcode.Math.LazyMath.normalizedVariance;
import static org.firstinspires.ftc.teamcode.Math.LazyMath.saturation;
import static org.firstinspires.ftc.teamcode.Math.LazyMath.tanh;
import static org.firstinspires.ftc.teamcode.Math.LazyMath.variance;
import static java.lang.Math.abs;

import com.pedropathing.control.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

import kotlin.Lazy;

public class Framework implements Controller {
    PIDFCoefficients coeff;
    PIDCoefficients amp;
    double bruteStrength, memorySize;
    double target, current, tolerance, lastError;
    double ku, kw, kb;
    double max, min, averageError, alpha;
    double maxOutput, minOutput;
    double lastSlope, predict, lastPredict;

    PIDF fast;
    PID weighted;

    boolean dangerZone, tryToNormalize;
    double D, C, dangerGain, lastTime;

    ElapsedTime time;

    private final double startP, startD, ampP, ampD;

    ArrayList<Double> gainChanges, predictedErrors, absPredicted, setpoints, outputHistory, slopes;

    public static class Builder {

    }

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

        coeff = coefficients;
        amp = ampCoefficients;

        dangerZone = false;

        gainChanges = new ArrayList<>();

        time = new ElapsedTime();
        lastTime = time.seconds();

        setpoints = new ArrayList<>();

        D = 0;
        C = 1;
        max = 100;
        min = -100;

        lastSlope = 0;
        lastError = 0;
        lastPredict = 0;

        alpha = 0.5;

        memorySize = 10;

        dangerGain = 1.0;

        startP = coeff.P;
        startD = coeff.D;
        ampP = amp.p;
        ampD = coeff.D;
    }

    public void assignAverageError(double average) {
        averageError = average;
    }

    public void setMemorySize(double size) {
        memorySize = size;
    }

    public void setDangerGain(double x) {
        dangerGain = clamp(x, 0, 1);
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

    /**
     * Defines {@code alpha}, which represents the {@code smoothness} of the code. How the values change and react.
     * <br>{@code Alpha} will define this velocity in a {@code 0 <= alpha <= 1} scale.
     * @param x the new value.
     */
    public void setAlpha(double x) {
        alpha = clamp(x, 0, 1);
    }

    public boolean useBrute(double err, double tolerance) {
        return !(err < tolerance && err > -tolerance);
    }

    public double brute(double err, double tolerance) {
        double s = LazyMath.closest(target, setpoints);

        if (current == s) return 0.0;

        if (s - target < tolerance && C > 0.5 && D <= 0.5) {
            target = s;
            err = target - current;
            return bruteStrength * Math.pow(LazyMath.lerp(1, 20, LazyMath.simplify(abs(err), 0, max)), 2) * Math.signum(err);
        }

        if (abs(err) > tolerance && C > 0.5 && D <= 0.5) {
            return bruteStrength * Math.pow(LazyMath.lerp(1, 20, LazyMath.simplify(abs(err), 0, max)), 2) * Math.signum(err);
        }

        return 0.0;
    }

    /**
     * {@code p3 = D(1 - C)}<br>{@code p2 = C(1 - s(y(t) - r(t), emin, emax)}<br>{@code p3 = C(1 - D)}
     * @param target the target value of the Framework. {@code y(t)}
     * @param current current value of the Framework. {@code r(t)}
     * @param avg the average value of the Framework. {@code e'}
     * @param accumulative the accumulative value of the Framework. {@code E}
     */
    public void updateGains(double target, double current, double avg, double accumulative) {
        ku = D * (1 - C);
        kw = C * (1 - LazyMath.simplify(target - current, min, max));
        kb = C * (1 - D);

        if (gainChanges.size() > memorySize) {
            gainChanges.remove(0);
        }

        gainChanges.add(kb);

        double derivativeDifference = startP - startD;

        coeff.P = lerp(coeff.P, startP * (0.5 + normalizedVariance(fast.getErrors(), Math.max(abs(min), abs(max)), 0, 1)) * (1 - C), alpha); // [0, startP * 1.5]
        coeff.D = lerp(coeff.D, coeff.P - derivativeDifference, alpha);
    }

    public double normalizeAcm() {
        return fast.getAccumulative()/ Math.max(abs(max), abs(min));
    }

    public void updateConfidence() {
        double errorC = 1 - clamp(normalizeAcm(), 0, 1);

        double kbChange = 1 - clamp(variance(gainChanges), 0, 1);

        double stable = 1 - clamp(normalizedVariance(fast.getErrors(), Math.max(abs(max), abs(min)), 0, 1), 0, 1);

        double predictedError =
                abs(predictAccumulative() - fast.getAccumulative()) / Math.max(fast.getAccumulative(), 1);
        double predictConfidence = 1 - clamp(predictedError, 0, 1);

        C = LazyMath.lerp(C, errorC * kbChange * stable * predictConfidence, alpha);

        updateGains(target, current, fast.getAverage(), fast.getAccumulative());

        double reqP = startP * (0.5 + normalizeAcm()); // startP * [0.5, 1.5]

        dangerGain = lerp(
                dangerGain,
                10 * Math.pow((reqP - coeff.P), 2),
                alpha
        );
    }

    public double predictAccumulative() {
        return LazyMath.average(absPredicted);
    }

    public void updatePredict() {
        predictedErrors.add(predict);
        absPredicted.add(abs(predict));

        if (absPredicted.size() > memorySize) {
            absPredicted.remove(0);
        }

        if (predictedErrors.size() > memorySize) {
            predictedErrors.remove(0);
        }
    }

    public double predictNext(boolean update) {
        double predict = fast.lastError + slope();

        if (update) {
            updatePredict();
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
        boolean safety = (C > D) && (D < 0.5);
        boolean acm = fast.getAccumulative() > min && fast.getAccumulative() < Math.max(abs(max), abs(min)) / 2;
        boolean stdDev = variance(getErrors()) > 2;

        if (s && safety && acm && stdDev) {
            if (!ignoreMax) {
                max = lerp(max, biggest(getErrors()), alpha);
            }

            if (!ignoreMin) {
                min = lerp(min, LazyMath.smallest(getErrors()), alpha);
            }

            maxOutput = biggest(outputHistory);
            minOutput = LazyMath.smallest(outputHistory);
        }

        if (max == 0) max = 1;
        if (min == 0) min = -1;
    }

    /**
     * [inserir documenação útil slk vapo]
     */
    public void updateDanger() {
        double slopeChange = saturation(variance(slopes), dangerGain);
        double outputAmplitude = saturation(variance(outputHistory), dangerGain);

        D = lerp(D, clamp(slopeChange * outputAmplitude, 0, 1), alpha);
        dangerZone = D > 0.7;
    }

    /**
     * Calculates given [target, current].<br>
     * Automatically calculates {@code C, D} and {@code P1, P2, P3} values.<br>
     * Returns {@code res(t) = u1(t)p1 + u2(t)p2 + u3(t)p3}
     * @param args {target, current};
     * @return the output.
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

        updateDanger();

        output = 0;

        double u = fast.calculate(target, current);

        weighted.setError(target, fast.getAverage());
        double w = weighted.calculate();

        predict = predictNext(true);

        output = (u * ku) + (w * kw) + ((fast.lastError - predict) * (coeff.D * alpha)) * kw + brute(err, tolerance) * kb;

        if (setpoints.contains(current)) {
            output = 0;
        }

        attemptToResetRanges();

        if (tryToNormalize) {
            output = LazyMath.simplify(output, minOutput, maxOutput);
        }

        outputHistory.add(output);

        if (outputHistory.size() > memorySize) {
            outputHistory.remove(0);
        }

        lastError = err;
        lastSlope = slope();
        lastTime = time.seconds();
        lastPredict = predict;

        slopes.add(slope());
        if (slopes.size() > memorySize) {
            slopes.remove(0);
        }

        return output;
    }
}