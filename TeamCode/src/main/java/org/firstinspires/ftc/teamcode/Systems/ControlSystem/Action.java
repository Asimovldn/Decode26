package org.firstinspires.ftc.teamcode.Systems.ControlSystem;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.Systems.ControlHandler;

import java.util.function.Function;

public class Action {
    private ControlHandler.Input button;
    private Runnable action;

    private boolean onCalled, onRel, rumble;

    private double biggerThan;

    @Nullable
    private Runnable followup;

    public Action(ControlHandler.Axis button, Runnable action) {
        this.button = button;
        this.action = action;

        defaults();
    }

    public Action(ControlHandler.Button grab, Runnable action) {
        this.button = grab;
        this.action = action;

        defaults();
    }

    public ControlHandler.Input grab() {
        return button;
    }

    public void defaults() {
        onCalled = true;
        onRel = false;
        rumble = false;
        followup = null;
    }

    /*
     Consider for buttons, recommended for buttons.
     */
    public Action onPressed() {
        onCalled = true;
        return this;
    }

    /*
     Recommended for axises.
     */
    public Action onMoved() {
        onCalled = true; return this;
    }

    public Action onReleased() {
        onRel = true; return this;
    }

    /*
        Activate once bigger then X, where x must fit
        0 < x < 1;
     */
    public Action onBiggerThan(double x) {
        assert 0 < x || x < 1 : "x does not fit 0 < x < 1";
        biggerThan = x;
        return this;
    };

    public Action thenVibrate() { rumble = true; return this; };

    public void andThen(Runnable then) {
        followup = then;
    }

    public void update() {
        if (onCalled && button.wasPressed()) {
            action.run();
            if (followup != null) {
                followup.run();
            }
        }

        if (onRel && button.wasReleased()) {
            action.run();
            if (followup != null) {
                followup.run();
            }
        }

        if (biggerThan > 0 && button.reading() > biggerThan) {
            action.run();
            if (followup != null) {
                followup.run();
            }
        }
    }

    public double reading() {
        return button.reading();
    }
}
