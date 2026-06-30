package org.firstinspires.ftc.teamcode.Systems.ControlSystem;

import java.util.function.Supplier;

public class Button {
    private Supplier<Boolean> input;
    private boolean last = false;

    public Button(Supplier<Boolean> inp) {
        input = inp;
    }

    public boolean isPressed() {
        return input.get();
    }

    public boolean wasPressed() {
        boolean current = isPressed();
        boolean pressed = current && !last;
        last = current;
        return pressed;
    }

    public boolean wasReleased() {
        boolean current = isPressed();
        boolean released = !current && last;
        last = current;
        return released;
    }
}
