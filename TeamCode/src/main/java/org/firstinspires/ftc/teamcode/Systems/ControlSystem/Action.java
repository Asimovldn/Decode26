package org.firstinspires.ftc.teamcode.Systems.ControlSystem;

public class Action {
    private Button button;
    private Runnable action;

    public Action(Button button, Runnable action) {
        this.button = button;
        this.action = action;
    }

    public void update() {
        if (button.wasPressed()) {
            action.run();
        }
    }
}
