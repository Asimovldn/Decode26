package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Systems.ControlSystem.Action;
import org.firstinspires.ftc.teamcode.Systems.ControlSystem.Button;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class ControlHandler {

    private final List<Action> actions = new ArrayList<>();
    private final HashMap<Buttons, Button> buttons = new HashMap<>();

    public ControlHandler(HardwareMap hm, Gamepad gamepad) {
        buttons.put(Buttons.A, new Button(() -> gamepad.a));
        buttons.put(Buttons.B, new Button(() -> gamepad.b));
        buttons.put(Buttons.X, new Button(() -> gamepad.x));
        buttons.put(Buttons.Y, new Button(() -> gamepad.y));
        buttons.put(Buttons.RIGHTBUMPER, new Button(() -> gamepad.right_bumper));
        buttons.put(Buttons.RIGHTTRIGGER, new Button(() -> gamepad.right_trigger_pressed));
        buttons.put(Buttons.LEFTBUMPER, new Button(() -> gamepad.left_bumper));
        buttons.put(Buttons.LEFTTRIGGER, new Button(() -> gamepad.left_trigger_pressed));
        buttons.put(Buttons.RIGHTSTICK, new Button(() -> gamepad.right_stick_button));
        buttons.put(Buttons.LEFTSTICK, new Button(() -> gamepad.left_stick_button));
    }

    public enum Buttons {
        A,
        B,
        X,
        Y,

        RIGHTBUMPER,
        LEFTBUMPER,

        RIGHTTRIGGER,
        LEFTTRIGGER,

        RIGHTSTICK,
        LEFTSTICK
    }

    public Button grab(Buttons btn) {
        return buttons.get(btn);
    }

    public void Add(Buttons btn, Runnable action) {
        actions.add(new Action(grab(btn), action));
    }

    public void update() {
        for (Action action : actions) {
            action.update();
        }
    }
}