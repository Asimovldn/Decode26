package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Systems.ControlSystem.Action;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;

import javax.lang.model.type.UnionType;

public class ControlHandler {

    private final List<Action> actions = new ArrayList<>();
    private final HashMap<Buttons, Button> buttons = new HashMap<>();
    private final HashMap<Axises, Axis> axis = new HashMap<>();

    public static HashMap<Flags, Boolean> rules = new HashMap<>();

    private Gamepad gamepad;

    public enum Controller {
        LOGITECH_F310,
        PS4
    }

    interface EnumReference {};

    public static class Input<T> {
        private Supplier<T> supplier;

        private EnumReference enumRef;

        public Input(Supplier<T> supplier, EnumReference en) {
            this.supplier = supplier;
            enumRef = en;
        }

        public T get() {
            return supplier.get();
        }

        public boolean wasPressed() {
            return get() != null;
        }

        public boolean wasReleased() {
            return get() != null;
        }

        /**
        Returns current {@code reading()} of the input in the {@code [0, 1]} range.
         For buttons: {@code 0 == false}, {@code 1 == true}
         For axis: values between {@code [0, 1]}.
         */
        public double reading() {
            if (get() instanceof Boolean) {
                return (boolean) get() ? 1 : 0;
            }

            return (double) get();
        }

        public EnumReference asEnum() {
            return enumRef;
        }
    }

    public static class Button extends Input<Boolean> {

        private boolean last;

        public Button(Supplier<Boolean> input, EnumReference en) {
            super(input, en);
        }

        public static Button fromInput(Input inp) {
            return new Button(inp.supplier, inp.asEnum());
        }

        public boolean wasPressed() {
            boolean current = get();
            boolean result = current && !last;
            last = current;
            return result;
        }

        public boolean wasReleased() {
            boolean current = get();
            boolean result = !current && last;
            last = current;
            return result;
        }

        @Override
        public double reading() {
            return get() ? 1 : 0;
        }
    }

    public static class Axis extends Input<Double> {

        private double last;

        public Double get() {
            if (Boolean.TRUE.equals(rules.get(Flags.UTILIZE_EXPONENTIAL))) {
                return Math.pow(super.supplier.get(), 2) /
                        (Math.pow(super.supplier.get(), 2) + Math.pow(1 - super.supplier.get(), 2));
            }

            return super.supplier.get();
        }

        public Axis(Supplier<Double> input, EnumReference en) {
            super(input, en);
        }

        public static Axis fromInput(Input inp) {
            return new Axis(inp.supplier, inp.asEnum());
        }

        public boolean wasPressed() {
            double current = get();
            boolean result = current > 0 && !(last > 0.1);
            last = current;
            return result;
        }

        public boolean wasReleased() {
            double current = get();
            boolean result = !(current > 0.1) && last > 0;
            last = current;
            return result;
        }

        @Override
        public double reading() {
            return get();
        }
    }

    public ControlHandler(HardwareMap hm, Gamepad gamepad) {
        this.gamepad = gamepad;

        create(Buttons.A, () -> gamepad.a);
        create(Buttons.B, () -> gamepad.b);
        create(Buttons.X, () -> gamepad.x);
        create(Buttons.Y, () -> gamepad.y);

        create(Buttons.RIGHTBUMPER, () -> gamepad.right_bumper);
        create(Buttons.LEFTBUMPER, () -> gamepad.left_bumper);

        create(Axises.RIGHTTRIGGER, () -> (double) gamepad.right_trigger);
        create(Axises.LEFTTRIGGER, () -> (double) gamepad.left_trigger);

        create(Axises.LEFTSTICK_Y, () -> (double) gamepad.left_stick_y);
        create(Axises.LEFTSTICK_X, () -> (double) gamepad.left_stick_x);
        create(Axises.RIGHTSTICK_Y, () -> (double) gamepad.left_stick_y);
        create(Axises.RIGHTSTICK_X, () -> (double) gamepad.left_stick_x);

        create(Buttons.RIGHTSTICK, () -> gamepad.right_stick_button);
        create(Buttons.LEFTSTICK, () -> gamepad.left_stick_button);

        create(Buttons.START, () -> gamepad.start);

        rules.put(Flags.UTILIZE_EXPONENTIAL, false);
    }

    public enum Buttons implements EnumReference {
        A,
        B,
        X,
        Y,

        RIGHTBUMPER,
        LEFTBUMPER,

        RIGHTSTICK,
        LEFTSTICK,

        START

    }

    public enum Axises implements EnumReference {
        RIGHTTRIGGER,
        LEFTTRIGGER,

        RIGHTSTICK_Y,
        RIGHTSTICK_X,
        LEFTSTICK_Y,
        LEFTSTICK_X
    }

    public enum Flags {
        UTILIZE_EXPONENTIAL
    }

    public void consider(Flags f, boolean state) {
        rules.replace(f, state);
    }

    public void create(Buttons btn, Supplier<Boolean> getter) {
        buttons.put(btn, new Button(getter, btn));
    }

    public void create(Axises ax, Supplier<Double> getter) {
        axis.put(ax, new Axis(getter, ax));
    }

    public Button grab(Buttons btn) {
        return buttons.get(btn);
    }

    public Axis grab(Axises ax) {
        return axis.get(ax);
    }

    public Action Add(Buttons btn, Runnable action) {
        Action act = new Action(grab(btn), action);
        actions.add(act);
        return act;
    }
    public Action Add(Axises ax, Runnable action) {
        Action act = new Action(grab(ax), action);
        actions.add(act);
        return act;
    }

    public void vibrate(int ms) {
        gamepad.rumble(ms);
    }

    public void update() {
        for (Action action : actions) {
            action.update();
        }
    }

    public double read(Buttons btn) {
        for (Action action : actions) {
            if (action.grab().asEnum() == btn) {
                return action.grab().reading();
            }
        }

        return 0;
    }

    public double read(Axises ax) {
        for (Action action : actions) {
            if (action.grab().asEnum() == ax) {
                return action.grab().reading();
            }
        }

        return 0;
    }
}