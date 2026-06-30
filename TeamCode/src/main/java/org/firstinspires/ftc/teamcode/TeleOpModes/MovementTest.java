package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

import static org.firstinspires.ftc.teamcode.TeleOpModes.MovementTest.drawCurrent;
import static org.firstinspires.ftc.teamcode.TeleOpModes.MovementTest.follower;
import static org.firstinspires.ftc.teamcode.TeleOpModes.MovementTest.telemetryM;

import android.content.Context;
import android.content.Intent;

@TeleOp(name = "Movement", group = "Testing")
public class MovementTest extends SelectableOpMode {
    public static TelemetryManager telemetryM;

    public static Follower follower;
    private Pose currentPose;
    public static PoseHistory poseHistory;

    public MovementTest() {
        super("Select a Test Mode", s -> {
            s.add("Movement With PIDF (Drive Coefficients)", MovementWithPIDF::new);
            s.add("Just Movement (Simple)", MovementSimple::new);
            s.add("Test Movement + Odometry", MovementPlusOdometryTest::new);
            s.add("Slow Movement Test", MovementSlow::new);
            s.add("Field Centric With PID", FieldCentricWithPID::new);
        });
    }

    @Override
    public void onSelect() {
        if (follower == null) {
            follower = Constants.createFollower(hardwareMap);
            PanelsConfigurables.INSTANCE.refreshClass(this);
        } else {
            follower = Constants.createFollower(hardwareMap);
        }

        follower.setStartingPose(new Pose());

        poseHistory = follower.getPoseHistory();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public static void drawCurrent() {
        try {
            DrawingHandler.drawRobot(follower.getPose());
            DrawingHandler.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void drawCurrentAndHistory() {
        DrawingHandler.drawPoseHistory(poseHistory);
        drawCurrent();
    }
};

class DrawingHandler {
    public static final double ROBOT_RADIUS = 9; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {
            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}

class MovementWithPIDF extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose());
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Teste do robô com PIDF (Drive) robô deve andar reto com ajuste de heading do Pedro Pathing.");
        telemetryM.debug("Não utilizar caso o PIDF esteja tunado errado ou tenhamos problemas de motores, caso isso seja um problema, use o MovementSimple.");
        telemetryM.update();
        follower.update();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        follower.startTeleopDrive();
        follower.update();
        drawCurrent();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
        MovementTest.drawCurrentAndHistory();
    }
};

class FieldCentricWithPID extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose());
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Teste do robô com PIDF (Drive) no modo Field Centric, robô deve andar reto com ajuste de heading do Pedro Pathing.");
        telemetryM.debug("Não utilizar caso o PIDF esteja tunado errado ou tenhamos problemas de motores/odometria, caso isso seja um problema, use o MovementSimple.");
        telemetryM.update(telemetry);
        follower.update();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        follower.startTeleopDrive();
        follower.update();
        drawCurrent();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();
        MovementTest.drawCurrentAndHistory();
    }
}

class MovementSimple extends OpMode {

    private boolean useBrakeMode;

    @Override
    public void init() {
        follower.setStartingPose(new Pose());
        useBrakeMode = false;
    }

    @Override
    public void start() {
        follower.deactivateAllPIDFs();
        follower.startTeleopDrive(useBrakeMode);
        follower.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) {
            useBrakeMode = !useBrakeMode;
        }
        telemetryM.debug("Testa o movimento sem nada mais, para testes de motores ou quando o PID está incorreto.");
        telemetryM.debug("Recomenda-se usar esse teste para testes mais simples ou para demonstração de movimento.");
        telemetryM.debug("Brake Mode está: ", useBrakeMode ? "\"ativado\"." : "\"desativado\".", "(aperte A para mudar)");
        telemetryM.update(telemetry);
        follower.update();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();
    }
};

class MovementPlusOdometryTest extends OpMode {

    private Pose position;

    @Override
    public void init() {
        follower.setStartingPose(new Pose());
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Testa o movimento e a odometria do robô, atualiza os valores e mostra eles na telemetria.");
        telemetryM.debug("Também desenha-se o caminho na tela de mapa do Panels.");
        telemetryM.update(telemetry);
        follower.update();
    }

    @Override
    public void start() {
        position = follower.getPose();
        follower.startTeleopDrive();
        follower.update();
        drawCurrent();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();

        position = follower.getPose();

        telemetryM.debug("X:" + position.getX());
        telemetryM.debug("Y:" + position.getY());
        telemetryM.debug("Heading:" + position.getHeading());
        telemetryM.debug("Total Heading:" + follower.getTotalHeading());
        telemetryM.update(telemetry);

        MovementTest.drawCurrentAndHistory();
    }
};

class MovementSlow extends OpMode {
    @Override
    public void init() {
        follower.setStartingPose(new Pose());
    }

    @Override
    public void init_loop() {
        telemetryM.debug("Testa o movimento com a velocidade dos motores reduzida pela metade.");
        telemetryM.debug("Útil para identificar motores defeituosos.");
        telemetryM.update(telemetry);
        follower.update();
    }

    @Override
    public void start() {
        follower.deactivateAllPIDFs();

        double x = follower.getDrivetrain().xVelocity();
        double y = follower.getDrivetrain().yVelocity();

        follower.getDrivetrain().setXVelocity(x / 2);
        follower.getDrivetrain().setYVelocity(y / 2);
        follower.getDrivetrain().setMaxPowerScaling(0.4);

        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        follower.update();
    };
};