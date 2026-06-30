package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

import static org.firstinspires.ftc.teamcode.TeleOpModes.SimplePath.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.TeleOpModes.SimplePath.follower;
import static org.firstinspires.ftc.teamcode.TeleOpModes.SimplePath.drawCurrent;
import static org.firstinspires.ftc.teamcode.TeleOpModes.SimplePath.pose;
import static org.firstinspires.ftc.teamcode.TeleOpModes.SimplePath.telemetryM;

@Autonomous
public class SimplePath extends SelectableOpMode {
    public static TelemetryManager telemetryM;

    public static Follower follower;
    private Pose currentPose;
    public static PoseHistory poseHistory;

    public SimplePath() {
        super("Select Pre-Made Path", s -> {
            s.folder("Simple Shape", a -> {
                a.add("Line", Line::new);
                a.add("Circle", Circle::new);
                // a.add("Square", Square::new);
            });
            s.folder("Complex Shapes", a -> {
                // a.add("Snake", Snake::new);
                // a.add("8-Shape", EightShape::new);
                a.add("Drift Square", DriftingSquare::new);
            });
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
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void drawCurrentAndHistory() {
        Drawing.drawPoseHistory(poseHistory);
        drawCurrent();
    }

    public static Pose pose(double x, double y) {
        return new Pose(x, y);
    }
};

class Drawing {
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
     * a Follower as an input, so an instance of the DashbaordDrawing class is not needed.
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

class Line extends OpMode {
    private Path forwards, backwards;
    private boolean forward = false;

    @Override
    public void init() {
        follower.setStartingPose(new Pose(72, 72));
    }

    @Override
    public void init_loop() {
        telemetryM.debug("O robô andará para frente e para trás com todos os PIDFs ativados (aproximadamente ~2 tiles).");
        telemetryM.debug("Você pode ajustar os valores do PIDF via Panels.");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }

    @Override
    public void start() {
        follower.activateAllPIDFs();
        forwards = new Path(new BezierLine(new Pose(72,72), new Pose(72 + 72,72)));
        forwards.setConstantHeadingInterpolation(0);
        backwards = new Path(new BezierLine(new Pose(72 + 72,72), new Pose(72,72)));
        backwards.setConstantHeadingInterpolation(0);
        follower.followPath(forwards);
    }

    @Override
    public void loop() {
        follower.update();
        drawCurrentAndHistory();

        if (!follower.isBusy()) {
            if (forward) {
                forward = false;
                follower.followPath(backwards);
            } else {
                forward = true;
                follower.followPath(forwards);
            }
        }
    }
}

class Circle extends OpMode {
    private PathChain circle;
    private double RADIUS = 20;

    @Override
    public void init() {
        follower.activateAllPIDFs();
        follower.setStartingPose(new Pose(67, 67));
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) {
            RADIUS += 5;
        }
        if (gamepad1.bWasPressed()) {
            RADIUS -= 5;
        }

        telemetryM.debug("Caminho circular de raio de aproximadamente ~" + RADIUS + " (em coordenadas do Pedro Pathing ");
        telemetryM.debug("Isso serve como teste de Heading e Centripental, o robô tentará se apontar para o círculo.");
        telemetryM.debug("(aperte A para aumentar o raio, B para diminuir)");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }

    @Override
    public void start() {
        circle = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(67, 67), new Pose(RADIUS + 67, 67), new Pose(RADIUS + 67, RADIUS + 67)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(67, RADIUS + 67))
                .addPath(new BezierCurve(new Pose(RADIUS + 67, RADIUS + 67), new Pose(RADIUS + 67, (2 * RADIUS) + 67), new Pose(67, (2 * RADIUS) + 67)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(67, RADIUS + 67))
                .addPath(new BezierCurve(new Pose(67, (2 * RADIUS) + 67), new Pose(-RADIUS + 67, (2 * RADIUS) + 67), new Pose(-RADIUS + 67, RADIUS + 67)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(67, RADIUS + 67))
                .addPath(new BezierCurve(new Pose(-RADIUS + 67, RADIUS + 67), new Pose(-RADIUS + 67, 67), new Pose(67, 67)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(67, RADIUS + 67))
                .build();
        follower.followPath(circle);
    }

    @Override
    public void loop() {
        follower.update();
        drawCurrentAndHistory();

        if (follower.atParametricEnd()) {
            follower.followPath(circle);
        }
    }
}

/*
class Snake extends OpMode {
    private double LENGTH = 72;
    private double HEIGHT = 36;
    private PathChain snake;

    @Override
    public void init() {
        follower.activateAllPIDFs();
        follower.setStartingPose(new Pose(72, 72));
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) {
            LENGTH += 1;
        }
        if (gamepad1.bWasPressed()) {
            LENGTH -= 1;
        }

        if (gamepad1.xWasPressed()) {
            HEIGHT += 1;
        }
        if (gamepad1.yWasPressed()) {
            HEIGHT -= 1;
        }

        telemetryM.debug("Caminho em formato de 8 com comprimento de " + LENGTH + " e altura " + HEIGHT + " (em coordenadas do Pedro Pathing ");
        telemetryM.debug("Isso serve como teste de Heading, pois o robô fara uma espécie de drift no caminho.");
        telemetryM.debug("(aperte A para aumentar o tamanho, B para diminuir) (aperte X para aumentar a altura, Y para diminuir)");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }

    public void start() {
        snake = follower.pathBuilder()
                .addPath(new BezierLine(pose(72, 72), pose(72 + LENGTH, 72 - HEIGHT)))
                .addPath(new BezierCurve(pose(72 + LENGTH, 72 - HEIGHT), pose(72 + LENGTH + LENGTH / 2, 72), pose(72 + LENGTH, 72 + HEIGHT)))
                .build();
    }
}
 */

class DriftingSquare extends OpMode {
    private PathChain square;
    private double Area = 1296;

    @Override
    public void init() {
        follower.activateAllPIDFs();
        follower.setStartingPose(new Pose(72, 72));
    }

    @Override
    public void init_loop() {
        if (gamepad1.aWasPressed()) {
            Area += 1;
        }
        if (gamepad1.bWasPressed()) {
            Area -= 1;
        }

        telemetryM.debug("Caminho quadrado com área de aproximadamente ~" + Area + " (em coordenadas do Pedro Pathing ");
        telemetryM.debug("Isso serve como teste de Heading e Translational, o robô tentará se apontar para o centro do quadrado.");
        telemetryM.debug("(aperte A para aumentar a área, B para diminuir)");
        telemetryM.update(telemetry);
        follower.update();
        drawCurrent();
    }

    @Override
    public void start() {
        square = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(72, 72), new Pose(72 + Math.sqrt(Area), 72)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(pose(72 + Math.sqrt(Area) / 2, 72 + Math.sqrt(Area) / 2)))
                .addPath(new BezierLine(new Pose(72 + Math.sqrt(Area), 72), new Pose(72 + Math.sqrt(Area), 72 + Math.sqrt(Area))))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(pose(72 + Math.sqrt(Area) / 2, 72 + Math.sqrt(Area) / 2)))
                .addPath(new BezierLine(new Pose(72 + Math.sqrt(Area), 72 + Math.sqrt(Area)), new Pose(72, 72 + Math.sqrt(Area))))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(pose(72 + Math.sqrt(Area) / 2, 72 + Math.sqrt(Area) / 2)))
                .build();
        follower.followPath(square);
    }

    @Override
    public void loop() {
        follower.update();
        drawCurrentAndHistory();

        if (follower.atParametricEnd()) {
            follower.followPath(square);
        }
    }
}

