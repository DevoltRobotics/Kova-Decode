package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Const.ASSIST_DOWN;
import static org.firstinspires.ftc.teamcode.Const.ASSIST_UP;
import static org.firstinspires.ftc.teamcode.Const.BALL_ALTO_CLOSE;
import static org.firstinspires.ftc.teamcode.Const.BALL_ALTO_OPEN;
import static org.firstinspires.ftc.teamcode.Const.BALL_STOP_CLOSE;
import static org.firstinspires.ftc.teamcode.Const.BALL_STOP_OPEN;
import static org.firstinspires.ftc.teamcode.Const.BALL_UP_DOWN;
import static org.firstinspires.ftc.teamcode.Const.BALL_UP_UP;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.HardwareCoquett;

@Configurable
public class AutonomousClose extends OpMode {

    private TelemetryManager panelsTelemetry;
    protected final HardwareCoquett robot;
    protected final Alliance alliance;
    private int pathState;
    private Paths paths;

    public static PathChain InitShoot;
    public static PathChain GrabPPG;
    public static PathChain ShootPPG;
    public static PathChain GrabPGP;
    public static PathChain ShootPGP;
    public static PathChain GrabGPP;
    public static PathChain ShootGPP;
    public static PathChain Park;
    boolean pathActivation;
    private final ElapsedTime autoTime = new ElapsedTime();
    private final ElapsedTime stateTime = new ElapsedTime();

    private final ElapsedTime asistenciaDelayTimer = new ElapsedTime();
    private boolean asistenciaDelayActive = false;



    public AutonomousClose(Alliance alliance) {
        this.alliance = alliance;
        robot = new HardwareCoquett(alliance);
    }

    @Override
    public void init_loop() {
        robot.update();
        robot.turret.aimingLimelight  = true;
        panelsTelemetry.debug("Init Path Running", robot.follower.isBusy());
        panelsTelemetry.update(telemetry);
    }


    @Override
    public void start() {
        pathState = 0;
        autoTime.reset();
        robot.shooter.aimingLimelight = true;
        robot.turret.aimingLimelight = true;
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        robot.init(hardwareMap);
        if (alliance == Alliance.BLUE) {
            robot.follower.setStartingPose(new Pose(20.5, 123, Math.toRadians(144)));
        } else {
            robot.follower.setStartingPose(new Pose(123, 124, Math.toRadians(38)));
        }

        paths = new Paths(robot.follower, alliance);
        robot.follower.setMaxPower(1);
        pathState = 0;
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        robot.ballStop.setPosition(BALL_STOP_OPEN);
        robot.ballAlto.setPosition(BALL_ALTO_OPEN);
    }

    @Override
    public void loop() {
        robot.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", robot.follower.getPose().getX());
        panelsTelemetry.debug("Y", robot.follower.getPose().getY());
        panelsTelemetry.debug("Heading", robot.follower.getPose().getHeading());
        panelsTelemetry.debug("State time", stateTime.seconds());
        panelsTelemetry.debug("Is Busy", robot.follower.isBusy());
            panelsTelemetry.debug("Path", robot.follower.getCurrentPath());
        panelsTelemetry.update(telemetry);
        if(autoTime.seconds()>=29.5){
            setPathState(7);
        }
    }

    public static class Paths {
        public Paths(Follower follower, Alliance alliance) {

            if (alliance == Alliance.BLUE) {
                // ================== BLUE ALLIANCE ==================
                InitShoot = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(20.500, 124.000), //Punto Inicio
                                        new Pose(50.000, 93.000)) //Punto Final
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(144))
                        .build();

                GrabPPG = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(50.000, 93.000), //Punto Inicial de la curva, es decir, el punto final del path pasado
                                        new Pose(55.000, 80.000), //Punto de control
                                        new Pose(19.000, 84.000) //Punto final de la curva
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                        .build();

                ShootPPG = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(
                                        new Pose(19.000, 84.000),
                                        new Pose(60.000, 83.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                        .build();

                GrabPGP = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(60.000, 83.000),
                                        new Pose(62.000, 58.000),
                                        new Pose(10.000, 57.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                        .build();

                ShootPGP = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(10.000, 57.000),
                                        new Pose(20.000, 52.000),
                                        new Pose(60.000, 83.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                        .build();

                GrabGPP = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(60.000, 83.000),
                                        new Pose(72.000, 28.000),
                                        new Pose(8.000, 35.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                        .build();

                ShootGPP = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(8.000, 35.000), new Pose(60.000, 83.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(144))
                        .build();

                Park = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(60.000, 83.000), new Pose(35.000, 78.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(144))
                        .build();

            } else {
                // ================== RED ALLIANCE ==================
                InitShoot = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(123.000, 124.000), new Pose(94.000, 93.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(38))
                        .build();

                GrabPPG = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(94.000, 93.000),
                                        new Pose(94.000, 80.000),
                                        new Pose(129.000, 86.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0),0.5)
                        .build();

                ShootPPG = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(129.000, 84.000), new Pose(83.000, 82.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(38))
                        .build();

                GrabPGP = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(83.000, 82.000),
                                        new Pose(76.000, 58.000),
                                        new Pose(136.000, 56.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0),0.5)
                        .build();

                ShootPGP = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(136.000, 58.000),
                                        new Pose(109.000, 56.000),
                                        new Pose(83.000, 82.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(38))
                        .build();

                GrabGPP = follower
                        .pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(83.000, 82.000),
                                        new Pose(72.000, 30.000),
                                        new Pose(135.000, 38.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0),0.5)
                        .build();

                ShootGPP = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(135.000, 36.000), new Pose(83.000, 82.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(38))
                        .build();

                Park = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(83.000, 82.000), new Pose(110.000, 79.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(38))
                        .build();
            }
        }
    }
//FALTA AJUSTAR CON EL NUEVO SERVO DE ALTO PELOTAS
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.follower.setMaxPower(1);
                robot.ballStop.setPosition(BALL_STOP_OPEN);
                robot.ballAlto.setPosition(BALL_ALTO_OPEN);
                robot.ballStop.setPosition(0.1);
                robot.ballAlto.setPosition(0.6);

                if(!pathActivation){
                    robot.follower.followPath(InitShoot);
                    pathActivation = true;
                }
                if (autoTime.seconds() >= 2.5) {
                    robot.ballUp.setPower(1);
                    robot.transferMotor.setPower(-0.6);
                    robot.intakeMotor.setPower(1);

                    if (!robot.isBallDetected()) {

                        if (!asistenciaDelayActive) {
                            asistenciaDelayActive = true;
                            asistenciaDelayTimer.reset();
                        }

                        if (asistenciaDelayTimer.seconds() >= 1.5) {
                            robot.asistencia.setPosition(ASSIST_UP);

                        }

                        if (asistenciaDelayTimer.seconds() >= 2) {
                            pathActivation = false;
                            setPathState(1);

                            robot.ballStop.setPosition(BALL_STOP_CLOSE);
                            robot.ballAlto.setPosition(BALL_ALTO_CLOSE);
                            robot.asistencia.setPosition(ASSIST_DOWN);
                        }
                    } else {
                        asistenciaDelayActive = false;
                    }
                }
                break;

            case 1:
                robot.turret.aimingLimelight = false;
                robot.follower.setMaxPower(1);

                robot.ballStop.setPosition(BALL_STOP_CLOSE);
                robot.ballAlto.setPosition(BALL_ALTO_CLOSE);

                if(!pathActivation){
                    robot.follower.followPath(GrabPPG, true);
                    pathActivation = true;
                }
                if (!robot.follower.isBusy()) {
                    robot.asistencia.setPosition(ASSIST_DOWN);
                    robot.ballUp.setPower(-1);
                    robot.turret.aimingLimelight = false;
                    robot.transferMotor.setPower(-0.8);
                    robot.intakeMotor.setPower(0.5);

                    double x = robot.follower.getPose().getX();
                    if ((alliance == Alliance.RED  && x > 122) || (alliance == Alliance.BLUE && x < 22)) {
                        pathActivation = false;
                        setPathState(2);
                    }
                }
                break;

            case 2:
                robot.shooter.aimingLimelight = true;
                robot.turret.aimingLimelight = true;

                if (!robot.follower.isBusy()) {

                    robot.transferMotor.setPower(0);
                    robot.intakeMotor.setPower(0);
                    robot.follower.followPath(ShootPPG, false);
                    robot.ballStop.setPosition(BALL_STOP_OPEN);
                    robot.ballAlto.setPosition(BALL_ALTO_OPEN);// Open
                    robot.follower.setMaxPower(1);

                    double x = robot.follower.getPose().getX();

                    if ((alliance == Alliance.RED  && x < 86) || (alliance == Alliance.BLUE && x > 58)) {


                        robot.ballUp.setPower(0.8);
                        robot.transferMotor.setPower(-0.6);
                        robot.intakeMotor.setPower(1);

                        if (!robot.isBallDetected()) {
                            if (!asistenciaDelayActive) {
                                asistenciaDelayActive = true;
                                asistenciaDelayTimer.reset();
                            }
                            if (asistenciaDelayTimer.seconds() >= 1.5) {
                                robot.asistencia.setPosition(1);

                            }
                            if (asistenciaDelayTimer.seconds() >= 3) {
                                robot.ballStop.setPosition(BALL_STOP_CLOSE);
                                robot.ballAlto.setPosition(BALL_ALTO_CLOSE);
                                robot.asistencia.setPosition(ASSIST_DOWN);
                                setPathState(3);
                            }
                        } else {
                            asistenciaDelayActive = false;
                        }
                    }
                }
                break;
            case 3:
                robot.turret.aimingLimelight = false;
                robot.follower.setMaxPower(1);

                robot.ballStop.setPosition(BALL_STOP_CLOSE);
                robot.ballAlto.setPosition(BALL_ALTO_CLOSE);

                if(!pathActivation){
                    robot.follower.followPath(GrabPGP, true);
                    pathActivation = true;
                }
                if (!robot.follower.isBusy()) {
                    robot.asistencia.setPosition(ASSIST_DOWN);
                    robot.ballUp.setPower(-1);
                    robot.turret.aimingLimelight = false;
                    robot.transferMotor.setPower(-0.8);
                    robot.intakeMotor.setPower(0.5);

                    double x = robot.follower.getPose().getX();
                    if ((alliance == Alliance.RED  && x > 131) || (alliance == Alliance.BLUE && x < 17)) {
                        pathActivation = false;
                        setPathState(4);
                    }
                }
                break;


            case 4:
                robot.shooter.aimingLimelight = true;
                robot.turret.aimingLimelight = true;

                if (!robot.follower.isBusy()) {

                    robot.transferMotor.setPower(0);
                    robot.intakeMotor.setPower(0);
                    robot.follower.followPath(ShootPGP, false);
                    robot.ballStop.setPosition(BALL_STOP_OPEN);
                    robot.ballAlto.setPosition(BALL_ALTO_OPEN);// Open
                    robot.follower.setMaxPower(1);

                    double x = robot.follower.getPose().getX();

                    if ((alliance == Alliance.RED  && x < 84) || (alliance == Alliance.BLUE && x > 58)) {


                        robot.ballUp.setPower(0.8);
                        robot.transferMotor.setPower(-0.6);
                        robot.intakeMotor.setPower(1);

                        if (!robot.isBallDetected()) {
                            if (!asistenciaDelayActive) {
                                asistenciaDelayActive = true;
                                asistenciaDelayTimer.reset();
                            }
                            if (asistenciaDelayTimer.seconds() >= 1.5) {
                                robot.asistencia.setPosition(1);

                            }
                            if (asistenciaDelayTimer.seconds() >= 3) {
                                robot.ballStop.setPosition(BALL_STOP_CLOSE);
                                robot.ballAlto.setPosition(BALL_ALTO_CLOSE);
                                robot.asistencia.setPosition(ASSIST_DOWN);
                                setPathState(5);
                            }
                        } else {
                            asistenciaDelayActive = false;
                        }
                    }
                }
                break;
                case 5:
                robot.turret.aimingLimelight = false;
                robot.follower.setMaxPower(1);

                if(!pathActivation){
                    robot.follower.followPath(GrabGPP, true);
                    pathActivation = true;
                }

                robot.ballStop.setPosition(BALL_STOP_CLOSE);
                robot.ballAlto.setPosition(BALL_ALTO_CLOSE);

                if (!robot.follower.isBusy()) {
                    robot.asistencia.setPosition(ASSIST_DOWN);
                    robot.ballUp.setPower(-1);
                    robot.turret.aimingLimelight = false;
                    robot.transferMotor.setPower(-0.8);
                    robot.intakeMotor.setPower(0.5);

                    double x = robot.follower.getPose().getX();
                    if ((alliance == Alliance.RED  && x > 131) || (alliance == Alliance.BLUE && x < 12)) {
                        pathActivation = false;
                        setPathState(6);
                    }
                }
                break;
            case 6:
                robot.shooter.aimingLimelight = true;
                robot.turret.aimingLimelight = true;

                if (!robot.follower.isBusy()) {

                    robot.transferMotor.setPower(0);
                    robot.intakeMotor.setPower(0);
                    robot.follower.followPath(ShootGPP, false);
                    robot.ballStop.setPosition(BALL_STOP_OPEN);
                    robot.ballAlto.setPosition(BALL_ALTO_OPEN);// Open
                    robot.follower.setMaxPower(1);

                    double x = robot.follower.getPose().getX();

                    if ((alliance == Alliance.RED  && x < 84) || (alliance == Alliance.BLUE && x > 58)) {


                        robot.ballUp.setPower(0.8);
                        robot.transferMotor.setPower(-0.6);
                        robot.intakeMotor.setPower(1);

                        if (!robot.isBallDetected()) {
                            if (!asistenciaDelayActive) {
                                asistenciaDelayActive = true;
                                asistenciaDelayTimer.reset();
                            }
                            if (asistenciaDelayTimer.seconds() >= 1.5) {
                                robot.asistencia.setPosition(1);

                            }
                            if (asistenciaDelayTimer.seconds() >= 2) {
                                robot.ballStop.setPosition(BALL_STOP_CLOSE);
                                robot.ballAlto.setPosition(BALL_ALTO_CLOSE);
                                robot.asistencia.setPosition(ASSIST_DOWN);
                                setPathState(7);
                            }
                        } else {
                            asistenciaDelayActive = false;
                        }
                    }
                }
                break;
            case 7:
                if(!pathActivation){
                    robot.follower.followPath(Park, true);
                    pathActivation=true;
                }
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        if(pState != pathState){
            stateTime.reset();
        }
        pathState = pState;
    }
}
