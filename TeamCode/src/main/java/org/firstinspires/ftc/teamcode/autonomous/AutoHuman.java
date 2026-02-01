package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.Const.ASSIST_DOWN;
import static org.firstinspires.ftc.teamcode.Const.ASSIST_UP;
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
public class AutoHuman extends OpMode {

    private TelemetryManager panelsTelemetry;
    protected final HardwareCoquett robot;
    protected final Alliance alliance;
    private int pathState = 0;
    private Paths paths;

    public static PathChain InitPos;
    public static PathChain GrabPPG;
    public static PathChain ShootPPG;
    public static PathChain GrabHuman;
    public static PathChain BackHuman;
    public static PathChain GrabHuman2;
    public static PathChain ShootHuman;
    public static PathChain LeavePos;
    private final ElapsedTime autoTime = new ElapsedTime();
    private final ElapsedTime stateTime = new ElapsedTime();

    private final ElapsedTime asistenciaDelayTimer = new ElapsedTime();
    private boolean asistenciaDelayActive = false;



    public AutoHuman(Alliance alliance) {
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
        setPathState(0);
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        robot.init(hardwareMap);
        if (alliance == Alliance.BLUE) {
            robot.follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        } else {
            robot.follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));
        }

        paths = new Paths(robot.follower, alliance);
        robot.follower.setMaxPower(1);
        pathState = 0;
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public Paths(Follower follower, Alliance alliance) {

            if (alliance == Alliance.BLUE) {
                // ================== BLUE ALLIANCE ==================
                InitPos = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(56.000, 8.000),

                                        new Pose(68.000, 19.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                        .build();

                GrabPPG = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(68.000, 19.000),
                                        new Pose(54.000, 37.000),
                                        new Pose(12.000, 36.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                        .build();

                ShootPPG = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(12.000, 36.000),

                                        new Pose(68.000, 19.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                        .build();

                GrabHuman = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(68.000, 19.000),

                                        new Pose(10.000, 10.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                        .build();

                BackHuman = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(10.000, 10.000),

                                        new Pose(29.000, 10.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                        .build();

                GrabHuman2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(29.000, 10.000),

                                        new Pose(10.000, 10.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                        .build();

                ShootHuman = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(10.000, 10.000),

                                        new Pose(68.000, 19.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                        .build();
                LeavePos = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(68.000, 19.000),

                                        new Pose(59.000, 31.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                        .build();
            } else {
                // ================== RED ALLIANCE ==================
                InitPos = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(88.000, 8.000),

                                        new Pose(83.000, 18.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                        .build();

                GrabPPG = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(83.000, 18.000),
                                        new Pose(89.000, 38.000),
                                        new Pose(132.000, 35.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                        .build();

                ShootPPG = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(132.000, 35.000),

                                        new Pose(83.000, 13.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                        .build();

                GrabHuman = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(83.000, 13.000),
                                        new Pose(111.000, 14.344),
                                        new Pose(134.000, 9.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                        .build();

                BackHuman = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(134.000, 9.000),

                                        new Pose(115.000, 9.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                GrabHuman2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(115.000, 9.000),

                                        new Pose(134.000, 9.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                ShootHuman = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(134.000, 9.000),
                                        new Pose(105.000, 15.000),
                                        new Pose(83.000, 13.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))

                        .build();

                LeavePos = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(83.000, 13.000),

                                        new Pose(108.000, 18.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                        .build();
            }
        }
    }

//ESTE AUTONOMO AGARRA LAS PELOTAS DEL HUMAN PLAYER, PERO HAY QUE AJUSTAR

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.follower.followPath(InitPos);
                robot.ballStop.setPosition(0);
                robot.ballAlto.setPosition(0.4);
                robot.shooter.aimingLimelight = true;
                robot.turret.aimingLimelight = true;

                if (autoTime.seconds() >= 4.0) {
                    robot.ballUp.setPower(BALL_UP_UP);
                    robot.transferMotor.setPower(-0.8);
                    robot.intakeMotor.setPower(1);
                    if (!robot.isDetected()) {

                        if (!asistenciaDelayActive) {
                            asistenciaDelayActive = true;
                            asistenciaDelayTimer.reset();
                        }

                        if (asistenciaDelayTimer.seconds() >= 3) {
                            robot.asistencia.setPosition(ASSIST_UP);
                            setPathState(1);
                        }
                    } else {
                        asistenciaDelayActive = false;
                    }
                }
                break;

            case 1:
                robot.transferMotor.setPower(0);
                robot.intakeMotor.setPower(0);
                robot.ballStop.setPosition(0.4);
                robot.ballAlto.setPosition(0);
                robot.follower.setMaxPower(1);
                robot.turret.aimingLimelight = false;

                if (!robot.follower.isBusy()) {
                    robot.asistencia.setPosition(ASSIST_DOWN);
                    robot.transferMotor.setPower(-1);
                    robot.intakeMotor.setPower(1);
                    robot.follower.followPath(GrabPPG, true);
                    robot.ballUp.setPower(BALL_UP_DOWN);
                    robot.turret.aimingLimelight = false;

                    double x = robot.follower.getPose().getX();
                    boolean reachedStack = (alliance == Alliance.RED  && x > 131) || (alliance == Alliance.BLUE && x < 13);

                    if (reachedStack) {
                        setPathState(2);
                    }
                }
                break;

            case 2:
                robot.transferMotor.setPower(0);
                robot.intakeMotor.setPower(0);
                robot.ballUp.setPower(0);
                robot.shooter.aimingLimelight = true;
                robot.turret.aimingLimelight = true;
                robot.ballStop.setPosition(0);
                robot.ballAlto.setPosition(0.4);

                if (!robot.follower.isBusy()) {

                    robot.follower.followPath(ShootPPG, false);
                    robot.follower.setMaxPower(1);

                    double x = robot.follower.getPose().getX();
                    boolean backToShooter =
                            (alliance == Alliance.RED  && x < 84) ||
                                    (alliance == Alliance.BLUE && x > 67);

                    if (backToShooter) {

                        robot.ballStop.setPosition(0);
                        robot.ballAlto.setPosition(0.4);  // Open
                        robot.ballUp.setPower(BALL_UP_UP);
                        robot.transferMotor.setPower(-0.8);
                        robot.intakeMotor.setPower(1);

                        if (!robot.isDetected()) {
                            if (!asistenciaDelayActive) {
                                asistenciaDelayActive = true;
                                asistenciaDelayTimer.reset();
                            }
                            if (asistenciaDelayTimer.seconds() >= 4) {
                                robot.asistencia.setPosition(ASSIST_UP);
                                setPathState(3);
                            }
                        } else {
                            asistenciaDelayActive = false;
                        }
                    }
                }
                break;

            case 3:
                robot.transferMotor.setPower(0);
                robot.intakeMotor.setPower(0);
                robot.asistencia.setPosition(ASSIST_DOWN);
                robot.ballStop.setPosition(0.4);
                robot.ballAlto.setPosition(0);
                robot.follower.setMaxPower(1);
                robot.turret.aimingLimelight = false;
                if (!robot.follower.isBusy()) {
                    robot.asistencia.setPosition(ASSIST_DOWN);
                    robot.transferMotor.setPower(-1);
                    robot.intakeMotor.setPower(1);
                    robot.follower.followPath(GrabHuman, true);
                    robot.ballUp.setPower(BALL_UP_DOWN);
                    robot.turret.aimingLimelight = false;


                    double x = robot.follower.getPose().getX();
                    boolean reachedSecondStack =
                            (alliance == Alliance.RED  && x > 133) ||
                                    (alliance == Alliance.BLUE && x < 11);

                    if (reachedSecondStack) {
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if (!robot.follower.isBusy()) {
                    robot.follower.followPath(BackHuman, true);
                    setPathState(5);
                }
                break;

            case 5:
                robot.transferMotor.setPower(0);
                robot.intakeMotor.setPower(0);
                robot.asistencia.setPosition(ASSIST_DOWN);
                robot.ballStop.setPosition(0.4);
                robot.ballAlto.setPosition(0);
                robot.follower.setMaxPower(1);
                robot.turret.aimingLimelight = false;
                if (!robot.follower.isBusy()) {
                    robot.asistencia.setPosition(ASSIST_DOWN);
                    robot.transferMotor.setPower(-1);
                    robot.intakeMotor.setPower(1);
                    robot.follower.followPath(GrabHuman2, true);
                    robot.ballUp.setPower(BALL_UP_DOWN);
                    robot.turret.aimingLimelight = false;


                    double x = robot.follower.getPose().getX();
                    boolean reachedSecondStack =
                            (alliance == Alliance.RED  && x > 133) ||
                                    (alliance == Alliance.BLUE && x < 11);

                    if (reachedSecondStack) {
                        setPathState(6);
                    }
                }
                break;


            case 6:
                robot.transferMotor.setPower(0);
                robot.intakeMotor.setPower(0);
                robot.ballUp.setPower(0);
                robot.shooter.aimingLimelight = true;
                robot.turret.aimingLimelight = true;
                robot.ballStop.setPosition(0);
                robot.ballAlto.setPosition(0.4);

                if (!robot.follower.isBusy()) {

                    robot.follower.followPath(ShootHuman, false);
                    robot.follower.setMaxPower(1);

                    double x = robot.follower.getPose().getX();
                    boolean backToShooter =
                            (alliance == Alliance.RED  && x < 82) ||
                                    (alliance == Alliance.BLUE && x > 67);

                    if (backToShooter) {

                        robot.ballStop.setPosition(0);
                        robot.ballAlto.setPosition(0.4);  // Open
                        robot.ballUp.setPower(BALL_UP_UP);
                        robot.transferMotor.setPower(-0.8);
                        robot.intakeMotor.setPower(1);

                        if (!robot.isDetected()) {
                            if (!asistenciaDelayActive) {
                                asistenciaDelayActive = true;
                                asistenciaDelayTimer.reset();
                            }
                            if (asistenciaDelayTimer.seconds() >= 4) {
                                robot.asistencia.setPosition(ASSIST_UP);
                                setPathState(7);
                            }
                        } else {
                            asistenciaDelayActive = false;
                        }
                    }
                }
                break;

            case 7:
                if (!robot.follower.isBusy()) {
                    robot.follower.followPath(LeavePos, true);
                    setPathState(8);
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
