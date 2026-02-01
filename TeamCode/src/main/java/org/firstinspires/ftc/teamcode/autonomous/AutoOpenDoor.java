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
public class AutoOpenDoor extends OpMode {

    private TelemetryManager panelsTelemetry;
    protected final HardwareCoquett robot;
    protected final Alliance alliance;
    private int pathState = 0;
    private Paths paths;

    public static PathChain InitShoot;
    public static PathChain GrabPGP;
    public static PathChain ShootPGP;
    public static PathChain OpenDoor;
    public static PathChain ShootDoor;
    public static PathChain OpenDoor2;
    public static PathChain ShootDoor2;
    public static PathChain GrabGPP;
    public static PathChain ShootGPP;
    private final ElapsedTime autoTime = new ElapsedTime();
    private final ElapsedTime stateTime = new ElapsedTime();

    private final ElapsedTime asistenciaDelayTimer = new ElapsedTime();
    private boolean asistenciaDelayActive = false;



    public AutoOpenDoor(Alliance alliance) {
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
                InitShoot = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(21.000, 123.000),

                                        new Pose(42.000, 102.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(143))

                        .build();

                GrabPGP = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(42.000, 102.000),
                                        new Pose(78.000, 58.000),
                                        new Pose(11.000, 59.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                        .build();

                ShootPGP = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(11.000, 59.000),
                                        new Pose(34.000, 62.000),
                                        new Pose(49.000, 94.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                        .build();

                OpenDoor = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(49.000, 94.000),

                                        new Pose(12.000, 61.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(120))

                        .build();

                ShootDoor = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(12.000, 61.000),
                                        new Pose(30.000, 73.000),
                                        new Pose(49.000, 94.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(143))

                        .build();

                OpenDoor2 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(49.000, 94.000),

                                        new Pose(12.000, 61.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(120))

                        .build();

                ShootDoor2 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(12.000, 61.000),
                                        new Pose(30.000, 73.000),
                                        new Pose(49.000, 94.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(143))

                        .build();

                GrabGPP = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(49.000, 94.000),
                                        new Pose(50.000, 82.000),
                                        new Pose(15.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(180))

                        .build();

                ShootGPP = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(15.000, 84.000),

                                        new Pose(60.000, 104.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(143))

                        .build();

            } else {
                // ================== RED ALLIANCE ==================
                InitShoot = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(123.000, 123.000),

                                        new Pose(102.000, 102.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(37))

                        .build();

                GrabPGP = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(102.000, 102.000),
                                    new Pose(66.000, 58.000),
                                    new Pose(133.000, 59.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                    .build();

                ShootPGP = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(133.000, 59.000),
                                    new Pose(110.000, 62.000),
                                    new Pose(95.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))

                    .build();

                OpenDoor = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.000, 94.000),
                                    new Pose(132.000, 61.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(60))

                    .build();

                ShootDoor = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(132.000, 61.000),
                                    new Pose(114.000, 73.000),
                                    new Pose(95.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(37))

                    .build();

                OpenDoor2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.000, 94.000),
                                    new Pose(132.000, 61.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(60))

                    .build();

                ShootDoor2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(132.000, 61.000),
                                    new Pose(114.000, 73.000),
                                    new Pose(95.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(37))

                    .build();

                GrabGPP = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(95.000, 94.000),
                                    new Pose(94.000, 82.000),
                                    new Pose(129.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(0))

                    .build();

                ShootGPP = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.000, 84.000),
                                    new Pose(84.000, 104.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(37))

                    .build();
            }
        }
    }

//ESTE AUTONOMO ABRE LA PUERTA, SE SUPONE QUE ES DE 15, PERO HAY QUE AJUSTAR
//A LO MEJOR ESTAN MAL LAS POCISIONES, AGREGLAR ESO
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.follower.followPath(InitShoot);
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
                    robot.follower.followPath(GrabPGP, true);
                    robot.ballUp.setPower(BALL_UP_DOWN);
                    robot.turret.aimingLimelight = false;

                    double x = robot.follower.getPose().getX();
                    boolean reachedStack = (alliance == Alliance.RED  && x > 132) || (alliance == Alliance.BLUE && x < 12);

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

                    robot.follower.followPath(ShootPGP, false);
                    robot.follower.setMaxPower(1);

                    double x = robot.follower.getPose().getX();
                    boolean backToShooter =
                            (alliance == Alliance.RED  && x < 96) ||
                                    (alliance == Alliance.BLUE && x > 48);

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
                    robot.follower.followPath(OpenDoor, true);
                    robot.ballUp.setPower(BALL_UP_DOWN);
                    robot.turret.aimingLimelight = false;


                    double x = robot.follower.getPose().getX();
                    boolean reachedSecondStack =
                            (alliance == Alliance.RED  && x > 131) ||
                                    (alliance == Alliance.BLUE && x < 11);

                    if (reachedSecondStack) {
                        setPathState(4);
                    }
                }
                break;

            case 4:
                robot.transferMotor.setPower(0);
                robot.intakeMotor.setPower(0);
                robot.ballUp.setPower(0);
                robot.shooter.aimingLimelight = true;
                robot.turret.aimingLimelight = true;
                robot.ballStop.setPosition(0);
                robot.ballAlto.setPosition(0.4);

                if (!robot.follower.isBusy()) {

                    robot.follower.followPath(ShootDoor, false);
                    robot.follower.setMaxPower(1);

                    double x = robot.follower.getPose().getX();
                    boolean backToShooter =
                            (alliance == Alliance.RED  && x < 96) ||
                                    (alliance == Alliance.BLUE && x > 48);

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
                                setPathState(5);
                            }
                        } else {
                            asistenciaDelayActive = false;
                        }
                    }
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
                    robot.follower.followPath(OpenDoor2, true);
                    robot.ballUp.setPower(BALL_UP_DOWN);
                    robot.turret.aimingLimelight = false;


                    double x = robot.follower.getPose().getX();
                    boolean reachedSecondStack =
                            (alliance == Alliance.RED  && x > 131) ||
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

                    robot.follower.followPath(ShootDoor2, false);
                    robot.follower.setMaxPower(1);

                    double x = robot.follower.getPose().getX();
                    boolean backToShooter =
                            (alliance == Alliance.RED  && x < 94) ||
                                    (alliance == Alliance.BLUE && x > 48);

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
                    robot.follower.followPath(GrabGPP, true);
                    robot.ballUp.setPower(BALL_UP_DOWN);
                    robot.turret.aimingLimelight = false;


                    double x = robot.follower.getPose().getX();
                    boolean reachedSecondStack =
                            (alliance == Alliance.RED  && x > 128) ||
                                    (alliance == Alliance.BLUE && x < 14);

                    if (reachedSecondStack) {
                        setPathState(8);
                    }
                }
                break;

            case 8:
                robot.transferMotor.setPower(0);
                robot.intakeMotor.setPower(0);
                robot.ballUp.setPower(0);
                robot.shooter.aimingLimelight = true;
                robot.turret.aimingLimelight = true;
                robot.ballStop.setPosition(0);
                robot.ballAlto.setPosition(0.4);

                if (!robot.follower.isBusy()) {

                    robot.follower.followPath(ShootGPP, false);
                    robot.follower.setMaxPower(1);

                    double x = robot.follower.getPose().getX();
                    boolean backToShooter =
                            (alliance == Alliance.RED  && x < 83) ||
                                    (alliance == Alliance.BLUE && x > 59);

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
