package org.firstinspires.ftc.teamcode.autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.HardwareCoquett;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;

@Configurable
public class AutonomousGlobal extends OpMode {

    private TelemetryManager panelsTelemetry;
    protected final HardwareCoquett robot;
    protected final Intake intake;
    protected final Alliance alliance;
    public Follower follower;
    private int pathState;
    private Paths paths;

    public static PathChain InitPos;
    public static PathChain GrabPPG;
    public static PathChain ShootPPG;
    public static PathChain GrabPGP;
    public static PathChain ShootPGP;
    public static PathChain GrabGPP;
    public static PathChain ShootGPP;
    public static PathChain LeavePos;

    public AutonomousGlobal(Alliance alliance) {
        this.alliance = alliance;
        robot = new HardwareCoquett(alliance);
        intake = new Intake(robot);
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        if (alliance == Alliance.RED) {
            follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        } else {
            follower.setStartingPose(new Pose(88, 8, Math.toRadians(90)));
        }

        paths = new Paths(follower, alliance);
        pathState = 0;
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public Paths(Follower follower, Alliance alliance) {

            if (alliance == Alliance.RED) {
                // ================== RED ALLIANCE ==================
                InitPos = follower
                        .pathBuilder()
                        .addPath(new BezierLine(new Pose(56.000, 8.000), new Pose(61.000, 24.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                        .build();

                GrabPPG = follower
                        .pathBuilder()
                        .addPath(new BezierCurve(new Pose(61.000, 24.000), new Pose(63.000, 37.000), new Pose(13.000, 35.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                        .build();

                ShootPPG = follower
                        .pathBuilder()
                        .addPath(new BezierLine(new Pose(13.000, 35.000), new Pose(61.000, 24.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                        .build();

                GrabPGP = follower
                        .pathBuilder()
                        .addPath(new BezierCurve(new Pose(61.000, 24.000), new Pose(63.000, 66.000), new Pose(13.000, 60.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                        .build();

                ShootPGP = follower
                        .pathBuilder()
                        .addPath(new BezierLine(new Pose(13.000, 60.000), new Pose(60.000, 75.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                        .build();

                GrabGPP = follower
                        .pathBuilder()
                        .addPath(new BezierCurve(new Pose(60.000, 75.000), new Pose(48.000, 87.000), new Pose(16.000, 84.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                        .build();

                ShootGPP = follower
                        .pathBuilder()
                        .addPath(new BezierLine(new Pose(16.000, 84.000), new Pose(46.000, 87.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(120))
                        .build();

                LeavePos = follower
                        .pathBuilder()
                        .addPath(new BezierLine(new Pose(46.000, 87.000), new Pose(47.000, 56.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(0))
                        .build();

            } else {
                // ================== BLUE ALLIANCE ==================
                InitPos = follower
                        .pathBuilder()
                        .addPath(new BezierLine(new Pose(88.000, 8.000), new Pose(84.500, 24.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(60))
                        .build();

                GrabPPG = follower
                        .pathBuilder()
                        .addPath(new BezierCurve(new Pose(84.500, 24.000), new Pose(80.000, 37.000), new Pose(130.000, 35.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                        .build();

                ShootPPG = follower
                        .pathBuilder()
                        .addPath(new BezierLine(new Pose(130.000, 35.000), new Pose(84.500, 24.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                        .build();

                GrabPGP = follower
                        .pathBuilder()
                        .addPath(new BezierCurve(new Pose(84.500, 24.000), new Pose(72.000, 62.000), new Pose(130.000, 60.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                        .build();

                ShootPGP = follower
                        .pathBuilder()
                        .addPath(new BezierLine(new Pose(130.000, 60.000), new Pose(87.000, 75.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                        .build();

                GrabGPP = follower
                        .pathBuilder()
                        .addPath(new BezierCurve(new Pose(87.000, 75.000), new Pose(80.000, 86.000), new Pose(131.000, 83.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(0))
                        .build();

                ShootGPP = follower
                        .pathBuilder()
                        .addPath(new BezierLine(new Pose(131.000, 83.000), new Pose(87.000, 74.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                        .build();

                LeavePos = follower
                        .pathBuilder()
                        .addPath(new BezierLine(new Pose(87.000, 74.000), new Pose(85.000, 54.000)))
                        .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(180))
                        .build();
            }
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(InitPos);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(GrabPPG, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(ShootPPG, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(GrabPGP, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(ShootPGP, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(GrabGPP, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(ShootGPP, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(LeavePos, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                break;
        }
        return pathState;
    }

    public void setPathState(int pState) {
        pathState = pState;
    }
}
