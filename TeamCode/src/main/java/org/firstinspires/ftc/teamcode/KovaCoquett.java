package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public abstract class KovaCoquett extends OpMode {

    // ------------------- ROBOT & SUBSYSTEMS -------------------
    protected final HardwareCoquett robot;
    protected final Intake intake;

    private static final double RGB_OFF    = 0.0;
    private static final double RGB_PINK = 0.85;
    boolean asisted;
    boolean asistedDown;
    boolean closed;
    private final ElapsedTime asistedTimer = new ElapsedTime();
    private final ElapsedTime closedTimer = new ElapsedTime();

    // --------------------------------------------------------------

    public KovaCoquett(Alliance alliance) {
        robot = new HardwareCoquett(alliance);
        intake = new Intake(robot);
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        // Show pink while in INIT
        if (robot.light != null) {
            robot.light.setPosition(RGB_PINK);
        }
    }

    @Override
    public void start() {
        robot.follower.startTeleOpDrive(true);
    }

    @Override
    public void loop() {
        boolean stateHigh = robot.laserInput.getState();
        boolean detected = stateHigh;
        // ------------------- DRIVE -------------------
        if (!gamepad1.b) {
            robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 0.5,
                    -gamepad1.left_stick_x * 0.5,
                    -gamepad1.right_stick_x * 0.5,
                    false
            );
            telemetry.addData("Modo lento", "Activado");
            robot.light.setPosition(0.277);
        } else {
            robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
            telemetry.addData("Modo lento", "Desactivado");
            robot.light.setPosition(0.5);
        }

        if (gamepad1.dpadUpWasPressed()) {
            robot.follower.setPose(new Pose());
        }
        if(robot.asistencia.getPosition()== 1){
            robot.light.setPosition(0.722);
        } else if (robot.asistencia.getPosition()==0.5) {
            robot.light.setPosition(0.5);
        }

        // ------------------- INTAKE  -------------------
        Intake.Command intakeCmd = intake.newCommand();
        intakeCmd.autoShoot             = gamepad2.a;
        intakeCmd.autoShootJustPressed  = gamepad2.aWasPressed();
        intakeCmd.autoShootFeedOverride = gamepad2.right_bumper;
        intakeCmd.manualIntakeForward = gamepad1.b;
        intakeCmd.manualIntakeReverse = gamepad1.a;
        intakeCmd.manualIndexerUp     = gamepad2.dpad_up;
        intakeCmd.autoBlockEnabled = (!gamepad2.x || !gamepad2.left_bumper);
        intakeCmd.shooterClearing = gamepad1.left_bumper;

        intake.update(intakeCmd);

        if (gamepad2.a) {
            robot.ballStop.setPosition(0.3);
            closed = false;
            closedTimer.reset();
        }else if (gamepad2.left_bumper) {
            robot.ballStop.setPosition(0.3);
            closed = false;
            closedTimer.reset();
        }else if (gamepad1.a) {
            robot.ballStop.setPosition(0.1);
            closed = true;
        }else if (!closed && asistedDown && closedTimer.seconds() > 0.5) {
            robot.ballStop.setPosition(0.1);
            closed = true;
        }

        if(gamepad1.a) {
            robot.noStuck.setPower(1);
        } else if (gamepad1.b) {
            robot.noStuck.setPower(-1);
        }else {
            robot.noStuck.setPower(0);
        }

        if (gamepad2.a) {
            robot.asistencia.setPosition(0.5);
            asisted = false;
        }

        if(gamepad2.dpad_right) {
            robot.asistencia.setPosition(1);
            asistedTimer.reset();
            asisted = true;
            asistedDown = false;
        }
        if(asisted && asistedTimer.seconds() > 0.25){
            robot.asistencia.setPosition(0.5);
            asisted = false;
            asistedDown = true;
        }
        if(gamepad2.dpad_left){
            robot.asistencia.setPosition(1);
            asisted = false;
            asistedDown = true;
        }

        if(gamepad2.a){
            robot.ballUp.setPower(0.8);
        }else {
            robot.ballUp.setPower(0);
        }

        // ------------------- SHOOTER -------------------
        if (gamepad2.left_trigger >= 0.3) {
            robot.shooter.shooterTargetVelocity = 1500;

            robot.shooter.aimingLimelight = false;
            robot.turret.aimingLimelight  = false;

        } else if (gamepad2.left_bumper) {
            robot.shooter.shooterTargetVelocity = -1500;

            robot.shooter.aimingLimelight = false;
            robot.turret.aimingLimelight  = false;

        } else if (gamepad2.right_trigger >= 0.3) {
            robot.shooter.aimingLimelight = true;
            robot.turret.aimingLimelight  = true;
        } else {
            robot.shooter.aimingLimelight = false;
            robot.turret.aimingLimelight  = false;

            robot.shooter.shooterTargetVelocity = 0;
        }

        // ------------------- TURRET -------------------
        if (gamepad1.y) {
            robot.turret.aimingLimelight = true;
        } else if (gamepad2.right_trigger < 0.3) {
            robot.turret.aimingLimelight = false;
            robot.torrettCoquette.setPower(-gamepad2.left_stick_x);
        }


        // ------------------- LIFT (LIBRO / SUBEBAJA) -------------------
        if (gamepad1.dpad_up) {
            robot.subiBajaMotor.setPower(0.5);
        } else if (gamepad1.dpad_down) {
            robot.subiBajaMotor.setPower(-1.0);
        } else {
            robot.subiBajaMotor.setPower(0.0);
        }

        // ------------------- ROBOT -------------------
        robot.update();
        // ------------------- TELEMETRY -------------------
        telemetry.addData("Sube bolas", intake.getIndexerPosition());
        telemetry.addData("Para bolas", intake.getGatePosition());
        telemetry.addData("Ángulo", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("Asisted", asisted);

        telemetry.update();
    }


}
