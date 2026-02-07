package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Const.ASSIST_DOWN;
import static org.firstinspires.ftc.teamcode.Const.ASSIST_UP;
import static org.firstinspires.ftc.teamcode.Const.BALL_UP_DOWN;
import static org.firstinspires.ftc.teamcode.Const.BALL_UP_UP;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Intake;

public abstract class KovaCoquett extends OpMode {

    // ------------------- ROBOT & SUBSYSTEMS -------------------
    protected final HardwareCoquett robot;
    boolean asisted;
    boolean asistedDown;
    boolean closed;
    boolean intakeOn;
    boolean intakeDelayActive = false;
    boolean intakeRunning = false;
    private final ElapsedTime intakeTimer = new ElapsedTime();
    private final ElapsedTime asistedTimer = new ElapsedTime();
    private final ElapsedTime closedTimer = new ElapsedTime();
    private final ElapsedTime matchTimer = new ElapsedTime();
    // --------------------------------------------------------------

    public KovaCoquett(Alliance alliance) {
        robot = new HardwareCoquett(alliance);
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.asistencia.setPosition(0);
        robot.luz.setPosition(0.722);
    }

    @Override
    public void start() {
        robot.follower.startTeleOpDrive(true);
        matchTimer.reset();
    }

    @Override
    public void loop() {
        // ------------------- DRIVE -------------------
        if (!gamepad1.b) {
            robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 0.5,
                    -gamepad1.left_stick_x * 0.5,
                    -gamepad1.right_stick_x * 0.5,
                    false
            );
            telemetry.addData("Modo lento", "Activado");
        } else {
            robot.follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false
            );
            telemetry.addData("Modo lento", "Desactivado");
        }

        if (gamepad1.dpadUpWasPressed()) {
            robot.follower.setPose(new Pose());
        }

        // ------------------- INTAKE  -------------------
        Intake.Command intakeCmd = robot.intake.newCommand();
        intakeCmd.autoShoot = gamepad2.a;
        intakeCmd.autoShootJustPressed = gamepad2.aWasPressed();
        intakeCmd.autoShootFeedOverride = gamepad2.right_bumper;
        intakeCmd.manualIntakeForward = gamepad1.right_bumper; //Out
        intakeCmd.manualIntakeReverse = gamepad1.left_bumper; //In
        intakeCmd.manualIndexerUp = gamepad2.dpad_up;
        intakeCmd.autoBlockEnabled = (!gamepad2.a || !gamepad2.left_bumper);
        intakeCmd.shooterClearing = gamepad1.left_trigger > 0.1;

        robot.intake.update(intakeCmd);

        robot.luz.setPosition(0.722);

        if (gamepad2.a) {
            robot.transferMotor.setPower(-0.8);
            robot.intakeMotor.setPower(1.0);
        }
//CAMBIAR POSISIONES DE X^2
        if (gamepad2.right_trigger > 0.1) {
            intakeTimer.reset();
            robot.ballAlto.setPosition(0.7);
            robot.ballStop.setPosition(0.1); //open
            intakeOn = true;
            closed = false;
            closedTimer.reset();
        } else if (gamepad2.left_bumper) {
            robot.ballStop.setPosition(0.1);
            robot.ballAlto.setPosition(0.7); //open
            intakeOn = false;
            closed = false;
            closedTimer.reset();
        } else if (gamepad1.right_bumper) {
            intakeTimer.reset();
            robot.ballStop.setPosition(0.5);
            robot.ballAlto.setPosition(0.2); //close
            intakeOn = false;
            closed = true;
        } else if (!closed && asistedDown && closedTimer.seconds() > 0.5) {
            intakeOn = false;
            robot.ballStop.setPosition(0);
            closed = true;
        }

        if (gamepad2.a) {
            robot.asistencia.setPosition(ASSIST_DOWN);
            intakeOn = true;
            gamepad2.rumble(0,1,1000);
            asisted = false;
        }

        if (gamepad2.dpad_right) {
            robot.asistencia.setPosition(ASSIST_UP);
            gamepad2.rumble(1,0,1000);
            asistedTimer.reset();
            asisted = true;
            asistedDown = false;
        }
        if (asisted && asistedTimer.seconds() > 0.25) {
            robot.asistencia.setPosition(ASSIST_DOWN);
            asisted = false;
            asistedDown = true;
        }
        if (gamepad2.dpad_left) {
            robot.asistencia.setPosition(ASSIST_DOWN);
            asisted = false;
            asistedDown = true;
        }

        if (gamepad2.a) {
            robot.ballUp.setPower(BALL_UP_UP);
        }else if (gamepad1.a) {
            robot.ballUp.setPower(BALL_UP_DOWN);
        }else {
            robot.ballUp.setPower(0);
        }

        if (gamepad2.y) {

            robot.ballAlto.setPosition(0);
        }else if (gamepad2.x){

            robot.ballAlto.setPosition(0.5);
        }

        // ------------------- SHOOTER -------------------
        if (gamepad2.left_trigger >= 0.3) {
            robot.shooter.shooterTargetVelocity = 1500; //Shoot Spped

            robot.shooter.aimingLimelight = false;
            robot.turret.aimingLimelight = false;

        } else if (gamepad2.left_bumper) {
            robot.shooter.shooterTargetVelocity = -1500; //In

            robot.shooter.aimingLimelight = false;
            robot.turret.aimingLimelight = false;

        } else if (gamepad2.right_trigger >= 0.3) {
            robot.shooter.aimingLimelight = true;
            robot.turret.aimingLimelight = true;
        } else {
            robot.shooter.aimingLimelight = false;
            robot.turret.aimingLimelight = false;

            robot.shooter.shooterTargetVelocity = 0;
        }

        // ------------------- TURRET -------------------
        if (gamepad1.y) {
            robot.turret.aimingLimelight = true;
        } else if (gamepad2.right_trigger < 0.3) {
            robot.turret.aimingLimelight = false;
            robot.torrettCoquette.setPower(-gamepad2.left_stick_x);
        }
//        while (matchTimer.seconds()>100){
//            gamepad1.rumble(1000);
//            gamepad2.rumble(1000);
//        }


        // ------------------- ELEVATOR -------------------

        if (gamepad1.dpad_right)
            robot.elevacionServo.setPower(1.0);
        else if (gamepad1.dpad_left) {
            robot.elevacionServo.setPower(-1.0);
        }else {
            robot.elevacionServo.setPower(0.0);
        }


        // ------------------- ROBOT -------------------
        robot.update();
        // ------------------- TELEMETRY -------------------
        telemetry.addData("Sube bolas", robot.intake.getIndexerPosition());
        telemetry.addData("Para bolas", robot.intake.getGatePosition());
        telemetry.addData("Ángulo", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("Detected", robot.isBallDetected());
        telemetry.update();
    }
}
