package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public abstract class KovaCoquett extends OpMode {

    private final HardwareCoquett robot;

    private final ElapsedTime intakeRollbackTimer = new ElapsedTime();
    private final ElapsedTime intakeShootRollbackTimer = new ElapsedTime();
    private final ElapsedTime shooterFlickTimer = new ElapsedTime();
    private Boolean habiaBola = null;

    public KovaCoquett(Alliance alliance) {
        robot = new HardwareCoquett(alliance);
    }

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void start() {
        robot.follower.startTeleOpDrive(true);
    }

    @Override
    public void loop() {
        if(!gamepad1.b){
            robot.follower.setTeleOpDrive(
                    gamepad1.left_stick_y*0.5,
                    gamepad1.left_stick_x *0.5,
                    gamepad1.right_stick_x *0.5,
                    false
            );
            telemetry.addData("Modo lento: ", "Activado");
            robot.light.setPosition(0.277);
        }else {
            robot.follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    false
            );
            robot.light.setPosition(0.5);
            telemetry.addData("Modo lento: ", "Desactivado");
        }

        if(gamepad1.dpadUpWasPressed()) {
            robot.follower.setPose(new Pose());
        }

        boolean hayBola = robot.detectaBolas.getDistance(DistanceUnit.MM) < 40;

        if (habiaBola == null) habiaBola = hayBola;

        double tragaBolasPower = 0;

        // --- CONTROL AUTOMÁTICO DE LANZAR ---
        if (gamepad1.a) {
            robot.paraBolas.setPosition(1);

            if (gamepad1.aWasPressed()) {
                if (hayBola) intakeShootRollbackTimer.reset();
                shooterFlickTimer.reset();
            }

            if (gamepad1.right_bumper) robot.subeBolas.setPosition(0.8);

            if (intakeShootRollbackTimer.seconds() < 0.2) {
                tragaBolasPower = 0.4;
            } else {
                tragaBolasPower = -1;
            }

            if(shooterFlickTimer.seconds() >= 3) {
                robot.subeBolas.setPosition(0.8);
            }
        } else if (gamepad2.b) {
            tragaBolasPower = 1;
        } else if (gamepad2.a) {
            tragaBolasPower = -1;
        } else if (gamepad2.right_bumper) {
            robot.subeBolas.setPosition(0.8);
        } else {
            robot.subeBolas.setPosition(0);
        }

        // --- BLOQUEO AUTOMÁTICO DE BOLAS ---
        if (!gamepad1.a) {
            if (hayBola && !gamepad2.b) {
                robot.paraBolas.setPosition(0.5);
                if (hayBola != habiaBola) intakeRollbackTimer.reset();
                if (intakeRollbackTimer.seconds() > 0.5 && intakeRollbackTimer.seconds() < 0.8) {
                    tragaBolasPower = 0.6;
                }
            } else {
                robot.paraBolas.setPosition(1);
            }
        }

        robot.tragaBolasMotor.setPower(tragaBolasPower);

        // --- SHOOTER ---
        if (gamepad1.left_trigger >= 0.3) {
            robot.shooter.shooterTargetVelocity = 1500;

            robot.shooter.aimingLimelight = false;
            robot.turret.aimingLimelight = false;

        } else if(gamepad1.left_bumper) {
            robot.shooter.shooterTargetVelocity = -1500;

            robot.shooter.aimingLimelight = false;
            robot.turret.aimingLimelight = false;

        } else if(gamepad1.right_trigger >= 0.3) {
            robot.shooter.aimingLimelight = true;
            robot.turret.aimingLimelight = true;
        } else {
            robot.shooter.aimingLimelight = false;
            robot.turret.aimingLimelight = false;

            robot.shooter.shooterTargetVelocity = 0;
        }

        // --- TORRETA ---
        if(gamepad1.y || gamepad2.right_trigger >= 0.3) {
            robot.turret.aimingLimelight = true;
        } else if(gamepad1.right_trigger < 0.3) {
            robot.turret.aimingLimelight = false;
            robot.torrettCoquette.setPower(-gamepad2.left_stick_x);
        }

        robot.follower.update();

        robot.update();

        // --- TELEMETRÍA ---
        telemetry.addData("Bolas rojas", robot.detectaBolas.red());
        telemetry.addData("Bolas verdes", robot.detectaBolas.green());
        telemetry.addData("Bolas azules", robot.detectaBolas.blue());
        telemetry.addData("Hay bola", hayBola);
        telemetry.addData("Sube bolas", robot.subeBolas.getPosition());
        telemetry.addData("Para bolas", robot.paraBolas.getPosition());
        telemetry.addData("Ángulo", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.update();

        habiaBola = hayBola;
    }
}
