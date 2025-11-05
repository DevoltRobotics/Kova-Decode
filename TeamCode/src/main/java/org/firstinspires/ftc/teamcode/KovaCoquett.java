package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TragabolasCoquette 😍😍❤️❤️")
public class KovaCoquett extends OpMode {

    private Follower follower;

    DcMotor tragaBolasMotor;
    DcMotor escupeBolasMotor;
    DcMotor disparadorMotor;

    CRServo torrettCoquette;
    Servo subeBolas;
    Servo paraBolas;

    RevColorSensorV3 detectaBolas;

    @Override
    public void init() {
        //motores
        follower = Constants.createFollower(hardwareMap);
        tragaBolasMotor = hardwareMap.dcMotor.get("intake");
        escupeBolasMotor = hardwareMap.dcMotor.get("shooter");
        disparadorMotor = hardwareMap.dcMotor.get("disparador");
        //subeBajaMotor = hardwareMap.dcMotor.get("subeBaja");

        //servos
        torrettCoquette = hardwareMap.crservo.get("torreta");
        paraBolas = hardwareMap.servo.get("paraBolas");
        subeBolas = hardwareMap.servo.get("subeBolas");

        disparadorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //extra
        detectaBolas = hardwareMap.get(RevColorSensorV3.class, "detectaBolas");
    }

    @Override
    public void start() {
        follower.startTeleOpDrive(true);
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                true // Robot Centric
        );

        detectaBolas.enableLed(true);

        boolean hayBola = detectaBolas.getDistance(DistanceUnit.MM) < 50;

        if (gamepad1.a) { //progra intake :)
            tragaBolasMotor.setPower(-1);
            paraBolas.setPosition(1);
        } else if (gamepad2.b) {
            tragaBolasMotor.setPower(1);
        } else if (gamepad2.a) {
            tragaBolasMotor.setPower(-1);
        } else {
            tragaBolasMotor.setPower(0);
        }

        if (!gamepad1.a) {
            if (hayBola && !gamepad2.b) {
                paraBolas.setPosition(0.3);
            } else {
                paraBolas.setPosition(1);
            }
        }

        //progra shooter :)
        if (gamepad1.left_trigger >= 0.1) {
            escupeBolasMotor.setPower(1);
            disparadorMotor.setPower(1);
        } else {
            escupeBolasMotor.setPower(0);
            disparadorMotor.setPower(0);
        }

        if (gamepad1.right_bumper) {
            subeBolas.setPosition(0.7);
        } else {
            subeBolas.setPosition(0);
        }

        //progra torrett coquette

        torrettCoquette.setPower(gamepad2.left_stick_x);

        follower.update();

        telemetry.addData("Bolas rojas ", detectaBolas.red());
        telemetry.addData("Bolas alpha/sigma ", detectaBolas.alpha());
        telemetry.addData("Bolas verdes ", detectaBolas.green());
        telemetry.addData("Bolas azules ", detectaBolas.blue());
        telemetry.addData("hay bola", hayBola);
        telemetry.addData("Sube bolas ", subeBolas.getPosition());
        telemetry.addData("Para bolas ", paraBolas.getPosition());
        telemetry.update();
    }
}