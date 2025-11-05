package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TragabolasCoquette 😍😍❤️❤️")
public class KovaCoquett extends OpMode {

    private Follower follower;

    // Motores principales
    DcMotor tragaBolasMotor;
    DcMotor escupeBolasMotor;
    DcMotor disparadorMotor;

    // Servos
    CRServo torrettCoquette;
    Servo subeBolas;
    Servo paraBolas;

    // Sensor para detectar bolas
    RevColorSensorV3 detectaBolas;

    // Timers para manejar tiempos y rollbacks
    ElapsedTime intakeRollbackTimer = new ElapsedTime();        // rollback cuando entra una nueva bola
    ElapsedTime intakeShootRollbackTimer = new ElapsedTime();   // rollback al disparar el shooter
    ElapsedTime shooterFlickTimer = new ElapsedTime();          // retardo para subir bola al disparador
    Boolean habiaBola = null;

    @Override
    public void init() {
        // Inicialización de hardware
        follower = Constants.createFollower(hardwareMap);
        tragaBolasMotor = hardwareMap.dcMotor.get("intake");
        escupeBolasMotor = hardwareMap.dcMotor.get("shooter");
        disparadorMotor = hardwareMap.dcMotor.get("disparador");

        torrettCoquette = hardwareMap.crservo.get("torreta");
        paraBolas = hardwareMap.servo.get("paraBolas");
        subeBolas = hardwareMap.servo.get("subeBolas");

        disparadorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
                true
        );

        detectaBolas.enableLed(true);

        boolean hayBola = detectaBolas.getDistance(DistanceUnit.MM) < 50;

        if (habiaBola == null) {
            habiaBola = hayBola;
        }

        double tragaBolasPower = 0;

        // --- CONTROL AUTOMÁTICO DE LANZAR ---
        if (gamepad1.a) {
            paraBolas.setPosition(1);

            // Si presionas A y ya hay bola detectada, se hace un rollback rápido
            // Esto empuja la bola un poquito hacia atrás para liberar presión del servo
            if (gamepad1.aWasPressed()) {
                if (hayBola) {
                    intakeShootRollbackTimer.reset(); // rollback de inicio
                }
                shooterFlickTimer.reset(); // temporizador para subir ultima bola
            }

            // Después de 2.5s, se empuja la ultima bola hacia el shooter con el servo
            if (shooterFlickTimer.seconds() >= 2.5) {
                subeBolas.setPosition(0.8);
            }

            // --- ROLLBACK #1: al iniciar el disparo automático ---
            // Si acabas de presionar A y había bola, retrocede un poco (0.2s)
            // para desatorar la bola antes de meterla otra vez.
            if (intakeShootRollbackTimer.seconds() < 0.2) {
                tragaBolasPower = 0.6; // rollback ligero
            } else {
                tragaBolasPower = -1;  // absorbe normalmente
            }
        } else if (gamepad2.b) {
            tragaBolasPower = 1;   // sacar bolas manualmente
        } else if (gamepad2.a) {
            tragaBolasPower = -1;  // meter bolas manualmente
        } else if (gamepad1.right_bumper) {
            subeBolas.setPosition(0.8); // subir bola manualmente
        } else {
            subeBolas.setPosition(0);
        }

        // --- CONTROL AUTOMÁTICO DE BLOQUEO DE BOLAS ---
        if (!gamepad1.a) {
            if (hayBola && !gamepad2.b) {
                paraBolas.setPosition(0.5);

                // Si acaba de detectarse una bola nueva (cambio de estado),
                // reinicia el temporizador del rollback automático.
                if (hayBola != habiaBola) {
                    intakeRollbackTimer.reset();
                }

                // --- ROLLBACK #2: cuando entra una nueva bola ---
                // Al detectar una bola nueva, el motor hace un pequeño rollback
                // entre los 0.5s y 0.8s después de haberla visto.
                // Esto sirve para despegarla del servo y evitar atascos.
                if (intakeRollbackTimer.seconds() > 0.5 && intakeRollbackTimer.seconds() < 0.8) {
                    tragaBolasPower = 0.6;
                }
            } else {
                paraBolas.setPosition(1);
            }
        }

        tragaBolasMotor.setPower(tragaBolasPower);

        // --- SHOOTER ---
        if (gamepad1.left_trigger >= 0.1) {
            escupeBolasMotor.setPower(1);
            disparadorMotor.setPower(1);
        } else {
            escupeBolasMotor.setPower(0);
            disparadorMotor.setPower(0);
        }

        // --- TORRETA ---
        torrettCoquette.setPower(gamepad2.left_stick_x);

        follower.update();

        // --- TELEMETRÍA ---
        telemetry.addData("Bolas rojas", detectaBolas.red());
        telemetry.addData("Bolas verdes", detectaBolas.green());
        telemetry.addData("Bolas azules", detectaBolas.blue());
        telemetry.addData("Hay bola", hayBola);
        telemetry.addData("Sube bolas", subeBolas.getPosition());
        telemetry.addData("Para bolas", paraBolas.getPosition());
        telemetry.update();

        // Actualiza estado anterior
        habiaBola = hayBola;
    }
}
