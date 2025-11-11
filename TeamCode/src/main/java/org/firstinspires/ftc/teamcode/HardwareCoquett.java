package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;

public class HardwareCoquett {

    // Motores principales
    public DcMotor tragaBolasMotor;

    public DcMotorEx escupeBolasMotor;
    public DcMotorEx disparadorMotor;

    // Servos
    public CRServo torrettCoquette;
    public Servo subeBolas;
    public Servo paraBolas;

    // Sensor de color / distancia
    public RevColorSensorV3 detectaBolas;

    // Follower para manejo de movimiento
    public Follower follower;

    public Limelight3A limelight;

    public Shooter shooter = new Shooter(this);

    public void init(HardwareMap hardwareMap) {
        // Inicializa el seguidor de trayectorias
        follower = Constants.createFollower(hardwareMap);

        // Motores
        tragaBolasMotor = hardwareMap.get(DcMotor.class, "intake");

        escupeBolasMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        disparadorMotor = hardwareMap.get(DcMotorEx.class, "disparador");

        disparadorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Servos
        torrettCoquette = hardwareMap.get(CRServo.class, "torreta");
        subeBolas = hardwareMap.get(Servo.class, "subeBolas");
        paraBolas = hardwareMap.get(Servo.class, "paraBolas");

        // Sensor
        detectaBolas = hardwareMap.get(RevColorSensorV3.class, "detectaBolas");
        detectaBolas.enableLed(true);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void update() {
        shooter.update();

        limelight.getLatestResult().getTa();
    }
}
