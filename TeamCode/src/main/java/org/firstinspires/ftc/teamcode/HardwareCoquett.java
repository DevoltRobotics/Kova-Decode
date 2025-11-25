package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

public class HardwareCoquett {

    // Motores principales
    public DcMotor tragaBolasMotor;

    public DcMotorEx escupeBolasMotor;
    public DcMotorEx disparadorMotor;
    public DcMotor subiBajaMotor;

    // Servos
    public CRServo torrettCoquette;
    public Servo subeBolas;
    public Servo paraBolas;
    public Servo light;

    // Sensor de color / distancia
    public RevColorSensorV3 detectaBolas;

    // Follower para manejo de movimiento
    public Follower follower;

    public Limelight3A limelight;

    public Shooter shooter = new Shooter(this);
    public Turret turret = new Turret(this);

    public final Telemetry panelsTelem = PanelsTelemetry.INSTANCE.getFtcTelemetry();

    public final Alliance alliance;

    public LLResult llResult = null;

    public HardwareCoquett(Alliance alliance) {
        this.alliance = alliance;
    }

    public void init(HardwareMap hardwareMap) {

        // Inicializa el seguidor de trayectorias
        follower = Constants.createFollower(hardwareMap);

        // Motores
        tragaBolasMotor = hardwareMap.get(DcMotor.class, "intake");

        escupeBolasMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        disparadorMotor = hardwareMap.get(DcMotorEx.class, "disparador");

        disparadorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        subiBajaMotor=hardwareMap.get(DcMotor.class, "subibaja");

        // Servos
        torrettCoquette = hardwareMap.get(CRServo.class, "torreta");
        subeBolas = hardwareMap.get(Servo.class, "subeBolas");
        paraBolas = hardwareMap.get(Servo.class, "paraBolas");
        light = hardwareMap.get(Servo.class, "light");

        // Sensor
        detectaBolas = hardwareMap.get(RevColorSensorV3.class, "detectaBolas");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.start();
        limelight.setPollRateHz(60);

        limelight.pipelineSwitch(0);
    }

    public void update() {
        llResult = limelight.getLatestResult();

        shooter.update();
        turret.update();

        panelsTelem.addData("LL isValid", llResult.isValid());
        panelsTelem.addData("LL tA", getAllianceTA());
        panelsTelem.addData("LL tX", getAllianceTX());

        panelsTelem.update();

        follower.update();
    }


    public Double getAllianceTA() {
        if (llResult != null && llResult.isValid() && !llResult.getFiducialResults().isEmpty()) {
            int id = llResult.getFiducialResults().get(0).getFiducialId();

            if (alliance == Alliance.ANY ||
                    (alliance == Alliance.RED && id == 24) ||
                    (alliance == Alliance.BLUE && id == 20)) {
                return llResult.getTa();
            }
        }

        return null;
    }


    public Double getAllianceTX() {
        if (llResult != null && llResult.isValid() && !llResult.getFiducialResults().isEmpty()) {
            int id = llResult.getFiducialResults().get(0).getFiducialId();

            if (alliance == Alliance.ANY ||
                    (alliance == Alliance.RED && id == 24) ||
                    (alliance == Alliance.BLUE && id == 20)) {
                return llResult.getTx();
            }
        }

        return null;
    }
}
