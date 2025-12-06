package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

public class HardwareCoquett {

    // Motores principales
    public DcMotor intake;

    public DcMotorEx shooterMotor;
    public DcMotorEx disparadorMotor;
    public DcMotor subiBajaMotor;

    // Servos
    public CRServo torrettCoquette;
    public CRServo ballUp;
    public CRServo noStuck;
    public Servo ballStop;
    public Servo light;
    public Servo asistencia;
    public DigitalChannel laserInput;

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
        intake = hardwareMap.get(DcMotor.class, "intake");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        disparadorMotor = hardwareMap.get(DcMotorEx.class, "disparador");
        laserInput = hardwareMap.get(DigitalChannel.class, "laser");
        laserInput.setMode(DigitalChannel.Mode.INPUT);

        disparadorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        subiBajaMotor=hardwareMap.get(DcMotor.class, "subibaja");


        // Servos
        torrettCoquette = hardwareMap.get(CRServo.class, "torreta");
        ballUp = hardwareMap.get(CRServo.class, "subeBolas");
        ballStop = hardwareMap.get(Servo.class, "paraBolas");
        asistencia = hardwareMap.get(Servo.class, "asistencia");
        noStuck = hardwareMap.get(CRServo.class, "stuck");
        light = hardwareMap.get(Servo.class, "light");



        // Sensor
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        asistencia.setDirection(Servo.Direction.REVERSE);

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