package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.HardwareCoquett;

@Configurable
public class Shooter {

    static InterpLUT lut = new InterpLUT();
    static {
        lut.add(0, 1650);
        lut.add(0.25, 1500);
        lut.add(0.42, 1400);
        lut.add(0.72, 1250);
        lut.add(3.2, 1050);
        lut.add(6, 1100);

        lut.createLUT();
    }

    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(
            0.01, 0, 0, 0
    );

    public static double kV = 0.0007;

    ElapsedTime detectionTimer = new ElapsedTime();

    HardwareCoquett robot;

    PIDFController shooterController = new PIDFController(shooterCoeffs);

    public boolean aimingLimelight = false;
    public double shooterTargetVelocity = 0;

    public Shooter(HardwareCoquett robot) {
        this.robot = robot;
    }

    public void update() {
        shooterController.setCoefficients(shooterCoeffs);
        Double tA = robot.getAllianceTA();

        if(aimingLimelight) {
            if (tA != null) {
                shooterTargetVelocity = lut.get(tA);
                detectionTimer.reset();
            } else if(detectionTimer.seconds() >= 2) {
                shooterTargetVelocity = 1200;
            }
        }

        shooterController.setTolerance(20);
        shooterController.setSetPoint(shooterTargetVelocity);

        double currentVelocity = -robot.shooterMotor.getVelocity();

        double power = (kV * shooterTargetVelocity) + shooterController.calculate(currentVelocity);

        robot.disparadorMotor.setPower(power);
        robot.shooterMotor.setPower(power);

        robot.panelsTelem.addData("shooter current velocity", currentVelocity);
        robot.panelsTelem.addData("shooter target velocity", shooterTargetVelocity);
    }
//TODO: Adjustar a disparar de lejos
}