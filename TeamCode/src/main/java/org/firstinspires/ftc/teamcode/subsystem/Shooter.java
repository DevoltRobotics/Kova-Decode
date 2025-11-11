package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.HardwareCoquett;

@Configurable
public class Shooter {

    static InterpLUT lut = new InterpLUT();

    static {
        lut.add(0, 3000);
        lut.add(5, 4000);
        lut.add(8, 5000);
        lut.add(10, 6000);

        lut.createLUT();
    }

    public static PIDFCoefficients shooterCoeffs = new PIDFCoefficients(
      0, 0, 0, 0
    );

    HardwareCoquett robot;

    PIDFController shooterController = new PIDFController(shooterCoeffs);

    public boolean aimingLimelight = false;
    public double shooterTargetVelocity = 0;

    public Shooter(HardwareCoquett robot) {
        this.robot = robot;
    }

    public void update() {
        double tA = robot.limelight.getLatestResult().getTa();

        if(aimingLimelight) {
            shooterTargetVelocity = lut.get(tA);
        }

        shooterController.setSetPoint(shooterTargetVelocity);

        double power = shooterController.calculate(robot.disparadorMotor.getVelocity());

        robot.disparadorMotor.setPower(power);
        robot.escupeBolasMotor.setPower(power);
    }

}
