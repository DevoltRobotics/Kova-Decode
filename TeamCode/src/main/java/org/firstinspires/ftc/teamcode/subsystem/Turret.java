package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.HardwareCoquett;

@Configurable
public class Turret {

    HardwareCoquett robot;

    public static PIDFCoefficients llTurretCoeffs = new PIDFCoefficients(
            0.03, 0, 0.0005, 0
    );

    PIDFController llTurretController = new PIDFController(llTurretCoeffs);

    public boolean aimingLimelight = false;

    public Turret(HardwareCoquett robot) {
        this.robot = robot;

        llTurretController.setTolerance(1.5);
        llTurretController.setMinimumOutput(0.03);
    }
    public void update() {
        if(aimingLimelight) {
            if(robot.getAllianceTX() != null) {
                llTurretController.setCoefficients(llTurretCoeffs);
                robot.torrettCoquette.setPower(llTurretController.calculate(robot.getAllianceTX(), 0));
            } else {
                robot.torrettCoquette.setPower(0);
            }
        }
    }

}
