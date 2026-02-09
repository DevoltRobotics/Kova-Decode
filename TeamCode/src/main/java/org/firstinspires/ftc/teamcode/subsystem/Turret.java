package org.firstinspires.ftc.teamcode.subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.HardwareCoquett;

@Configurable
public class Turret {

    HardwareCoquett robot;

    public static PIDFCoefficients llTurretCoeffs = new PIDFCoefficients(
            0.018, 0, 0.0001, 0
    );

    public static double minPower = 0.04;
    public static double llTolerance = 0.1;

    // --- Nuevo: Filtro exponencial ---
    // Cuanto menor sea alpha, más suave el movimiento (0.1–0.3 recomendable)
    public static double filterAlpha = 0.8;
    private double filteredOutput = 0.0;

    PIDFController llTurretController = new PIDFController(llTurretCoeffs);

    public boolean aimingLimelight = false;

    public Turret(HardwareCoquett robot) {
        this.robot = robot;
    }

    public void update() {
        if (aimingLimelight) {
            if (robot.getAllianceTX() != null) {
                if (robot.alliance == Alliance.BLUE) {
                    llTurretController.setSetPoint(3);
                } else {
                    llTurretController.setSetPoint(3);
                }

                llTurretController.setCoefficients(llTurretCoeffs);
                llTurretController.setTolerance(llTolerance);
                llTurretController.setMinimumOutput(minPower);

                double rawOutput = llTurretController.calculate(robot.getAllianceTX());
                double smoothOutput = applyExponentialFilter(rawOutput);

                robot.torrettCoquette.setPower(smoothOutput);

                robot.panelsTelem.addData("turret error", llTurretController.getPositionError());
                robot.panelsTelem.addData("raw output", rawOutput);
                robot.panelsTelem.addData("filtered output", smoothOutput);
            } else {
                robot.torrettCoquette.setPower(0);
                filteredOutput = 0;
            }
        }

        robot.panelsTelem.addData("turret power", robot.torrettCoquette.getPower());
    }

    // --- Método de suavizado exponencial ---
    private double applyExponentialFilter(double input) {
        filteredOutput = filterAlpha * input + (1 - filterAlpha) * filteredOutput;
        return filteredOutput;
    }
}
