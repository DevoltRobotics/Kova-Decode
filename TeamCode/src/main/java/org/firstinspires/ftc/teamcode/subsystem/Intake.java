package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareCoquett;
public class Intake {

    private final HardwareCoquett robot;

    private final ElapsedTime intakeRollbackTimer = new ElapsedTime();
    private final ElapsedTime intakeShootRollbackTimer = new ElapsedTime();
    private final ElapsedTime shooterFlickTimer = new ElapsedTime();

    private Boolean previousHasBall = null;

    public static class Command {
        public boolean autoShoot;
        public boolean autoShootJustPressed;
        public boolean autoShootFeedOverride;
        public boolean manualIntakeForward;
        public boolean manualIntakeReverse;
        public boolean manualIndexerUp;
        public boolean autoBlockEnabled;
        public boolean shooterClearing;
    }

    public Intake(HardwareCoquett robot) {
        this.robot = robot;
    }

    public Command newCommand() {
        return new Command();
    }

    public void update(Command cmd) {
        boolean hasBall = hasBall();

        if (previousHasBall == null) {
            previousHasBall = hasBall;
        }

        double intakePower = 0.0;
        double paraBolasPos = robot.ballStop.getPosition();
        double subeBolasPos = robot.ballUp.getPower();

        // --- AUTO SHOOT (gamepad2.a) ---
        if (cmd.autoShoot) {
            if (cmd.autoShootJustPressed) {
                if (hasBall) {
                    intakeShootRollbackTimer.reset();
                }
                shooterFlickTimer.reset();
            }

            if (intakeShootRollbackTimer.seconds() < 0.2) {
                intakePower = 0.4;
            } else {
                intakePower = -1.0;
            }

        }
        // --- MANUAL INTAKE / INDEXER (gamepad2) ---
        else if (cmd.manualIntakeForward) {
            intakePower = 1.0;
        } else if (cmd.manualIntakeReverse) {
            intakePower = -1.0;
        }

        // --- AUTO BALL BLOCKING ---
        /*if (cmd.autoBlockEnabled && !cmd.shooterClearing) {
            if (hasBall && !cmd.manualIntakeForward) {
                if (hasBall != previousHasBall) {
                    intakeRollbackTimer.reset();
                }

                double t = intakeRollbackTimer.seconds();
                if (t > 0.5 && t < 0.8) {
                    intakePower = 0.6;
                }
            }
        }*/

        robot.intake.setPower(intakePower);

        previousHasBall = hasBall;
    }

    // ------------- Auto Methods -------------

    public void intakeIn() {
        robot.intake.setPower(1.0);
    }

    public void intakeOut() {
        robot.intake.setPower(-1.0);
    }

    public void stopIntake() {
        robot.intake.setPower(0.0);
    }

    public void indexerUp() {
        robot.ballUp.setPower(0.8);
    }

    public void indexerDown() {
        robot.ballUp.setPower(0.0);
    }

    public void openGate() {
        robot.ballStop.setPosition(1.0);
    }

    public void blockGate() {
        robot.ballStop.setPosition(0.5);
    }

    public boolean hasBall() {
        return robot.laserInput.getState();
    }

    public double getIndexerPosition() {
        return robot.ballUp.getPower();
    }

    public double getGatePosition() {
        return robot.ballStop.getPosition();
    }
}
