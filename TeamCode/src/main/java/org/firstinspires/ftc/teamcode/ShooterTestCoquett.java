package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp
public class ShooterTestCoquett extends OpMode {

    public static double shooterTarget = 0;

    HardwareCoquett robot = new HardwareCoquett(Alliance.ANY);

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.shooter.shooterTargetVelocity = shooterTarget;
        robot.update();

        if(gamepad1.aWasPressed()) {
            robot.shooter.aimingLimelight = !robot.shooter.aimingLimelight;
        }
    }
}
