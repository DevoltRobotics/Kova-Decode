package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "avanzaGrande", group = "Autonomous")
@Configurable // Panels
public class avanzaGrande extends OpMode {

    HardwareCoquett robot = new HardwareCoquett(Alliance.ANY);

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.follower.setPose(new Pose());
        robot.follower.startTeleOpDrive();
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        if(timer.seconds() <= 1.27) {
            robot.follower.setTeleOpDrive(0.5, 0, 0);
        } else {
            robot.follower.setTeleOpDrive(0, 0, 0);
        }

        robot.update();
    }

}