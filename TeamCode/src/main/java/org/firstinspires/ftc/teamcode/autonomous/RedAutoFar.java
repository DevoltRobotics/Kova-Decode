package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Alliance;
@Disabled
@Autonomous(name = "\uD83D\uDFE5Far Side\uD83D\uDFE5", group = "AutonomousFar")
public class RedAutoFar extends AutonomousFar {
    public RedAutoFar() {
        super(Alliance.RED);
    }
}
