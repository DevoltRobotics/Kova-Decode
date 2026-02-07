package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Alliance;
@Disabled
@Autonomous(name = "\uD83D\uDFE6Far Side\uD83D\uDFE6", group = "AutonomousFar")
public class BlueAutoFar extends AutonomousFar {
    public BlueAutoFar() {
        super(Alliance.BLUE);
    }
}
