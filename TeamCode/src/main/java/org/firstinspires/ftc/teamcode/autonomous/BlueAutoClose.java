package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "\uD83D\uDFE6Close Side\uD83D\uDFE6", group = "AutonomousClose")
public class BlueAutoClose extends AutonomousClose {
    public BlueAutoClose() {
        super(Alliance.BLUE);
    }
}
