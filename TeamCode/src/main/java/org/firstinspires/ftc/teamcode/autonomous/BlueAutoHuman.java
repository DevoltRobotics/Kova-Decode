package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "\uD83D\uDFE6Human Player\uD83D\uDFE6", group = "AutoHuman")
public class BlueAutoHuman extends AutoHuman {
    public BlueAutoHuman() {
        super(Alliance.BLUE);
    }
}
