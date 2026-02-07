package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "\uD83D\uDFE5Human Player\uD83D\uDFE5", group = "AutoHuman")
public class RedAutoHuman extends AutoHuman {
    public RedAutoHuman() {
        super(Alliance.RED);
    }
}

