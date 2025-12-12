package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "\uD83D\uDFE5Close Side\uD83D\uDFE5", group = "Autonomous")
public class RedAutoClose extends AutonomousClose {
    public RedAutoClose() {
        super(Alliance.RED);
    }
}
