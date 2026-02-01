package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "\uD83D\uDFE6Open Door\uD83D\uDFE6", group = "AutoOpenDoor")
public class BlueAutoDoor extends AutoOpenDoor {
    public BlueAutoDoor() {
        super(Alliance.BLUE);
    }
}
