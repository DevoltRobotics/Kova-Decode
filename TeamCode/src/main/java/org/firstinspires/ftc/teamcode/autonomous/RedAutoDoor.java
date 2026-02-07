package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Alliance;
@Disabled
@Autonomous(name = "\uD83D\uDFE5Open Door\uD83D\uDFE5", group = "AutoOpenDoor")
public class RedAutoDoor extends AutoOpenDoor {
    public RedAutoDoor() {
        super(Alliance.RED);
    }
}
