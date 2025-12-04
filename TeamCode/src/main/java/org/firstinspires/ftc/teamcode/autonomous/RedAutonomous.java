package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "RedAllianceAuto", group = "Autonomous")
public class RedAutonomous extends AutonomousGlobal {
    public RedAutonomous() {
        super(Alliance.RED);
    }
}
