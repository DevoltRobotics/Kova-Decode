package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Alliance;

@Autonomous(name = "BlueAllianceAuto", group = "Autonomous")
public class BlueAutonomous extends AutonomousGlobal {
    public BlueAutonomous() {
        super(Alliance.BLUE);
    }
}
