package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto (Red Left)")
public class AutoRedLeft extends AutoSuper {
    AutoRedLeft() {
        super(Alliance.RED, AllianceSide.LEFT);
    }
}
