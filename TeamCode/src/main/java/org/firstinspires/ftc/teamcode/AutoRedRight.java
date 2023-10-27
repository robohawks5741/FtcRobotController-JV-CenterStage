package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto (Red Right)")
public class AutoRedRight extends AutoSuper {
    AutoRedRight() {
        super(Alliance.RED, AllianceSide.RIGHT);
    }
}
