package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto (Blue Right)")
public class AutoBlueRight extends AutoSuper {
    AutoBlueRight() {
        super(Alliance.BLUE, AllianceSide.RIGHT);
    }
}
