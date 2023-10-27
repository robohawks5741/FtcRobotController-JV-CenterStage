package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto (Blue Left)")
public class AutoBlueLeft extends AutoSuper {
    AutoBlueLeft() {
        super(Alliance.BLUE, AllianceSide.LEFT);
    }
}
