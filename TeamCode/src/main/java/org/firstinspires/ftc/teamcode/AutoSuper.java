package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

//@Disabled
class AutoSuper extends OpMode {
    final protected Alliance alliance;
    final protected AllianceSide side;

    AutoSuper(Alliance alliance, AllianceSide side) {
        this.alliance = alliance;
        this.side = side;
    }
    enum Alliance {
        RED,
        BLUE
    }
    enum AllianceSide {
        LEFT,
        RIGHT
    }

    @Override public void init() { }
    @Override public void start() { }

    @Override public void loop() { }
}