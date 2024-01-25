package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.Servo

class PlowMatic (
    private val plow: Servo
) {
    companion object {
        @JvmStatic val PLOW_HEIGHT_LOWERED = 0.175;
        @JvmStatic val PLOW_HEIGHT_RAISED = 0.51;
    }

    public fun lower() {
        plow.position = PLOW_HEIGHT_LOWERED
    }

    public fun raise() {
        plow.position = PLOW_HEIGHT_RAISED
    }

    public val position
        get() = plow.position
}