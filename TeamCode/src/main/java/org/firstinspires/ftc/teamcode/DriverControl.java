package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.*;

import android.icu.text.Transliterator;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Driver Control (android)")
public class DriverControl extends OpMode {
    // Declare OpMode members.
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;

    private DcMotorEx hangerDrive = null;

    private Servo frontPlow = null;

    private IMU imu = null;

    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotorEx.class, "left");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right");
        hangerDrive = hardwareMap.get(DcMotorEx.class, "hanger");
        frontPlow = hardwareMap.get(Servo.class, "plowFront");
        imu = hardwareMap.get(IMU.class, "imu");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // causes the motors to actively stop instead of coasting
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangerDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sets the direction of the hanger drive motor, can be changed if necessary
        hangerDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        // IMU orientation/calibration
        LogoFacingDirection logo = LogoFacingDirection.RIGHT;
        UsbFacingDirection usb = UsbFacingDirection.FORWARD;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
    }

    @Override
    public void start() {
        imu.resetYaw();
        frontPlow.setPosition(0.175);
    }

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

//        imu.getRobotYawPitchRollAngles();

//        double theta;
        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        boolean frontplow = gamepad1.dpad_up;
        boolean frontplownegative = gamepad1.dpad_down;
        boolean NormalSpeed = gamepad1.right_trigger <= 0.5;
        boolean FastBoi = gamepad1.right_trigger >= 0.5;
        boolean hangerRaise = gamepad1.right_bumper;
        boolean hangerContract = gamepad1.left_bumper;
        double hangerForce = 0.25;
        if (hangerRaise) {
            hangerDrive.setPower(hangerForce);
        } else if (hangerContract) {
            hangerDrive.setPower(hangerForce * (-1));
        }

        if (NormalSpeed) {
            leftPower = Range.clip(drive + turn, -0.4, 0.4);
            rightPower = Range.clip(drive - turn, -0.4, 0.4);
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        }
        if (FastBoi) {
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        }

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        if (frontplow) {
            frontPlow.setPosition(0.51);
        }
        if (frontplownegative) {
            frontPlow.setPosition(0.175);
        }
        // Send calculated power to wheels

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Show the elapsed game time and wheel power.

        telemetry.addData("Plow Servo Position", "(%.4f)", frontPlow.getPosition());
    }
}

