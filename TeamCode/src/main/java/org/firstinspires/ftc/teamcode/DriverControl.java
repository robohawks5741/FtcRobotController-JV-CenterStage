package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.*;

import static org.firstinspires.ftc.teamcode.MacrosKt.clamp;
import static org.firstinspires.ftc.teamcode.MacrosKt.stickCurve;

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

    private IMU imu = null;

    private PlowMatic plow;

    int hangerLocation;

    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotorEx.class, "left");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right");
        hangerDrive = hardwareMap.get(DcMotorEx.class, "hanger");
        imu = hardwareMap.get(IMU.class, "imu");
        plow = new PlowMatic(hardwareMap.get(Servo.class, "plowFront"));

        plow.raise();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        hangerDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangerDrive.setTargetPosition(300);
        hangerDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hangerDrive.setVelocity(1500);
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
        plow.raise();
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

        double drive = stickCurve(-gamepad1.left_stick_y);
//        double turn = clamp(gamepad1.right_stick_x + gamepad1.left_stick_x, -1.0, 1.0);
        double turn = stickCurve(gamepad1.right_stick_x);

        boolean gottaGoFast = gamepad1.right_trigger >= 0.5;

        boolean hangerRaise = gamepad1.right_bumper;
        boolean hangerContract = gamepad1.left_bumper;

        // Process bumper input to change the desired angle
        if (hangerRaise) {
            hangerLocation = (hangerLocation + 4);
        } else if (hangerContract) {
            hangerLocation = (hangerLocation - 4);
        }

        double leftPowerPre = clamp(drive + turn, -1.0, 1.0);
        double rightPowerPre = clamp(drive - turn, -1.0, 1.0);

        if (gottaGoFast) {
            leftPower = leftPowerPre;
            rightPower = rightPowerPre;
        } else {
            leftPower = Range.scale(leftPowerPre, -1.0, 1.0, -0.4, 0.4);
            rightPower = Range.scale(rightPowerPre, -1.0, 1.0, -0.4, 0.4);
        }
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        if (gamepad1.dpad_up) plow.raise();
        else if (gamepad1.dpad_down) plow.lower();
        // Send calculated power to wheels

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Show the elapsed game time and wheel power.

        telemetry.addData("Plow Servo Position", "(%.4f)", plow.getPosition());
        hangerDrive.setTargetPosition(hangerLocation);
        telemetry.addData("Hanger Location", hangerLocation);
    }
}

