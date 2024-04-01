package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//@Autonomous
@Autonomous(name = "SensorTest")
public class SensorTest extends OpMode {
    private DcMotorEx leftDrive = null;
    private DcMotorEx rightDrive = null;

    private DcMotorEx hangerDrive = null;

    private Rev2mDistanceSensor Dist1 = null;

    private IMU imu = null;

    private PlowMatic plow;

    int hangerLocation;

    @Override public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotorEx.class, "left");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right");
        hangerDrive = hardwareMap.get(DcMotorEx.class, "hanger");
        imu = hardwareMap.get(IMU.class, "imu");
        plow = new PlowMatic(hardwareMap.get(Servo.class, "plowFront"));
        Dist1 = hardwareMap.get(Rev2mDistanceSensor.class, "dist1");

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
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo, usb)));
        imu.resetYaw();
    }

    @Override public void start() {
        imu.resetYaw();
        plow.raise();

    }
    @Override public void loop() {
       double frontDistance = (Dist1.getDistance(DistanceUnit.CM));
       Orientation rotation = (imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES));
       float yawRot = (rotation.secondAngle);
       telemetry.addData("Front Distance", frontDistance);
       telemetry.addData("Orientation", rotation);
       telemetry.addData("Yaw", yawRot);

    }
}