package org.firstinspires.ftc.teamcode.TeleOp.mechanisms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    public DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
    public IMU rev_imu;
    public void init(HardwareMap hwMap) {
        frontLeftMotor = hwMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hwMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hwMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hwMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rev_imu = hwMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot RevOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
        rev_imu.initialize(new IMU.Parameters(RevOrientation));


    }

    public void robotOrientedDrive(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0; // change this number to like 0.3 when at an outreach event

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftMotor.setPower(maxSpeed * (frontLeftPower / maxPower));
        backLeftMotor.setPower(maxSpeed * (backLeftPower / maxPower));
        frontRightMotor.setPower(maxSpeed * (frontRightPower / maxPower));
        backRightMotor.setPower(maxSpeed * (backRightPower / maxPower));
    }

    public void fieldOrientedDrive(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward,strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta -
                rev_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.robotOrientedDrive(newForward, newStrafe, rotate);
    }
}
