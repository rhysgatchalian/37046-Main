package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@Config
public class SubSystem {

    // ---------------------------------------------------------------
    // HuskyLens / AprilTag following tuning (shows up in FTC Dashboard
    // since this class is @Config).
    // ---------------------------------------------------------------

    // HuskyLens reports block centers in a 320x240 image.
    public static int HUSKYLENS_FRAME_WIDTH = 320;

    // Proportional gain converting pixel error -> turn power.
    // Flip the sign if the robot turns away from the tag instead of towards it.
    public static double APRILTAG_TURN_GAIN = 0.006;

    // Cap on how hard the robot is allowed to spin while following a tag.
    public static double APRILTAG_MAX_TURN_POWER = 0.4;

    // Pixel error smaller than this is treated as "centered" (prevents jitter).
    public static int APRILTAG_TURN_DEADBAND = 4;

    // Works with any Road Runner drive (SampleMecanumDrive, SampleTankDrive, ...).
    private Drive drive;

    private HuskyLens huskyLens;

    // Motors Setup

    private DcMotorEx motor1, motor2, motor3, motor4;
    private double motor1_RPM, motor2_RPM, motor3_RPM, motor4_RPM;

    // Servos Setup

    private Servo servo1;
    private CRServo servo2;

    public void init(HardwareMap hardwareMap, Drive drive) {
        this.drive = drive;

        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        motor4 = hardwareMap.get(DcMotorEx.class, "motor4");

        motor1.setDirection(DcMotorEx.Direction.FORWARD);
        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        motor3.setDirection(DcMotorEx.Direction.REVERSE);
        motor4.setDirection(DcMotorEx.Direction.REVERSE);

        motor1_RPM = 0;
        motor2_RPM = 0;
        motor3_RPM = 0;
        motor4_RPM = 0;

        motor1.setPower(0.01);
        motor2.setPower(0.01);
        motor3.setPower(0.01); 
        motor4.setPower(0.01);

        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        // Camera Setup (HuskyLens over I2C, configured with the name "huskylens")
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    // ---------------------------------------------------------------
    // AprilTag following
    //
    // This does NOT block. Call it once per loop iteration (e.g. every
    // pass through your TeleOp/Auto while-loop) and it will keep
    // rotating the robot in place so it faces the most recently seen
    // AprilTag. Call stopAprilTagFollowing() to stop turning.
    // ---------------------------------------------------------------

    /** Turns the robot to face the first AprilTag the HuskyLens sees. */
    public void followAprilTag() {
        followAprilTag(-1);
    }

    /**
     * Turns the robot to face a specific AprilTag id. If that tag isn't
     * currently visible, the robot stops turning until it is seen again.
     *
     * @param tagId the AprilTag id to track, or -1 to track whichever tag is seen first
     **/
    public void followAprilTag(int tagId) {
        HuskyLens.Block target = null;
        for (HuskyLens.Block block : huskyLens.blocks()) {
            if (tagId == -1 || block.id == tagId) {
                target = block;
                break;
            }
        }

        if (target == null) {
            stopAprilTagFollowing();
            return;
        }

        int error = target.x - (HUSKYLENS_FRAME_WIDTH / 2);
        double turnPower;
        if (Math.abs(error) <= APRILTAG_TURN_DEADBAND) {
            turnPower = 0;
        } else {
            turnPower = Range.clip(-error * APRILTAG_TURN_GAIN, -APRILTAG_MAX_TURN_POWER, APRILTAG_MAX_TURN_POWER);
        }

        drive.setDrivePower(new Pose2d(0, 0, turnPower));
    }

    /** Stops any rotation started by followAprilTag(). */
    public void stopAprilTagFollowing() {
        drive.setDrivePower(new Pose2d(0, 0, 0));
    }

    /** Returns every AprilTag currently visible to the HuskyLens. */
    public HuskyLens.Block[] getVisibleAprilTags() {
        return huskyLens.blocks();
    }


    // ---------------------------------------------------------------
    // Motor Systems


    // Intake

    public void startIntake() {
        motor1.setPower(1.0);
    }

    public void stopIntake() {
        motor1.setPower(0.01);
    }

    // Launcher

    public void startLauncher() {
        motor2.setPower(1.0);
    }

    public void stopLauncher() {
        motor2.setPower(0.01);
    }

    // Passthrough

    public void startPass() {
        motor3.setPower(1.0);
    }

    public void stopPass() {
        motor3.setPower(0.01);
    }

    // Climb

    public void climb() {
        motor4.setTargetPosition(1000);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setPower(1.0);
    }

    // ---------------------------------------------------------------
    // Servo Systems

    // Servo1
    public void setServo1Position(double position) {
        servo1.setPosition(position);
    }

    // Servo2
    public void startServo2(boolean direction) {
        servo2.setDirection(direction ? CRServo.Direction.FORWARD : CRServo.Direction.REVERSE);
        servo2.setPower(1);
    }

    public void stopServo2() {
        servo2.setPower(0);
    }
}
