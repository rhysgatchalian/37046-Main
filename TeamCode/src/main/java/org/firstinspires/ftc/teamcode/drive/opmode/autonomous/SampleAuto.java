package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

// Imported Libraries
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SubSystem;

// Other Libraries
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;



@Autonomous(group = "drive")
public class SampleAuto extends LinearOpMode {




    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);         // Change to SampleTankDrive if using tank drive
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SubSystem subSystem = new SubSystem();
        subSystem.init(hardwareMap, drive);


        waitForStart();

        if (isStopRequested()) return;
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .build();

        drive.followTrajectory(trajectory);



        Trajectory trajectory2 = drive.trajectoryBuilder(new Pose2d())
                .turn(Math.toRadians(90))
                .build();

        drive.followTrajectory(trajectory2);
    }
}