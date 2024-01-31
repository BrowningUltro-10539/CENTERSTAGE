package org.firstinspires.ftc.teamcode.Common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeRunCommand;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
@Autonomous
public class RED_RIGHT_AUTONOMOUS_GUESS extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode(){
        CommandScheduler.getInstance().reset();
        this.robot = new Robot(hardwareMap, true);

        //start position
        Pose2d startPose = new Pose2d(11.5, -62, Math.toRadians(90));
        robot.driveSubsystem.setPoseEstimate(startPose);
        robot.reset();

        //Left tape
        TrajectorySequence toCenterTape = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(12, -42), Math.toRadians(110))
                .splineTo(new Vector2d(12, -15), Math.toRadians(90))
                .build();

        //Center tape
        TrajectorySequence toLeftTape = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(12, -42), Math.toRadians(110))
                .splineTo(new Vector2d(9,-30), Math.toRadians(0))
                .build();

        //Right tape
        TrajectorySequence toRightTape = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(12, -42), Math.toRadians(110))
                .splineTo(new Vector2d(30, -30), Math.toRadians(0))
                .build();

        TrajectorySequence toLeftBackdrop = robot.driveSubsystem.trajectorySequenceBuilder(toLeftTape.end())
                .splineTo(new Vector2d(49, -28.5), Math.toRadians(0))
                .build();

        TrajectorySequence toRightBackdrop = robot.driveSubsystem.trajectorySequenceBuilder(toRightTape.end())
                .splineTo(new Vector2d(49, -41.5), Math.toRadians(0))
                .build();

        TrajectorySequence toCenterBackdrop = robot.driveSubsystem.trajectorySequenceBuilder(toCenterTape.end())
                .splineTo(new Vector2d(49, -35), Math.toRadians(0))
                .build();

        TrajectorySequence toPark = robot.driveSubsystem.trajectorySequenceBuilder(toCenterTape.end())
                .splineTo(new Vector2d(28, -12), Math.toRadians(10))
                .splineTo(new Vector2d(61, -59), Math.toRadians(0))
                .build();


        while (!isStarted() && !isStopRequested()) {
            robot.reset();

            for (LynxModule module : robot.getControllers()) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toCenterTape),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new IntakeRunCommand(robot.intake, -1,1),
                                new WaitCommand(1500),
                                new IntakeRunCommand(robot.intake, 0,1),
                                new WaitCommand(1500),
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem,toPark)
                        )
                )
        );

        robot.reset();

        while (opModeIsActive()) {
            robot.read();

            CommandScheduler.getInstance().run();

            PoseStorage.currentPose = robot.driveSubsystem.getPoseEstimate();

            robot.intake.loop();
            robot.lift.loop();


            robot.write();

            telemetry.addData("Robot Pose: ", robot.driveSubsystem.getLocalizer().getPoseEstimate());
            telemetry.update();

            for (LynxModule module : robot.getControllers()) {
                module.clearBulkCache();
            }


        }


    }
}
