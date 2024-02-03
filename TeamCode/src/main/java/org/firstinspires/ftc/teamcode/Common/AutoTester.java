package org.firstinspires.ftc.teamcode.Common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Vision.TensorFlowBlueMarkerDetection;
import org.firstinspires.ftc.teamcode.commands.Auto.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeRunCommand;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.Teleop.DepositAndRetractCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Autonomous
public class AutoTester extends LinearOpMode {
    private Robot robot;

    private static final boolean USE_WEBCAM = true;

    private TfodProcessor tfod;

    private VisionPortal visionPortal;

    private static final String[] LABELS = {
            "Box",
    };

    public enum MarkerState{LEFT, CENTER, RIGHT}

    TensorFlowBlueMarkerDetection.MarkerState markerPos = TensorFlowBlueMarkerDetection.MarkerState.CENTER;
    @Override
    public void runOpMode() {


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        CommandScheduler.getInstance().reset();
        this.robot = new Robot(hardwareMap, true);

        //start position
        Pose2d startPose = new Pose2d(12, -62, Math.toRadians(90));
        robot.driveSubsystem.setPoseEstimate(startPose);
        robot.reset();

        //Left tape
        TrajectorySequence toCenterTape = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(12, -12))
                .build();
        TrajectorySequence toCenterBackdrop = robot.driveSubsystem.trajectorySequenceBuilder(toCenterTape.end())
                .lineTo(new Vector2d(12, -10))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(50, -10))
                .lineTo(new Vector2d(50, -35))
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
                        new IntakeRunCommand(robot.intake, -0.58, 3),
                        new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toCenterBackdrop),
                        new LiftPositionCommand(robot.lift, 10, 200, 200, 2),
                        new InstantCommand(() -> robot.outtake.update(OuttakeSubsystem.ArmState.RELEASE)),
                        new DepositAndRetractCommand(robot)
                )
        );

        robot.reset();

        while (opModeIsActive()) {
            robot.read();

            CommandScheduler.getInstance().run();

            PoseStorage.currentPose = robot.driveSubsystem.getPoseEstimate();

            robot.intake.loop();
            robot.lift.loop();
            robot.outtake.loop();



            robot.write();

            telemetry.addData("Robot Pose: ", robot.driveSubsystem.getLocalizer().getPoseEstimate());
            telemetry.update();

            for (LynxModule module : robot.getControllers()) {
                module.clearBulkCache();
            }


        }

    }

}
