package org.firstinspires.ftc.teamcode.commands.Auto;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Vision.RedPipeline;
import org.firstinspires.ftc.teamcode.Vision.TensorFlowBlueMarkerDetection;
import org.firstinspires.ftc.teamcode.commands.IntakeRunCommand;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.Teleop.DepositAndRetractCommand;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class AutoFarBlue extends LinearOpMode {
    private Robot robot;

    private static final boolean USE_WEBCAM = true;

    private TfodProcessor tfod;

    private RedPipeline pipeline;

    private VisionPortal visionPortal;

    private static final String[] LABELS = {
            "Box",
    };

    public enum MarkerState {LEFT, CENTER, RIGHT}

    MarkerState markerPos = MarkerState.CENTER;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        this.robot = new Robot(hardwareMap, true);

        //start position
        Pose2d startPose = new Pose2d(-36, 62, Math.toRadians(-90));
        robot.driveSubsystem.setPoseEstimate(startPose);
        robot.reset();

        initTfod();

        //Center tape
        TrajectorySequence toCenterTape = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, 15))
                .build();
        TrajectorySequence toCenterPark = robot.driveSubsystem.trajectorySequenceBuilder(toCenterTape.end())
                .lineTo(new Vector2d(-36, 12))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(55, 12))
                .build();

        //Right tape
        TrajectorySequence toRightTape = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, -30))
                .lineTo(new Vector2d(-34, -30))
                .build();
        TrajectorySequence toRightPark = robot.driveSubsystem.trajectorySequenceBuilder(toRightTape.end())
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-36, -12))
                .lineTo(new Vector2d(55, -12))
                .build();

        //Left tape
        TrajectorySequence toLeftTape = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, -30))
                .turn(Math.toRadians(-90))
                .lineTo(new Vector2d(-38, -30))
                .build();
        TrajectorySequence toLeftPark = robot.driveSubsystem.trajectorySequenceBuilder(toLeftTape.end())
                .lineTo(new Vector2d(-36, -12))
                .lineTo(new Vector2d(55, -12))
                .build();


        while (!isStarted() && !isStopRequested()) {
            robot.reset();

            for (LynxModule module : robot.getControllers()) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
            //TFOD


            telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
            telemetryTfod();
            telemetry.update();
        }

        switch(markerPos){
            case CENTER:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toCenterTape),
                                new IntakeRunCommand(robot.intake, -0.40, .35),
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toCenterPark)
                        )
                );
                break;
            case LEFT:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toLeftTape),
                                new IntakeRunCommand(robot.intake, -0.40, .35),
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toLeftPark)
                        )
                );
                break;
            case RIGHT:
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toRightTape),
                                new IntakeRunCommand(robot.intake, -0.40, .35),
                                new TrajectorySequenceFollowerCommand(robot.driveSubsystem, toRightPark)
                        )
                );
                break;
        }


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

    public void initTfod() {

        tfod = new TfodProcessor.Builder()
                .setModelAssetName("BlueModel.tflite")

                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        builder.enableCameraMonitoring(true);
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.80f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }

    /**
     * Function to add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("", " ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (x >= 0 && x <= 300) {
                markerPos = MarkerState.RIGHT;
            } else if (x >= 300 && x <= 800) {
                markerPos = MarkerState.CENTER;
            } else if (x >= 800 && x <= 1200) {
                markerPos = MarkerState.LEFT;
            }
        }
    }

}

