package org.firstinspires.ftc.teamcode.Common;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.rr.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

//@Autonomous
public class BLUE_LEFT_AUTONOMOUS extends LinearOpMode {

    private Robot robot;

    OpenCvCamera camera;

    // camera settings (need to adjust for our cam)
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    int LEFT = 16;
    int MIDDLE = 14;
    int RIGHT = 19;

    OpenCvPipeline tensorFlowRedMarkerDetection;

    @Override
    public void runOpMode(){
        CommandScheduler.getInstance().reset();
        this.robot = new Robot(hardwareMap, true);

        //start position
        Pose2d startPose = new Pose2d(11.5, 62, Math.toRadians(-90));
        robot.driveSubsystem.setPoseEstimate(startPose);
        robot.reset();

        //Left tape
        TrajectorySequence toCenterTape = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(12, 42), Math.toRadians(-110))
                .splineTo(new Vector2d(12, 15), Math.toRadians(-90))
                .build();

        //Center tape
        TrajectorySequence toRightTape = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(12, 42), Math.toRadians(-110))
                .splineTo(new Vector2d(11, 30), Math.toRadians(0))
                .build();

        //Right tape
        TrajectorySequence toLeftTape = robot.driveSubsystem.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(12, 42), Math.toRadians(-110))
                .splineTo(new Vector2d(32, 30), Math.toRadians(0))
                .build();

        TrajectorySequence toRightBackdrop = robot.driveSubsystem.trajectorySequenceBuilder(toRightTape.end())
                .splineTo(new Vector2d(49, 28.5), Math.toRadians(0))
                .build();

        TrajectorySequence toLeftBackdrop = robot.driveSubsystem.trajectorySequenceBuilder(toLeftTape.end())
                .splineTo(new Vector2d(49, 41.5), Math.toRadians(0))
                .build();

        TrajectorySequence toCenterBackdrop = robot.driveSubsystem.trajectorySequenceBuilder(toCenterTape.end())
                .splineTo(new Vector2d(49, 35), Math.toRadians(0))
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        tensorFlowRedMarkerDetection = new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                return null;
            }
        };

        camera.setPipeline(tensorFlowRedMarkerDetection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        while(!isStarted() && !isStopRequested()){
            robot.reset();

            for(LynxModule module : robot.getControllers()){
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }

//            ArrayList<AprilTagDetection> currentDetections = tensorFlowRedMarkerDetection.getLatestDetections();

//            if(currentDetections.size() != 0){
//                boolean tagFound = false;
//
//                for (AprilTagDetection tag : currentDetections) {
//                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                        if (tagFound) {
//                            telemetry.addLine("Tag of interest is in sight!/n/nLocation data:");
//                            tagToTelemetry(tagOfInterest);
//                        } else {
//                            telemetry.addLine("Don't see tag of interest :(");
//
//                            if (tagOfInterest == null) {
//                                telemetry.addLine("(The tag has never been seen)");
//                            } else {
//                                telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                                tagToTelemetry(tagOfInterest);
//                            }
//                        }
//
//                    } else {
//                        telemetry.addLine("Don't see tag of interest :(");
//
//                        if (tagOfInterest == null) {
//                            telemetry.addLine("(The tag has never been seen)");
//                        } else {
//                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                            tagToTelemetry(tagOfInterest);
//                        }
//
//                    }
//
//                    telemetry.update();
//                    sleep(20);
//                }
//                if (tagOfInterest != null) {
//                    telemetry.addLine("Tag snapshot:/n");
//                    tagToTelemetry(tagOfInterest);
//                    telemetry.update();
//                } else {
//                    telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//                    telemetry.update();
//                }
//                if (tagOfInterest == null) {
//                    CommandScheduler.getInstance().schedule(
//                            new SequentialCommandGroup(
//                                    //idk
//                            )
//                    );
//                } else if (tagOfInterest.id == 16) { //Left tape
//                    CommandScheduler.getInstance().schedule(
//                            new SequentialCommandGroup(
//                                    new AutoPreloadCommandV2Medium(robot, toLeftTape),
//                                    new WaitCommand(10),
//
//
//                                    )
//                    );
//
//                } else if (tagOfInterest.id == 14) { //Center tape
//                    CommandScheduler.getInstance().schedule(
//                            new SequentialCommandGroup(
//                                    new AutoPreloadCommandV2Medium(robot, toCenterTape),
//                                    new WaitCommand(10),
//
//
//                                    )
//                    );
//
//                } else if (tagOfInterest.id == 19) { //Right tape
//                    CommandScheduler.getInstance().schedule(
//                            new SequentialCommandGroup(
//                                    new AutoPreloadCommandV2Medium(robot, toRightTape),
//                                    new WaitCommand(10),
//
//
//                                    )
//                    );
//
//                }
//
//            }
//
//        }

            robot.reset();

            while (opModeIsActive()) {
                robot.read();

                CommandScheduler.getInstance().run();


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

}
