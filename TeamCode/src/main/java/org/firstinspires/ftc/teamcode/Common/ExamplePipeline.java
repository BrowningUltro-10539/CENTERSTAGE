package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.BluePipeline;
import org.firstinspires.ftc.vision.VisionPortal;


public class ExamplePipeline extends LinearOpMode {

    private BluePipeline pipeline;
    private VisionPortal portal;

    @Override
    public void runOpMode(){
        pipeline = new BluePipeline();
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Position: ", pipeline.getMarkerPosition());
        }

    }
}
