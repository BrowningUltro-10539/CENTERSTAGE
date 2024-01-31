package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;

public class PoseStorage {
    //setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels));
    public static Pose2d currentPose = new Pose2d();
    public static List<Integer> lastTrackingEncPositions= new ArrayList<>();
    public static List<Integer> lastTrackingEncVels= new ArrayList<>();

}
