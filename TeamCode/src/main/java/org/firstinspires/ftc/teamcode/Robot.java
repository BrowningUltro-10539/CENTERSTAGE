package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.rr.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HangerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.List;

public class Robot {
    public MecanumDriveSubsystem driveSubsystem;

    public IntakeSubsystem intake;
    public LiftSubsystem lift;
    public HangerSubsystem hanger;
//    public HangerAngleSubsystem hangerAng;
    public OuttakeSubsystem outtake;
    public AirplaneSubsystem launcher;

    private boolean isAuto = false;

    public Pose2d robotPose;

    private List<LynxModule> controllers;

    public Robot(HardwareMap hardwareMap, boolean isAuto){
        this.isAuto = isAuto;


        driveSubsystem = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap), true);
        intake = new IntakeSubsystem(hardwareMap, isAuto);
        lift = new LiftSubsystem(hardwareMap, isAuto);
//        hanger = new HangerSubsystem(hardwareMap, isAuto);
        outtake = new OuttakeSubsystem(hardwareMap, isAuto);
//        hangerAng = new HangerAngleSubsystem(hardwareMap, isAuto);
        launcher = new AirplaneSubsystem(hardwareMap);
        hanger = new HangerSubsystem(hardwareMap);

        if(isAuto){
            lift.lift.encoder.reset();
//            hanger.hangerMotor.encoder.reset();
        }

        controllers = hardwareMap.getAll(LynxModule.class);
    }

    public Robot(HardwareMap hardwareMap){
        this(hardwareMap, false);
    }

    public void read(){
        intake.read();
        lift.read();
//        hanger.read();
        outtake.read();
//        hangerAng.read();
        launcher.read();

        if(isAuto){
            driveSubsystem.getPoseEstimate();
        }
    }

    public void write(){
        intake.write();
        lift.write();
//        hanger.write();
        outtake.write();
//        hangerAng.write();
        launcher.write();

        if(isAuto){
            driveSubsystem.update();
        }
    }

    public void reset(){
        lift.lift.resetEncoder();
//        hanger.hangerMotor.resetEncoder();
//        hangerAng.hangerAngleMotor.resetEncoder();
        launcher.update(AirplaneSubsystem.airplaneServoState.CLOSED);
        outtake.update(OuttakeSubsystem.ArmState.INTAKE);
        hanger.update(HangerSubsystem.ServoState.DOWN);
    }

    public List<LynxModule> getControllers(){
        return controllers;
    }

    public Pose2d getPose(){
        return robotPose;
    }


}
