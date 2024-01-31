package org.firstinspires.ftc.teamcode.Common;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.checkerframework.checker.units.qual.A;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.Teleop.DepositAndRetractCommand;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.old.HangerAngleSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.List;

@TeleOp
public class DriveOpmode extends CommandOpMode {
    private Robot robot;




    //    StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, );
    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, false);
        robot.reset();



//        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, , );


        for (LynxModule module : robot.getControllers()){
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

//        myLocalizer.setPoseEstimate(PoseStorage.currentPose);


    }



    @Override
    public void run(){

        robot.read();

//        myLocalizer.update();


        if(gamepad1.right_trigger > 0.25){
            schedule(new InstantCommand(() ->robot.intake.update(IntakeSubsystem.IntakeState.INTAKE)));
        } else if (gamepad1.left_trigger > 0.25) {
            schedule(new InstantCommand(() ->robot.intake.update(IntakeSubsystem.IntakeState.PUSH_OUT)));
        }else{
            schedule(new InstantCommand(() ->robot.intake.update(IntakeSubsystem.IntakeState.PAUSE)));
        }

        if(gamepad1.cross){
            schedule(new InstantCommand(() -> robot.launcher.update(AirplaneSubsystem.airplaneServoState.OPEN)));
        }

        schedule(new MecanumDriveCommand(robot.driveSubsystem, () -> -gamepad1.left_stick_y, ()-> -gamepad1.left_stick_x, () -> gamepad1.right_stick_x));

        if(gamepad2.dpad_up){
            schedule(new LiftPositionCommand(robot.lift, 15, 200, 200, 2));
        }

        if(gamepad2.dpad_left){
            schedule(new LiftPositionCommand(robot.lift, 23, 200, 200, 2));
        }

        if(gamepad2.dpad_down){
            schedule(new LiftPositionCommand(robot.lift, 0, 200, 200, 2));
        }

        if(gamepad2.cross) {
            schedule(new DepositAndRetractCommand(robot));
        }

//        if(gamepad2.square){
//            schedule(new ParallelCommandGroup(
//                    new HangerAnglePositionCommand(robot.hangerAng, 90, 100, 100, 1),
//                    new HangerPositionCommand(robot.hanger, 7.5, 200, 200, 1)
//            ));
//        }
//
//        if(gamepad2.triangle){
//            schedule(new ParallelCommandGroup(
//                    new HangerAnglePositionCommand(robot.hangerAng, 45, 100, 100, 1),
//                    new HangerPositionCommand(robot.hanger, 4, 100, 100, 2)
//            ));
//        }
//
//        if(gamepad2.circle){
//            schedule(new ParallelCommandGroup(
//                    new HangerAnglePositionCommand(robot.hangerAng, 0, 100, 100, 1),
//                    new HangerPositionCommand(robot.hanger, 0, 100, 100, 2)
//            ));
//        }

        robot.lift.loop();
//        robot.hanger.loop();
        robot.outtake.loop();
        robot.intake.loop();
//        robot.hangerAng.loop();
        robot.launcher.loop();

        CommandScheduler.getInstance().run();

        robot.write();

        for(LynxModule module : robot.getControllers()){
            module.clearBulkCache();
        }




    }

    @Override
    public void reset(){
        CommandScheduler.getInstance().reset();
    }

}