package org.firstinspires.ftc.teamcode.Common;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.LiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.Teleop.DepositAndRetractCommand;
import org.firstinspires.ftc.teamcode.subsystems.AirplaneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@TeleOp
public class DriveOpmode extends CommandOpMode {
    private Robot robot;

    private NavxMicroNavigationSensor NAV_X;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;


    //    StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, );
    @Override
    public void initialize(){
        CommandScheduler.getInstance().reset();

        robot = new Robot(hardwareMap, false);
        robot.reset();
// Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        NAV_X = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");


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

//        schedule(new MecanumDriveCommand(robot.driveSubsystem, () -> -gamepad1.left_stick_y, ()-> -gamepad1.left_stick_x, () -> gamepad1.right_stick_x));

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
            schedule(new InstantCommand(() -> robot.outtake.update(OuttakeSubsystem.ArmState.RELEASE)));
        }

        if(gamepad2.square){
            schedule(new DepositAndRetractCommand(robot));
        }


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.

        double botHeading = Math.toRadians(90) + NAV_X.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

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