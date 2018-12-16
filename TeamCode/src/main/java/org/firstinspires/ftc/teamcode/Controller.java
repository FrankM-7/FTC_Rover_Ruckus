//hello
//hi
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Controller", group="4719")

public class Controller extends LinearOpMode {


    HardwareP robot = new HardwareP();

    @Override
    public void runOpMode() {


        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

           /// driveStraightForward();
            //driveStraightSideways();
            driveStraightFinal();


            if (gamepad1.dpad_up) {
                robot.hingeMotor.setPower(.6);

            } else if (gamepad1.dpad_down) {
                robot.hingeMotor.setPower(-.6);
            } else {
                robot.hingeMotor.setPower(0);
            }


        }

    }

    public void driveStraightForward() {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double leftFSpeed;
        double rightBSpeed;

        double target = robot.mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = robot.leftBack.getCurrentPosition();  //Starting position

        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (Math.abs(gamepad1.left_stick_y) > .4 && (gamepad1.left_stick_x) == 0) {  //While we have not passed out intended distance
            robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Current direction

            leftSpeed = -gamepad1.left_stick_y + (robot.zAccumulated - target) / 100;  //Calculate speed for each side
            rightSpeed = -gamepad1.left_stick_y - (robot.zAccumulated - target) / 100;
            leftFSpeed = 0 + (robot.zAccumulated-target)/ 100;
            rightBSpeed = 0 - (robot.zAccumulated - target) / 100;

            leftSpeed = com.qualcomm.robotcore.util.Range.clip(leftSpeed, -1, 1);
            rightSpeed = com.qualcomm.robotcore.util.Range.clip(rightSpeed, -1, 1);

            robot.leftBack.setPower(leftSpeed/2);
            robot.rightFront.setPower(rightSpeed/2);
            robot.rightBack.setPower(rightBSpeed);
            robot.leftFront.setPower(leftFSpeed);

            telemetry.addData("1. Left", robot.leftBack.getPower());
            telemetry.addData("2. Right", robot.rightFront.getPower());
            telemetry.addData("3. LeftF", robot.leftFront.getPower());
            telemetry.addData("4. RightB", robot.rightBack.getPower());
            telemetry.update();
        }

        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
    }
    public void driveStraightSideways() {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double leftFSpeed;
        double rightBSpeed;

        double target = robot.mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = robot.leftBack.getCurrentPosition();  //Starting position

        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (Math.abs(gamepad1.left_stick_x) > .4 && (gamepad1.right_stick_y) == 0) {  //While we have not passed out intended distance
            robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Current direction

            leftSpeed = 0 + (robot.zAccumulated - target) / 100;  //Calculate speed for each side
            rightSpeed = 0 - (robot.zAccumulated - target) / 100;
            leftFSpeed = gamepad1.left_stick_x + (robot.zAccumulated-target)/ 100;
            rightBSpeed = gamepad1.left_stick_x - (robot.zAccumulated - target) / 100;

            leftSpeed = com.qualcomm.robotcore.util.Range.clip(leftSpeed, -1, 1);
            rightSpeed = com.qualcomm.robotcore.util.Range.clip(rightSpeed, -1, 1);

            robot.leftBack.setPower(leftSpeed);
            robot.rightFront.setPower(rightSpeed);
            robot.rightBack.setPower(rightBSpeed/2);
            robot.leftFront.setPower(leftFSpeed/2);

            telemetry.addData("1. Left", robot.leftBack.getPower());
            telemetry.addData("2. Right", robot.rightFront.getPower());
            telemetry.addData("3. LeftF", robot.leftFront.getPower());
            telemetry.addData("4. RightB", robot.rightBack.getPower());
            telemetry.update();
        }

        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
    }
    public void driveStraightFinal() {
        double leftSpeed; //Power to feed the motors
        double rightSpeed;
        double leftFSpeed;
        double rightBSpeed;

        double target = robot.mrGyro.getIntegratedZValue();  //Starting direction
        double startPosition = robot.leftBack.getCurrentPosition();  //Starting position

        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0) {  //While we have not passed out intended distance
            robot.zAccumulated = robot.mrGyro.getIntegratedZValue();  //Current direction

            leftSpeed = -gamepad1.left_stick_y + (robot.zAccumulated - target) / 100;  //Calculate speed for each side
            rightSpeed = -gamepad1.left_stick_y - (robot.zAccumulated - target) / 100;
            leftFSpeed = gamepad1.left_stick_x + (robot.zAccumulated-target)/ 100;
            rightBSpeed = gamepad1.left_stick_x - (robot.zAccumulated - target) / 100;

            leftSpeed = com.qualcomm.robotcore.util.Range.clip(leftSpeed, -1, 1);
            rightSpeed = com.qualcomm.robotcore.util.Range.clip(rightSpeed, -1, 1);

            robot.leftBack.setPower(leftSpeed/2);
            robot.rightFront.setPower(rightSpeed/2);
            robot.rightBack.setPower(rightBSpeed);
            robot.leftFront.setPower(leftFSpeed);

            telemetry.addData("1. Left", robot.leftBack.getPower());
            telemetry.addData("2. Right", robot.rightFront.getPower());
            telemetry.addData("3. LeftF", robot.leftFront.getPower());
            telemetry.addData("4. RightB", robot.rightBack.getPower());
            telemetry.update();
        }

        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftFront.setPower(0);
    }
}
