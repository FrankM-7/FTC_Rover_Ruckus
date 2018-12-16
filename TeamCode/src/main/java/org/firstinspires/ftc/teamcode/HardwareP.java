

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class HardwareP
{

    public DcMotor  rightFront   = null;
    public DcMotor  leftFront  = null;
    public DcMotor  rightBack     = null;
    public DcMotor  leftBack   = null;
    public DcMotor  hingeMotor = null;
    public Servo    drop =null;
    int zAccumulated;  //Total rotation left/right
    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
    //  public WebcamName webcamName1 = null;
    //public Servo    leftClaw    = null;
    // public Servo    rightClaw   = null;

    // public static final double MID_SERVO       =  0.5 ;
    //public static final double ARM_UP_POWER    =  0.45 ;
    //  //public static final double ARM_DOWN_POWER  = -0.45 ;


    HardwareMap hwMap1           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public HardwareP(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap1) {
        // Save reference to Hardware map
        hwMap1 = ahwMap1;

        // Define and Initialize Motors
        rightFront  = hwMap1.get(DcMotor.class, "rightFront");
        leftFront  = hwMap1.get(DcMotor.class, "leftFront");
        rightBack = hwMap1.get(DcMotor.class, "rightBack");
        leftBack    = hwMap1.get(DcMotor.class, "leftBack");
        hingeMotor = hwMap1.get(DcMotor.class, "hingeMotor");
        drop = hwMap1.get(Servo.class,"drop");
        sensorGyro = hwMap1.gyroSensor.get("gyro");

        //webcamName1 =hwMap1.get(WebcamName.class, "Webcam 1");

        rightFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        hingeMotor.setDirection(DcMotor.Direction.REVERSE);

        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID

        // Set all motors to zero power
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
        hingeMotor.setPower(0);

        drop.setPosition(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hingeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        // leftClaw  = hwMap.get(Servo.class, "left_hand");
        //  rightClaw = hwMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(MID_SERVO);
        // rightClaw.setPosition(MID_SERVO);
    }
}


