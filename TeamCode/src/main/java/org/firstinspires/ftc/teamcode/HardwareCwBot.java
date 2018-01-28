package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareCwBot
{
    /* Public OpMode members. */
    public DcMotor rightRearMotor = null;
    public DcMotor leftRearMotor = null;
    public Servo    phone       = null;

    DeviceInterfaceModule dim;
    AnalogInput dsFront;
    AnalogInput dsLeft;
    double systemVoltage;

    public static final double MID_SERVO       =  0.4 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareCwBot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        dim = hwMap.get(DeviceInterfaceModule.class, "DIM1");   //  Use generic form of device mapping
//        AnalogInput ds = hardwareMap.get(AnalogInput.class, "Ultrasound");
        dsFront = new AnalogInput(dim,7);
        dsLeft = new AnalogInput(dim,6);
        systemVoltage = dsFront.getMaxVoltage();

        // Define and Initialize Motors
        rightRearMotor = hwMap.dcMotor.get("RightRear");
        leftRearMotor = hwMap.dcMotor.get("LeftRear");
        rightRearMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        rightRearMotor.setPower(0.0);
        leftRearMotor.setPower(0.0);
        //rightRearMotor.setMaxSpeed(2000); // Defaults to 1120???

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        phone = hwMap.servo.get("phone");
        phone.setPosition(MID_SERVO);
    }

    public double getFrontDistance() // in cm
    {
        return Math.round(dsFront.getVoltage()/systemVoltage*1024.0);
    }

    public double getLeftDistance() // in cm
    {
        return Math.round(dsLeft.getVoltage()/systemVoltage*1024.0);
    }

    public void setBlueLED(boolean on)
    {
        dim.setLED(0,on);
    }

    public void setRedLED(boolean on)
    {
        dim.setLED(1,on);
    }

    public void resetTickPeriod()
    {
        period.reset();
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

