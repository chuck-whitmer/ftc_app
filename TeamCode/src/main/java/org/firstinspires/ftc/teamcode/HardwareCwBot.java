package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class HardwareCwBot
{
    /* Public OpMode members. */
    public DcMotor backRight = null;
    public DcMotor backLeft = null;
    public DcMotor frontRight = null;
    public DcMotor frontLeft = null;
    public Servo    phone       = null;

    DeviceInterfaceModule dim;
    AnalogInput dsFront;
    AnalogInput dsLeft;
    double systemVoltage;

    public static final double MID_SERVO =  0.4 ;

    // The IMU sensor object
    BNO055IMU imu;
    BNO055IMU.Parameters imuParameters;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    DcMotor[] allMotors;
    static final int FL = 0;
    static final int FR = 1;
    static final int BL = 2;
    static final int BR = 3;
    double[] rotationArray;
    double[] powerFactor;
    ElapsedTime driveTimer = new ElapsedTime();

    /* Constructor */
    public HardwareCwBot()
    {}

    // 4" Tetrix wheels
    //public static final double ticksPerCm = 37.734; // 1.0/.02879 for Stealth // 1.0/0.02905 for Tetrix
    //public static final double ticksPerInch = 88.225; // = 2.54 * ticksPerCm;
    //public static final double wheelBase = 1455.7; // 16.5 * ticksPerInch;

    // Mecanum wheels - Neverest Orbital 20
    public static final double ticksPerCm = 17.29;
    public static final double ticksPerInch = 43.9; // = 2.54 * ticksPerCm;
    public static final double wheelBase = 1455.7; // 16.5 * ticksPerInch;

    public static int inches(double len)
    {
        return (int) Math.round(len * ticksPerInch);
    }

    public static int cms(double len)
    {
        return (int) Math.round(len * ticksPerCm);
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap)
    {
        Log.i("robot.init","init");

        // Save reference to Hardware map
        hwMap = ahwMap;
        dim = hwMap.get(DeviceInterfaceModule.class, "DIM1");   //  Use generic form of device mapping
//        AnalogInput ds = hardwareMap.get(AnalogInput.class, "Ultrasound");
        dsFront = new AnalogInput(dim,7);
        dsLeft = new AnalogInput(dim,6);
        systemVoltage = dsFront.getMaxVoltage();
        setBlueLED(false);
        setRedLED(false);

        // Define and Initialize Motors
        backRight = hwMap.dcMotor.get("backRight");
        backLeft = hwMap.dcMotor.get("backLeft");
        frontRight = hwMap.dcMotor.get("frontRight");
        frontLeft = hwMap.dcMotor.get("frontLeft");

        allMotors = new DcMotor[] {frontLeft, frontRight, backLeft, backRight};
        rotationArray = new double[]{-1.0, 1.0, -1.0, 1.0};
        powerFactor = new double[] {1.0, 1.0, 1.0, 1.0};

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : allMotors)
        {
            m.setPower(0.0);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Define and initialize ALL installed servos.
        phone = hwMap.servo.get("phone");
        phone.setPosition(MID_SERVO);

        imuParameters = new BNO055IMU.Parameters();
        imuParameters.accelRange = BNO055IMU.AccelRange.G16;
        imuParameters.gyroRange = BNO055IMU.GyroRange.DPS500;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "IMU");
        imu.initialize(imuParameters);
    }

    public double[] getDrivePowersFromAngle(double angle) {
        double[] unscaledPowers = new double[4];
        unscaledPowers[0] = Math.sin(angle + Math.PI / 4);
        unscaledPowers[1] = Math.cos(angle + Math.PI / 4);
        unscaledPowers[2] = unscaledPowers[1];
        unscaledPowers[3] = unscaledPowers[0];
        return unscaledPowers;
    }

    void DriveByPid(int ticks, LinearOpMode caller)
    {
        int timeStep = 20; // 20 msec
        double P = 0.0;
        double I = 0.0;
        double D = 0.0;

        // Experiment: Accelerate with a power ramp of 1.0 per second for 0.5 seconds,
        // then decelerate at the same rate. We travel 639 ticks.
        // => 2*(1/2*a*t^2) = 639 ticks.  a = 2556 ticks/sec/sec
        // Same experiment with same ramp rate for 1.0 seconds up and 1.0
        // seconds down gives a = 2542 ticks/sec/sec
        double fullPowerSpeed = 2548.0; // Ticks per second with power=1.0
        double acc = fullPowerSpeed; // Since we are happy to get there in 1 sec.
        // So ramp at fullPowerSpeed/sec, but max out the speed at 0.9 power, so we
        // have room to adjust upward.
        double rampUpDownDist = 2.0*(0.5*acc*0.9*0.9); // about 48 inches
        double t1;
        double t2;
        double t3;
        if (ticks < rampUpDownDist)
        {
            t1 = Math.sqrt(ticks/acc);
            t2 = t1;
            t3 = 2.0*t1;
        }
        else
        {
            t1 = 0.9;
            t2 = (ticks-rampUpDownDist)/(0.9*fullPowerSpeed) + t1;
            t3 = t2 + 0.9;
        }
        // Now we have a model of where we should be at any time.
        // for t<t1:  d = 0.5*acc*t^2
        // for t1<t<t2:  d = rampUpDownDist/2 + (t-t1)*0.9*fullPowerSpeed
        // for t>t2: d = rampUpDownDist/2 + (t-t1)*0.9*fullPowerSpeed
        //                  -0.5*acc*(t-t2)^2

        int pos0 = AverageEncoders();
        double startTime = driveTimer.time();
        while (caller.opModeIsActive() && driveTimer.time()-startTime < t3)
        {

        }
        Log.i("tag", "msg");



    }

    int AverageEncoders()
    {
        int sum = 0;
        for (DcMotor m : allMotors)
            sum += m.getCurrentPosition();
        return sum/4;
    }

    double WiggleWalk(int transverseTicks, double heading, LinearOpMode caller)
    {
        double theta;
        if (transverseTicks < 0)
        {
            theta = 180.0/Math.PI*Math.acos(1.0+transverseTicks/wheelBase);
            PivotOnLeftWheel(heading+theta, caller);
            PivotOnRightWheel(heading, caller);
        }
        else
        {
            theta = 180.0/Math.PI*Math.acos(1.0-transverseTicks/wheelBase);
            PivotOnRightWheel(heading-theta, caller);
            PivotOnLeftWheel(heading, caller);
        }
        return wheelBase * Math.sin(Math.PI/180*theta);
    }

    void RunToEncoder2(int ticks, LinearOpMode caller)
    {
        int maxSetSpeed = 1120;
        double maxSpeed = 800.0;
        int maxAcceleration = 4000;
        int maxDeceleration = 8000;
        double multiplier = 2.0;
        double deltaT = 0.020;

        //caller.telemetry.addData("Run2", "%d ticks", ticks);
        //caller.telemetry.update();
        //waitForTick(3000);

        int n = allMotors.length;
        int[] targets = new int[n];
        for (int i=0; i<n; i++) {
            DcMotor motor = allMotors[i];
            targets[i] = motor.getCurrentPosition() + ticks;
        }
        double[] lastSpeeds = new double[n];
        double lastTime = driveTimer.time()-deltaT;
        int remainingTicks = Math.abs(ticks);
        double highestSpeed = 0.0;
        double dt = 0.020;
        while (caller.opModeIsActive() && (remainingTicks > 6 || highestSpeed > 2*dt*maxDeceleration))
        {
            double time = driveTimer.time();
            dt = time - lastTime;
            lastTime = time;
            for (int i=0; i<n; i++)
            {
                double maxNewSpeed;
                double minNewSpeed;

                double newSpeed = multiplier * (targets[i] - allMotors[i].getCurrentPosition());
                if (newSpeed > 0.0)
                {
                    maxNewSpeed = Math.min(lastSpeeds[i] + dt*maxAcceleration,maxSpeed);
                    minNewSpeed = lastSpeeds[i] - dt*maxDeceleration;
                }
                else
                {
                    maxNewSpeed = lastSpeeds[i] + dt*maxDeceleration;
                    minNewSpeed = Math.max(lastSpeeds[i] - dt*maxAcceleration,-maxSpeed);
                }
                if (newSpeed > maxNewSpeed) newSpeed = maxNewSpeed;
                if (newSpeed < minNewSpeed) newSpeed = minNewSpeed;
                double newPower  = newSpeed/ maxSetSpeed;
                if (Math.abs(newPower)<0.25)
                    newPower = Math.signum(newPower)*0.25;
                allMotors[i].setPower(newPower);
                lastSpeeds[i] = newSpeed;
            }
            //caller.telemetry.addData("remain", "%d", remainingTicks);
            //caller.telemetry.update();
            waitForTick(20);
            remainingTicks = 0;
            highestSpeed = 0.0;
            for (int i=0; i<n; i++)
            {
                int dist = Math.abs(targets[i] - allMotors[i].getCurrentPosition());
                if (remainingTicks < dist) remainingTicks = dist;
                double speed = Math.abs(lastSpeeds[i]);
                if (highestSpeed < speed) highestSpeed = speed;
            }
        }
        for (int i=0; i<n; i++)
        {
            allMotors[i].setPower(0.0);
        }
    }

    void TestRun1(double rampTime, double runTime, double maxPower, LinearOpMode caller)
    {
        long timeStep = 50;  // 50 msec cycles
        double deltaT = timeStep / 1000.0;
        double endTime = 2.0 * rampTime + runTime;
        double t=0.0;
        double startTime = driveTimer.time();
        while (caller.opModeIsActive() && (t=driveTimer.time()-startTime)<endTime+0.4)
        {
            logEncoders(t);
            double f0 = ramp(rampTime,runTime,t);
            double f1 = ramp(rampTime,runTime,t+deltaT);
            double power = 0.5*(f0+f1)*maxPower;
            for (int i=0; i<allMotors.length; i++)
            {
                allMotors[i].setPower(power);
            }
            waitForTick(timeStep);
        }
        for (int i=0; i<allMotors.length; i++)
            allMotors[i].setPower(0.0);
        logEncoders(t);
    }

    void logEncoders(double t)
    {
        String msg = String.format("drive: %7.3f %6d %6d %6d %6d",
                t,
                allMotors[0].getCurrentPosition(),
                allMotors[1].getCurrentPosition(),
                allMotors[2].getCurrentPosition(),
                allMotors[3].getCurrentPosition()
        );
        Log.i("foo",msg);
    }

    double ramp(double rampTime, double runTime, double t)
    {
        double endTime = 2.0*rampTime + runTime;
        if (t < 0.0 || t > endTime)
            return 0.0;
        if (t < rampTime)
            return t/rampTime;
        else if (t < rampTime + runTime)
            return 1.0;
        else
            return (endTime - t)/ rampTime;
    }

    void RunToEncoder3(int ticks, LinearOpMode caller)
    {
        int maxSetSpeed = 1120;
        double maxSpeed = 400.0;   // was 800
        int maxAcceleration = 1000;  // was 4000
        int maxDeceleration = 2000;  // was 8000
        double multiplier = 0.10;
        double deltaT = 0.020;

        //caller.telemetry.addData("Run2", "%d ticks", ticks);
        //caller.telemetry.update();
        //waitForTick(3000);

        int leftTarget = 
                frontLeft.getCurrentPosition() + backLeft.getCurrentPosition() + 2*ticks;
        int rightTarget =
                frontRight.getCurrentPosition() + backRight.getCurrentPosition() + 2*ticks;
        double lastLeftSpeed = 0.0;
        double lastRightSpeed = 0.0;
        double lastTime = driveTimer.time()-deltaT;
        int remainingTicks = Math.abs(2*ticks);
        double highestSpeed = 0.0;
        double dt = 0.020;
        while (caller.opModeIsActive() && (remainingTicks > 12 || highestSpeed > 2*dt*maxDeceleration))
        {
            double time = driveTimer.time();
            dt = time - lastTime;
            lastTime = time;
            // Left
            {
                double maxNewSpeed;
                double minNewSpeed;
                int pos = frontLeft.getCurrentPosition() + backLeft.getCurrentPosition();
                double newSpeed = multiplier * (leftTarget - pos);
                if (newSpeed > 0.0)
                {
                    maxNewSpeed = Math.min(lastLeftSpeed + dt*maxAcceleration,maxSpeed);
                    minNewSpeed = lastLeftSpeed - dt*maxDeceleration;
                }
                else
                {
                    maxNewSpeed = lastLeftSpeed + dt*maxDeceleration;
                    minNewSpeed = Math.max(lastLeftSpeed - dt*maxAcceleration,-maxSpeed);
                }
                if (newSpeed > maxNewSpeed) newSpeed = maxNewSpeed;
                if (newSpeed < minNewSpeed) newSpeed = minNewSpeed;
                double newPower  = newSpeed/ maxSetSpeed;
                if (Math.abs(newPower)<0.25)
                    newPower = Math.signum(newPower)*0.25;
                frontLeft.setPower(newPower);
                backLeft.setPower(newPower);
                lastLeftSpeed = newSpeed;
            }
            // Right
            {
                double maxNewSpeed;
                double minNewSpeed;
                int pos = frontRight.getCurrentPosition() + backRight.getCurrentPosition();
                double newSpeed = multiplier * (rightTarget - pos);
                if (newSpeed > 0.0)
                {
                    maxNewSpeed = Math.min(lastRightSpeed + dt*maxAcceleration,maxSpeed);
                    minNewSpeed = lastRightSpeed - dt*maxDeceleration;
                }
                else
                {
                    maxNewSpeed = lastRightSpeed + dt*maxDeceleration;
                    minNewSpeed = Math.max(lastRightSpeed - dt*maxAcceleration,-maxSpeed);
                }
                if (newSpeed > maxNewSpeed) newSpeed = maxNewSpeed;
                if (newSpeed < minNewSpeed) newSpeed = minNewSpeed;
                double newPower  = newSpeed/ maxSetSpeed;
                if (Math.abs(newPower)<0.25)
                    newPower = Math.signum(newPower)*0.25;
                frontRight.setPower(newPower);
                backRight.setPower(newPower);
                lastRightSpeed = newSpeed;
            }
            //caller.telemetry.addData("remain", "%d", remainingTicks);
            //caller.telemetry.update();
            waitForTick(20);
            int leftDist =
                    leftTarget - frontLeft.getCurrentPosition() - backLeft.getCurrentPosition();
            int rightDist =
                    rightTarget - frontRight.getCurrentPosition() - backRight.getCurrentPosition();
            remainingTicks = Math.max(Math.abs(leftDist),Math.abs(rightDist));
            highestSpeed = Math.max(Math.abs(lastLeftSpeed),Math.abs(lastRightSpeed));
        }
        for (int i=0; i<allMotors.length; i++)
        {
            allMotors[i].setPower(0.0);
        }
    }

    void TurnToHeading(double target, LinearOpMode caller)
    {
        int maxSetSpeed = 1120;
        double maxSpeed = 800.0;
        int maxAcceleration = 2000;
        int maxDeceleration = 4000;
        double multiplier = 10.0;
        double dt = 0.020;

        int n = allMotors.length;

        double lastSpeed = 0.0;
        double lastTime = driveTimer.time()-dt;
        double remainingAngle = diffHeading(target);
        while (caller.opModeIsActive() && (Math.abs(remainingAngle) > 0.5 || Math.abs(lastSpeed) > dt*maxDeceleration))
        {
            double time = driveTimer.time();
            dt = time - lastTime;
            lastTime = time;
            double maxNewSpeed;
            double minNewSpeed;

            double newSpeed = multiplier * remainingAngle;
            if (newSpeed > 0.0)
            {
                maxNewSpeed = Math.min(lastSpeed + dt*maxAcceleration,maxSpeed);
                minNewSpeed = lastSpeed - dt*maxDeceleration;
            }
            else
            {
                maxNewSpeed = lastSpeed + dt*maxDeceleration;
                minNewSpeed = Math.max(lastSpeed - dt*maxAcceleration,-maxSpeed);
            }
            if (newSpeed > maxNewSpeed) newSpeed = maxNewSpeed;
            if (newSpeed < minNewSpeed) newSpeed = minNewSpeed;
            double newPower  = newSpeed/ maxSetSpeed;
            if (Math.abs(newPower)<0.07)
                newPower = Math.signum(newPower)*0.07;
            frontLeft.setPower(-newPower);
            backLeft.setPower(-newPower);
            frontRight.setPower(newPower);
            backRight.setPower(newPower);
            lastSpeed = newSpeed;
            waitForTick(20);
            remainingAngle = diffHeading(target);
        }
        for (int i=0; i< allMotors.length; i++)
        {
            allMotors[i].setPower(0.0);
        }
    }

    void PivotOnLeftWheel(double target, LinearOpMode caller)
    {
        int maxSetSpeed = 1120;
        double maxSpeed = 800.0;
        int maxAcceleration = 2000;
        int maxDeceleration = 4000;
        double multiplier = 10.0;
        double dt = 0.020;

        int n = allMotors.length;
        for (int i=0; i<n; i++)
        {
            allMotors[i].setPower(0.0);
        }

        double lastSpeed = 0.0;
        double lastTime = driveTimer.time()-dt;
        double remainingAngle = diffHeading(target);
        while (caller.opModeIsActive() && (Math.abs(remainingAngle) > 0.5 || Math.abs(lastSpeed) > dt*maxDeceleration))
        {
            double time = driveTimer.time();
            dt = time - lastTime;
            lastTime = time;
            double maxNewSpeed;
            double minNewSpeed;

            double newSpeed = multiplier * remainingAngle;
            if (newSpeed > 0.0)
            {
                maxNewSpeed = Math.min(lastSpeed + dt*maxAcceleration,maxSpeed);
                minNewSpeed = lastSpeed - dt*maxDeceleration;
            }
            else
            {
                maxNewSpeed = lastSpeed + dt*maxDeceleration;
                minNewSpeed = Math.max(lastSpeed - dt*maxAcceleration,-maxSpeed);
            }
            if (newSpeed > maxNewSpeed) newSpeed = maxNewSpeed;
            if (newSpeed < minNewSpeed) newSpeed = minNewSpeed;
            double newPower  = newSpeed/ maxSetSpeed;
            if (Math.abs(newPower)<0.07)
                newPower = Math.signum(newPower)*0.07;
            allMotors[1].setPower(newPower);
            lastSpeed = newSpeed;
            waitForTick(20);
            remainingAngle = diffHeading(target);
        }
        for (int i=0; i<n; i++)
        {
            allMotors[i].setPower(0.0);
        }
    }

    void PivotOnRightWheel(double target, LinearOpMode caller)
    {
        int maxSetSpeed = 1120;
        double maxSpeed = 800.0;
        int maxAcceleration = 2000;
        int maxDeceleration = 4000;
        double multiplier = 15.0;
        double dt = 0.020;

        int n = allMotors.length;
        for (int i=0; i<n; i++)
        {
            allMotors[i].setPower(0.0);
        }

        double lastSpeed = 0.0;
        double lastTime = driveTimer.time()-dt;
        double remainingAngle = diffHeading(target);
        while (caller.opModeIsActive() && (Math.abs(remainingAngle) > 0.5 || Math.abs(lastSpeed) > dt*maxDeceleration))
        {
            double time = driveTimer.time();
            dt = time - lastTime;
            lastTime = time;
            double maxNewSpeed;
            double minNewSpeed;

            double newSpeed = multiplier * remainingAngle;
            if (newSpeed > 0.0)
            {
                maxNewSpeed = Math.min(lastSpeed + dt*maxAcceleration,maxSpeed);
                minNewSpeed = lastSpeed - dt*maxDeceleration;
            }
            else
            {
                maxNewSpeed = lastSpeed + dt*maxDeceleration;
                minNewSpeed = Math.max(lastSpeed - dt*maxAcceleration,-maxSpeed);
            }
            if (newSpeed > maxNewSpeed) newSpeed = maxNewSpeed;
            if (newSpeed < minNewSpeed) newSpeed = minNewSpeed;
            double newPower  = newSpeed/ maxSetSpeed;
            if (Math.abs(newPower)<0.07)
                newPower = Math.signum(newPower)*0.07;
            allMotors[0].setPower(-newPower);
            lastSpeed = newSpeed;
            waitForTick(20);
            remainingAngle = diffHeading(target);
        }
        for (int i=0; i<n; i++)
        {
            allMotors[i].setPower(0.0);
        }
    }


    void RunToEncoder(int ticks, LinearOpMode caller)
    {
        int n = allMotors.length;
        int[] targets = new int[n];
        for (int i=0; i<n; i++)
        {
            DcMotor motor = allMotors[i];
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            targets[i] = (int)(ticks*powerFactor[i]);
            motor.setTargetPosition(targets[i]);
        }
        for (int i=0; i<allMotors.length; i++)
        {
            allMotors[i].setPower(0.5 * powerFactor[i]);
        }

        boolean[] isDone = new boolean[n];
        boolean allDone = false;
        while (caller.opModeIsActive() && !allDone)
        {
            allDone = true;
            for (int i=0; i<n; i++)
            {
                DcMotor motor = allMotors[i];
                if (!isDone[i])
                {
                    if (!motor.isBusy())
                    {
                        motor.setPower(0.0);
                        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        isDone[i] = true;
                    } else {
                        allDone = false;
                    }
                }
            }
        }
    }

    final public int DRIVE = 0;
    final public int GOTOHEADING = 1;
    final public int RIGHTWHEELPIVOT = 2;
    final public int LEFTWHEELPIVOT = 3;
    final public int CHECKIMU = 4;

    void RunProgram(int[] prog, LinearOpMode caller)
    {
        int  nSteps = prog.length/2;
        for (int iStep = 0; iStep<nSteps; iStep++)
        {
            int instruction = prog[2*iStep];
            int data = prog[2*iStep+1];
            switch (instruction)
            {
                case DRIVE:
                    RunToEncoder3(data, caller);
                    break;
                case GOTOHEADING:
                    TurnToHeading((double) data, caller);
                    break;
                case RIGHTWHEELPIVOT:
                    PivotOnRightWheel((double) data, caller);
                    break;
                case LEFTWHEELPIVOT:
                    PivotOnLeftWheel((double) data, caller);
                    break;
                case CHECKIMU:
                    Quaternion q = imu.getQuaternionOrientation();
                    if (q.magnitude() < 0.9)
                    {
                        caller.telemetry.addData("Say","IMU died!!!");
                        caller.telemetry.update();
                        imu.initialize(imuParameters);
                        setRedLED(true);
                        waitForTick(2000);
                    }
            }
        }
    }

    double getHeading()
    {
        Quaternion q = imu.getQuaternionOrientation();
        double c = q.w;
        double s = q.z;
        double norm = Math.sqrt(c*c+s*s);
        if (norm < 0.5)
        {
            //telemetry.addData("Say", "IMU died!");
            //telemetry.update();
            return 0.0;
        }
        c /= norm;
        s /= norm;
        double halfAngle = Math.atan2(s,c);
        return (360.0 / Math.PI * halfAngle + 900.0) % 360.0 - 180.0; // Return -180 to +180.
    }

    double diffHeading(double target)
    {
        double diff = target - getHeading();
        diff = (diff + 720.0) % 360.0;
        if (diff > 180.0) diff -= 360;
        return diff;
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
