/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="cwbot: Telop Tank", group="cwbot")
public class cwbotTeleopTank_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCwBot   robot           = new HardwareCwBot();              // Use a K9'shardware
    // The IMU sensor object
    BNO055IMU imu;

    DcMotor[] allMotors;
    double[] powerFactor;
    ElapsedTime timer = new ElapsedTime();

    final double ticksPerCm = 1.0/.02879; // 1.0/0.02905;
    final double ticksPerInch = 2.54 * ticksPerCm;

    final int DRIVE = 0;
    final int GOTOHEADING = 1;
    final int RIGHTWHEELPIVOT = 2;
    final int LEFTWHEELPIVOT = 3;
    final int CHECKIMU = 4;

    int[] BlueNearProgram =
            {
                    DRIVE, (int) (24.0*ticksPerInch),
                    CHECKIMU, 0,
                    RIGHTWHEELPIVOT, -27,
                    DRIVE, (int) (20.0*ticksPerInch),
                    RIGHTWHEELPIVOT, 90
            };

//    After BlueNearProgram we have readings: Front voltage 0.313, left voltage 0.528
//      Measuring, we get front = 25 1/2", left = 42 3/4"
//      The stone is not centered at 24", but instead at 23 5/16"

    int[] BlueFarProgram =
            {
                    DRIVE, (int) (24.0*ticksPerInch),
                    CHECKIMU, 0,
                    LEFTWHEELPIVOT, 90,
                    DRIVE, (int) (4.0 * ticksPerInch)
            };
    int[] BlueFarMultiGlyph =
            {
                    DRIVE, (int) (-12.0 * ticksPerInch),
                    RIGHTWHEELPIVOT, -75,
                    DRIVE, (int) (29.0 * ticksPerInch),
                    RIGHTWHEELPIVOT, 0,
                    DRIVE, (int) (-9.0 * ticksPerInch),
                    LEFTWHEELPIVOT, 90,
                    DRIVE, (int) (-2.0 * ticksPerInch)
            };

    BNO055IMU.Parameters parameters;

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // A B
        allMotors = new DcMotor[] {robot.leftRearMotor, robot.rightRearMotor};
        powerFactor = new double[] {1.0, 1.0};


        parameters = new BNO055IMU.Parameters();
        parameters.accelRange = BNO055IMU.AccelRange.G16;
        parameters.gyroRange = BNO055IMU.GyroRange.DPS500;
        //parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;gyr
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        robot.setBlueLED(false);
        robot.setRedLED(false);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.right_bumper)
            {
                /*
                // Near Blue Autonomous
                RunProgram(BlueNearProgram);
                // Wait for ultrasound sensors to converge.
                robot.resetTickPeriod();
                robot.waitForTick(1500);
                double frontDistance = robot.getFrontDistance()/2.54; // inches
                double leftDistance = robot.getLeftDistance()/2.54; // inches
                // We expect about 25" in front. If it is significantly less, we assume that
                // we are seeing the cryptobox columns, and therefore add 4".

                telemetry.addData("measure","front %.1f left %.1f", frontDistance, leftDistance);
                telemetry.update();
                robot.waitForTick(4000);

                if (frontDistance > 30.0)
                {
                    frontDistance = 25.0;
                    robot.setBlueLED(true);
                }
                if (frontDistance < 23.0) frontDistance += 4.0;
                // Move up halfway to cryptobox.
                double deltaY = (frontDistance - 4.0)/2.0;
                // Left sensor face is 8.0" from center of robot.
                // The center of this cryptobox is assumed to be 60.0 - 1.5 inches.
                int box = 0;
                double boxCenterX = 58.75 - 8.0 + 7.625 * box;
                double deltaX = boxCenterX - leftDistance;
                double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
                double heading = (180.0/Math.PI) * Math.atan2(deltaY, deltaX);

                PivotOnRightWheel(heading);
                RunToEncoder2(distance * ticksPerInch);
                PivotOnRightWheel(90.0);

                robot.resetTickPeriod();
                robot.waitForTick(1500);
                leftDistance = robot.getLeftDistance()/2.54; // inches

                telemetry.addData("measure","left %.1f", leftDistance);
                telemetry.update();
                robot.waitForTick(4000);

                deltaX = boxCenterX - leftDistance;
                if (Math.abs(deltaX) > 0.5/2.54)
                {
                    deltaY -= WiggleWalk(deltaX,16.5,90.0);

                    robot.resetTickPeriod();
                    robot.waitForTick(1500);
                    leftDistance = robot.getLeftDistance()/2.54; // inches

                    telemetry.addData("measure","left %.1f", leftDistance);
                    telemetry.update();
                    robot.waitForTick(4000);
                    deltaX = boxCenterX - leftDistance;
                    if (Math.abs(deltaX) > 0.5/2.54)
                    {
                        RunToEncoder2(-2.0 * ticksPerInch);
                        deltaY += 2.0;
                        deltaY -= WiggleWalk(deltaX, 16.5, 90.0);
                    }
                }


//                heading = (180.0/Math.PI) * Math.atan2(deltaY, deltaX);
//                PivotOnRightWheel(heading);
                RunToEncoder2(deltaY * ticksPerInch);
                */
                RunToEncoder2(2000);
            }
            if (gamepad1.left_bumper)
            {
                RunProgram(BlueFarProgram);
                ParkBlueFar(34.25);
                // Try MultiGlyph
                robot.resetTickPeriod();
                robot.waitForTick(1000);

                RunProgram(BlueFarMultiGlyph);
                ParkBlueFar(34.5+7.63);
            }

            float x = gamepad1.right_stick_x;
            float y = -gamepad1.right_stick_y; // Negate to get +y forward.
            float rotation = gamepad1.left_stick_x;

            // A B

            float b = y - rotation;
            float a = y + rotation;

            float biggest = Math.max(Math.abs(a),Math.abs(b));
            if (biggest < 1.0f) biggest = 1.0f;

            robot.rightRearMotor.setPower(b/biggest);
            robot.leftRearMotor.setPower(a/biggest);

            int encoderA = robot.leftRearMotor.getCurrentPosition();
            int encoderB = robot.rightRearMotor.getCurrentPosition();

            Quaternion q = imu.getQuaternionOrientation();
            // The sonar only refreshes at 6.7 Hz.
            // We will average over 1 second to reduce noise.
            double vFront = robot.getFrontDistance();
            double vLeft = robot.getLeftDistance();
            telemetry.addData("Q", "%.5f %.5f %.5f %.5f",q.w,q.x,q.y,q.z);
            telemetry.addData("heading", "%.1f",getHeading());
            telemetry.addData("Encoders","%d %d", encoderA,encoderB);
            telemetry.addData("ds",  "%.2f %.2f", vFront, vLeft);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            //sleep(40);
            robot.waitForTick(40);
        }
    }

    void ParkBlueFar(double xTarget) // 34.5 for center
    {
        // Wait for ultrasound sensors to converge.
        robot.resetTickPeriod();
        robot.waitForTick(1500);
        double frontDistance = robot.getFrontDistance()/2.54; // inches
        double leftDistance = robot.getLeftDistance()/2.54; // inches

        telemetry.addData("measure","front %.1f left %.1f", frontDistance, leftDistance);
        telemetry.update();
        robot.waitForTick(2000);

        // Move up halfway to cryptobox.
        double deltaY = (frontDistance - 4.0)/2.0;
        // Left sensor face is 8.0" from center of robot.
        // The center of this cryptobox is assumed to be 36.0 - 1.5 inches.
        double boxCenterX = xTarget - 8.0;
        double deltaX = boxCenterX - leftDistance;
        double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
        double heading = (180.0/Math.PI) * Math.atan2(deltaY, deltaX);

        PivotOnLeftWheel(heading);
        RunToEncoder2(distance * ticksPerInch);
        PivotOnLeftWheel(90.0);

        robot.resetTickPeriod();
        robot.waitForTick(1500);
        leftDistance = robot.getLeftDistance()/2.54; // inches
        deltaY += 2.0;
//        telemetry.addData("measure","left %.1f", leftDistance);
//        telemetry.update();
//        robot.waitForTick(2000);

        deltaX = boxCenterX - leftDistance;
        if (Math.abs(deltaX) > 0.5/2.54)
        {
            deltaY -= WiggleWalk(deltaX,16.5,90.0);

            robot.resetTickPeriod();
            robot.waitForTick(1500);
            leftDistance = robot.getLeftDistance()/2.54; // inches

//            telemetry.addData("measure","left %.1f", leftDistance);
//            telemetry.update();
//            robot.waitForTick(2000);
            deltaX = boxCenterX - leftDistance;
            if (Math.abs(deltaX) > 0.5/2.54) {
                RunToEncoder2(-2.0 * ticksPerInch);
                deltaY += 2.0;
                deltaY -= WiggleWalk(deltaX, 16.5, 90.0);
            }
        }
        RunToEncoder2(deltaY * ticksPerInch);
    }

    double WiggleWalk(double deltaX, double wheelBase, double heading)
    {
        double theta;
        if (deltaX < 0.0)
        {
            theta = 180.0/Math.PI*Math.acos(1.0+deltaX/wheelBase);
            PivotOnLeftWheel(heading+theta);
            PivotOnRightWheel(heading);
        }
        else
        {
            theta = 180.0/Math.PI*Math.acos(1.0-deltaX/wheelBase);
            PivotOnRightWheel(heading-theta);
            PivotOnLeftWheel(heading);
        }
        return wheelBase * Math.sin(Math.PI/180*theta);
    }


    void RunProgram(int[] prog)
    {
        int  nSteps = prog.length/2;
        for (int iStep = 0; iStep<nSteps; iStep++)
        {
            int instruction = prog[2*iStep];
            int data = prog[2*iStep+1];
            switch (instruction)
            {
                case DRIVE:
                    RunToEncoder2(data);
                    break;
                case GOTOHEADING:
                    TurnToHeading((double) data);
                    break;
                case RIGHTWHEELPIVOT:
                    PivotOnRightWheel((double) data);
                    break;
                case LEFTWHEELPIVOT:
                    PivotOnLeftWheel((double) data);
                    break;
                case CHECKIMU:
                    Quaternion q = imu.getQuaternionOrientation();
                    if (q.magnitude() < 0.9)
                    {
                        telemetry.addData("Say","IMU died!!!");
                        telemetry.update();
                        imu.initialize(parameters);
                        robot.setRedLED(true);
                        robot.waitForTick(2000);
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
            telemetry.addData("Say", "IMU died!");
            telemetry.update();
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

    void RunToEncoder2(double ticks)
    {
        RunToEncoder2((int)ticks);
    }

    void RunToEncoder2(int ticks)
    {
        int maxSetSpeed = 1120;
        double maxSpeed = 800.0;
        int maxAcceleration = 4000;
        int maxDeceleration = 8000;
        double multiplier = 2.0;
        double deltaT = 0.020;

        int n = allMotors.length;
        int[] targets = new int[n];
        for (int i=0; i<n; i++) {
            DcMotor motor = allMotors[i];
            targets[i] = motor.getCurrentPosition() + ticks;
        }
        double[] lastSpeeds = new double[n];
        double lastTime = timer.time()-deltaT;
        int remainingTicks = Math.abs(ticks);
        double highestSpeed = 0.0;
        double dt = 0.020;
        while (opModeIsActive() && (remainingTicks > 6 || highestSpeed > 2*dt*maxDeceleration))
        {
            double time = timer.time();
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
            telemetry.addData("delta t","%.3f",dt);
            telemetry.update();
            robot.waitForTick(20);
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

    void TurnToHeading(double target)
    {
        int maxSetSpeed = 1120;
        double maxSpeed = 800.0;
        int maxAcceleration = 2000;
        int maxDeceleration = 4000;
        double multiplier = 10.0;
        double dt = 0.020;

        int n = allMotors.length;

        double lastSpeed = 0.0;
        double lastTime = timer.time()-dt;
        double remainingAngle = diffHeading(target);
        while (opModeIsActive() && (Math.abs(remainingAngle) > 0.5 || Math.abs(lastSpeed) > dt*maxDeceleration))
        {
            double time = timer.time();
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
            allMotors[1].setPower(newPower);
            lastSpeed = newSpeed;
            robot.waitForTick(20);
            remainingAngle = diffHeading(target);
        }
        for (int i=0; i<n; i++)
        {
            allMotors[i].setPower(0.0);
        }
    }

    void PivotOnLeftWheel(double target)
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
        double lastTime = timer.time()-dt;
        double remainingAngle = diffHeading(target);
        while (opModeIsActive() && (Math.abs(remainingAngle) > 0.5 || Math.abs(lastSpeed) > dt*maxDeceleration))
        {
            double time = timer.time();
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
            robot.waitForTick(20);
            remainingAngle = diffHeading(target);
        }
        for (int i=0; i<n; i++)
        {
            allMotors[i].setPower(0.0);
        }
    }

    void PivotOnRightWheel(double target)
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
        double lastTime = timer.time()-dt;
        double remainingAngle = diffHeading(target);
        while (opModeIsActive() && (Math.abs(remainingAngle) > 0.5 || Math.abs(lastSpeed) > dt*maxDeceleration))
        {
            double time = timer.time();
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
            robot.waitForTick(20);
            remainingAngle = diffHeading(target);
        }
        for (int i=0; i<n; i++)
        {
            allMotors[i].setPower(0.0);
        }
    }


    void RunToEncoder(int ticks)
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
        while (opModeIsActive() && !allDone)
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

    void ResetMotor(DcMotor motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
