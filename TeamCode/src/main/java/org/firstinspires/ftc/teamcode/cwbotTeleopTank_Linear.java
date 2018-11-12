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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@TeleOp(name="cwbot: Teleop Drive", group="cwbot")
public class cwbotTeleopTank_Linear extends LinearOpMode
{
    /* Declare OpMode members. */
    HardwareCwBot robot = new HardwareCwBot();



    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (drive r presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //robot.backRight.setPower(gamepad1.right_bumper ? 1.0 : 0.0);
            //robot.backLeft.setPower(gamepad1.left_bumper ? 1.0 : 0.0);


            float x = gamepad1.left_stick_x;
            float y = -gamepad1.left_stick_y; // Negate to get +y forward.
            float rotation = -gamepad1.right_stick_x;
            float speedControl = 0.5f*(1.0f + gamepad1.left_trigger);
            double biggestControl = Math.sqrt(x*x+y*y);
            double biggestWithRotation = Math.sqrt(x*x+y*y+rotation*rotation);

            double angle = Math.atan2(y,-x) - Math.PI/2.0;

            double[] powers = robot.getDrivePowersFromAngle(angle);
            double pow2 = 0.0;
            for (int i=0; i<robot.allMotors.length; i++)
            {
                double pow = powers[i]*biggestControl + rotation * robot.rotationArray[i];
                powers[i] = pow;
                pow2 += pow*pow;
            }

            if (biggestWithRotation != 0.0) {
                double scale = Math.sqrt(pow2);
                for (int i = 0; i < robot.allMotors.length; i++) {
                    robot.allMotors[i].setPower(
                            powers[i]/scale*biggestWithRotation*speedControl);
                }
            }
            else
            {
                for (int i = 0; i < robot.allMotors.length; i++)
                    robot.allMotors[i].setPower(0.0);
            }

            if (gamepad1.right_bumper)
                TestAuto();

            int encoderA = robot.frontLeft.getCurrentPosition();
            int encoderB = robot.backLeft.getCurrentPosition();
            int encoderC = robot.frontRight.getCurrentPosition();
            int encoderD = robot.backRight.getCurrentPosition();

            Quaternion q = robot.imu.getQuaternionOrientation();
            // The sonar only refreshes at 6.7 Hz.
            // We will average over 1 second to reduce noise.
            double vFront = robot.getFrontDistance();
            double vLeft = robot.getLeftDistance();
            telemetry.addData("Q", "%.5f %.5f %.5f %.5f",q.w,q.x,q.y,q.z);
            telemetry.addData("heading", "%.1f",robot.getHeading());
            telemetry.addData("Encoders","%6d %6d %6d %6d", encoderA,encoderB,encoderC,encoderD);
            telemetry.addData("ds",  "%.2f %.2f", vFront, vLeft);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            //sleep(40);
            robot.waitForTick(40);
        }
    }

    void TestAuto()
    {
        robot.TestRun1(0.1, 1.5, 0.5, this);
    }

    void ParkBlue(double xTarget) // 34.5 for center
    {
        int sonarTicks = 700;

        // Wait for ultrasound sensors to converge.
        robot.resetTickPeriod();
        robot.waitForTick(sonarTicks);
        double frontDistance = robot.getFrontDistance()/2.54; // inches
        double leftDistance = robot.getLeftDistance()/2.54; // inches

        if (leftDistance < 13.0)
        {
            robot.WiggleWalk(HardwareCwBot.inches(3.0),90.0, this);
            robot.resetTickPeriod();
            robot.waitForTick(sonarTicks);
            frontDistance = robot.getFrontDistance()/2.54; // inches
            leftDistance = robot.getLeftDistance()/2.54; // inches
            if (leftDistance < 13.0) return;
        }

        //telemetry.addData("measure","front %.1f left %.1f", frontDistance, leftDistance);
        //telemetry.update();
        //robot.waitForTick(2000);

        // Move up halfway to cryptobox.
        double deltaY = (frontDistance - 4.0)/2.0;
        // Left sensor face is 8.0" from center of robot.
        // The center of this cryptobox is assumed to be 36.0 - 1.5 inches.
        double boxCenterX = xTarget - 8.0;
        double deltaX = boxCenterX - leftDistance;
        double distance = Math.sqrt(deltaX*deltaX + deltaY*deltaY);
        double heading = (180.0/Math.PI) * Math.atan2(deltaY, deltaX);

        robot.PivotOnLeftWheel(heading, this);
        robot.RunToEncoder2(HardwareCwBot.inches(distance), this);
        robot.PivotOnLeftWheel(90.0, this);

        robot.resetTickPeriod();
        robot.waitForTick(sonarTicks);
        leftDistance = robot.getLeftDistance()/2.54; // inches
        deltaY += 2.0;
//        telemetry.addData("measure","left %.1f", leftDistance);
//        telemetry.update();
//        robot.waitForTick(2000);

        deltaX = boxCenterX - leftDistance;
        if (Math.abs(deltaX) > 0.5/2.54)
        {
            deltaY -= robot.WiggleWalk(robot.inches(deltaX),90.0, this)
                    / robot.ticksPerInch;

            robot.resetTickPeriod();
            robot.waitForTick(sonarTicks);
            leftDistance = robot.getLeftDistance()/2.54; // inches

//            telemetry.addData("measure","left %.1f", leftDistance);
//            telemetry.update();
//            robot.waitForTick(2000);
            deltaX = boxCenterX - leftDistance;
            if (Math.abs(deltaX) > 0.5/2.54) {
                robot.RunToEncoder2(robot.inches(-2.0), this);
                deltaY += 2.0;
                deltaY -= robot.WiggleWalk(robot.inches(deltaX), 90.0, this)
                        / robot.ticksPerInch;
            }
        }
        robot.RunToEncoder2(robot.inches(deltaY), this);
    }
}
