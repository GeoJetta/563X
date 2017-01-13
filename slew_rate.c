/*----------------------------------------------------------------------------------------------------*/
/*                                                                                                    */
/*                      MOTOR CONTROL, CURRENT MANAGEMENT, AND LESS WHEEL SLIP                        */
/*                                                                                                    */
/* Adapted from original code - Source:  http://www.vexforum.com/showpost.php?p=225727&postcount=25   */
/* Article: http://www.vexmen.com/2014/02/troubles-tripping-circuit-breakers-with-10-big-393-motors/  */
/*                                                                                                    */
/*----------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  definitions and variables for the motor slew rate controller.              */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
 
#define MOTOR_NUM               kNumbOfTotalMotors
#define MOTOR_MAX_VALUE         127
#define MOTOR_MIN_VALUE         (-127)
#define MOTOR_DEFAULT_SLEW_RATE 10      // Default will cause 375mS from full fwd to rev
#define MOTOR_FAST_SLEW_RATE    256     // essentially off
#define MOTOR_TASK_DELAY        15      // task 1/frequency in mS (about 66Hz)
 
// Array to hold requested speed for the motors
int motorReq[ MOTOR_NUM ];
 
// Array to hold "slew rate" for the motors, the maximum change every time the task
// runs checking current mootor speed.
int motorSlew[ MOTOR_NUM ];
 
/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  Task  - compares the requested speed of each motor to the current speed    */
/*          and increments or decrements to reduce the difference as necessary */
/*                                                                             */
/*-----------------------------------------------------------------------------*/
 
task MotorSlewRateTask()
{
    int motorIndex;
    int motorTmp;
 
    // Initialize stuff
    for(motorIndex=0;motorIndex<MOTOR_NUM;motorIndex++)
    {
        motorReq[motorIndex] = 0;
        motorSlew[motorIndex] = MOTOR_DEFAULT_SLEW_RATE;
    }
 
    // run task until stopped
    while( true )
    {
        // run loop for every motor
        for( motorIndex=0; motorIndex<MOTOR_NUM; motorIndex++)
        {
            // So we don't keep accessing the internal storage
            motorTmp = motor[ motorIndex ];
 
            // Do we need to change the motor value ?
            if( motorTmp != motorReq[motorIndex] )
            {
                // increasing motor value
                if( motorReq[motorIndex] > motorTmp )
                {
                    motorTmp += motorSlew[motorIndex];
                    // limit
                    if( motorTmp > motorReq[motorIndex] )
                        motorTmp = motorReq[motorIndex];
                    }
 
                // decreasing motor value
                if( motorReq[motorIndex] < motorTmp )
                {
                    motorTmp -= motorSlew[motorIndex];
                    // limit
                    if( motorTmp < motorReq[motorIndex] )
                        motorTmp = motorReq[motorIndex];
                }
 
                // finally set motor
                motor[motorIndex] = motorTmp;
            }
        }
 
        // Wait approx the speed of motor update over the spi bus
        wait1Msec( MOTOR_TASK_DELAY );
        }
}
