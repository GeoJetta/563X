//	REMEMBER TO INCLUDE GyroLib.c IN CODE! & to #define GYRO & LEFTENC & RIGHTENC & TICKS_PER

/*The purpose of this code is to output a coordinate value given an original starting position and angle. It should update the position as it moves around the field.*/

/*Some Explanations for some things:
Angle variables - these seem a bit confusing, but I promise you they are way more confusing than that.
> old_angle stores what the angle was last cycle
> gps_array[3], or GetGyroAngle() is the current angle
> new_angle is the difference between the two, or the angle change between cycles
*/

//Init global values for later functions
int old_x = 0; //x coordinate to be added to
int old_y = 0; //y coordinate to be added to
int old_angle = 0; //angle from before to subtract from current

int gps_array[4]; //array for storing multiple sensor values



void gps_sensors () //Takes sensor values and outputs as real-world values
{

	//	1. Convert encoder ticks to inches

	gps_array[0] = 0;

	gps_array[1] = -gps_array[1] + sensorValue[LEFTENC] / (TICKS_PER / 4 ); //Since wheels are 4 inches and there are 360 ticks per revolution, 360/4 = 90 ticks per inch
	gps_array[2] = -gps_array[2] + sensorValue[RIGHTENC] / (TICKS_PER / 4 ); //Subtract the old value to find the change in amount, not total

	//	2. Convert (?) gyro to degrees

	gps_array[3] = GyroGetAngle(); //gyro value with drift accounted for (JPearman Gyro Lib - if having problems, implement complementary filter)

}

void gps () //Takes encoder and gyro inputs, finds radius of arc and outputs (x, y) coordinates and angle relative to starting point
{

	/*	Explanation: All manuevers will take place on a curve (straight lines can be represented as curves with very large radii),
	so if we cut up every motion into a curve it will represent motion better than straight lines. From curves and a bit of trig
	we can find the coordinates relative to where we started.																																		*/

	//	1. Init Values

	gps_sensors(); //store all sensor values in gps_array[]

	int dist = 0; //distance travelled
	int radius = 0; //radius of circle
	int new_x = 0; //new x value (added to old)
	int new_y = 0; //new y value (added to old)
	int new_angle = 0; //Angle in-between

	//	2a. Find radius of circle

	dist = ( gps_array[1] + gps_array[2] ) / 2;

	new_angle = gps_array[3] - old_angle; //new angle. Check this math

	radius = ( ( 360 * dist ) / ( new_angle + 0.0000001 ) ) / ( 2 * PI ); //radius of circle taken from the proportion of angle to 360 degrees to distance travelled to circumference

	//	2b. Find coordinates

	new_x = ( -cos( new_angle ) * radius ) - radius; //trig to find x amount moved
	new_y = sin( new_angle ) * radius; //trig to find y amount moved

	//	2c. ROTATION MATRIX MOTHERKITTENERS - this may or may not be a horrible idea. In the case of the first, re-write this entire code! Yay!

	new_x = new_x * cos( old_angle ) + new_y * sin( old_angle ); //rotates x value by the old angle we subtracted
	new_y = -new_x * sin( old_angle ) + new_y * cos( old_angle ); //rotates y value by the old angle we subtracted

	//	3. Return values

	old_x = old_x + new_x; //Add new values to originals
	old_y = old_y + new_y;

	old_angle = gps_array[3]; //Set angle for the next cycle

	//Now (old_x, old_y) will give the coordinates.

}


//void gps_goto ( int x, int y, int ang, int pwr = 127 ) //Go to a specified coordinate and angle
//{

//	reset_ds_dist();

//	//	1a. Find angle to point

//	int angle = 0 /*tan( ( old_y - y ) / ( old_x - x ) )*/;

//	//	1b. Turn to that angle

//	point_ang( angle, pwr );

//	//	2a. Find distance to point

//	int distance = sqrt( ( ( old_y - y ) ^ 2 + ( old_x - x ) ^ 2 ) )

//	//	2b. Go to point

//	reset_ds_dist();

//	ds_dist( distance, pwr );

//	//	3. Turn to angle

//	reset_ds_dist();

//	point_ang( ang, pwr );

//}
