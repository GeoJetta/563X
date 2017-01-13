
//DRIVER CONTROL - BASE FUNCTIONS

const	int tsArray[128] = {
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 22, 23, 24, 24,
		25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
		28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
		33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
		37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
		41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
		46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
		52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
		61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
		71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
		80, 81, 83, 84, 84, 86, 86, 87, 87, 88,
		88, 89, 89, 90, 90,127,127,127
};

int trueSpeed(int power){
	return ((power>0)?1:-1)*tsArray[power*((power>0)?1:-1)];
}

void drive ( int l = 127, int r = l, int sped = true ) //Default power max forward, default left and right drive equal to this power
{

	motor [ L ] = l; //Set left drive equal to left power

	motor [ R ] = r;

}

void lift ( int l = 127, int r = l )
{

	motor [ L1 ] = l; //top
	motor [ L2 ] = l;
	motor [ L3 ] = l;
	motor [ L4 ] = l; //bottom

	motor [ R4 ] = r; //bottom
	motor [ R3 ] = r;
	motor [ R2 ] = r;
	motor [ R1 ] = r; //top

}

void haltLift ()
{

	motor[L1] = 5;
	motor[L2] = 5;
	motor[L3] = 5;
	motor[L4] = 5;

	motor[R4] = 5;
	motor[R3] = 5;
	motor[R2] = 5;
	motor[R1] = 5;

}



//MISC FUNCTIONS


//TEST ALL LIFT MOTORS INDIVIDUALLY (IN PAIRS)

//void motTest ()
//{

//	motor[L1] = 127;
//	motor[R1] = 127;

//	wait1Msec(2000);

//	lift(0);

//	motor[L2] = 127;
//	motor[R2] = 127;

//	wait1Msec(2000);

//	lift(0);

//	motor[L3] = 127;
//	motor[R3] = 127;

//	wait1Msec(2000);

//	lift(0);

//	motor[L4] = 127;
//	motor[R4] = 127;

//	wait1Msec(2000);

//	lift(0);

//}
