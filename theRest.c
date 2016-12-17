//NOTE: if negative encoder values start messing stuff up, use absolute values in ds

int old_ds_dist[3]; //Create a place to store original encoder values to subtract

void reset_ds_dist () //Allow user to reset starting values whenever needed
{

	old_ds_dist[1] = sensorValue[LEFTENC]; //Set array to current encoder values to subtract from
	old_ds_dist[2] = sensorValue[RIGHTENC];

}

void point ( int pwr, string dir, int err = 5, int mult = 0.1 ) //Point turn keeping opposite encoder values. No ramp up, just checking if each side is opposite
{

	int l; //left sign value
	int r; //right sign value

	if ( dir == "left" ) //Check if strings need quotes.
	{

		l = -1; //multipliers for drive values
		r = 1;

	}

	else if ( dir == "right" )
	{

		l = 1;
		r = -1;

	}

	drive( pwr * l, pwr * r );

	if ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] - err < abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] + err && abs( sensorValue[LEFTENC] ) - old_ds_dist[1] + err > abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] - err ) //If they are turning an equal amount within margin of error, set power to the same
	{

		drive( pwr * l, pwr * r );

	}

	else if ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] - err < abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] + err )
	{

		drive( pwr * l, pwr * ( 1 - mult ) * r );

	}

	else if ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] + err > abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] - err )
	{

		drive( pwr * ( 1 - mult ) * l, pwr * r );

	}


}

void ds ( int pwr, int err = 5, int mult = 0.1 ) //Drive straight. No ramp up, just checking each side is equal
{

	drive( pwr, pwr );

	if ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] - err < abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] + err && abs( sensorValue[LEFTENC] ) - old_ds_dist[1] + err > abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] - err ) //If they are straight within margin of error, set power to the same
	{

		drive( pwr, pwr );

	}

	else if ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] - err < abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] + err )
	{

		drive( pwr, pwr * ( 1 - mult ) );

	}

	else if ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] + err > abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] - err )
	{

		drive( pwr * ( 1 - mult ), pwr );

	}

}

void ds_dist ( int dist, int pwr = 127, int ramp = 5, int slow_dist = 12 ) //Drive straight for a certain distance (Includes ramp-up)
{

	for (int i; i < pwr; i++) //Ramp up power smoothly
	{

		ds( i );

		wait1MSec( ramp );

	}

	while ( ( ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] ) + ( abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] ) ) / 180 < dist - slow_dist ) //Go at pwr until within slow_dist of target
	{

		ds( pwr );

	}

	while ( ( ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] ) + ( abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] ) ) / 180 < dist )
	{

		ds( pwr * -( ( ( ( abs( sensorValue[LEFTENC] ) - old_ds_dist[1] ) + ( abs( sensorValue[RIGHTENC] ) - old_ds_dist[2] ) ) / 180 ) - dist ) / 20 );

	}

	drive( 0, 0 );

}

void point_ang ( int ang, int pwr = 127, int ramp = 5, int slow_angle = 20 ) //point turn to a specific angle
{

	string dir;

	if ( GyroGetAngle() - ang >= 0 ) //if current angle - wanted angle is greater than 0, turn right
	{

		dir = "right";

	}

	else if ( GyroGetAngle() - ang < 0 ) //if current angle - wanted angle is greater than 0, turn right
	{

		dir = "left";

	}

	for (int i; i < pwr; i++) //Ramp up power smoothly
	{

		point( i, dir );

		wait1MSec( ramp );

	}

	while ( /*abs(*/ ( GyroGetAngle() - ang ) /*)*/ > 0 + slow_angle ) //Go at pwr until within slow_angle of target
	{

		point( pwr, dir );

	}

	while ( /*abs*/( GyroGetAngle() - ang /*)*/ >= 0 )
	{

		point( pwr * /*abs(*/ GyroGetAngle() - ang /*)*/ / 20, dir );

	}

	drive( 0, 0 );

}
