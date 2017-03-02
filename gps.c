int x_pos;
int y_pos;
int lastSense[2];

void gpsCalc( int lDist, int rDist = lDist, int ang )
{

	lDist -= lastSense[0];
	rDist -= lastSense[1];

	x_pos += (lDist+rDist)/2*cosDegrees(ang);
	y_pos += (lDist+rDist)/2*sinDegrees(ang);

	lastSense[0] += lDist;
	lastSense[1] += rDist;

}

task gps()
{

	while( true )
	{

		gpsCalc( SensorValue[lEnc], SensorValue[rEnc], SensorValue[gyro] );

		wait1Msec( 10 );

	}

}
