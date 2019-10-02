#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

#include "constantes.hpp"

#define INDICE_45 136
#define DIST_MIN 0.1
#define DETECT_LIGNEDROITE 50


float dRect(float angle)
{
	return (LARGEUR_VOITURE/2/sin(angle));
}

float commandDirection(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	float consigne_angle = 0; // [rad]
	float Range_min = INDICE_CENTRE-INDICE_45;
	float Range_max = INDICE_CENTRE + INDICE_45;
	/******* recherche du cas d'étude ********/
	int Compt(0);
	int somme(0);
	for (int i( Range_min ); i<= Range_max; i++)
	{
		if(DIST_MIN > scan_in->ranges[i] || scan_in->ranges[i] > DISTANCE_MAX )
		{
			Compt++;
			somme += i;
		}
	}
	if(Compt>DETECT_LIGNEDROITE)
	{
		consigne_angle = (somme/Compt-INDICE_CENTRE)*scan_in->angle_increment;// moyenne des indices par l'increment pour retrouver une consigne d'angle
		return consigne_angle;
	}


	/***** Recherche des min ******/
	float maxDRect = 0;
	int i_maxDRect = 0;
	float minDRect = 0;


	for (int i(Range_min); i<= Range_max; i++)
	{
		float ranges_i;
		if(DIST_MIN < scan_in->ranges[i] < DISTANCE_MAX)
		{
			ranges_i = scan_in->ranges[i];
		}
		else
		{
			ranges_i = DISTANCE_MAX;
		}

			minDRect = ranges_i;
			for(int j(i-INDICE_45); i<= i + INDICE_45; i++)
			{
				float ranges_j;
				if(DIST_MIN < scan_in->ranges[i] < DISTANCE_MAX)
				{
					ranges_j = scan_in->ranges[j];
				}
				else
				{
					ranges_j = DISTANCE_MAX;
				}

				//theta = fabs(i-j)*scan_in->angle_increment);
				if(ranges_j < dRect(fabs(i-j)*scan_in->angle_increment))
				{
					if(minDRect>ranges_j)
					{
						minDRect = ranges_j;
					}
				}
			}

			if(maxDRect<minDRect)
			{
				maxDRect = minDRect;
				i_maxDRect = i;
			}
	}

	consigne_angle = (i_maxDRect-INDICE_CENTRE)*scan_in->angle_increment;
	return consigne_angle;
}
