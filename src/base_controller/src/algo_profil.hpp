#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Float32MultiArray.h"


#include "constantes.hpp"

#define INDICE_45 136
#define DIST_MIN 0.1
#define DIST_MAX 5
#define DETECT_LIGNEDROITE 50

#define DIST_MAX_ARRIV 3
#define PERCENT_ARRIV 0.2


float marge_erreur(1.5);

float dRect(float angle)
{
	return (LARGEUR_VOITURE*marge_erreur/2/sin(angle));
}

float commandDirection(const sensor_msgs::LaserScan::ConstPtr& scan_in, ros::Publisher rect_pub)
{
	float consigne_angle = 0; // [rad]
	int Range_min = INDICE_CENTRE-INDICE_45;
	int Range_max = INDICE_CENTRE + INDICE_45;
	/******* recherche du cas d'étude ********/
	int Compt(0);
	int somme(0);
	//ROS_INFO("I'm in !");
	for (int i( Range_min ); i<= Range_max; i++)
	{
		if(scan_in->ranges[i]==0 || scan_in->ranges[i] > DIST_MAX )
		{
			Compt++;
			somme += i;
		}
	}
	if(Compt>DETECT_LIGNEDROITE)
	{
		consigne_angle = (somme/Compt-INDICE_CENTRE)*scan_in->angle_increment;// moyenne des indices par l'increment pour retrouver une consigne d'angle

		//ROS_INFO("ligne droite");
		return consigne_angle;
	}

	// DETECT LIGNE POUR ARRIVE
	float compt(0);
	for (int i(Range_min);i<= Range_max;i++)
	{
		if(scan_in->ranges[i] == 0 || scan_in->ranges[i] >= DIST_MAX_ARRIV)
		{	compt+=1;}
	}
	if(compt/INDICE_45/2 > PERCENT_ARRIV)
	{ROS_INFO("\n##LIGNE DROITE#\n#");}
	else{ROS_INFO("\n##VIRAGE##\n");}


	//ROS_INFO("Virage");

	/***** Recherche des min ******/
	float maxDRect = 0;
	int i_maxDRect = 0;
	float minDRect = 0;


	for (int i(Range_min); i<= Range_max; i++)
	{
		float ranges_i;


		if(scan_in->ranges[i] > DIST_MAX || scan_in->ranges[i]==0)
		{
			ranges_i = DISTANCE_MAX;
		}
		else
		{
			ranges_i = scan_in->ranges[i];
		}

			minDRect = ranges_i;
			for(int j = (i-INDICE_45+100); j<=( i + INDICE_45+100); j++)
			{
				if(i==j)
				{
					continue;
				}


				// On normalise entre la dist max et la dist min


				float ranges_j;
				if(scan_in->ranges[j] > DISTANCE_MAX || scan_in->ranges[j] == 0)
				{
					ranges_j = DISTANCE_MAX;
				}
				else
				{
					ranges_j = scan_in->ranges[j];
				}

				// On regarde la projection du robot dans la direction
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

  std_msgs::Float32MultiArray msgs_rect;
  msgs_rect.data = {consigne_angle, maxDRect};

  rect_pub.publish(msgs_rect);
	return consigne_angle;
}
