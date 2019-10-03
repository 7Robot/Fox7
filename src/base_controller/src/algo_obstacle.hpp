#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

#include "constantes.hpp"

#define DISTANCE_SECURITE 0.40

float commandDirection(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	float consigne_angle=0; // [rad]


	//De base, on envoit 0 en consigne d angle, jusqu a ce qu on voit quelquechose devant
	
	//Pour chaque point dans un angle de 45° de chaque côté:
		//On calcule l'angle qu'on observe
		//On recupère la distance de l obstacle observe, on la traite (max, min ...)
		//On calcule les coordonnées x et y de cet obstacle
		//On vérifie si cet obstacle est dans le rectangle de securite devant la voiture
			//Si il l'est, on stocke les donnees liees a l'obstacle
			//et on verifie si c est l obstacle preponderant
		
		//On calcul la commande en cas d'obstacle en calculant l'angle d evitement
	

	float angle_check=PI/4;
	float distance=0;
	float x=0;
	float y=0;
	float a_increment=scan_in->angle_increment;
	
	for(int i=INDICE_MIN+271; i<INDICE_MAX-270; i++)
	{
		angle_check+=a_increment;
		distance=scan_in->ranges[i];

		if ((distance>0.17)&&(distance<5))
		{
			if (angle_check<PI/2)
			{
				x=cos(angle_check)*distance;
				y=sin(angle_check)*distance;

				if (((-1*LARGEUR_VOITURE)/2<x)&&(x<(1*LARGEUR_VOITURE)/2))
				{
					if ((0<y)&&(y<DISTANCE_SECURITE))
					{
						//ya un obstacle
						//on calcul l angle pour esquiver
						ROS_INFO("YA UN OBSTACLE A DROITE");
						float esquive=atan((LARGEUR_VOITURE/2-x)/y);
					
						if (abs(esquive)>abs(consigne_angle))
						{
							consigne_angle=esquive;
						}				
					}
				}
			}
			else
			{
				x=cos(PI-angle_check)*distance;
				y=sin(PI-angle_check)*distance;

				if (((-1*LARGEUR_VOITURE)/2<x)&&(x<(1*LARGEUR_VOITURE)/2))
				{
					if ((0<y)&&(y<DISTANCE_SECURITE))
					{
						//ya un obstacle
						//on calcul l angle pour esquiver
						ROS_INFO("YA UN OBSTACLE A GAUCHE");
						float esquive=-atan((LARGEUR_VOITURE/2-x)/y);
					
						if (abs(esquive)>abs(consigne_angle))
						{
							consigne_angle=esquive;
						}				
					}
				}
			}
		}
	}
	

	consigne_angle=-1*consigne_angle;


	return consigne_angle;
}

