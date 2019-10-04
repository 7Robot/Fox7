#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <vector>
#include <deque>

#include "constantes.hpp"

#define Y_MIN 0.15
#define PAS 0.01
#define PAS_ANGLE 4 // 4*scan_in->angle_increment

float traiterDist(int i, const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	float dist=scan_in->ranges[i];

	if(dist>scan_in->range_max)
		dist=scan_in->range_max;
	if(dist<scan_in->range_min)
		dist=scan_in->range_min;

	if(dist>DISTANCE_MAX || dist==0)
		dist=DISTANCE_MAX;

	return dist;
}

float trouverMax(const sensor_msgs::LaserScan::ConstPtr& scan_in, float* dist_max)
{
	float dist;
	std::deque<float> liste;
	liste.push_back(0);
	for(int i=INDICE_CENTRE+ANGLE_MIN/scan_in->angle_increment; i<INDICE_CENTRE+ANGLE_MAX/scan_in->angle_increment; i++)
	{
		dist=traiterDist(i, scan_in);
		if(dist>liste.back())
		{
			liste.push_back(dist);
			if(liste.front()<0.95*liste.back())
				liste.pop_front();
		}
	}
	float dist_moy, angle_moy;
	for(int i=0; i< liste.size(); i++)
	{
		dist_moy+=liste[i];
		angle_moy+=i;
	}		
	dist_moy=dist_moy/liste.size();
	angle_moy=(angle_moy/liste.size()-INDICE_CENTRE)*scan_in->angle_increment;

	*dist_max=dist_moy;
	return angle_moy;
}


float calculOuverture(float dmax)
{
	float ouverture;
	// a dvp evidemment
	ouverture=30*PI/180;

	return ouverture;
}

float genererRectangle(float angle_max, float dmax, const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	float ygmin=dmax;
	float ydmin=dmax;
	bool obstacle;

	float beta, h;
	float x=1.2*LARGEUR_VOITURE/2;
	
	//////////////////
	// Ligne gauche //
	//////////////////
	obstacle=false;
	for(float y=Y_MIN; y<dmax && !obstacle; y+=PAS)
	{
		beta=atan(y/x);
		h=y/sin(beta);

		if(traiterDist(INDICE_CENTRE+(angle_max+beta-PI/2)/scan_in->angle_increment, scan_in) <= h)
		{
			obstacle=true;
			ygmin=y;
		}

	}
	if(ygmin>dmax)
		ygmin=dmax;

	//////////////////
	// Ligne droite //
	//////////////////
	obstacle=false;		
	for(float y=Y_MIN; y<dmax && !obstacle; y+=PAS)
	{
		beta=atan(y/x);
		h=y/sin(beta);

		if(traiterDist(INDICE_CENTRE+(angle_max+beta-PI/2)/scan_in->angle_increment, scan_in) <= h)
		{
			obstacle=true;
			ydmin=y;
		}

	}
	if(ydmin>dmax)
		ydmin=dmax;

	return std::min(ygmin, ydmin);
}

float commandDirection(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	float consigne_angle=0; // [rad]

	// On trouve dmax et angle_max
	float angle_dmax;
	float dmax;
	angle_dmax=trouverMax(scan_in, &dmax);

	// Determine angle d'ouverture
	float angle_ouverture=calculOuverture(dmax);

	// Pour chaque angle dans l'ouverture on trouve le rectangle associé dans le but de trouver le rectangle le plus long
	std::deque<float> liste;
	liste.push_back(0);
	float longueur_rectangle=0;
	for(float angle=angle_dmax-angle_ouverture; angle<angle_dmax+angle_ouverture; angle+=PAS_ANGLE*scan_in->angle_increment)
	{
		//ROS_INFO("angle_boucle=%f", angle);
		longueur_rectangle=genererRectangle(angle, dmax, scan_in);

		if(longueur_rectangle>liste.back())
		{
			liste.push_back(longueur_rectangle);
			if(liste.front()<0.95*liste.back())
				liste.pop_front();
		}
	}

	// on a maintenant la direction a viser
	for(int i=0; liste.size(); i++)
		consigne_angle+=liste[i];
	consigne_angle=(consigne_angle/liste.size()-INDICE_CENTRE)*scan_in->angle_increment;

	return consigne_angle;
}

