#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <vector>
#include <deque>

#include "constantes.hpp"

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

float trouverMaxNew(const sensor_msgs::LaserScan::ConstPtr& scan_in, float* dist_max)
{
	float dmax;
	deque<float>

	*dist_max=dmax;
	return angle_max;
}

float trouverMax(const sensor_msgs::LaserScan::ConstPtr& scan_in, float* dist_max)
{
	// cherche dmax
	float dmax=0;
	float dist=0;
	for(int i=INDICE_CENTRE+ANGLE_MIN/scan_in->angle_increment; i<INDICE_CENTRE+ANGLE_MAX/scan_in->angle_increment; i++)
	{
		dist=traiterDist(i, scan_in);

		// cherche max
		if(dist>dmax)
			dmax=dist;
	}
	*dist_max=dmax;
	//ROS_INFO("dmax=%f", dmax);

	// on fait la moyenne sur un certain pourcentage
	float pourcentage=0.95;
	int imoymax=0;
	int nbi=0;
	for(int i=INDICE_CENTRE+ANGLE_MIN/scan_in->angle_increment; i<INDICE_CENTRE+ANGLE_MAX/scan_in->angle_increment; i++)
	{	
		//ROS_INFO("scan[%d]=%f", i, scan_in->ranges[i]);
		// on borne distance vu
		dist=traiterDist(i, scan_in);

		if(dist>dmax*pourcentage )
		{
			//ROS_INFO("added");
			imoymax+=i;
			++nbi;
		}
	}
	imoymax=imoymax/nbi;
	//ROS_INFO("imoymax=%d, nbi=%d", imoymax,nbi);

	return (imoymax-INDICE_CENTRE)*scan_in->angle_increment;
}

float calculOuverture(float dmax)
{
	float ouverture;
	// a dvp evidemment
	ouverture=30*PI/180;

	return ouverture;
}

float genererRectangle(float angle, float angle_ecart, float* longueur_rectangle, const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	//ROS_INFO("angle_ecart(deg)=%f", angle_ecart*180/PI);

	//trouve dmin dans plage [angle-angle_ecart; angle+angle_ecart]
	float dmin=scan_in->ranges[INDICE_CENTRE+(angle-angle_ecart)/scan_in->angle_increment]; // longueur_rectangle
	for(float angle_plage=angle-angle_ecart+scan_in->angle_increment; angle_plage<angle+angle_ecart; angle_plage+=5*scan_in->angle_increment)
	{
		if(scan_in->ranges[INDICE_CENTRE+angle_plage/scan_in->angle_increment]<dmin)
		{
			//ROS_INFO("plop");
			dmin=scan_in->ranges[INDICE_CENTRE+angle_plage/scan_in->angle_increment];
		}
		//ROS_INFO("scan=%f\ndmin=%f", scan_in->ranges[INDICE_CENTRE+angle_plage/scan_in->angle_increment], dmin);
	}
	*longueur_rectangle=dmin;

	// calcule largeur_rectangle
	return 2*dmin*tan(angle_ecart);
}

float commandDirection(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
	float consigne_angle=0; // [rad]

	/*
	-> cherche Dmax
		-> fixe une distance max ou on voit
		-> trouver dmax
		-> recuperer tous indices supérieurs a un pourcentage de la distance max
		-> centre de cette zone (moyenne), on obtient meilleur angle max

	-> determiner angle d'ouverture en fonction de Dmax
		-> au plus distance courte au plus angle grand

	-> pour chaque angle dans la zone d'ouverture (ou avec un pas defini) trouver rectangle max
		-> generer rectangle (i_angle_ecart=1)
			-> trouver distance min dans plage (formee par angle ecart autour de l'angle)
			-> calculer largeur_plage en fonction de dmin et angle_ecart

		-> tant que largeur_plage<largeur_voiture (ou autre condition de fin au cas ou ?)
			-> agrandir plage/angle_ecart
			-> generer rectangle

		-> si ce rectangle est le plus grand qu'on ai calculé jusque la
			-> long_rectangle_max
			-> imax et donc rectangle_angle_max

	-> on vise la direction en rectangle_angle_max
	*/

	// On trouve dmax et angle_max
	float angle_moy_max;
	float dmax;

	angle_moy_max=trouverMax(scan_in, &dmax);
	//angle_moy_max=trouverMaxNew(scan_in, &dmax);
	ROS_INFO("angle_moy_max=%f dmax=%f", angle_moy_max, dmax);

	// Determine angle d'ouverture
	float angle_ouverture=calculOuverture(dmax);
	//ROS_INFO("angle_ouverture=%f", angle_ouverture);

	// Pour chaque angle dans l'ouverture on trouve le rectangle associé dans le but de trouver le rectangle le plus long
	
	float angle_ecart;
	float largeur_rectangle=0;
	float longueur_rectangle=0;
	float longueur_rectangle_max=0;
	std::vector<float> angle_moy_rectangle;
	for(float angle=angle_moy_max-angle_ouverture; angle<angle_moy_max+angle_ouverture; angle+=5*scan_in->angle_increment)
	{
		//ROS_INFO("angle_boucle=%f", angle);
		// generer 1er rectangle
		angle_ecart=scan_in->angle_increment;
		largeur_rectangle=genererRectangle(angle, angle_ecart, &longueur_rectangle, scan_in);

		//ROS_INFO("largeur_1er_rectangle=%f", largeur_rectangle);

		// on continue de l'agrandir jusqu'a ce que sa largeur depasse celle de la voiture
		while(largeur_rectangle<LARGEUR_VOITURE && angle_ecart<=85*PI/180 && angle_ecart>=-85*PI/180)
		{
			angle_ecart+=2*scan_in->angle_increment;
			largeur_rectangle=genererRectangle(angle, angle_ecart, &longueur_rectangle, scan_in);
		}
		//ROS_INFO("longueur_rectangle=%f", longueur_rectangle);
		//ROS_INFO("largeur_rectangle=%f", largeur_rectangle);
		//ROS_INFO("fin boucle");

		if(longueur_rectangle>=longueur_rectangle_max*0.95 && longueur_rectangle<=longueur_rectangle_max)
		{
			//ROS_INFO("add size=%d", (int)angle_moy_rectangle.size());
			angle_moy_rectangle.push_back(angle);
		}

		// on verif si on a trouve le plus long jusque la
		else if(longueur_rectangle>longueur_rectangle_max)
		{
			longueur_rectangle_max=longueur_rectangle;
			angle_moy_rectangle.clear();
			angle_moy_rectangle.push_back(angle);
			//ROS_INFO("found max");
		}
	}

	// on a maintenant la direction a viser
	ROS_INFO("size=%d", (int)angle_moy_rectangle.size());
	for(int i=0; i<angle_moy_rectangle.size(); i++)
		consigne_angle+=angle_moy_rectangle[i];
	consigne_angle=consigne_angle/angle_moy_rectangle.size();
	ROS_INFO("consigne_angle=%f", consigne_angle);

	return consigne_angle;
}

