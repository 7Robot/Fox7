// LIDAR
// ranges[0] -> ranges[810]
// centre 405

#define PI 3.14519265

#define GPIO_SERVO 17
#define GPIO_ESC 18

#define ANGLE_MAX 3.141592654/2 // [rad]
#define ANGLE_MIN -3.141592654/2 // [rad]

#define CMD_ANGLE_MAX 0.69813 // 40 degre en [rad]
#define CMD_ANGLE_MIN -0.69813 // -40 degre en [rad]

#define INDICE_MAX 810
#define INDICE_CENTRE 405
#define INDICE_MIN 0

#define CMD_SPEED_MAX 1
#define CMD_SPEED_MIN 0.0 // Garder positive ! ou changer commandSpeed

#define LARGEUR_VOITURE 0.35 // 0.15 + marge securite

#define DISTANCE_MAX 5
#define DISTANCE_MIN 0.4
#define DIST_URGENCE 0.2


