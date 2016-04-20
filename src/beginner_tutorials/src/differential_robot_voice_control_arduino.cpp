#include "ros/ros.h"		        //Obligatorio ponerlo para trabajar con ROS
#include "geometry_msgs/Twist.h"	
#include "turtlesim/Color.h"
#include "std_msgs/String.h"        //El tipo String de ROS
#include "string.h"

std::string cadena;                 //declaramos una variable global
void chatterCallback(const std_msgs::String::ConstPtr& voice_command)   //esta funcion esta espera recibir un dato prar publicarlo
{		  
  ROS_INFO("I heard: [%s]", voice_command->data.c_str());  //imprime el dato que recibe
  cadena = voice_command->data.c_str();         //guarda el dato que recibe en la variable global
}					

int main(int argc, char **argv)
{
  ros::init(argc, argv, "differential_robot_voice_control_arduino");      //se inicia  el nodo con el nombre: "differential_robot_voice_control_arduino"
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("recognizer/output", 1000, chatterCallback);//el nodo se suscribe al topico "recognizer/output"

	//begin talker
  
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("arduino/cmd_vel", 1000); //el nodo publica al topico "robot_0/cmd_vel"

  ros::Rate loop_rate(10); //Define la frecuencia en hercios de polling (10 veces por segundo). 
 

 	std::string avz;    //Se declaran las variables a utilizar.
	avz = "avanza";
	std::string rtcd;
	rtcd = "retrocede";     //Estas variables son los mando que esperamos recibir del usuario.
	std::string der;
	der = "derecha";
	std::string izq;
	izq = "izquierda";
    std::string stop;
    stop = "alto";
   /* std::string acl;
    acl = "acelera";
    std::string len;
    len = "lento";*/
    std::string hor;
    hor = "horario";
    std::string ahor;
    ahor = "antihorario";

  int count = 0;
  while (ros::ok()) //Mientras el sistema ROS esté operativo
  {

    geometry_msgs::Twist msg;   //Se define un mensaje de tipo Twist definido en geometry_msgs, que sera publicado.
    msg.linear.x = 0;           //inicializamos las componentes del dato Twist en 0;
    msg.linear.y = 0;           
    msg.linear.z = 0;
    msg.angular.x = 0;          //min=0 --- max=10;
    msg.angular.y = 0;
    msg.angular.z = 0;
    ROS_INFO("El Robot Avanza: linear:%f",msg.linear.x);

    if  (cadena.compare(avz) == 0)          //comparamos la varible global con los mandos que esperamos recibir		
		{
			msg.linear.x = 5;               //asignamos un valor a la componente en x del dato Twist
            msg.angular.z = 0;
            ROS_INFO("El Robot Avanza: linear:%f",msg.linear.x);    //imprime la funcion que esta realizando el robot
		}
    if (cadena.compare(rtcd) == 0)
		{
			msg.linear.x = -5;
            msg.angular.z = 0; 
            ROS_INFO("El Robot Retrocede");
		}
	if (cadena.compare(der) == 0)		
		{
            msg.linear.x = 0;
			msg.angular.z = -3; 
            ROS_INFO("El Robot gira a la derecha");
		}
    if (cadena.compare(izq) == 0)		
		{
			msg.linear.x = 0;
			msg.angular.z = 3; 
            ROS_INFO("El Robot gira a la izquierda");
		}
    /*if (cadena.compare(acl) == 0)		
		{
			msg.linear.x = 10; 
            ROS_INFO("El Robot acelera");
		}
    if (cadena.compare(len) == 0)		
		{
			msg.linear.x = 1; 
            ROS_INFO("El Robot va lento");
		}*/
    if (cadena.compare(hor) == 0)		
		{
			msg.linear.x = 5;
            msg.angular.z = -3; 
            ROS_INFO("Robot haciendo circulos en sentido horario");  
		}
    if (cadena.compare(ahor) == 0)		
		{
			msg.linear.x = 5;
            msg.angular.z = 3; 
            ROS_INFO("Robot haciendo circulos en sentido horario");  
		}
    if (cadena.compare(stop) == 0)		
		{
			msg.linear.x = 0;
            msg.angular.z = 0; 
            ROS_INFO("Robot detenido");
		}
    else
        {
            ROS_INFO("Dame una orden");
        }

    chatter_pub.publish(msg); //variable que publica mensaje tomado de msg


    ros::spinOnce(); 

    loop_rate.sleep(); //Aplica el sleep que definimos antes. Publica 10 veces/seg
    ++count;
  }		

  ros::spin();

  return 0;
}
