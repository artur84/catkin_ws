#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include "social_fcns.h"
using namespace std;

//*******************************************************************************
  //Esta funcion recibe position, orientacion y velocidad de un peaton
  // y la posicion y orientacion de un objeto y calcula
  //si existe una interaccion entre ellos 
  // devuelve uno si hay interaccion 0 en caso contrario
  // y los valores por defecto de la gaussiana
  //Los angulos deben estar en radianes (-Pi,Pi)
//*******************************************************************************
int  EvObjInt(double  x1,double y1,double theta1,double vx1,double vy1,double  x2,double y2,double theta2,social_filter::int_data* Data)
{
  double mu_x=0.0;
  double mu_y=0.0;
  double s_x=0.0;
  double s_y=0.0;
 
  //Desviacion estandar para los radianes de cada formacion, aprox. 9 grados
 
  float  Sigma=M_PI/20; 

  int z=0;
  //Verificar distancia social y velocidad relativa baja
  float euclidianDistance, vel;
  vel=   sqrt(pow(vx1,2) + pow(vy1,2)); 
  euclidianDistance = sqrt(pow(x2-x1,2)+ pow(y2-y1,2) );
  //  cout<<"d: "<<euclidianDistance <<" vel "<<vel<<endl;
  if(euclidianDistance < 3.0 &&  vel < 0.4)
    {
      struct dataAlign DataAl;
      float PM[2];
      Alignment(x1,y1,theta1, x2,y2, theta2,&DataAl);
   
      //calcular PM 
      PM[0]=(x2+x1)/2;
      PM[1]=  (y2+y1)/2;
    
      //case vis a vis with an object
      if (fabs(DataAl._alfa)< 2.5*Sigma && fabs(DataAl._beta)< 2.5*Sigma)
	{
	  z=HOI;// human object interaction
	  mu_x=PM[0];
	  mu_y=PM[1];
	  s_x= euclidianDistance/4;
	  s_y= 3*euclidianDistance/5;
	  
	}       
    }  

  Data->angle=atan2(y2-y1,x2-x1);
  Data->media_x=mu_x;
  Data->media_y=mu_y;
  Data->sd_x=s_x;
  Data->sd_y=s_y;   
 
  return z;
}

//**************************************************************************************************	
//*******************************************************************************
  //Esta funcion recibe position, orientacion y velocidad de un par de peatones y calcula
  //si esta en alguna formacion Vis-vis, V, L, C
  // Si coincide con un caso listado devuelve el tipo de formation y una gaussiana
  // si no devuelve cero y los valores por defecto de la gaussiana
//Los angulos deben estar en radianes (-Pi,Pi)
//*******************************************************************************
int  EvFormation(double x1,double y1,double theta1,double vx1,double vy1, double x2,double y2, double theta2, double vx2, double vy2,social_filter::int_data* Data, double interaction_space)
{
  double mu_x=0.0;
  double mu_y=0.0;
  double s_x=0.0;
  double s_y=0.0;
  double x_meet1,y_meet1,x_meet2,y_meet2,m=0;//,D=0;
  //std::cout<<"theta1 "<< theta1/GRAD2RADIAN<<"theta2 " << theta2/GRAD2RADIAN << endl;  
  //Desviacion estandar para los radianes de cada formacion, aprox. 12 grados
  float  Sigma=M_PI/15; 

  int z=0;
  //Verificar distancia social y velocidad relativa baja
  float euclidianDistance, L,xc,yc;   
  euclidianDistance = sqrt(pow(x2-x1,2)+ pow(y2-y1,2) );
  if(euclidianDistance < 3.0 &&  sqrt(pow((vx2-vx1),2) + pow((vy2-vy1),2)) < 5.0)
    {
      struct dataAlign DataAl;
      float PM[2];
      Alignment(x1,y1,theta1, x2,y2, theta2,&DataAl);
   
      //calcular Punto Medio 
      PM[0]=(x2+x1)/2;
      PM[1]=  (y2+y1)/2;
    
      //std::cout<<"alfa "<< DataAl._alfa/GRAD2RADIAN<<"beta " << DataAl._beta/GRAD2RADIAN << endl;

      // cout<< "limite"<<2.5*Sigma/GRAD2RADIAN<<endl;
      //case vis a vis
      if (fabs(DataAl._alfa)< 2.0*Sigma && fabs(DataAl._beta)< 2.0*Sigma)
	{
	  //	  std::cout<<"alfa "<< DataAl._alfa/GRAD2RADIAN<<"beta " << DataAl._beta/GRAD2RADIAN << endl;
	  z=VISVIS;
	  mu_x=PM[0];
	  mu_y=PM[1];
	  ///////////////////////////////////////////////////////////////////////
	  ///////////////// TODO : dynamique interaction space //////////////////
	  ///////////////// add the same for other configuration ////////////////
	  ///////////////////////////////////////////////////////////////////////
	  s_x= interaction_space * log(1 + euclidianDistance);
	  s_y= interaction_space * 1.3;//1.3;
	  ///////////////////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////
	  if(y2!=y1){
	    m=-(x2-x1)/(y2-y1);
	    y_meet1=PM[1]+ m*0.5*euclidianDistance*sqrt(1/(1 + pow(m,2) ) );
	    y_meet2=PM[1]- m*0.5*euclidianDistance*sqrt(1/(1 + pow(m,2) ) );
	    if(x2 !=x1){
	    x_meet1=PM[0] + (y_meet1 - PM[1])/m;
	    x_meet2=PM[0] + (y_meet2 - PM[1])/m;
	    }
	    else{
	      x_meet1=PM[0]+ 0.5*euclidianDistance;
	      x_meet2=PM[0]- 0.5*euclidianDistance;
	    }
	  }
	  else{
	    y_meet1=PM[1];
	    x_meet1=PM[0] +0.5*euclidianDistance;
	    y_meet2=PM[1];
	    x_meet2=PM[0] - 0.5*euclidianDistance;
	  }
	 
	}       
 
      
      // //The value to compare have been chosen related to the models proposed by F-Formation 
      // //case V
      // if (DataAl.side !=0 && fabs((22.5)*GRAD2RADIAN- fabs(DataAl._alfa))< 2.0*Sigma && fabs((22.5*GRAD2RADIAN)-fabs(DataAl._beta))< 2.0*Sigma)
      // 	{
      // 	  z=V_FORM;
      // 	  xc=(tan(theta2)*x2 -y2-tan(theta1)*x1+y1)/(tan(theta2)-tan(theta1));
      // 	  yc= (xc-x1)*tan(theta1)+ y1;
      // 	  mu_x= (xc + PM[0])/2;
      // 	  mu_y= (yc + PM[1])/2;
      // 	  s_x=euclidianDistance/3.0;
      // 	  L= sqrt(pow(xc-PM[0],2) + pow(yc-PM[1],2));
      // 	  s_y=2*L;
      // 	  if(xc!=PM[0]){
      // 	    m=(yc-PM[1])/(xc-PM[0]);
      // 	    y_meet1=yc + m*0.5*euclidianDistance*sqrt(1/(1 + pow(m,2) ) );
      // 	    y_meet2=yc - m*0.5*euclidianDistance*sqrt(1/(1 + pow(m,2) ) );
      // 	    if(yc != PM[1]){
      // 	    x_meet1=xc + (y_meet1 - yc)/m;
      // 	    x_meet2=xc + (y_meet2 - yc)/m;
      // 	    }
      // 	    else{
      // 	      x_meet1=xc + 0.5*euclidianDistance;
      // 	      x_meet2=xc - 0.5*euclidianDistance;
      // 	    }
      // 	  }
      // 	  else{
      // 	    y_meet1=yc+0.5*euclidianDistance ;
      // 	    x_meet1=xc ;
      // 	    y_meet2=yc - 0.5*euclidianDistance;
      // 	    x_meet2=xc ;
      // 	  }
	  
      // 	}
     
      //case L
      if(DataAl.side!=0 && fabs((45*GRAD2RADIAN)- fabs(DataAl._alfa))< 1.5*Sigma && fabs((45*GRAD2RADIAN)-fabs(DataAl._beta))< 1.5*Sigma)
	{
	  z=L_FORM;
	  xc=(tan(theta2)*x2 -y2-tan(theta1)*x1+y1)/(tan(theta2)-tan(theta1));
	  yc= (xc-x1)*tan(theta1)+ y1;
	  // mu_x= (xc + PM[0])/2;
	  // mu_y= (yc + PM[1])/2;
	  mu_x= xc ;
	  mu_y= yc ;
	  ///////////////////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////
	  s_x=interaction_space * euclidianDistance/2;
	  L= sqrt(pow(xc-PM[0],2) + pow(yc-PM[1],2) );
	  s_y=interaction_space * L;  
	  ///////////////////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////
	  if(xc!=PM[0]){
	    m=(yc-PM[1])/(xc-PM[0]);
	    y_meet1=yc + m*0.75*euclidianDistance*sqrt(1/(1 + pow(m,2) ) );
	    y_meet2=yc - m*0.75*euclidianDistance*sqrt(1/(1 + pow(m,2) ) );
	    if(yc != PM[1]){
	    x_meet1=xc + (y_meet1 - yc)/m;
	    x_meet2=xc + (y_meet2 - yc)/m;
	    }
	    else{
	      x_meet1=xc + 0.75*euclidianDistance;
	      x_meet2=xc - 0.75*euclidianDistance;
	    }
	  }
	  else{
	    y_meet1=yc + 0.75*euclidianDistance ;
	    x_meet1=xc ;
	    y_meet2=yc - 0.75*euclidianDistance;
	    x_meet2=xc ;
	  }
	  
	  //To have only one point
	  if(sqrt( pow(y_meet1-PM[1],2) + pow(x_meet1-PM[0],2) ) < sqrt( pow(y_meet2-PM[1],2) + pow(x_meet2-PM[0],2) )){
	    x_meet1=x_meet2;
	    y_meet1=y_meet2;
	  }
	  else{
	    x_meet2=x_meet1;
	    y_meet2=y_meet1;
	  }
	 
	} 
     
      //case C
      if( DataAl.side!=0 && fabs((75*GRAD2RADIAN)- fabs(DataAl._alfa))< Sigma && fabs( (7.5*GRAD2RADIAN)-fabs(DataAl._beta))< Sigma)
	{
	  z=C_FORM;
	  xc=(tan(theta2)*x2 -y2-tan(theta1)*x1+y1)/(tan(theta2)-tan(theta1));
	  yc= (xc-x1)*tan(theta1)+ y1;
	  mu_x= (xc + PM[0])/2;
	  mu_y= (yc + PM[1])/2;
	  ///////////////////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////
	  s_x=interaction_space * euclidianDistance/4;
	  L= sqrt(pow(xc-PM[0],2) + pow(yc-PM[1],2));
	  s_y=interaction_space * L/3;  
	  ///////////////////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////
	  ///////////////////////////////////////////////////////////////////////
	 
	  if(xc!=PM[0]){
	    m=(yc-PM[1])/(xc-PM[0]);
	    y_meet1=mu_y + m*0.5*euclidianDistance*sqrt(1/(1 + pow(m,2) ) );
	    y_meet2=mu_y - m*0.5*euclidianDistance*sqrt(1/(1 + pow(m,2) ) );
	    if(yc != PM[1]){
	      x_meet1=mu_x + (y_meet1 - mu_y)/m;
	      x_meet2=mu_x + (y_meet2 - mu_y)/m;
	    }
	    else{
	      x_meet1=mu_x + 0.5*euclidianDistance;
	      x_meet2=mu_x - 0.5*euclidianDistance;
	    }
	  }
	  else{
	    y_meet1=mu_y+0.5*euclidianDistance ;
	    x_meet1=mu_x ;
	    y_meet2=mu_y - 0.5*euclidianDistance;
	    x_meet2=mu_x ;
	  }
	   //To have only one point
	  if(sqrt( pow(y_meet1-PM[1],2) + pow(x_meet1-PM[0],2) ) < sqrt( pow(y_meet2-PM[1],2) + pow(x_meet2-PM[0],2) )){
	    x_meet1=x_meet2;
	    y_meet1=y_meet2;
	  }
	  else{
	    x_meet2=x_meet1;
	    y_meet2=y_meet1;
	  }


	}
      /*
      if(z>0)
	{
	  cout<<"formacion de tipo "<<z<<endl;
	  cout<<"theta 1 "<<theta1/GRAD2RADIAN<<" theta2 "<<theta2/GRAD2RADIAN<<endl;  
	  cout<<"alfa "<< DataAl._alfa/GRAD2RADIAN<<"beta " << DataAl._beta/GRAD2RADIAN << endl;
	}
      */ 
    }  

  Data->angle=atan2(y2-y1,x2-x1);
  Data->media_x=mu_x;
  Data->media_y=mu_y;
  Data->sd_x=s_x;
  Data->sd_y=s_y; 
  Data->meet_points.push_back(x_meet1);  
  Data->meet_points.push_back(y_meet1); 
  Data->meet_points.push_back(x_meet2); 
  Data->meet_points.push_back(y_meet2); 
  return z;
}

//**************************************************************************************************									      
void Alignment(double x1,double y1,double theta1, double x2,double y2,double theta2, struct dataAlign *Data)
{
  //input: position and orientation of two pedestrian, angle in radians
  //output: angle between the vector following the orientation of each pedestrian
  //        and the straight line between the two positions
  
  //Get unitary vector for AB
  double AB[2];
  double BA[2], A[2], B[2];
  double m;
  int side;
  double xp1,yp1,xp2,yp2,_alfa,_beta;


  AB[0]=(x2-x1);
  AB[1]=(y2-y1);
  
 
  //Get unitary vector for A
  m=tan(theta1);
  if(theta1 >= -M_PI/2 && theta1 <= M_PI/2)
    xp1= x1 + 1.0;
  else
    xp1= x1 - 1.0;
  
  yp1 = m*(xp1-x1)+ y1;
 
  A[0]=(xp1-x1);
  A[1]=(yp1-y1);
  //Get angle between A y AB
  //_alfa= atan(AB(1)*A(2)- AB(2)*A(1), AB(1)*A(1) + AB(2)*A(2));
  _alfa= atan2(AB[0]*A[1]-AB[1]*A[0],AB[0]*A[0]+AB[1]*A[1]);
  
  //Get unitary vector for BA
  //TODO: check here why it changes when we have negative values
  BA[0]=-AB[0];
  BA[1]=-AB[1];
  //Get unitary vector for B
  m=tan(theta2);
  if(theta2 >= -M_PI/2 && theta2 <= M_PI/2)
    xp2= x2 + 1.0;
  else
    xp2= x2 - 1.0;
  
  yp2 = m*(xp2-x2)+ y2;
  
  B[0]=(xp2-x2);
  B[1]= (yp2-y2);
  
  //Get angle between B y BA
  
  _beta= atan2(BA[0]*B[1]- BA[1]*B[0], BA[0]*B[0] + BA[1]*B[1]);
  
  //side = 1 si estan del mismo lado o los dos estan sobre la misma linea, 0 en caso contrario 
  int sign1,sign2;
  if((x2-x1)*(yp1-y1)-(y2-y1)*(xp1-x1)>=0)
    sign1=1;
  else
    sign1=-1;

  if((x2-x1)*(yp2-y1)-(y2-y1)*(xp2-x1)>=0)
    sign2=1;
  else
    sign2=-1;

  if (sign1 == sign2)
    side = 1;
  else
    side = 0;
  

  //Return angles in radians
  Data->_alfa=_alfa;
  Data->_beta=_beta;
  Data->side= side;
} 

/**************************************************************************************************/
/******
Function for Calculating Personal space as a Gaussian function bi-dimensional
xk,yk 	coordinates of point to calculate
xp,yp 	coordinates pedestrian
dir 	direction of pedestrian in radians
****/
double PSpace(double xk, double yk, double xp, double yp, double dir, double pspace_size)
{  

 double a,c,x,y;
 double sigma_x, sigma_y;


 double z=0.0;
 float A =1.0;
// double x0=0.0,y0=0.0,theta=0.0;
 double phi=dir;//-M_PI/2;
// Changing coordinates of the point in a frame with center xp,yp
// and rotating phi degrees, this is done to calculate Gaussian
// in the origin and with ellipse shape in the right
x=(xk-xp)*cos(phi)+(yk-yp)*sin(phi);
y=(xk-xp)*(-1)*sin(phi)+(yk-yp)*cos(phi);



  if( x < 0)  //the point is at left
    {
    sigma_x=pspace_size;
    sigma_y=pspace_size;   
    }
  else
    {
    sigma_y=pspace_size;
    sigma_x=2*sigma_y; //to have a half ellipse at right
    }
 
  //This equations has been taken from Gaussian Function in Wikipedia
  //In our case we will have theta=0 then we proceed to simplify 
  //the equations
  //  a = pow(cos(theta),2)/(2*pow(sigma_x,2)) + pow(sin(theta),2)/(2*pow(sigma_y,2));
  a=1/(2*pow(sigma_x,2));
 // b = sin(2*theta)/(4*pow(sigma_x,2)) - sin(2*theta)/(4*pow(sigma_y,2)) ;
 // The signs has been changed to have conterclockwise rotation
  //  b=0;
   
 // c = pow(sin(theta),2)/(2*pow(sigma_x,2)) + pow(cos(theta),2)/(2*pow(sigma_y,2));
    c = 1/(2*sigma_y*sigma_y);
  
 //return (A*exp( - (a*(x-x0)^2 + 2*b*(x-x0)*(y-y0) + c*(y-y0)^2))) ;
    z=A*exp(-(a*x*x  + c*y*y));
   if(z>0.001) //For avoiding very small values
     {
       // cout<<"value PSpace "<< z <<endl;
       return z;
     }
   else return 0;

}
 
//funcion que evalua un punto en una gaussiana
//recibe un punto y los parametros de la gaussiana y devuelve el valor 
/*******************************************************************************/
double EvalGauss(double x, double y,const social_filter::int_data Data )
{
 
  double a, b, c,theta,z;
  double sigma_x, sigma_y;
  double A=1.0;
  double x0,y0;
  sigma_x=Data.sd_x;
  sigma_y=Data.sd_y;
  theta=Data.angle;
  x0=Data.media_x;
  y0=Data.media_y;

  //This equations has been taken from Gaussian Function in Wikipedia
  //In our case we will have theta=0 then we proceed to simplify 
  //the equations
   a = pow(cos(theta),2)/(2*pow(sigma_x,2)) + pow(sin(theta),2)/(2*pow(sigma_y,2));
   // a=1/(2*pow(sigma_x,2));
   b = sin(2*theta)/(4*pow(sigma_x,2)) - sin(2*theta)/(4*pow(sigma_y,2)) ;
 // The signs has been changed to have conterclockwise rotation
   
   c = pow(sin(theta),2)/(2*pow(sigma_x,2)) + pow(cos(theta),2)/(2*pow(sigma_y,2));
 
  
   z=A*exp( - (a*pow(x-x0,2) + 2*b*(x-x0)*(y-y0) + c*pow(y-y0,2))); 

   if(z>0.001) //For avoiding very small values
   return z;
   else return 0;
}

void transformPosetoMap(tf::TransformListener* listener,social_filter::humanPose on_odom, social_filter::humanPose* on_map)
{
  //changes a humanPose in frame /robot_i/odom to frame /map
 
  geometry_msgs::PoseStamped in;
  geometry_msgs::PoseStamped out;

  in.header.frame_id = on_odom.header.frame_id;
  in.header.stamp= on_odom.header.stamp;
  in.pose.position.x=on_odom.x;
  in.pose.position.y=on_odom.y;
  in.pose.position.z=0.0;
  geometry_msgs::Quaternion local_orientation=  tf::createQuaternionMsgFromYaw( on_odom.theta);	
  in.pose.orientation.x=local_orientation.x;
  in.pose.orientation.y=local_orientation.y;
  in.pose.orientation.z=local_orientation.z;
  in.pose.orientation.w=local_orientation.w;
  if(listener->canTransform("/map", in.header.frame_id, on_odom.header.stamp ) ) {  
    listener->transformPose("/map",in,out);
    on_map->x=out.pose.position.x;
    on_map->y=out.pose.position.y;
    on_map->theta=tf::getYaw(out.pose.orientation);
  }
  else{
    //TODO:find a best solution to the case the transform can not be done
    //std::cout<<"Could not transform from "<< in.header.frame_id.c_str()<<" to \map" << std::endl;
    on_map->x=0.0;
    on_map->y=0.0;
    on_map->theta=0.0;  
  }
 
}

//************************************************************************
bool group_tightness(social_filter::int_data a, social_filter::int_data b,int M[MAX_P][MAX_P])
{
  //this function check if the groups a, b satisfy the intra-group
  //tightness criterion
  int edges_p_q, hat_edges_p_q, edges_p, hat_edges_p, edges_q, hat_edges_q;
  social_filter::int_data c;

  hat_edges_p_q= min_edges(elements(a,b,&c));
  hat_edges_p= min_edges(a.id_members.size());
  hat_edges_q= min_edges(b.id_members.size());
  edges_p= n_edges(a,M);
  edges_q= n_edges(b,M);
  edges_p_q = n_edges(c,M);
 
  return edges_p_q >= hat_edges_p_q + edges_p - hat_edges_p + edges_q - hat_edges_q;
}

//************************************************************************

int n_edges(social_filter::int_data a, int form_matrix[MAX_P][MAX_P]){
  //return the number of edges in a by checking in the matrix
  //of f-formations
 vector<uint8_t>::iterator it;
 vector<uint8_t>::iterator it2;
 int edges=0;
 for(it=a.id_members.begin() ; it < a.id_members.end()-1; it++)
   for(it2=it + 1; it2 < a.id_members.end(); it2++){
     edges+=form_matrix[*it][*it2];
   }
 return edges;
}

//************************************************************************
int min_edges(int k)
{
  //function that gives the minimum number of edges for
  //a group
  if(k==1)
    return 0;
  else 
    if (k==2)
      return 1;
    else 
      if (k%2 ==0)
	return 	(k/2)*(k/2);
      else return ( (k-1)/2 )*( 1 + (k-1)/2 );
}

//************************************************************************
int elements(social_filter::int_data a,social_filter::int_data b,social_filter::int_data* c){
  //returns the number of elements of a union b
  //and cluster the two groups
 
  uint i;
  uint j;
  //copying all a members
  c->id_members=a.id_members;

  for(i=0 ; i < b.id_members.size(); i++){
    j=0;
    while(j < a.id_members.size()){
      if( b.id_members[i]==a.id_members[j])
	break;
      j++;
    }
    if(j==a.id_members.size())
      c->id_members.push_back( b.id_members[i]);
  } 
  c->type=CIRCULAR;
  c->media_x=0.0;
  c->media_y=0.0;
  c->sd_x=0.0;
  c->sd_y=0.0;

  return c->id_members.size();
}

double ips(double x, double y,double theta, double x_hi,double y_hi,double theta_hi,double v_hi){
  double d=0.0,z=0.0;
 double d_e= 2.5; //distance of effect for the IPS
 double d_l=1.0;//distance of decay for the IPS, this is additive then the effect is 3.5 meters
 //Sound speed 343 m/s
 double c=3.43;
 //frecuencia la 440Hz
 // double f=1.0; 
 //source speed
 //double vs=3.0;
 double factordistance=0.0;

 //this is done because there is a rotation not considered in 
 //the equation for IPS
 //  if( theta_hi < 0.0)
 //    theta_hi = theta_hi + 3.1416;
 //    else
 //     theta_hi = theta_hi - 3.1416;
 d=sqrt(pow(x-x_hi,2)+ pow(y-y_hi,2) );
 //d= distEuclide(x,y,x_hi,y_hi);
 if( d< d_e)
   factordistance=1.0;
 else 
   if( (d_e <= d) && (d <= d_e + d_l) )
     factordistance = 1 - ((d - d_e)/d_l);
   else
     factordistance = 0;
       
 //align is used to get the value of angle between the vector and 
 // the common line between point and source
 //calculating discomfort due to IPS
 struct dataAlign data;
 Alignment(x,y,theta,x_hi,y_hi,theta_hi,&data);

 // z=factordistance*f*( c/( c-v_hi*cos(data._beta)  )  );
  z=factordistance*( c/( c-v_hi*cos(data._beta)  )  );
 //normalizing before add
 //we suppose now that v_hi is 2.5 then we can calculate a fixed value
 //to keep ips in [0,.5]
 return (2.0*z/7.38 );
 //return (z/ (c / (c-v_hi) ) );

}

/*********************************************************/
