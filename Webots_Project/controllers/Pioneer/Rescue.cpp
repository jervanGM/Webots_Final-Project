#include "Rescue.h"
#include <string>
#include "math.h"
#include <stdlib.h>
#include <algorithm>

//////////////////////////////////////////////
using namespace std;
using namespace webots;
Rescue::Rescue() : DifferentialWheels()
{
    // Inicializas valores de los atributos
    _time_step = 32;

    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED;
  
    _mode = FORWARD;
    sum_y=0;
    green = 0;
    red = 0;
    blue = 0;
    color_counter=0;
    percentage_yellow=0.0;
    percentage_green=0.0;
    OBJECTIVE_X=0.0;
    OBJECTIVE_Y=0.2;
    BACK_X=BACK_Y=0;
    angle=0;
    compass=0;
    sum_l=0;
    sum_r=0;
    X = Y = 0.0;
    deg=0.0;
    // Guradas los sensores en un array
    for(int i=0;i<NUM_DISTANCE_SENSOR;i++){
      string str = to_string(i);
      _distance_sensor[i] = getDistanceSensor("ds"+str);
      _distance_sensor[i]->enable(_time_step);
    }
    //Obtienes los dispositivos y los habilitas
    _my_compass=getCompass("compass");
    _my_compass->enable(_time_step);
    _my_gps=getGPS("gps");
    _my_gps->enable(_time_step);
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
    
    
}

//////////////////////////////////////////////

Rescue::~Rescue()
{
    // Desactiva dispositivos y para el robot
    _spherical_camera->disable();
    for (int i=0; i<NUM_DISTANCE_SENSOR; i++) {
        _distance_sensor[i]->disable();
    }
    _my_compass->disable();
    _my_gps->disable();
    this->disableEncoders();
    _forward_camera->disable();
    setSpeed(0.0,0.0);
}
 
//////////////////////////////////////////////
void Rescue::controlLine(){
  switch (_mode){
            case STOP:
                cout<<"STOP"<<endl;
                addSpeed(0,0);
                break;
            case FORWARD:
                cout<<"FORWARD"<<endl;
                controlForward();
                break;
            case TURN_LEFT:
                cout<<"TURN_LEFT"<<endl;
                addSpeed(-MAX_SPEED/2,MAX_SPEED/2);
                break;
            case TURN_RIGHT:
                cout<<"TURN_RIGHT"<<endl;
                addSpeed(MAX_SPEED/2,-MAX_SPEED/2);
                break;
            case TURN_LEFT_CONTROL:
                cout<<"TURN_LEFT_CONTROL"<<endl;
                addSpeed(MAX_SPEED/2,MAX_SPEED);
                break;
            case TURN_RIGHT_CONTROL:
                cout<<"TURN_RIGHT_CONTROL"<<endl;
                addSpeed(MAX_SPEED,MAX_SPEED/2);
                break;
            default:
                break;
        }
}
/////////////////////////////////////////////////

void Rescue::controlForward(){//Control del robot e torno a angle
    
    if(compass<angle-1) addSpeed(MAX_SPEED,MAX_SPEED/2);
    else if(compass>(angle+1)) addSpeed(MAX_SPEED/2,MAX_SPEED);
    else addSpeed(MAX_SPEED,MAX_SPEED);
    
}

/////////////////////////////////////////////////////////////////////////////
bool Rescue::yellowLine(){
    vector<float>HVS;
    //Tamaño de la imagen
    int image_width_s = _spherical_camera->getWidth();
    int image_height_s = _spherical_camera->getHeight();
    sum_y=0;
    const unsigned char *image_s = _spherical_camera->getImage();
    for (int x = 0; x < image_width_s; x++) {
        for (int y = image_height_s/2; y < image_height_s; y++) {
            green = _spherical_camera->imageGetGreen(image_s, image_width_s, x, y);
            red = _spherical_camera->imageGetRed(image_s, image_width_s, x, y);
            blue = _spherical_camera->imageGetBlue(image_s, image_width_s, x, y);
            
            //Pasas los resultados a HSV y se comprueba que se encuentra en el rango de colores
            HVS=RGB2HSV(red,green,blue);
            bool SPACEH=HVS[0]==240;
            bool SPACES=(HVS[2]>0.8558 && HVS[2]<=1);
            bool SPACEV=HVS[1]>80 && HVS[1]<155;
            //Cumple las condiciones    
            bool YELLOW=SPACEH==true && SPACES==true && SPACEV==true;
                
            if (YELLOW) sum_y++;
         }
     }
     percentage_yellow = (sum_y / (float) (image_width_s * image_height_s/2)) * 100;
     
     //Detecta si hay linea amarilla   
     if(percentage_yellow>0.5){
       color_counter++;
       cout<<"Linea amarilla"<<endl;
     }
     //En ciertos mapas necesita un valor minimo de detección llamado color_counter   
     if( color_counter>=12) return true;
        
     return false;
}
/////////////////////////////////////////////////////////////////////////////
void Rescue::addSpeed(double left_speed,double right_speed){
      this->_left_speed=left_speed;
      this->_right_speed=right_speed;
}
/////////////////////////////////////////////////////////////////////////////
void Rescue::getSensorsValue(){
  const double *gps_value;
  const double *compass_value;
  //Valores del gps
  gps_value=_my_gps->getValues();
  X=gps_value[2];
  Y=gps_value[0];
  //Valores de la brújula   
  compass_value=_my_compass->getValues();
  compass = convert_bearing_to_degrees(compass_value)-45;
  //Valores de los sensores delanteros de distancia
  dist_sensor=0;
  for(int i=0;i<5;i++){
     ir_val[i] = _distance_sensor[i]->getValue();
     if(ir_val[i]>0) dist_sensor++;
  }
  for(int i=11;i<=15;i++){
      ir_val[i] = _distance_sensor[i]->getValue();
      if(ir_val[i]>0) dist_sensor++;
  }
}
/////////////////////////////////////////////////////////////////////////////
void Rescue::bug0(){
    //Obtiene los valores de los sensores
    getSensorsValue();
    double delta_x=OBJECTIVE_X-X;
    double delta_y=Y-OBJECTIVE_Y;
    double rad=atan2(delta_y,delta_x);
    //Pasas a grados
    deg = rad * (180.0 / M_PI);
    
}


//////////////////////////////////////////////
//******************************************//
//////////////////////////////////////////////



void Rescue::initialPosition(int angulo_salida){
  
    if(compass<angulo_salida-2 && compass>=angulo_salida-180) addSpeed(MAX_SPEED/2,-MAX_SPEED/2);
    else if(compass>angulo_salida+2 && compass<angulo_salida-179) addSpeed(-MAX_SPEED/2,MAX_SPEED/2);
}
/////////////////////////////////////////////////////////////////////////////
void Rescue::Go()
{
    bool mapa7=false;//Indica si el robot se encuentra en scenario7
    double time=0;
    bool line=false;//Marca si ha detectado linea amarilla
    while (step(_time_step) != -1 && line==false) {
        cout<<angle<<endl;
        getSensorsValue();
        if(Y<-2.35287 && X<-6) mapa7=true;
        if((int)compass%180==0) angle=(int)compass;
        if((int)compass>=-180 && (int)compass<-160) angle=-180;
        //Distintas situaciones en el recorrido
        bool OBSTACULO_FRONTAL=ir_val[0]>0 && ir_val[15]>0 && (angle<10 && angle>-10);
        bool RINCON_IZQUIERDA=ir_val[0]>0 && ir_val[3]>0;
        bool MURO_IZQUIERDA=ir_val[3]>800 ;
        bool MURO_DERECHA=ir_val[12]>800;
        bool GIRA_NORTE_180= dist_sensor==0 && (compass>170 || compass<-170);
        bool ESQUINA_DERECHA=(ir_val[15]>0 || ir_val[12]>500) ;
        bool PRUEBA=ir_val[14]>0;
        bool ESQUINA_IZQUIERDA=(ir_val[0]>0 || ir_val[1]>0) && OBSTACULO_FRONTAL==false ;
        //Distintos actos según la situación obtenida a lo largo del mapa
        if(OBSTACULO_FRONTAL)_mode=TURN_RIGHT;
        else if(MURO_IZQUIERDA){
            if(ir_val[3]>ir_val[4]) _mode=TURN_RIGHT_CONTROL;
            else if(ir_val[3]<ir_val[4] ) _mode=TURN_LEFT_CONTROL;
            else _mode=FORWARD;
        }
        else if(MURO_DERECHA){
            if(ir_val[11]>=ir_val[12]) _mode=TURN_RIGHT_CONTROL;
            else _mode=TURN_LEFT_CONTROL;
        }
        else if(ESQUINA_DERECHA || PRUEBA)_mode=TURN_LEFT_CONTROL;
        else if(ESQUINA_IZQUIERDA)_mode=TURN_RIGHT_CONTROL;
        else _mode=FORWARD;
        //Eventos con prioridad
        if(dist_sensor==0 && X<=0){
            _mode=FORWARD;
            angle=deg;
        }
        if( GIRA_NORTE_180) _mode=TURN_LEFT_CONTROL;
        if(mapa7==true)mapaSiete();
        bug0();
        //Evita que en ciertos lugares del mapa cometa errores como al inicio o en una parte del mapa 7
        if(X>-7)line=yellowLine();
        if(X>7.5 && mapa7==true) line=true;
        //Llamada a función del robot
        controlLine();
        //Prioridad no controlada con controlLine()
        if(RINCON_IZQUIERDA)addSpeed(30,-30);
        //Orientación inicial
        if(time<=45 && (compass<-2 || compass>2)) initialPosition(0);
       
        // Establece las velocidades de los motores del robot
        setSpeed(_left_speed,_right_speed);
        time++;
    }
    //Aññade las ubicaciones desde donde ha entrado el robot a la área de busqueda de las personas
    OBJECTIVE_X=BACK_X=X;
    OBJECTIVE_Y=BACK_Y=Y;
}
/////////////////////////////////////////////////////////////////////////////
double Rescue::convert_bearing_to_degrees(const double* in_vector)
{
    double rad = atan2(in_vector[0], in_vector[2]);
    double degr = rad * (180.0 / M_PI);

    return degr;
}


/////////////////////////////////////////////////////////////////////////////
void Rescue:: mapaSiete(){//Movimiento zigzag en esta parte del mapa 7
    if(X>2.3 && Y<-2.5){
       if(X>2.3 && X<4.2){
         _mode=FORWARD;
         angle=30;
       }
       else if(X>=4.2 && X<8.2){
         _mode=FORWARD;
         angle=-15;
       }
    }
}



//////////////////////////////////////////////
//******************************************//
//////////////////////////////////////////////



void Rescue::Search(){
  double X_POS=0,Y_POS=0;
  double distancia=0;
  int time=0;
  int time_360=0;
  int time_90=0;
  int fases=0;
  bool salir=false;
  //Numero de personas encontradas
  int n=0;
  bool verde=false;
  
  while(step(_time_step) != -1 && salir==false){
      getSensorsValue();
      if((int)compass%90==0)angle=(int)compass;
      if(X_POS>0)distancia=sqrt(pow(X-X_POS,2)+pow(Y-Y_POS,2));
      //Condiciones de obstáculos durante la busqueda
      bool MURO=ir_val[0]>200 || ir_val[15]>200;
      bool MURO_DERECHA=ir_val[12]>500;
      bool MURO_IZQUIERDA=ir_val[3]>0;
      bool AREA_PERSONA1=distancia<2 && X_POS>0;
      
      switch(fases){
          case 0: //Fase de busqueda
                  if(MURO){
                    addSpeed(30,-30);
                  }
                  else if(MURO_DERECHA ){
                    if(ir_val[12]>ir_val[11] && ir_val[11]>0) addSpeed(30,60);
                    else if(ir_val[12]<ir_val[11] && ir_val[11]>0) addSpeed(60,30);
                    else if(ir_val[12]==ir_val[11] && ir_val[11]>0) addSpeed(60,60);
                  }
                  else if(MURO_IZQUIERDA ){
                    if(ir_val[4]>ir_val[3] && ir_val[4]>0) addSpeed(30,60);
                    else if(ir_val[4]<ir_val[3] && ir_val[4]>0) addSpeed(60,30);
                    else if(ir_val[4]==ir_val[3] && ir_val[4]>0) addSpeed(60,60);
                  }
                  else{
                    _left_speed=100;
                    _right_speed=100;
                  }
                  //Si no se encuentra en el área del objetivo ya encontrado
                  if(!AREA_PERSONA1){
                    cout<<"AREA"<<endl;
                    verde=cameraProcess();
                  }
                  //Control para evitar que el robot salga de la zona de busqueda
                  if(X<=8.5 &&(Y<-2.14 || (Y>2.1 && Y<3.5))){
                      angle=-90;
                      controlForward();
                  }
                  if(X<6.5) addSpeed(60,20);
                  //Encuentra objetivo
                  if(verde==true && ir_val[15]>0 && ir_val[0]>0 && AREA_PERSONA1==false){
                      if(n==1)n=2;
                      fases=1;
                  }
                  break;
          case 1://Fase de parada
                 addSpeed(0,0);
                 if(n==0){
                    double x_compass=-compass;
                    //Posiciones del primer objetivo encontrado
                    X_POS=X+1*cos(x_compass*M_PI/180);
                    Y_POS=Y+1*sin(x_compass*M_PI/180);
                    n=1;
                 }
                   time+=32;
                 
                 if(time>=2000) fases=2;//Tiempo de espera
                 break;
          case 2:
                 time=0;
                 addSpeed(100,-100);
                 time_360++;
                 if(time_360>35 && n==1) fases=3;
                 else if(time_360>35 && n==2) fases=4;
                 break;
          case 3:
                 time_360=0;
                 addSpeed(-20,20);
                 time_90++;
                 if(time_90>30) fases=0;
                 break;
          case 4: bug0();
                  angle=deg;
                  controlForward();
                  if(MURO) addSpeed(-40,40);
                  else if(ir_val[1]>0 ) addSpeed(60,30);
                  //Se encuentra cerca de la entrada
                  if((int)X==(int)BACK_X && (int)Y==(int)BACK_Y){
                    salir=true;
                  }
                  break;
      } 
      cout<<"Personas encontradas:"<<n<<endl;
      setSpeed(_left_speed,_right_speed);
      
    }
}
/////////////////////////////////////////////////////////////////////////////

bool Rescue::cameraProcess(){
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = _forward_camera->getHeight();
    vector<float>HVS;
    sum_l=0;
    sum_r=0;
    const unsigned char *image_f = _forward_camera->getImage();
    for(int i=0;i<image_width_f;i++){
        for(int j=0;j<image_height_f;j++){
            green=_forward_camera->imageGetGreen(image_f,image_width_f,i,j);
            red=_forward_camera->imageGetRed(image_f,image_width_f,i,j);
            blue=_forward_camera->imageGetBlue(image_f,image_width_f,i,j);
            HVS=RGB2HSV(red,green,blue);
            bool SPACEH=HVS[0]>111 && HVS[0]<120;
            bool SPACES=HVS[2]>0.2 && HVS[2]<=1;
            bool SPACEV=HVS[1]>20 && HVS[1]<155;
            bool GREEN=SPACEH==true && SPACES==true && SPACEV==true;
            //Lado de la cámara donde se encuentra el objetivo
            if(GREEN && i<=image_width_f/2) sum_l++;
            else if(GREEN && i>image_width_f/2) sum_r++;
            
        }
      }
        //Control del robot por cámara
        if(sum_l!=0 || sum_r!=0){
            if(sum_l>sum_r) addSpeed(50,60);
            else if(sum_l<sum_r) addSpeed(60,50);
            else addSpeed(90,90);
            
            return true;
        }
    
      return false;
}

/////////////////////////////////////////////////////////////////////////////

vector <float>Rescue:: RGB2HSV(unsigned char red,unsigned char green,unsigned char blue){
    vector<float>HSV;
    float R=(float)red;
    float G=(float)green;
    float B=(float)blue;
    //Expresiones de conversión a HSV
    HSV.push_back(180+((atan((sqrt(3)*(G-B))/((R-G)+(R-B)))*180)/M_PI));
    HSV.push_back((R+G+B)/3);
    HSV.push_back(1-(min({R,G,B})/HSV[1]));
    
    return HSV;
}


//////////////////////////////////////////////
//******************************************//
//////////////////////////////////////////////



void Rescue::Back(){
   color_counter=0;
   int initial_time=0;
   while(step(_time_step) != -1 && X>-8.4){
       getSensorsValue();
       
       if(initial_time<25){
         if(compass<0)compass+=360;
         initialPosition(180);
         initial_time++;
       }
       else{
       //Parte de abajo del mapa o parte de arriba
           if(BACK_Y<-2.5) folloWall();
           else homeComing();
           if(compass<0)compass+=360;
           controlLine();
       }
       
       
       setSpeed(_left_speed,_right_speed);
   }
}
/////////////////////////////////////////////////////////////////////////////
void Rescue::folloWall(){
//Control del seguimiento del muro
   bool MURO_IZQUIERDA=ir_val[3]>0 || ir_val[4]>0;
   bool MURO_FRONTAL=ir_val[0]>400 && ir_val[15]>400;
   bool ESQUINA=ir_val[1]>0;
   if(MURO_FRONTAL)_mode=TURN_RIGHT;
   else if(MURO_IZQUIERDA){
       if(ir_val[3]>ir_val[4])_mode=TURN_RIGHT_CONTROL;
       else if(ir_val[3]<ir_val[4])_mode=TURN_LEFT_CONTROL;
       else _mode=FORWARD;
   }
   else if(ESQUINA)_mode=TURN_RIGHT_CONTROL;
}

void Rescue::homeComing(){
  //Condiciones en el recorrido  
  bool OBSTACULO_FRONTAL=ir_val[0]>0 && ir_val[15]>0 && X<=6;
  bool MURO_DERECHA= ir_val[12]>800;
  bool MURO_IZQUIERDA=ir_val[3]>800;
  bool ESQUINA_IZQUIERDA=ir_val[1]>0;
  bool ESQUINA_DERECHA=ir_val[14]>0;
  //Manejo de las condiciones
  if(OBSTACULO_FRONTAL) _mode=TURN_LEFT;
  else if(ESQUINA_DERECHA)_mode=TURN_LEFT_CONTROL;
  else if(ESQUINA_IZQUIERDA)_mode=TURN_RIGHT_CONTROL;
  else if(MURO_DERECHA){
  
    if(ir_val[12]>ir_val[11])_mode=TURN_LEFT_CONTROL;
    else if(ir_val[12]<ir_val[11])_mode=TURN_RIGHT_CONTROL;
    else _mode=FORWARD;
    
  }
  else if(MURO_IZQUIERDA){
    if(ir_val[4]>ir_val[3])_mode=TURN_LEFT_CONTROL;
    else if(ir_val[4]<ir_val[3])_mode=TURN_RIGHT_CONTROL;
    else _mode=FORWARD;
  }
  //Establecimiento de objetivos y bug0
  if(dist_sensor==0){
     if(X<1){
      OBJECTIVE_X=-8;
      OBJECTIVE_Y=-0.65;
     }
     else{
       OBJECTIVE_X=1;
      OBJECTIVE_Y=0;
     }
     bug0();
     _mode=FORWARD;
     if(deg<0)deg+=360;
     angle=deg;
     
  }
}
/////////////////////////////////////////////////////////////////////////////