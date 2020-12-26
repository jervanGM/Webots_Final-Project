
#ifndef GO_SEARCH_BACK_H_
#define GO_SEARCH_BACK_H_

#include <webots/DifferentialWheels.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <vector>

using namespace std;
using namespace webots;

/**
*Numero  de sensores ultrasonidos que dispone el robot
*/
#define NUM_DISTANCE_SENSOR 16
/**
*Máxima velocidad de las ruedas del robot
*/
#define MAX_SPEED           100
/**
*@author Javier Galán Méndez
*@date 26-11-2019
*@version 1.0
*@brief Controlador de un robot de busqueda y localización de dos personas en distintos tipos de escenarios
*/
class Rescue:public DifferentialWheels{

  public:
    /**
    *Contructor de la clase Rescue, inicializa todas las variables y habilita todos los sensores.
    *La velocidad del robot se inicializa en modo FORWARD, que andará recto.
    *Es usado para crear un robot de busqueda
    *@code
        int main(){
          Rescue *mirobot=new Rescue();
        }
    *@endcode
    */
    Rescue();
    
    /**
    *Usado para liberar la memoria de los sensores y resetear el robot a sus valores por defecto.
    *Se llama al usar la keyword delete a la clase creada
    *@code
        int main(){
          Rescue *mirobot=new Rescue();
          delete mirobot; 
        }
    *@endcode
    */
    ~Rescue();
    /**
    *Funcion para asignar un valor en concreto a los atributos _left_speed y _right_speed
    *@param left_speed Valor a asignar al atributo _left_speed
    *@param right_speed Valor a asignar al atributo _right_speed
    */
    void addSpeed(double left_speed,double right_speed);
    /**
    *Funcion generalmente usada en modo FORWARD para manener una dirección concreta dada por el 
    *atributo angle.Si el robot se sale de ur rango en torno al valor de angle, este se corrige girando en sentido contrario
    *hacia donde se mueve el robot por error
    * En este ejemplo, seguirá una dirección en torno a 90º :
    *@code
        if(ir_recto==true){
           _mode=FORWARD;
           angle=90;
           mirobot->controlForward();
        }
    *@endcode
    */
    void controlForward();
    /**
    *Cada vez que se llama, dará los valores de los sensores inicializados por el robot
    *(exceptuando las cámaras) para su posterior tratamiento. Ejemplo:
    *@code
         mi_robot-> getSensorsValue();
         cout<<"Compass:"<<compass<<endl;
         cout<<"X:"<<X<<", Y:"<<Y<<endl;
         cout<<"Valor sensor de distancia 0:"<<ir_val[0]<<endl;
      }
        
    *@endcode
    */
    void getSensorsValue();
    /**
    *Aplica la tecnica del bug0 en base a una posición objetivo y asigna un valor a deg que se
    *actualiza cada vez que se llama. Se suele usar en conjunto con controlForward.
    *@code
        bug0();//Da un valor a deg
        angle=deg;
        controlForward();
    *@endcode
    *La tecnica de bug0 es simple de aplicar, se da un objetivo al robot y este irá a ese objetivo tomando como 
    *referencia la dirección hacia el mismo. Mientras va, se deberán usar los sensores para esquivar objetos y usar controlLine()
    *para evitarlos
    */
    void bug0();
    /**
    *Función principal para los modos del robot, cada vez que se llama, mira cual fue el ultimo modo 
    *activado y asigna unas ciertas velocidades a addSpeed() segun el modo requerido.SI el modo es FORWARD, en la función llamará a 
    *control forward. En el ejemplo, el modo será girar a la derecha, por lo que llamará a addSpeed(MAXSPEED,-MAXSPEED/2) para 
    *girar a la derecha:
    *@code
        _mode=TURN_RIGHT;
        controlLine();
    *@endcode
    */
    void controlLine();
    
    /**
    *Primera función principal de las fases de Go,Search,Back. En esta fase, se ha aplicado un bug0 en torno al centro del mapa (0,0)
    *e irá hacia ese objetivo esquivando obstaculos mediante las diferentes situaciones y valores de los sensores de distancia
    *del robot. Al parar ese objetivo (X>0), el robot tendrá como dirección la de la salida del mapa, es decir, donde supuestamente
    *se encuentran los objetivos. El robot puede tomar direrentes rutas según como se encuentren los obtáculos, las principales salidas
    *que suele tomar son la central y la de abajo, el cual esta ultima tendrá que dar media vuelta para poder entrar por el pasillo que conduce
    *hacia dicha salida. Go se suele utilizar principalmente en el main, y Go llamará a las correspondientes funciones para realizar su
    *correcto funcionamiento. Go termina cuando yellowLine() devuelve un true, indicando que ha llegado a la salida. 
    */
    void Go();
    /**
    *Esta función es usada principalmente en Go, su función principal es la busqueda de la linea amarilla que indica que es
    *la salida del mapa. Esta función utiliza los metodos de la cámara esférica para analizar la imagen pixel por pixel en busca del 
    *nivel de amarillo concreto para considerarlo linea de llegada. Utiliza un contador y rangos para verificar que esa linea es la 
    *correcta.
    *@code
        while(yellowLine==false){//El robot andará hacia delante hasta que encuentre una linea amarilla
            _mode=FORWARD;
            controlLine();
        }
    *@endcode
    */
    bool yellowLine();
    /**
    *Inicializa el robot a una orientación general dada por parametro a la ida(Go()) o vuelta (Back()) del mapa,
    *permitiendo evitar errores de orientación. Orienta el robot dando unas velocidades a addSpeed()
    *@code
      while(1){
          if(time<45){//Al inicio, el robot girará hasta estar orientado a 0º
              initialPosition(0);
              time++;
          }
          setSpeed(_left_speed,_right_speed);//Método de DifferentialWheels para dar una velocidad al robot;
      }
    *@endcode
    *@param angulo_salida angulo dado en la llamada a la que se quiere iniciar el robot
    */
    void initialPosition(int angulo_salida);
    /**
    *Convierte los valores dados por el metodo getValues() de Compass en grados. 
    *@param in_vector Vector de valores de compass que se tratan con funciones de math.h para pasarlos a radianes y a grados
    */
    double convert_bearing_to_degrees(const double* in_vector);
    /**
    *Función especifia usada en Go(), indica una situación especial dada en el mapa de prueba
    *scenario7 el cual en Go() el robot detectará si se encuentra en dicho mapa y llamará a esta función para
    *desplazar el robot por dicho mapa mediante bug0.
    */
    void mapaSiete();
    /**
    *Segundo método principar de la clase Rescue, en este método tiene como finalidad buscar dos 
    *objetivos o cilindros verdes por un area la cual no puede salir hasta haber encontrado a los objetivos.
    *Utiliza una maquina de estados para seguir una serie de fases cuando encuentra sus objetivos:
    *@brief-Fase 0:Busca a los objetivos.Utiliza cameraProcess() para buscar los objetivos y los sensores para esquivar obtáculos en el prceso
    *@brief-Fase 1:Cuando encuentra al objetivo, el robot se para delante del mismo 2s. Establece la posición de dicho objetivo para no confundirlo comoo si fuera un segundo objetivo
    *@brief-Fase 2:Después de pararse, da una vuelta completa y segun si ha encontrado 1 o 2 objetivos irá a la fase 3 o 4
    *@brief-Fase 3:Posiciona el robot 90º a la izquierda y pasa a la fase 0 para buscar el segundo objetivo
    *@brief-Fase 4:Una vez que encuentra el segundo objetivo, utiliza bug0 para ir al lugar de donde llegó para pasar a la siguente fase
    */
    void Search();
    /**
    *Método utilizado en Search(), este método, a igual que yellowLine(),alaniza pixel por pixel para encontrar posibles objetivos
    *de distinta tonalidad de verde y si encuentra, se moverá en dirección a donde se encuentra mediante la posición en de la imagen
    *de la cámara en la que se encuentra.Devuelve un booleano si encuentra objetivos.
    *@code
      bool verde=false;
      while(1){
        verde=cameraProcess();
        setSpeed(_left_speed,_right_sped);
        if(verde==true) cout<<"He encontrado objetivo"<<endl;
      }
    *@endcode
    *Este método usa addSpeed() para mover el robot hacia el objeto, por lo que en la llamada debe estar setSpeed().
    */
    bool cameraProcess();
    /**
    *Convierte el espacio de colores RGB a HSV para un reconocimiento mejor de colores. Devuelve un vector HVS con 
    *Hue(HSV[0]), Saturation(HSV[2]), Value(HSV[1]).
    *@code
        vector <float> HSV=RGB2HSV(red_val,green_val,blue_val);
        cout<<"H:"<<HSV[0]<<", S:"<<HSV[2]<<", V:"<<HSV[1]<<endl;
    *@endcode
    *@param red canal rojo del espacio RGB
    *@param green canal verde del espacio RGB
    *@param blue canal azul del espacio RGB
    */
    vector<float> RGB2HSV(unsigned char red,unsigned char green,unsigned char blue);
    /**
    *Tercer y último método principal de Rescue, llama a initialPosition para situarse a 180º con respecto al mapa
    *y según de donde haya venido, utilizará folloWall() o homeComing(). El destino de las dos funciones es llegar al inicio del mapa
    *y finalizar la seguencia cuando llege a dicho inicio.
    */
    void Back();
    /**
    *En este método, el robot sigue el muro con los sensores de distancia laterales esquivando obstáculos con los sesonres de distancia
    *frontales. 
    */
    void folloWall();
    /**
    *Este método emplea la misma metodología que Go(), primero el robot debe ir al centro del mapa esquivando obstáculos
    *con los sensores de distancia, el robot gira a  la izquierda si se encuentra un obstaculo frontal.
    *Despues de llegar al centro del mapa, el robot se le marca otro objetivo que será el inicio del mapa siguiendo la misma metodología.
    */
    void homeComing();
    
   private:
     /**
      *Modos de funcionamiento del robot:
      *-STOP:para el robot.
      *-FORWARD:el robot avanza hacia una dirección.
      *-TURN_LEFT:gira en su eje vertical hacia la izquierda.
      *-TURN_RIGHT:gira en su eje vertical hacia la derecha.
      *-TURN_LEFT_CONTROL:gira ligeramente hacia la izquierda-
      *-TURN_RIGHT_CONTROL:gira ligeramente hacia la derecha.
     */
      enum Mode {
            STOP,
            FORWARD,
            TURN_LEFT,
            TURN_RIGHT,
            TURN_LEFT_CONTROL,
            TURN_RIGHT_CONTROL
        };
      Mode _mode;
      /**Velocidades de la rueda izquierda y derecha*/
      double _left_speed, _right_speed;
      //Canales verde,rojo y azul de las camaras
      unsigned char green, red, blue;
      /**Tiempo de salto de la simulación*/
      int _time_step;
      /**Contador de pixeles amarillos en la imagen*/
      int sum_y;
      /**Porcentage de amarillo en la imagen*/
      double percentage_yellow;
      /**Numeros de sensores activos*/
      int dist_sensor;
      /**Array de valores de los sensores de distancia*/
      double ir_val[NUM_DISTANCE_SENSOR];
      /**Contador de color para saltar de Go() a Search()*/
      int color_counter;
      /**Atributos de orientación del robot*/
      double angle,compass;
      /**Posición de y con respecto al mapa*/
      double Y;
      /**Posicion de x con respecto al mapa*/
      double X;
      /**Angulo de bug0 del robot con respecto a la posición objetivo*/
      double deg;
      /**Objetivos correspondientes en cada punto del proceso, cambia según las fases o parte de ellas*/
      double OBJECTIVE_X,OBJECTIVE_Y;
      /**Posiciónes x e y de llegada al final del mapa, indican por donde ha entado el robot*/
      double BACK_X,BACK_Y;
      /**Indicadores de dirección de la camara, indica por donde el robot debe ir en base a que
         mitad de la cámara se encuentra el objetivo*/
      int sum_l,sum_r;
      /**Porcentaje de verde en la imagen de la camara frontal.*/
      double percentage_green;
      //Atributos de los sensores
      Camera *_spherical_camera;
      DistanceSensor * _distance_sensor[NUM_DISTANCE_SENSOR];
      Compass * _my_compass;
      GPS *_my_gps;
      Camera *_forward_camera;
      double max=0;
};

#endif