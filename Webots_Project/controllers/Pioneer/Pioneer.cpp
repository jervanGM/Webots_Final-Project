#include "Rescue.h"

/**
 * @brief Main program.
 */
int main(int argc, char **argv)
{   Rescue* robot=new Rescue();
    robot->Go();
    robot->Search();
    robot->Back();
    cout<<"FIN"<<endl;
    delete robot;
    return 0;
}
