#include "tinyxml.h"
#include <stdio.h>

/*
g++ -c -Wall -Wno-unknown-pragmas -Wno-format -O3   tinyxml.cpp -o tinyxml.o
g++ -c -Wall -Wno-unknown-pragmas -Wno-format -O3   tinyxmlparser.cpp -o tinyxmlparser.o
g++ -c -Wall -Wno-unknown-pragmas -Wno-format -O3   tinyxmlerror.cpp -o tinyxmlerror.o
g++ -c -Wall -Wno-unknown-pragmas -Wno-format -O3   tinystr.cpp -o tinystr.o
g++ -c -Wall -Wno-unknown-pragmas -Wno-format -O3   xmlrri.cpp -o xmlrri.o
g++ -o xmlrri  tinyxml.o tinyxmlparser.o tinyxmlerror.o tinystr.o xmlrri.o 
 * 
 * */

int main()
{

	//
	// We start with the 'demoStart' todo list. Process it. And
	// should hopefully end up with the todo list as illustrated.
	//
        const char* msg = "<RobData Id=\"5986\" Ts=\"765.315969\"><Ts_act>765.311368</Ts_act><P_act X=\"397.589\" Y=\"-34.9865\" Z=\"304.967\" Rx=\"-179.995\" Ry=\"-0.000497347\" Rz=\"90.0006\"/><J_act J1=\"-0.0877707\" J2=\"0.494737\" J3=\"0.240517\" J4=\"1.21592e-06\" J5=\"0.83563\" J6=\"1.48301\"/><Ts_des>765.356440</Ts_des><P_des X=\"397.589\" Y=\"-34.9865\" Z=\"304.967\" Rx=\"-179.995\" Ry=\"-0.000497347\" Rz=\"90.0006\"/><J_des J1=\"-0.0877707\" J2=\"0.494737\" J3=\"0.240517\" J4=\"1.21592e-06\" J5=\"0.83563\" J6=\"1.48301\"/><AppData>0</AppData></RobData>";
        TiXmlDocument doc;
        doc.Parse( msg );
        TiXmlElement* element = doc.FirstChildElement( "RobData" ); 
        double x;
        if (element){
                TiXmlElement* Pact = element->FirstChildElement("P_act");
                Pact->QueryDoubleAttribute( "X", &x );
                printf("%.5lf\n", x);
                TiXmlElement* Jact = element->FirstChildElement("J_act");
                Jact->QueryDoubleAttribute( "J1", &x );
                printf("%.5lf\n", x);
        }
        

}
