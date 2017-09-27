#include <iostream>
#include <math.h>

inline void transform(double y[], const double x[], const double T[]){
    y[0] = x[0]*T[0] + x[1]*T[1] + x[2]*T[2] + T[3];
    y[1] = x[0]*T[4] + x[1]*T[5] + x[2]*T[6] + T[7];
    y[2] = x[0]*T[8] + x[1]*T[9] + x[2]*T[10] + T[11];
}
inline void assign(double x[], double y[]){
    x[0] = y[0];
    x[1] = y[1];
    x[2] = y[2];
}
extern "C" {
void bestPoint(double *PC, int lr, double *T, double *ret)
{
    const int n = 480*640;
    double bp[3] = {0.0, 0.0, 0.0};
    double xyz_world[3];
    if (lr == 0) {  // r
        double center[3] = {1.467, -0.42714, 0.92472};
        for (int j = 0; j < n; j++){
            if(fabs(PC[j*3]) < 1e-9)  // bad point
                continue;
                
            transform(xyz_world, &PC[j*3], T);
            if(fabs(xyz_world[0]-center[0]) < 0.08 && fabs(xyz_world[1]-center[1]) < 0.06 && 
               fabs(xyz_world[2]-center[2]) < 0.05 && xyz_world[1] < bp[2])
               assign(bp, xyz_world);
        }
    }
    else {   // l
        double center[3] = {1.467, 0.42714, 0.92472};
        for (int j = 0; j < n; j++){
            if(fabs(PC[j*3]) < 1e-9)  // bad point
                continue;
                
            transform(xyz_world, &PC[j*3], T);
            if(fabs(xyz_world[0]-center[0]) < 0.08 && fabs(xyz_world[1]-center[1]) < 0.06 && 
               fabs(xyz_world[2]-center[2]) < 0.05 && xyz_world[1] > bp[2])
               assign(bp, xyz_world);
        }
    }
    assign(ret, bp);
}
}

