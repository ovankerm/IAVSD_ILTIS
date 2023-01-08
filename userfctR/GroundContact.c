/**
 * @file Calspan.c
 *
 * This file implements the Calspan model for wheel-ground contact forces as described in [1].
 *
 * [1]J.-C. Samin and P. Fisette, Symbolic Modeling of Multibody Systems, vol. 112. Dordrecht: Springer Netherlands, 2003.
 * [2]S. Frik, G. Leister, and W. Schwartz, “Simulation of the IAVSD Road Vehicle Benchmark Bombardier Iltis with FASIM, MEDYNA, NEWEUL and SIMPACK,” Vehicle System Dynamics, vol. 22, pp. 215–253, Jan. 1993.

 *
 * Creation date: 22/07/2018
 *
 * (c) Universite catholique de Louvain
 */

#include "math.h"
#include "mbs_matrix.h"
#include "GroundContact.h"

double ComputeRadial(double Xw, double Zw, double Kw, double Rw, double *Q, double *ng, int hole, int left, int simple, int poule, int cosine){
    /*
        Xw : x position of the wheel
        Zw : z position of the wheel
        Kw : spring constant of the tyre
        Rw : radius of the wheel
        Q : point of application of the force (to be filled)
        ng : vector normal to the ground (to be filled)
        hole : =1 if hole ; =0 if bump
    */

    double Zp;

    if((poule && (Xw <= 6 || Xw >= 6.5)) || (poule && left) || (cosine && (Xw <= 0.97 || Xw >=5.97)) || (!poule && !cosine)){
        Zp = 0;
        Q[1] =Xw; Q[2] =0; Q[3] =Zp;
        ng[1]=0; ng[2]=0; ng[3]=1;

        if (Zw-Rw<0){
            return Kw*(Rw-Zw);
        }

        return 0.0;
    } else {
        double slope; // d(road profile)/d(x)
        double DZ;    // Zw-Zp
        double DZq, DXq, H, L_QG;

        if(poule){
            slope = 0.2/0.5;
            Zp = -0.2 + 0.2*(Xw-6)/0.5;
        } else {
            Zp =    0.1*    (1-cos(2*M_PI*(Xw-0.97)/5)); // [m]  Road profile
            slope = 0.2*M_PI/5*sin(2*M_PI*(Xw-0.97)/5); // [-]  Corresponding slope
            if (hole){
                Zp    *=-1;
                slope *=-1;
            }
        }

        // printf("%f\n", Zw);
        if (Zp <= Zw-Rw){
            Q[1] =0; Q[2] =0; Q[3] =0;
            ng[1]=0; ng[2]=0; ng[3]=1;

            return 0.0;
        }

        if(simple){
            printf("road : %f, radius : %f, wheel : %f\n", Zp, Rw, Zw);
            Q[1] =Xw; Q[2] =0; Q[3] =Zp;
            return Kw*(Zp + Rw - Zw);
        }
        // Computation, by Similar triangles
        DZ = Zw-Zp;
        DZq = slope*slope*DZ/(1+slope*slope);
        DXq = DZq/slope;

        // Fill values
        Q[1] = Xw+DXq;
        Q[2] = 0;
        Q[3] = Zp+DZq;

        H = sqrt(1+slope*slope);
        L_QG = DZ/H; // length Point Q to Wheel center

        ng[1] = (-DXq)/L_QG;
        ng[2] = 0;
        ng[3] = (Zw-Q[3])/L_QG ;

        return Kw*(Rw-DZ)/H;
    }
}

double ComputeSimpleRadialForce(double Xw, double Zw, double Kw, double Rw, double *Q, double *ng, int hole){
    double Zp;    // ground height under wheel center
    // Road profile from [2]
    if (Xw <= 0.97 || Xw >=5.97){
        Zp = 0;
    } else {
        Zp = 0.1 * (1-cos(2*M_PI*(Xw-0.97)/5));
    }
    Zp *= hole ? -1 : 1;

    ng[1]=0; ng[2]=0; ng[3]=1;

    if (Zp <= Zw-Rw){
        Q[1] = 0; Q[2] = 0; Q[3] = 0;
        return 0.0;
    } else {
       Q[1] =Xw; Q[2] =0; Q[3] =Zp;
       return Kw*(Zp + Rw - Zw);
    }
}

double ComputeRadialForce(double Xw, double Zw, double Kw, double Rw, double *Q, double *ng, int hole)
{
    /*
        Xw : x position of the wheel
        Zw : z position of the wheel
        Kw : spring constant of the tyre
        Rw : radius of the wheel
        Q : point of application of the force (to be filled)
        ng : vector normal to the ground (to be filled)
        hole : =1 if hole ; =0 if bump
    */
    double Zp;    // ground height under wheel center

    // Road profile from [2]
    if (Xw <= 0.97 || Xw >=5.97){
        Zp = 0;
        Q[1] =Xw; Q[2] =0; Q[3] =Zp;
        ng[1]=0; ng[2]=0; ng[3]=1;

        if (Zw-Rw<0){
            return Kw*(Rw-Zw);
        }

        return 0.0;
    }
    else{
        double slope; // d(road profile)/d(x)
        double DZ;    // Zw-Zp
        double DZq, DXq, H, L_QG;

        // From road profile
        Zp =    0.1*    (1-cos(2*M_PI*(Xw-0.97)/5)); // [m]  Road profile
        slope = 0.2*M_PI/5*sin(2*M_PI*(Xw-0.97)/5); // [-]  Corresponding slope
        if (hole){
            Zp    *=-1;
            slope *=-1;
        }
        // Ground contact condition
        if (Zp <= Zw-Rw){
            Q[1] =0; Q[2] =0; Q[3] =0;
            ng[1]=0; ng[2]=0; ng[3]=1;

            return 0.0;
        }
        // Computation, by Similar triangles
        DZ = Zw-Zp;
        DZq = slope*slope*DZ/(1+slope*slope);
        DXq = DZq/slope;

        // Fill values
        Q[1] = Xw+DXq;
        Q[2] = 0;
        Q[3] = Zp+DZq;

        H = sqrt(1+slope*slope);
        L_QG = DZ/H; // length Point Q to Wheel center

        ng[1] = (-DXq)/L_QG;
        ng[2] = 0;
        ng[3] = (Zw-Q[3])/L_QG ;

        return Kw*(Rw-DZ)/H;
    }
}

double ComputeRadialForce_Belgian_road(double Xw, double Zw, double Kw, double Rw, double *Q, double *ng, int left)
{
    /*
        Xw : x position of the wheel
        Zw : z position of the wheel
        Kw : spring constant of the tyre
        Rw : radius of the wheel
        Q : point of application of the force (to be filled)
        ng : vector normal to the ground (to be filled)
        
    */
    double Zp;    // ground height under wheel center
    double slope = 0.2/0.5 ;
    if (Xw >= 6 && Xw <= 6.5 && left){
        Zp = -0.2 + 0.2*(Xw-6)/0.5;
        

        if (Zp <= Zw-Rw){
            Q[1] =0; Q[2] =0; Q[3] =0;
            ng[1]=0; ng[2]=0; ng[3]=1;

            return 0.0;
        }
        // Correction for normal force
        double DZ, DZq, DXq; 
        DZ = Zw-Zp;
        DZq = slope*slope*DZ/(1+slope*slope);
        DXq = DZq/slope;

        // Fill values
        Q[1] = Xw+DXq;
        Q[2] = 0;
        Q[3] = Zp+DZq;

        double H = sqrt(1+slope*slope);
        double L_QG = DZ/H; // length Point Q to Wheel center

        ng[1] = (-DXq)/L_QG;
        ng[2] = 0;
        ng[3] = (Zw-Q[3])/L_QG ;
        return Kw*(Rw-Zw + Zp);
    }
    else{
        double slope; // d(road profile)/d(x)
        double DZ;    // Zw-Zp
        
        // From road profile
        Zp =   0; // [m]  Road profile
        slope = 1.0 ; // [-]  Corresponding slope
        
        // Ground contact condition
        if (Zp <= Zw-Rw){
            Q[1] =0; Q[2] =0; Q[3] =0;
            ng[1]=0; ng[2]=0; ng[3]=1;

            return 0.0;
        }
        // Computation, by Similar triangles
        DZ = Zw-Zp;
        
        Q[1] =Xw; Q[2] =0; Q[3] =Zp; // just to try

        ng[1] = 0;
        ng[2] = 0;
        ng[3] = 1 ;

        return Kw*(Rw-DZ);
    }
}

double ComputeRadialForce_Bumpy(double Xw, double Yw, double Zw, double Kw, double Rw, double *Q, double *ng, int left)
{
    /*
        Xw : x position of the wheel
        Zw : z position of the wheel
        Kw : spring constant of the tyre
        Rw : radius of the wheel
        Q : point of application of the force (to be filled)
        ng : vector normal to the ground (to be filled)
        
    */
    double Zp;    // ground height under wheel center
    double slope ;
    

    if (Xw >= 3.15 ){
        Zp = sin(Xw)/15 + sin(Yw)/15;
        slope = cos (Xw)/15 - sin(Yw)/15;
        if (left){
            Zp *= -1;
            slope *= -1;
        }

        if (Zp <= Zw-Rw){
            Q[1] =0; Q[2] =0; Q[3] =0;
            ng[1]=0; ng[2]=0; ng[3]=1;

            return 0.0;
        }
        // Correction for normal force
        double DZ, DZq, DXq; 
        DZ = Zw-Zp;
        DZq = slope*slope*DZ/(1+slope*slope);
        DXq = DZq/slope;

        // Fill values
        Q[1] = Xw+DXq;
        Q[2] = 0;
        Q[3] = Zp+DZq;

        double H = sqrt(1+slope*slope);
        double L_QG = DZ/H; // length Point Q to Wheel center

        ng[1] = (-DXq)/L_QG;
        ng[2] = 0;
        ng[3] = (Zw-Q[3])/L_QG ;
        return Kw*(Rw-Zw + Zp);
    }
    else{
        double slope; // d(road profile)/d(x)
        double DZ;    // Zw-Zp
        
        // From road profile
        Zp =   0; // [m]  Road profile
        slope = 1.0 ; // [-]  Corresponding slope
        
        // Ground contact condition
        if (Zp <= Zw-Rw){
            Q[1] =0; Q[2] =0; Q[3] =0;
            ng[1]=0; ng[2]=0; ng[3]=1;

            return 0.0;
        }
        // Computation, by Similar triangles
        DZ = Zw-Zp;
        
        Q[1] =Xw; Q[2] =0; Q[3] =Zp; // just to try

        ng[1] = 0;
        ng[2] = 0;
        ng[3] = 1 ;

        return Kw*(Rw-DZ);
    }
}

void Calspan(double *Flat,double *Myaw, double Fvert, double Anglis, double Angcamb, double Gliss, double A0, double A1, double A2, double A3, double A4,
             double Aomegat, double B1, double B2, double B3, double B4, double SN, double AF1, double AF2, double AF3)
{
    // internal coefficient
    double f, g, alphaBar, alphaphi;
    // If all coefficents are 0.0 use the ones from [2]
    if (A0==0 && A1==0 && A2==0 && A3==0 && A4==0 && Aomegat==0 && B1==0 && B2==0 && B3==0 && B4==0 && SN==0 && AF1==0 && AF2==0 && AF3==0){
        // Force coefficient
        A0 =  2625.00; //[N]
        A1 =    14.47; //[-]
        A2 = 12930.00; //[N]
        A3 =     2.29; //[-]
        A4 = 18175.00; //[N]
        Aomegat = 1;   //[-]
        // Friction coefficient
        B1 = -0.464e-4; //[N^-1]
        B2 =  0.000;    //[s/m]
        B3 =  1.216;    //[-]
        B4 =  2.18e-11; //[N^-2]
        // Skid number ratio
        SN = 1.0274;    //[-]
        // Torque coefficient
        AF1 = -0.171e-4; //[m/N]
        AF2 =  0.171e-4; //[m/N]
        AF3 =  0.000;    //[m]
    }

    // Tire side Force: f
    f = (B1*Fvert+B3+B4*Fvert*Fvert)*SN;// +B2*v_t*SN neglected because B2=0 and v_t:=the tyre velocity

    // Equivalent slide slip : alphaB
    if (Fvert<=Aomegat*A2){
        alphaphi =(-A2*A3*(A4-Fvert)*Fvert)/(A4*(A1*Fvert*(Fvert-A2)-A0*A2))*Angcamb;
        alphaBar = (A1*Fvert*(Fvert-A2)-A0*A2)/(A2*f*Fvert)*(Anglis+alphaphi);
    }
    else{
        alphaphi = (-A2*A3*Aomegat*(A4-Aomegat*A2))/(-A0*A4)*Angcamb; // denominator simplified: A4*(A1*A2*Aomegat*(Aomegat-1)-A0)*Angcamb & (Aomegat-1)=0
        alphaBar = (-A0)/(f*Fvert)*(Anglis+alphaphi); // numerator simplified : A1*A2*Aomegat*(Aomegat-1)-A0 & (Aomegat-1)=0
    }


    // Sideforce shaping function
    if(fabs(alphaBar)<3){
        g = alphaBar-alphaBar*fabs(alphaBar)/3+alphaBar*alphaBar*alphaBar/27;
    }
    else{
        g = alphaBar/fabs(alphaBar);
    }

    // Lateral force
    Flat[0] = f*Fvert*g*Gliss;

    // Aligning torque
    Myaw[0] = (AF1*Fvert+AF2*fabs(Flat[0]))*Flat[0]; // -AF3*Fvert*Angcamb

    return;
}