/**
 * @file Calspan.h
 *
 * Header of Calspan.c
 *
 * Creation date: 22/07/2018
 *
 * (c) Universite catholique de Louvain
 */
#ifndef GROUNDCONTACT_h
#define GROUNDCONTACT_h

double ComputeSimpleRadialForce(double Xw, double Zw, double Kw, double Rw, double *P, double *ng, int hole);

double ComputeRadialForce(double Xw, double Zw, double Kw, double Rw, double *P, double *ng, int hole);

void Calspan(double *Flat,double *Myaw, double Fvert, double Anglis, double Angcamb, double Gliss, double A0, double A1, double A2, double A3, double A4,
             double Aomegat, double B1, double B2, double B3, double B4, double SN, double AF1, double AF2, double AF3);
#endif