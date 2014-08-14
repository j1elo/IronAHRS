/*
 * razor_math.h
 *
 * Created: 29/06/2012 21:02:15
 * Author: Juan Navarro
 */

#ifndef RAZOR_MATH_H_
#define RAZOR_MATH_H_

double Vector_Dot_Product(double v1[3], double v2[3]);
void Vector_Cross_Product(double out[3], double v1[3], double v2[3]);
void Vector_Scale(double out[3], double v[3], double scale);
void Vector_Add(double out[3], double v1[3], double v2[3]);
void Matrix_Multiply(double a[3][3], double b[3][3], double out[3][3]);
//void Matrix_Vector_Multiply(double a[3][3], double b[3], double out[3]);
void init_rotation_matrix(double m[3][3], double yaw, double pitch, double roll);

#endif // RAZOR_MATH_H_
