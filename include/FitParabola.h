/*********************************************
FitParabola
Claire Lackner
August 16, 2006

Fits a second order curve to a set points
used in soft contacts to calculate the analytic curve in 
the neighborhood of a contact
The curve is of the form z = Ax^2 +By^2 +Cxy
RotateParaboloid takes the fit and rotates it such that the mixed term
goes to zero, leaving z = R1x^2 + R2y^2 
where R1 and R2 are the radii of curvature
*********************************************/

#include "matvec3D.h"

inline void FitParaboloid( vec3 *points, int numpts, double *coeffs );
inline void RotateParaboloid( double *coeff, double *R1, double *R2, mat3 *Rot, double *rotAngle );

#define FIT_TINY 1.0e-6
#define CONCAVE_WARNING 1.0e-3

void FitParaboloid( vec3 *points, int numpts, double *coeffs )
{
	//std::cerr << "Numpts: " << numpts;
	//trying to solve Ax=z for x where x is the vector of the coefficients
	//x_1=coeff x^2, x_2=coeff y^2, x_3=coeff xy

	mat3 A, A_add, A_inverse;
	vec3 z, z_add, x;

		z.set(0, 0, 0);
	A=mat3::ZERO;
	
	for( int i = 0; i < numpts; i++ )
	{
		z_add.set(points[i].x()*points[i].x()*points[i].z(), 
				points[i].y()*points[i].y()*points[i].z(),
				points[i].x()*points[i].y()*points[i].z());
		z+=z_add;

		double M [9]={points[i].x()*points[i].x()*points[i].x()*points[i].x(),	//0,0
					points[i].x()*points[i].x()*points[i].y()*points[i].y(), //1,0
					points[i].x()*points[i].x()*points[i].x()*points[i].y(), //2,0
					points[i].x()*points[i].x()*points[i].y()*points[i].y(), //0,1
					points[i].y()*points[i].y()*points[i].y()*points[i].y(), //1,1
					points[i].x()*points[i].y()*points[i].y()*points[i].y(), //2,1
					points[i].x()*points[i].x()*points[i].x()*points[i].y(), //0,2
					points[i].x()*points[i].y()*points[i].y()*points[i].y(), //1,2
					points[i].x()*points[i].x()*points[i].y()*points[i].y()}; //2,2
		
		A_add.set(M);

		A+=A_add;
	}

	A_inverse = A.inverse();
	
	x=A_inverse*z;

	coeffs[0] = x.x();
	coeffs[1] = x.y();
	coeffs[2] = x.z();
	for (int i=0; i<3; i++) {
		//almost planar
		if ( fabs(coeffs[i]) < FIT_TINY ) {
			coeffs[i] = 0.0;
		} 
		/*
		else if ( coeffs[i] < 0) {
			//concave
			if ( fabs(coeffs[i]) > CONCAVE_WARNING ) {
				//if really concave, display warning
				std::cerr << "Warning: concave fit at soft contact: " << coeffs[i] << std::endl;
			}
			coeffs[i] = 0;
		}
		*/
	       
	}
	//std::cerr<<"\nRadii of curvature: "<<coeffs[0]<<"\t"<<coeffs[1]<<"\t"<<coeffs[2]<<std::endl;

}

void RotateParaboloid( double *coeff, double *R1, double *R2, mat3 *Rot, double *rotAngle )
{
	//rotate a given parabola so that the xy coefficient is zero
	//fill out the rads (radii of curvature) and the rotation that
	//takes you from the first (contact) frame into the rotated frame

	//tan(2theta) = C/(B-A)

	//R^t*[[A C/2][C/2 B]]*R = [[rads[0] 0][0 rads[1]]
	//where R is the rotation matrix [[cos(theta) -sin(theta)][sin(theta) cos(theta)]]


	double theta, sintheta, costheta;
	vec3 col1, col2, col3;

	if( fabs(coeff[2]) > FIT_TINY )
	{
		if( coeff[0] != coeff[1] )
			theta = atan( coeff[2]/(coeff[1] - coeff[0])  )/2.0;
		else
			theta = M_PI/2;

		*rotAngle = theta;
		sintheta = sin(theta);
		costheta = cos(theta);
	
		col1.set( costheta, sintheta, 0.0 );
		col2.set( -1.0*sintheta, costheta, 0.0 );
		col3.set( 0.0, 0.0, 1.0 );
		Rot->set( col1, col2, col3 );

		double temp1 = (coeff[0]*costheta*costheta + coeff[1]*sintheta*sintheta 
					- coeff[2] * sintheta*costheta)*2;
		double temp2 = (coeff[0]*sintheta*sintheta + coeff[1]*costheta*costheta 
					+ coeff[2] * sintheta*costheta)*2;
		if( fabs(temp1) > FIT_TINY ) {
			*R1 = 1/temp1;
		} else { 
			*R1 = -1.0;
		}
		if( fabs(temp2) > FIT_TINY ) {
			*R2 = 1/temp2;
		} else {
			*R2 = -1.0;
		}
		//the else's take care of flat case, which will be addressed
		//in finding the relative radii of curvature
	}
	else
	{
		if( fabs(coeff[0]) > FIT_TINY ) {
			*R1 = 1/(2*coeff[0]);
		} else { 
			*R1 = -1.0;
		}
		if( fabs(coeff[1]) > FIT_TINY ) {
			*R2 = 1/(2*coeff[1]);
		} else {
			*R2 = -1.0;
		}

		*rotAngle = 0;
		*Rot = mat3::IDENTITY;

	}

	//std::cerr<<"Coeffs on entry: " << coeff[0] << " " << coeff[1] << " " << coeff[2] << std::endl;
	//std::cerr<<"Radii of curvature: "<<*R1<<"\t"<<*R2<<std::endl;

	//final form of eqn z = 1/2r1*x^2 +1/2r2*y^2
}
