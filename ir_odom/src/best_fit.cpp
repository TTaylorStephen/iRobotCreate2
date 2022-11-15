// Given 3 axis values for magnetometer data, this function fits the 
// data to a sphere using a Least Squares/Gauss Newton iterative method 
// minimizes sum of ((x-a)^2+(y-b)^2+(z-c)^2 -r^2)^2  
//
// Inputs:  
//		Data - Raw X, Y, and Z magnetometer data
//
// Outputs: 
// 		Center - Estimated C(x,y,z) of data due to hard iron effects 
//		Radius - Estimated radius 
///////////////////////////////////////////////////////////////////////


#include "mpu9250.h"

int j=0, k=0, it=0, count=1; 
long double r=0, ri=0, r0=0, x0=0, y0_=0, z0=0, p0=0, r_fin=0;
long double x_p=0, y_p=0, z_p=0, r_p=0, f=0;
int samp_num=3000;
bool start = false; 
long double x_sum=0, y_sum=0, z_sum=0, x_avg=0, y_avg=0, z_avg=0;

Eigen::MatrixXd A  =  Eigen::MatrixXd::Zero(samp_num,4);
Eigen::VectorXd	b  =  Eigen::VectorXd::Zero(samp_num);
Eigen::Vector4d x  =  Eigen::Vector4d::Zero();

Eigen::MatrixXd J  =  Eigen::MatrixXd::Zero(samp_num,4);  
Eigen::VectorXd d  =  Eigen::VectorXd::Zero(samp_num);
Eigen::Vector4d xp =  Eigen::Vector4d::Zero();
Eigen::Vector4d g  =  Eigen::Vector4d::Zero();


void Adc::sphereFit(std::vector<double> mag_data){

	if(start==false){
		std::cout << "Collecting Data - Please rotate sensor about all axes" << std::endl;
		std::cout << std::setprecision(16) << std::fixed;
		start=true;
	} 	

	if(j<samp_num){
		//expanded linear function for sphere at f = (ri^2-r^2) = (x-a)^2+(y-b)^2+(z-c)^2 -r^2  
		//collect samples and populate A and b matrices according to f
		A.row(j) << 2*mag_data.at(0), 2*mag_data.at(1), 2*mag_data.at(2), -1; 	
		b(j) = (mag_data.at(0)*mag_data.at(0))+(mag_data.at(1)*mag_data.at(1))+(mag_data.at(2)*mag_data.at(2));	
		++j;	
	}	
	else if(j==samp_num){
		if(it==0){
			std::cout << " A matrix is = \n" << A << std::endl;
			x = (A.transpose()*A).ldlt().solve(A.transpose()*b); //solve x = (AtA)^-1*(Atb) for initial estimates
			r = sqrt(x[0]*x[0]+x[1]*x[1]+x[2]*x[2]-x[3]);	     //get initial radius estimate from governing equation 
			std::cout << " initial radius is = " << r << std::endl;
			for(auto const& h : x){std::cout << " initial center is = " << h << std::endl;}
			++it;
		}
		if(k<samp_num){
			//calculate estimated radius 
			//ri = 1; //uncomment if using raw data 
			ri = sqrt((A(k,0)-x[0])*(A(k,0)-x[0])+(A(k,1)-x[1])*(A(k,1)-x[1])+(A(k,2)-x[2])*(A(k,2)-x[2]));
			
			//populate jacobian and minimizing vector
			J.row(k) << -1*(A(k,0)-x[0])/ri, -1*(A(k,1)-x[1])/ri, -1*(A(k,2)-x[2])/ri, -1;
			d(k) = -1*(ri-r);
			++k;	
		}
		else if(k==samp_num){
			xp=(J.transpose()*J).ldlt().solve(J.transpose()*d); //calculate partial values for x, y, z, and r
			g=J.transpose()*d;									//calculate right hand side of equation
			x[0]+=xp[0], x[1]+=xp[1], x[2]+=xp[2], r+=xp[3];	//sum partial values
			//std::cout << " Jacobian = \n" << J << std::endl;
			std::cout << " ri value = " << ri << std::endl;
			std::cout << " g vector = \n" << g.norm() <<std::endl;
			//std::cout << " xp = \n" << xp << std::endl;
			
			//check for convergence 	
			//break out of loop when norm of right hand side is very small	
			//at this point the partial values are at ~5 digits precision					
			if(g.norm()<0.01){ 	
				j=samp_num+1; //break out of loop														
				std::cout << "final center is at = (" << x[0] << ", " << x[1] << ", " << x[2] << ")" << std::endl;
				std::cout << "final radius = \n" << r << std::endl;	
			}
			k=0;	//reset k for next iteration
		}
	}

}


/****** method derived from: Forbes, Alistair B. Least Squares Best-Fit geometric elements, NPL report April 1989 ******/
