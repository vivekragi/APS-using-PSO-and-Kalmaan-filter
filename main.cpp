#include <bits/stdc++.h>
#define R 6400
using namespace std;

// PSO parameters
double pso_position[2];
double pso_velocity[2];
double measurement[2];

// Cost Function Constants
double k_square = 10.0;
double mew = 0.012;

// Kalman parameters
double kalman_position[2];
double kalman_velocity[2];

// Normal Distribution
double sample(double variance) {
    double u = ((double) rand() / (RAND_MAX)) * 2 - 1;
    double v = ((double) rand() / (RAND_MAX)) * 2 - 1;
    double r = u * u + v * v;
    if (r == 0 || r > 1)
        return sample(variance);
    double c = sqrt(-2 * log(r) / r);
    return variance*u*c;
}

// Distance between 2 points given gps coordinates
double distance(double phi1,double phi2,double lamda1,double lamda2)
{
    double delta_phi = phi1 - phi2;
    double delta_lamda = lamda1 - lamda2;
    double a = sin(delta_phi/2.0)*sin(delta_phi/2.0) + cos(phi1)*cos(phi2)*sin(delta_lamda/2.0)*sin(delta_lamda/2.0);
    double c = 2.0*atan2(sqrt(a),sqrt(1-a));
    double dist = R*c;
    cout<<"Distance: "<<dist<<endl;
    return dist;
}

// Angle to Destination given 2 gps coordinates
double bearing(double phi1,double phi2,double lamda1,double lamda2){
    double delta_lamda = lamda1 - lamda2;
    double bear = atan2(sin(delta_lamda)*cos(phi2), cos(phi1)*sin(phi2) - cos(delta_lamda)*sin(phi1)*cos(phi2));
    cout<<"Bearing: "<<bear<<endl;
    return bear;
}


// Given distance, bearing and current location this gives final coordinates
double heading_error(double phi1,double lamda1,double final_phi2,double final_lamda2,double bear,double dist)
{
    double theta  = bear;
    double delta = dist/R;
    double phi2 = asin(sin(phi1)*cos(delta) + cos(phi1)*sin(delta)*cos(theta));
    double lamda2 = lamda1 + atan2(sin(theta)*sin(delta)*cos(phi1), cos(delta) - sin(phi1)*sin(phi2));
    double error = bearing(phi2,final_phi2,lamda2,final_lamda2);
    cout<<"Final Lat-Lon and error: "<<phi2<<" "<<lamda2<<" "<<error<<endl;
    return error;
}


///////////////////////////////////////////////////////////////////////////////////////////// KALMAN FILTER //////////////////////////////////////////////////////
// Kalman Filter for 2D
void kalman(double acceleration[]){
    // Kalman Matrices
    double kalman_gain[2];
    double position_prediction_noise[2];
    double velocity_prediction_noise[2];
    double position_measurement_noise[2];
    

    // Variances
    double delta_p[2];
    double delta_v[2];
    double sigma_sq_a[2];
    double sigma_sq_z[2];
    double position_covariance[2];
    double velocity_covariance[2];
    double measurement_covariance[2];

    //Time step
    double time = 0.004;
    // For 2D

    for(int i=0;i<2;i++){

        // Noise - UNIFORM RANDOM DISTRIBUTION
        position_prediction_noise[i] = sample(0.05);
        velocity_prediction_noise[i] = sample(0.05);
        position_measurement_noise[i] = sample(0.05);

        // Math Model (Process)
        kalman_position[i] += kalman_velocity[i]*time + 0.5*acceleration[i]*time*time;
        kalman_velocity[i] += acceleration[i]*time;

        // Covariance means error
        sigma_sq_a[i] = sample(0.03);
        sigma_sq_z[i] = sample(0.03);
        delta_p[i] = 0.5*sigma_sq_a[i]*time*time;
        delta_v[i] = sigma_sq_a[i]*time;
        

        position_covariance[i] = delta_p[i] + position_prediction_noise[i];
        velocity_covariance[i] = delta_v[i] + velocity_prediction_noise[i];
        measurement_covariance[i] =  sigma_sq_z[i];

        // Measurements with noise (Comes from GPS)
        measurement[i] += position_measurement_noise[i];

        // Kalman Gain = error in process/(error in process + error in measurement)
        kalman_gain[i] = position_covariance[i]/(position_covariance[i] + measurement_covariance[i]);

        // Kalman Position
        kalman_position[i] = kalman_gain[i]*measurement[i] + (1 - kalman_gain[i])*kalman_position[i];

        // Updating covariance
        position_covariance[i] = kalman_gain[i]*measurement_covariance[i] + (1 - kalman_gain[i])*position_covariance[i];
        velocity_covariance[i] = (1 - kalman_gain[i])*velocity_covariance[i];
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// PSO //////////////////////////////////////////////////////

// Equation of cycloid for y
double curve_y(double position[]){
    double theta = atan(position[0]/position[1]);
    double y = -0.5*k_square*((1-cos(theta)) + mew*(theta + sin(theta)));
    return y;
}

// Equation of cycloid for x
double curve_x(double position[]){
    double theta = atan(position[0]/position[1]);
    double x = 0.5*k_square*((theta - sin(theta)) + mew*(1 - cos(theta)));
    return x;
}

// PSO loop for 2D
void PSO(double position[],double velocity[]){
    int i,j;
    int pso_iterations=100;
    double w = 0.01;
    double c = 0.02;
    double wdamp = 0.68;
    double best_position[2];
    double max_val = 10.0;
    double min_val = -10.0;
    // Initialize
    for(i=0;i<2;i++){
        pso_position[i] = position[i];
        pso_velocity[i] = velocity[i];
    }
    // Best Position of Particle will always on the cycloid
    best_position[0] = curve_y(pso_position);
    best_position[1] = curve_x(pso_position);

    // Main Loop
    for(i=0;i<pso_iterations;i++){
        for(j=0;j<2;j++){
            double r = 1 - sample(1.0);
            if(pso_position[i]>max_val)
                pso_position[i] = max_val;
            else if(pso_position[i] < min_val)
                pso_position[i] = min_val;
            if(pso_velocity[i]>max_val)
                pso_velocity[i] = max_val;
            else if(pso_velocity[i] < min_val)
                pso_velocity[i] = min_val;
            pso_velocity[j] = w*pso_velocity[j] + c*r*(best_position[j] - pso_position[j]);
            pso_position[j] = pso_position[j] + pso_velocity[j];
        }
        w *= wdamp;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double cost_function(double position[])
{
    double cost[2];
    double theta = atan(position[0]/position[1]);
    cost[0] = -0.5*k_square*((1-cos(theta)) + mew*(theta + sin(theta)));
    cost[1] = 0.5*k_square*((theta - sin(theta)) + mew*(1 - cos(theta)));
    double a = rand()%11 + 1;
    double b = rand()%11 + 1;
    double total_cost = (a*cost[0]*cost[0] + b*cost[1]*cost[1])/(a+b);
    return total_cost;
}

int main()
{
    // Problem Definition
    double phi1 = 17.54486;
    double phi2 = 17.492938;
    double lamda1 = 78.57185;
    double lamda2 = 78.537458;
    double initial_position[3];
    double final_position[3];
    double pso_cost;
    double kalman_cost;
    double position[2];
    double velocity[2];
    double acceleration[2];
    double error[2];
    initial_position[0] = 500.0;
    initial_position[1] = -distance(phi1,phi2,lamda1,lamda2);
    initial_position[2] = 0.0;
    final_position[0] = 0.0; // Altimeter Reading
    final_position[1] = 0.0;
    final_position[2] = bearing(phi1,phi2,lamda1,lamda2);

    acceleration[0] = -3.8 + 1.0*sample(0.1);
    acceleration[1] = 3.8 + 1.0*sample(0.1);
    for(int i=0;i<2;i++){
        velocity[i] = 10.0 + sample(0.1);
        position[i] = initial_position[i];
        pso_position[i] = initial_position[i];
        kalman_position[i] = initial_position[i];
    	measurement[i] = kalman_position[i];
    }
    ofstream output_file;
    output_file.open("out.csv");
    output_file<<"PSO_Cost,Kalman_Cost,Hybrid_Cost,Error_Cost,RMS_Cost,\n";
    cout<<endl;
    cout<<"READY!"<<endl;
    int t = 500;
    while(t--){
        cout<<"**************************Entered Loop**************************"<<endl;
        kalman(acceleration);
        PSO(position,velocity);
        kalman_cost = cost_function(kalman_position);
        cout<<"Kalman cost: "<<kalman_cost<<endl;
        pso_cost = cost_function(pso_position);
        cout<<"PSO cost: "<<pso_cost<<endl;
        for(int i=0;i<2;i++){
            position[i] = kalman_cost*pso_position[i] + pso_cost*kalman_position[i];
            position[i] /= (kalman_cost + pso_cost);
            velocity[i] = kalman_cost*pso_velocity[i] + pso_cost*kalman_velocity[i];
            velocity[i] /= (kalman_cost + pso_cost);
            error[i] = (position[i] - kalman_position[i]); //Kalman
            if(kalman_cost < pso_cost)
                pso_position[i] = kalman_position[i];
            else{
                acceleration[i] += error[i]*0.5*0.004;
                if(acceleration[i] > 100)
                    acceleration[i] = 100.0;
                else if(acceleration[i] < -100)
                    acceleration[i] = -100.0;
            }
        }
        if(kalman_position[0]<=0.01)
                acceleration[0] = 0.0;
        double hybrid_cost = cost_function(position); //PSO
        cout<<"Hybridized Cost: "<<hybrid_cost<<endl;
        cout<<"Errors: "<<error[0]<<" "<<error[1]<<endl;
        double rms_error = sqrt(hybrid_cost*hybrid_cost + error[0]*error[0] + error[1]*error[1]); //RMS
        cout<<"RMS Error: "<<rms_error<<endl;
        output_file<<pso_cost<<","<<kalman_cost<<","<<hybrid_cost<<","<<error[0]*error[0] + error[1]*error[1]<<","<<rms_error<<"\n";
    }
    output_file.close();
    return 0;
}
