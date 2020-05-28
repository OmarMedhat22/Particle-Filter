/* particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
num_particles =  1000;
std::default_random_engine gen;

std::normal_distribution<double> N_x(x,std[0]);
std::normal_distribution<double> N_y(y,std[1]);
std::normal_distribution<double> N_theta(theta,std[2]);

for(int i=0;i<num_particles;i++){

Particle particle;
particle.id = i;
particle.x = N_x(gen);
particle.y = N_y(gen);
particle.theta = N_theta(gen);
particle.weight = 1;
particles.push_back(particle);
weights.push_back(1);
}
is_initialized = true;


}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	default_random_engine gen;

	for(int i=0;i<num_particles;i++){

double new_x;
double new_y;
double new_theta;
if(yaw_rate == 0)
{

new_x = particles[i].x + velocity*delta_t*cos(particles[i].theta);
new_y = particles[i].y + velocity*delta_t*sin(particles[i].theta);
new_theta = particles[i].theta;
}
else
{
new_x = particles[i].x +velocity/yaw_rate*(sin(particles[i].theta + yaw_rate*delta_t) - sin(particles[i].theta) );
new_y = particles[i].y +velocity/yaw_rate*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate*delta_t) );
new_theta = particles[i].theta + yaw_rate*delta_t;
}

std::normal_distribution<double> N_x(new_x,std_pos[0]);
std::normal_distribution<double> N_y(new_y,std_pos[1]);
std::normal_distribution<double> N_theta(new_theta,std_pos[2]);

particles[i].x = N_x(gen);
particles[i].y = N_y(gen);
particles[i].theta = N_theta(gen);


} 

}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.



}
	std::vector<double> new_weight;

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
	//for(int i = 0;i<new_weight.size();i++){

	//	new_weight[0]

	//}

	for (int i = 0; i < particles.size(); i++) {

std::vector<LandmarkObs> observations_transf;
double weight_in_particle = 1;
double single_weight;
for (int j=0; j<observations.size();j++){
LandmarkObs obs_t;
obs_t.id = i;
obs_t.x = particles[i].x + cos(particles[i].theta)*observations[j].x - sin(particles[i].theta)*observations[j].y  ;
obs_t.y = particles[i].y + sin(particles[i].theta)*observations[j].x + cos(particles[i].theta)*observations[j].y  ;
observations_transf.push_back(obs_t);


vector <double> dists;
double min_dist = 1000000;
int min_map;
for (int z=0; z<map_landmarks.landmark_list.size(); z++) {                 
                    double dst = dist(map_landmarks.landmark_list[z].x_f,map_landmarks.landmark_list[z].y_f, obs_t.x, obs_t.y);
                    // dists.push_back(dst);
                    if(dst<min_dist){

                    	min_dist = dst;
                    	min_map = z ;
                    }


}
// vector<double>::iterator result = min_element(begin(dists), end(dists));
//Map::single_landmark_s lm = map_landmarks.landmark_list[distance(begin(dists), result)];
//obs_t.id = lm.id_i;

double x_diffrence = pow((observations_transf[j].x - map_landmarks.landmark_list[min_map].x_f),2);
double y_diffrence = pow((observations_transf[j].y - map_landmarks.landmark_list[min_map].y_f),2);
double standard_devition_x = std_landmark[0]*std_landmark[0]*2;
double standard_devition_y = std_landmark[1]*std_landmark[1]*2;
double normal_std = (1/(std_landmark[0]*std_landmark[1]*(22/7)*2));

//single_weight =( 1/(exp(((pow((observations_transf[j].x - map_landmarks.landmark_list[min_map].x_f),2))/(std_landmark[0]*std_landmark[0]*2))+
	//((pow((observations_transf[j].y - map_landmarks.landmark_list[min_map].y_f),2))/(std_landmark[1]*std_landmark[1]*2)))))
	// * (1/(std_landmark[0]*std_landmark[1]*(22/7)*2));
	// if(single_weight !=0)
single_weight = ( 1/exp(x_diffrence/standard_devition_x) ) * ( 1/exp(y_diffrence/standard_devition_y) ) * normal_std ;
if(single_weight !=0){
	weight_in_particle = single_weight * weight_in_particle;

}

cout << " single weights 1   = "<< single_weight;
//cout << " single weights 1   = "<< x_diffrence;
//cout << " single weights 2   = "<< y_diffrence;
//cout << " single weights 3   = "<< standard_devition_x;
//cout << " single weights 4   = "<< standard_devition_y;
//cout << " single weights 5   = "<< normal_std;
 
 
}
particles[i].weight = weight_in_particle;
weights[i] = weight_in_particle;
cout << "weights in the  = "<< weights[i] ;

}

	//for (int i = 0; i < particles.size(); i++) {

	//}

}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

		default_random_engine gen;
		discrete_distribution<int> distribution(weights.begin(),weights.end());
vector<Particle> resample_particle;

for(int i=0; i< num_particles;i++){

resample_particle.push_back(particles[distribution(gen)]);
}
particles = resample_particle;


}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
