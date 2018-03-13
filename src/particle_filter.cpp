/*
 * particle_filter.cpp
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

void ParticleFilter::setNumParticles(unsigned int num) {
  num_particles = num;
  particles.resize(num_particles);
  weights.resize(num_particles);
}

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  default_random_engine gen;
  
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);
  
  //std::cout << "initial GPS: " << x << ", " << y << ", " << theta << std::endl;
  for (int i = 0; i < num_particles; i++) {
    particles[i].id = i;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].weight = 1.0;
  }
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  double curr_x = 0.0;
  double curr_y = 0.0;
  double curr_theta = 0.0;
  
  default_random_engine gen;
  //std::cout << "control: " << velocity << ", " << yaw_rate << std::endl;
  for (int i = 0; i < num_particles; i++) {
    curr_x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
    curr_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
    curr_theta = particles[i].theta + yaw_rate * delta_t;
    
    //std::cout << "predict: " << curr_x << ", " << curr_y << ", " << curr_theta << std::endl;
    
    normal_distribution<double> dist_x(curr_x, std_pos[0]);
    normal_distribution<double> dist_y(curr_y, std_pos[1]);
    normal_distribution<double> dist_theta(curr_theta, std_pos[2]);
    
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  for (unsigned int i = 0; i < observations.size(); i++) {
    double min_d = numeric_limits<double>::max();
    for (unsigned int j = 0; j < predicted.size(); j++) {
      double d = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
      if (d < min_d) {
        min_d = d;
        observations[i].id = predicted[j].id;
      }
    }
  }
}

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
  for (int i = 0; i < num_particles; i++) {
    // extract landmarks within sensor_range
    //std::cout << "======" << "PARTICLE " << i << " ======" << std::endl;
    //std::cout << "particle location " << particles[i].x << ", " << particles[i].y << std::endl;
    std::vector<LandmarkObs> landmarks;
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      double xp = particles[i].x;
      double yp = particles[i].y;
      double lmx = map_landmarks.landmark_list[j].x_f;
      double lmy = map_landmarks.landmark_list[j].y_f;
      int lmid = map_landmarks.landmark_list[j].id_i;
      
      if (dist(lmx, lmy, xp, yp) < sensor_range) {
        LandmarkObs obj;
        obj.x = lmx;
        obj.y = lmy;
        obj.id = lmid;
        
        landmarks.push_back(obj);
        //cout << "LANDMARKS (range): " << lmx << ", " << lmy << ", " << lmid << std::endl;
      }
    }
    
    // 1. Transform
    std::vector<LandmarkObs> transformed_observation;
    for (unsigned int j = 0; j < observations.size(); j++) {
      double xp = particles[i].x;
      double yp = particles[i].y;
      double theta = particles[i].theta;
      double xc = observations[j].x;
      double yc = observations[j].y;
      
      // transformed observation
      double xm = cos(theta) * xc - sin(theta) * yc + xp; // position x in global coordinate
      double ym = sin(theta) * xc + cos(theta) * yc + yp; // position y in global coordinate
      
      LandmarkObs obj;
      obj.x = xm;
      obj.y = ym;
      transformed_observation.push_back(obj);
    }
    
    // 2. Associate
    dataAssociation(landmarks, transformed_observation);
    
    // 3. Update Weights
    particles[i].weight = 1.0;
    for (unsigned int j = 0; j < transformed_observation.size(); j++) {
      double x_obs = transformed_observation[j].x;
      double y_obs = transformed_observation[j].y;
      double mu_x = map_landmarks.landmark_list[transformed_observation[j].id-1].x_f;
      double mu_y = map_landmarks.landmark_list[transformed_observation[j].id-1].y_f;
      double gauss_norm = (1.0 / (2 * M_PI * (std_landmark[0] * std_landmark[1])));
      double exponent = ((x_obs - mu_x)*(x_obs - mu_x))/(2 * (std_landmark[0]*std_landmark[0]))
                      + ((y_obs - mu_y)*(y_obs - mu_y))/(2 * (std_landmark[1]*std_landmark[1]));
      double weight = gauss_norm * exp(-exponent);
      particles[i].weight *= weight;
    }
    weights[i] = particles[i].weight;
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  std::vector<Particle> new_particles;
  double beta = 0.0;
  int index = rand() % num_particles;
  for (int i = 0; i < num_particles; i++) {
    beta = beta + (2.0 * ((double) rand() / (RAND_MAX)) * *std::max_element(weights.begin(), weights.end()));
    while (weights[index] < beta) {
      beta = beta - weights[index];
      index = (index + 1) % num_particles;
    }
    new_particles.push_back(particles[index]);
  }
  particles.swap(new_particles);
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
