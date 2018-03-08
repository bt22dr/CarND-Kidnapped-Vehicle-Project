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
  
  std::cout << "initial GPS: " << x << ", " << y << ", " << theta << std::endl;
  for (int i = 0; i < num_particles; i++) {
    particles[i].id = i;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].weight = 1.0;
    
    std::cout << "initial particle: " << particles[i].x << ", " << particles[i].y << ", " << particles[i].theta << std::endl;
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
  std::cout << "control: " << velocity << ", " << yaw_rate << std::endl;
  for (int i = 0; i < num_particles; i++) {
    curr_x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
    curr_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
    curr_theta = particles[i].theta + yaw_rate * delta_t;
    
    std::cout << "predict: " << curr_x << ", " << curr_y << ", " << curr_theta << std::endl;
    
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
  /*
  double min_d = numeric_limits<double>::max();
  for (unsigned int i = 0; i < predicted.size(); i++) {
    double d = dist(xm, ym, landmarkx, landmarky);
    if (d < min_d) {
      min_d = d;
      association_id = map_landmarks.landmark_list[k].id_i;
    }
  }
  */
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
  
  for (unsigned int i = 0; i < observations.size(); i++) {
    std::cout << "OBSERVATION " << observations[i].id << ":" << observations[i].x << ", " << observations[i].y << std::endl;
  }
  
  for (int i = 0; i < num_particles; i++) {
    int particle_id = particles[i].id;
    for (unsigned int j = 0; j < observations.size(); j++) {
      // 1. Transform
      double xp = particles[i].x;
      double yp = particles[i].y;
      double theta = particles[i].theta;
      double xc = observations[j].x;
      double yc = observations[j].y;
      
      double xm = cos(theta) * xc - sin(theta) * yc + xp;
      double ym = sin(theta) * xc + cos(theta) * yc + yp;
      
      std::vector<LandmarkObs> predicted;
      for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); k++) {
        
      }
      
      particles[i].sense_x.push_back(xm);
      particles[i].sense_y.push_back(ym);
      
      /*
      // 2. Associate
      int observation_id = observations[j].id;
      int association_id;
      double min_d = numeric_limits<double>::max();
      for (unsigned int k = 0; k < map_landmarks.landmark_list.size(); k++) {
        double landmarkx = map_landmarks.landmark_list[k].x_f;
        double landmarky = map_landmarks.landmark_list[k].y_f;
        double d = dist(xm, ym, landmarkx, landmarky);
        if (d > sensor_range)
          continue;
        if (d < min_d) {
          min_d = d;
          association_id = map_landmarks.landmark_list[k].id_i;
        }
      }
      */
      // 3. Update Weights
      // double gauss_norm = (1.0 / (2 * math.M_PI * std_landmark[0] * std_landmark[1]));
      // double exponent =
      // particles[i].sense_x.push_back(xm);
      // particles[i].sense_y.push_back(ym);
      // particles[i].associations.push_back(observations[j].id);
    }
  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

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
