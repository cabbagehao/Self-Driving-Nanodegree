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
#include <math.h>

#include "particle_filter.h"

using namespace std;

std::default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) 
{
    num_particles = 100;
    std::vector<int> associations;
    std::vector<double> sense_x;
    std::vector<double> sense_y;   
    

    std::normal_distribution<double> dist_x (0, std[0]); 
    std::normal_distribution<double> dist_y (0, std[1]); 
    std::normal_distribution<double> dist_theta (0, std[2]); 
    // Init particles.
    for(int i=0; i<num_particles; i++)
    {

        float x_gauss = x + dist_x(gen);
        float y_gauss = y + dist_y(gen);
        float theta_gauss = theta + dist_theta(gen);

        weights.push_back(1);
        Particle particle_init = {i, x_gauss, y_gauss, theta_gauss, 1, associations, sense_x, sense_y};
        particles.push_back(particle_init);
    }

    is_initialized = true;
}



void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) 
{
    const double EPS = 0.001;
    normal_distribution<double> dist_x(0, std_pos[0]);
    normal_distribution<double> dist_y(0, std_pos[1]);
    normal_distribution<double> dist_phi(0, std_pos[2]);    
    // linear model.
    if(fabs(yaw_rate) < EPS)
    {
        for(int i = 0; i < num_particles; i++)
        {
            double theta = particles[i].theta;
            particles[i].x += velocity*delta_t*cos(theta) + dist_x(gen);
            particles[i].y += velocity*delta_t*sin(theta) + dist_y(gen);
            particles[i].theta += dist_phi(gen);
        }
    }
    // bycicle model
    else
    {
        for(int i=0; i< num_particles; i++)
        {
            double theta = particles[i].theta;
            particles[i].x += velocity/yaw_rate * (sin(theta + yaw_rate*delta_t) - sin(theta)) + dist_x(gen);
            particles[i].y += velocity/yaw_rate * (cos(theta) - cos(theta + yaw_rate*delta_t)) + dist_y(gen);
            particles[i].theta += yaw_rate*delta_t + dist_phi(gen);
        }      
    }
}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) 
{
    /*
        Used the nearest Neighborhood.  
        predicted是某个particle在range范围内所有的landmark。
    */
    for(int i=0; i<observations.size(); i++)
    {
        int min_distance_id = 0;
        double min_distance = std::numeric_limits<double>::max();      
        for(int j=0; j<predicted.size(); j++)
        {
            double distance =  dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
            if(min_distance > distance)
            {
                min_distance = distance;
                min_distance_id = predicted[j].id;
            }
        }

        observations[i].id = min_distance_id;
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
     std::vector<LandmarkObs> observations, Map map_landmarks) 
{
    /*  
        update all particles' weight, preparing to resample.
    */
    float theta = M_PI / 2;
    for(int i=0; i < num_particles; i++)
    {  
        // Find all the landmarks that within the detect range.
        std::vector<LandmarkObs> predicteds;        
        LandmarkObs predicted;
        for(int j=0; j < map_landmarks.landmark_list.size(); j++)
        {
            double lx = map_landmarks.landmark_list[j].x_f;
            double ly = map_landmarks.landmark_list[j].y_f;

            predicted.id = map_landmarks.landmark_list[j].id_i;
            predicted.x = lx;
            predicted.y = ly;
            double distance = dist(particles[i].x, particles[i].y, lx, ly);
            if(distance < sensor_range)
            {
                predicteds.push_back(predicted); 
            }    
        }
       
        /*  observations 是car 能 观察到的所有landmark结果。
            obs_landmarks是将观察的结果应用到这个particle上，并转换为map坐标系。
                每个particle不需要再观察（测量），直接使用car的观察结果。
                假设这个结果就是自己观察到的，然后去计算和landmark实际结果的偏差。
                如果particle和car近，这样变换出来偏差并不大，如果隔得远，则偏差很大，权重变小。
            particle的方向theta就代表了车辆坐标系与map坐标系的夹角。因此转换为map坐标系时相当于向右旋转theta，所以为x' = x*cosθ-y*sinθ的形式。
        */
        std::vector<LandmarkObs> obs_landmarks;
        for(int t=0; t< observations.size(); t++)
        {
            double obs_x = observations[t].x;
            double obs_y = observations[t].y;
            double trans_x = particles[i].x + (obs_x*cos(particles[i].theta)-obs_y*sin(particles[i].theta));
            double trans_y = particles[i].y + (obs_x*sin(particles[i].theta)+obs_y*cos(particles[i].theta));
            
            LandmarkObs obs_landmark;
            obs_landmark.x = trans_x;
            obs_landmark.y = trans_y;
            obs_landmarks.push_back(obs_landmark);
        }
     
        // predicted是这个particle能观察到的所有landmark。
        // obs_landmarks是这个particle观察到的landmark的map坐标。 
        // 对这些坐标与观察到的landmark进行绑定。 可能一个landmark被多次观察（测量）到。
        dataAssociation(predicteds , obs_landmarks);

    
        // If all observation beyond the sensor_range, do not update weights.
        if(observations.size() == 0 )  return;

        // 更新每个particle的weight
        double sig_x = std_landmark[0];
        double sig_y = std_landmark[1];
        double gauss_norm= (1/(2 * M_PI * sig_x * sig_y));
        double weight_tmp = 1.0;
        double mu_x;
        double mu_y; 

        // 对该particle的每一个观察值（测量值）进行遍历
        for(int t=0; t < obs_landmarks.size(); t++)
        {
            // // If no landmark matched, set weight = 0
            // if(particles[i].associations.size() == 0)
            // {
            //     std::cout << "particles[" << i << "].associations.size() == 0" << endl;
            //     particles[i].weight = 0;
            //     weights[i] = 0;
            //     continue;
            // }
            // 找到这个测量值对应的哪一个landmark
            for(int k=0; k<map_landmarks.landmark_list.size(); k++)
            {
                if(obs_landmarks[t].id == map_landmarks.landmark_list[k].id_i)
                {
                    mu_x = map_landmarks.landmark_list[k].x_f;
                    mu_y = map_landmarks.landmark_list[k].y_f;
                    break;
                }
            }
            double x_obs = obs_landmarks[t].x;
            double y_obs = obs_landmarks[t].y;   

            //计算这个观测值与该landmark实际结果的偏差，并根据偏差累乘概率。
            double exponent= ( pow((x_obs - mu_x),2) ) / (2 * sig_x*sig_x) + ( pow((y_obs - mu_y),2) ) / (2 * sig_y*sig_y);
            weight_tmp *= gauss_norm * exp(-exponent);
        }
        // 当计算完这个particle所有观测值的概率累乘后，更新它的权重
        // 所有particle的权重可能都非常非常小，但这不影响，后面会normalize.
        weights[i] = weight_tmp ; 
        particles[i].weight = weight_tmp;   // did not use.
   
    }

    // normalize. 是全概率为1.
    double sum_weight = 0.0;

    for(int k = 0; k < weights.size(); k++)
    {
        sum_weight += weights[k];
    }
    for(int k = 0; k < weights.size(); k++)
    {
        weights[k] /= sum_weight;
        // particles[k].weight /= sum_weight;
    }
}


/*
    resample采用了一个轮盘算法，所有概率排成一个轮盘（列表）
    取所有概率的最大值max(w)，每次让指针转动2*max(w)*random(0,1)
    如果weights[index] < beta(指针的值),则把index加1，beta减去weights[index]，while这个过程直到weights[index] >= beta
    相当于指针拨动后，停在某个index对应的概率那个扇形，就把指针没有停的那些扇形代表的概率全都减掉，取到那个index的particle。
    反正就是把所有概率放在一个轮盘（全概率肯定为1，也就是轮盘正好是个圆），拨动指针，指针停在哪里就取它。（其实和累加后取随机数方法差不多）
*/

void ParticleFilter::resample() 
{

    double max_w = *max_element(begin(weights), end(weights));;
    double beta = 0;

    std::default_random_engine random(time(NULL));
    std::uniform_int_distribution<int> index_rand(0, num_particles);
    std::uniform_real_distribution<double> p_rand(0, 1);

    int index = index_rand(random);  
    std::vector<Particle> particles_tmp;
    for(int i=0; i < num_particles; i++)
    {
        beta += p_rand(random) * 2 * (max_w);
        while(weights[index] < beta)
        {
            //cout << beta << "   " << weights[index] << " " << index << endl;
            beta -= weights[index];
            index = (1+index) % weights.size();
        }
        particles_tmp.push_back(particles[index]);
    }
    particles = particles_tmp;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    // d
    //Clear the previous associations
    particle.associations.clear();
    particle.sense_x.clear();
    particle.sense_y.clear();

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;

    return particle;
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

void ParticleFilter::pack_obsevations(vector<LandmarkObs>& noisy_observations, string sense_x, string sense_y) 
{
    /*
        package sensex and sense_y to noisy_observations.
    */
    std::vector<float> x_sense;
    std::istringstream iss_x(sense_x);

    std::copy(std::istream_iterator<float>(iss_x),
    std::istream_iterator<float>(),
    std::back_inserter(x_sense));

    std::vector<float> y_sense;
    std::istringstream iss_y(sense_y);

    std::copy(std::istream_iterator<float>(iss_y),
    std::istream_iterator<float>(),
    std::back_inserter(y_sense));

    for(int i = 0; i < x_sense.size(); i++)
    {
        LandmarkObs obs;
        obs.x = x_sense[i];
        obs.y = y_sense[i];
        noisy_observations.push_back(obs);
    }
}