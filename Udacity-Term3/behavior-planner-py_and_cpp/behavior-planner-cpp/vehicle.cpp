#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    state = "CS";
    max_acceleration = -1;
    time(&this->time_stamp);
}

Vehicle::~Vehicle() {}

// TODO - Implement this method.
void Vehicle::update_state(map<int,vector < vector<int> > > predictions) {
	/*
    Updates the "state" of the vehicle by assigning one of the
    following values to 'self.state':

    "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is 
       traffic in front of it, in which case it will slow down.

    "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
       behavior for the "KL" state in the new lane.

    "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
       BEHIND itself and will adjust speed to try to get behind that vehicle.

    INPUTS
    - predictions 
    A dictionary. The keys are ids of other vehicles and the values are arrays
    where each entry corresponds to the vehicle's predicted location at the 
    corresponding timestep. The FIRST element in the array gives the vehicle's
    current position. Example (showing a car with id 3 moving at 2 m/s):

    {
      3 : [
        {"s" : 4, "lane": 0},
        {"s" : 6, "lane": 0},
        {"s" : 8, "lane": 0},
        {"s" : 10, "lane": 0},
      ]
    }

    */
    
    int best_lane = _find_best_lane_for_now(predictions);
    cout <<"lane test: " << this->goal_lane << ' ' << this->lane << " " << best_lane<<endl;
    if(best_lane == this->lane)
      this->state = "KL";
    else if( best_lane < this->lane)
    {
      if( this->state != "PLCR")
        this->state = "PLCR";
      else
      {
        map<int, vector<vector<int> > >::iterator it = predictions.begin();
        vector<vector<vector<int> > > at_behind;
        while(it != predictions.end())
        {
        	int v_id = it->first;
            vector<vector<int> > v = it->second;
    
            if((v[0][0] == best_lane) && (v[0][1] == this->s))
            {
            	at_behind.push_back(v);
    
            }
            it++;
        }
        if(at_behind.size() == 0)
          this->state = "LCR";
      }
    }
    else
    {
      if( this->state != "PLCL")
        this->state = "PLCL";
      else
      {
        map<int, vector<vector<int> > >::iterator it = predictions.begin();
        vector<vector<vector<int> > > at_behind;
        while(it != predictions.end())
        {
        	int v_id = it->first;
            vector<vector<int> > v = it->second;
    
            if((v[0][0] == best_lane) && (v[0][1] == this->s))
            {
            	at_behind.push_back(v);
    
            }
            it++;
        }
        if(at_behind.size() == 0)
          this->state = "LCL";
      }
    }


}

void Vehicle::configure(vector<int> road_data) {
	/*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}

string Vehicle::display() {

	ostringstream oss;
	
	oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    
    return oss.str();
}

void Vehicle::increment(int dt = 1) {

	this->s += this->v * dt;
    this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t) {

	/*
    Predicts state of vehicle in t seconds (assuming constant acceleration)
    */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}

bool Vehicle::collides_with(Vehicle other, int at_time) {

	/*
    Simple collision detection.
    */
    vector<int> check1 = state_at(at_time);
    vector<int> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {

	Vehicle::collider collider_temp;
	collider_temp.collision = false;
	collider_temp.time = -1; 

	for (int t = 0; t < timesteps+1; t++)
	{
      	if( collides_with(other, t) )
      	{
			collider_temp.collision = true;
			collider_temp.time = t; 
        	return collider_temp;
    	}
	}

	return collider_temp;
}

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {
   
	/*
    Given a state, realize it by adjusting acceleration and lane.
    Note - lane changes happen instantaneously.
    */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
    	realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
    	realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
    	realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
    	realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
    	realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
    	realize_prep_lane_change(predictions, "R");
    }

}

void Vehicle::realize_constant_speed() {
	a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {

	int delta_v_til_target = target_speed - v;
    int max_acc = min(max_acceleration, delta_v_til_target);

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while(it != predictions.end())
    {
       
    	int v_id = it->first;
    	
        vector<vector<int> > v = it->second;
        
        if((v[0][0] == lane) && (v[0][1] > s))
        {
        	in_front.push_back(v);

        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
    	int min_s = 1000;
    	vector<vector<int>> leading = {};
    	for(int i = 0; i < in_front.size(); i++)
    	{
    		if((in_front[i][0][1]-s) < min_s)
    		{
    			min_s = (in_front[i][0][1]-s);
    			leading = in_front[i];
    		}
    	}
    	
    	int next_pos = leading[1][1];
    	int my_next = s + this->v;
    	int separation_next = next_pos - my_next;
    	int available_room = separation_next - preferred_buffer;
    	max_acc = min(max_acc, available_room);
    }
    
    return max_acc;

}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
	this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction) {
	int delta = -1;
    if (direction.compare("L") == 0)
    {
    	delta = 1;
    }
    int lane = this->lane + delta;

    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > at_behind;
    while(it != predictions.end())
    {
    	int v_id = it->first;
        vector<vector<int> > v = it->second;

        if((v[0][0] == lane) && (v[0][1] <= this->s))
        {
        	at_behind.push_back(v);

        }
        it++;
    }
    if(at_behind.size() > 0)
    {

    	int max_s = -1000;
    	vector<vector<int> > nearest_behind = {};
    	for(int i = 0; i < at_behind.size(); i++)
    	{
    		if((at_behind[i][0][1]) > max_s)
    		{
    			max_s = at_behind[i][0][1];
    			nearest_behind = at_behind[i];
    		}
    	}
    	int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
    	int delta_v = this->v - target_vel;
    	int delta_s = this->s - nearest_behind[0][1];
    	if(delta_v != 0)
    	{

    		int time = -2 * delta_s/delta_v;
    		int a;
    		if (time == 0)
    		{
    			a = this->a;
    		}
    		else
    		{
    			a = delta_v/time;
    		}
    		if(a > this->max_acceleration)
    		{
    			a = this->max_acceleration;
    		}
    		if(a < -this->max_acceleration)
    		{
    			a = -this->max_acceleration;
    		}
    		this->a = a;
    	}
    	else
    	{
    		int my_min_acc = max(-this->max_acceleration,-delta_s);
    		this->a = my_min_acc;
    	}

    }

}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10) {

	vector<vector<int> > predictions;
    for( int i = 0; i < horizon; i++)
    {
      vector<int> check1 = state_at(i);
      vector<int> lane_s = {check1[0], check1[1]};
      predictions.push_back(lane_s);
  	}
    return predictions;

}

int Vehicle::_find_best_lane_for_now(map<int, vector< vector<int> > > predictions)
{
    time_t now;
    double past_time = difftime(time(&now), this->time_stamp);
    double left_time = 20 - past_time;
    vector<int> priority_lanes;
    vector<int> lanes;
    
    lanes.push_back(this->goal_lane);
    if(this->goal_lane != this->lane)
        lanes.push_back(this->lane);
    
    vector<int> all_lane;
    all_lane.push_back(0);
    all_lane.push_back(1);
    all_lane.push_back(2);
    all_lane.push_back(3);
    for(int i=0; i < all_lane.size(); i++)
    {
        if(all_lane[i] !=this->lane and all_lane[i] !=this->goal_lane)
            lanes.push_back(all_lane[i]);
    }
    
    int best_lane = this->lane;  
    for(int i=0; i< lanes.size(); i++)
    {
        map<int, vector<vector<int> > >::iterator it = predictions.begin();
        vector<vector<vector<int> > > in_front;
        while(it != predictions.end())
        {
        	int v_id = it->first;
            vector<vector<int> > v = it->second;
    
            if((v[0][0] == this->goal_lane) && (v[0][1] >= this->s))
            {
            	in_front.push_back(v);
            }
            it++;
        }
        if(in_front.size() == 0)
        {
            best_lane = lanes[i];
            break;
        }
        else
        {
            // Find the closest car. calc it's v.
            double min_s = 10000;
        	vector<vector<int> > nearest_front = {};
        	for(int i = 0; i < in_front.size(); i++)
        	{
        		if((in_front[i][0][1]) < min_s)
        		{
        			min_s = in_front[i][0][1];
        			nearest_front = in_front[i];
        		}
        	}
        	int target_vel = nearest_front[1][1] - nearest_front[0][1];
            if(left_time * target_vel >= this->goal_s - this->s)
            {
              best_lane = lanes[i];
              break;
            }
            if(i == lanes.size()) 
                cout << "No lane can finish.";
        }
    }
    return best_lane;
}
