#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <vector>
#include "classifier.h"

double gaussians_calc(double x, double mu, double sigma)
{
    return exp(-((x - mu)*(x - mu))/(2*sigma*sigma)) / (sigma * sqrt(2*M_PI));
}

int find_pos(vector<string> array, string x)
{
    for(int i=0; i<array.size(); i++)
    {
        if(array[i] == x)
        {
            return i;
        }
    }
    return -1;
}


/**
 * Initializes GNB
 */
GNB::GNB() {
    N_features = 4;
    N_labels = 3;
}

GNB::~GNB() {}

void GNB::train(vector<vector<double> > data, vector<string> labels)
{
    // this-> possible_labels =  array_uniq(labels)
    this-> N_features = data[0].size();
    this-> N_labels = this-> possible_labels.size();
    N_labels = this-> N_labels;
    N_features = this-> N_features;
    double label_sum [N_labels][N_features];
    vector<int> count_label(3, 0);
    // init.
    for(int i=0; i < N_labels; i++)
    {
        for(int j=0; j < N_features; j++)
        {
            label_sum[i][j] = 0;
            this->std[i][j] = 0;
        }
    }

    // calc sum first.
    for(int i=0; i < data.size(); i++)
    {
        int pos = find_pos(this-> possible_labels, labels[i]);
        count_label[pos] += 1;
        for(int j=0; j<N_features; j++)
        {
            
            label_sum[pos][j] += data[i][j];
        }
    }   
    // calc mean
    for(int i=0; i<N_labels; i++)
    {
        for(int j=0; j<N_features;  j++)
        {
            this->mean[i][j] = (double)label_sum[i][j] / data.size();
            //cout << "mean: " << mean[i][j] << endl;
        }
    }
    // calc std
    for(int i=0; i < data.size(); i++)
    {
        int pos = find_pos(this-> possible_labels, labels[i]);
        double delta;
        for(int j=0; j<N_features; j++)
        {
            delta = (data[i][j] - this->mean[pos][j]);
            this->std[pos][j] +=  delta * delta;
        }

    }   
    for(int i=0; i<N_labels; i++)
    {
        for(int j=0; j<N_features; j++)
        {
            double tmp = this->std[i][j];
            this->std[i][j] = sqrt(tmp / count_label[i]);
        }
    }

}

string GNB::predict(vector<double> sample)
{
    /*
        Once trained, this method is called and expected to return 
        a predicted behavior for the given observation.

        INPUTS

        observation - a 4 tuple with s, d, s_dot, d_dot.
          - Example: [3.5, 0.1, 8.5, -0.2]

        OUTPUT

        A label representing the best guess of the classifier. Can
        be one of "left", "keep" or "right".
        """
        # TODO - complete this
    */
    double max_possibility = 0;
    double possibility;
    int max_possibility_pos;
    for(int i=0; i< this->N_labels; i++)
    {
        possibility = 0;
        for(int j=0; j < this-> N_features; j++)
        {
            possibility += gaussians_calc(sample[j], mean[i][j], std[i][j]);
            
        }
        //cout << possibility << endl;
        if(possibility > max_possibility)
        {
            max_possibility = possibility;
            max_possibility_pos = i;
        }
    }

    //cout << this->possible_labels[max_possibility_pos] << endl;
    return this->possible_labels[max_possibility_pos];

}

