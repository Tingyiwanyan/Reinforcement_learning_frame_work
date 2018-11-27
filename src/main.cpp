#include "Hello.h"
#include "mdp_state.h"
#include "simulate_grid2d.h"
#include "simulate_net.h"
#include "ssp.h"
#include "methods.h"
//#include <boost/filesystem.hpp>
#include <map>
#include <queue>
#include <ctime>
#include <cstdlib>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>

using namespace std;
using namespace mdp_planner;

int main(int argc, char *argv[])
{
  int num_rows = 20;
  int num_cols = 20;
  vector<double> vec1,vec2;
  double resolution = 1;
  //vector<transform2> tf2_starts, sf2_goals;
  point2d_t origin(-num_cols/2 + 0.5, -num_rows/2 + 0.5);
  point2d_t start(3.5,-3.5);
  point2d_t goal(-0.5,0.5);
  MDP_Grid2D::Ptr pGrid2d = MDP_Grid2D::Ptr(new MDP_Grid2D(num_cols, num_rows, resolution, origin));
  MDP_Net::Ptr pNet = MDP_Net::Ptr(new MDP_Net(pGrid2d));
  pNet->getState(start)->type = START;
  pNet->getState(goal)->type = GOAL;
  pNet->getState(goal)->optimal_value = 100;
  pNet->getState(goal)->optimal_action = ZERO;
  //std::vector<int> mdp_obstacle_ids;
  vector<mdp_state_t*> obstacles;
  pNet->mdp_obstacle_ids.push_back(1);
  pNet->mdp_obstacle_ids.push_back(3);
  pNet->mdp_obstacle_ids.push_back(4);
  pNet->mdp_obstacle_ids.push_back(6);
  pNet->mdp_obstacle_ids.push_back(30);
  pNet->mdp_obstacle_ids.push_back(29);
  pNet->mdp_obstacle_ids.push_back(40);
  pNet->mdp_obstacle_ids.push_back(45);
  pNet->mdp_obstacle_ids.push_back(46);
  pNet->mdp_obstacle_ids.push_back(10);
  pNet->mdp_obstacle_ids.push_back(50);
  pNet->mdp_obstacle_ids.push_back(53);
  pNet->mdp_obstacle_ids.push_back(35);

  for(int i=0;i<pNet->mdp_obstacle_ids.size();i++)
  {
    pNet->mdp_states[pNet->mdp_obstacle_ids[i]]->optimal_value = -100;
    pNet->mdp_states[pNet->mdp_obstacle_ids[i]]->type = OBSTACLE;
    pNet->mdp_states[pNet->mdp_obstacle_ids[i]]->optimal_action = ZERO;
  }

  std::cout << "\nmap indices" << std::endl;
    for (int i = pGrid2d -> n_rows - 1; i >= 0; i--) {
        for (int j = 0; j < pGrid2d -> n_cols; j++) {
            std::cout << i * pGrid2d -> n_cols + j << " ";
        }
        std::cout << std::endl;
    }
    std::cout<<pGrid2d->cell_centers.size()<<std::endl;
    for(int j=0;j<10;j++){
    double error_one_trail;
    for(int i=0;i<pNet->mdp_states.size();i++){

      if(pNet->mdp_states[i]->type != OBSTACLE)
      {
        error_one_trail = mdp_planner::Trail_update_(pNet->mdp_states[i],pNet);
        //error_one_trail = mdp_planner::Trail_update_(start,pParams,pNet);
        //if(error<error_one_trail)
        //{
        //  error = error_one_trail;
        //}
      }
      //std::cout<<"error is"<<error<<std::endl;
    }
    std::cout<<"error is"<<error_one_trail<<std::endl;
  }
  std::cout << "\nmap indices" << std::endl;
  for (int i = pGrid2d -> n_rows - 1; i >= 0; i--) {
      for (int j = 0; j < pGrid2d -> n_cols; j++) {
          //std::cout << i * pGrid2d -> n_cols + j << " ";
          std::cout<< (int)pNet->mdp_states[i*pGrid2d->n_cols+j]->optimal_value<<" ";
          //my_file2<<pNet->mdp_states[i*pGrid2d->n_cols+j]->optimal_value<<",";
      }
      std::cout << std::endl;
  }
  mdp_planner::uniform_transition_initialization(pNet);
  std::cout<<pNet->mdp_states[2]->probs[1]<<std::endl;
  SSP *pSSP = new SSP(pNet);

  //pSSP->initTransMatrix(pNet);
  std::cout<<"goals "<<pNet->getState(goal)->id<<std::endl;
  pSSP->MFPT(pNet, pNet->getState(goal)->id);
  std::cout << "reachabilities" << std::endl;
  //ofstream my_file;
  //my_file.open("reachability.txt");

  for (int i = pNet -> n_rows - 1; i >= 0; i--) {
      for (int j = 0; j < pNet -> n_cols; j++) {
          //std::cout << i * pGrid2d -> n_cols + j << " ";
          std::cout<< (int)pSSP->reachability[i * pNet -> n_cols + j]  << " ";
          //my_file<<pSSP->reachability[i * pNet -> n_cols + j]<<",";
      }
      std::cout << std::endl;
  }
  /*
  std::cout<<"transition probability"<<std::endl;
  for(int i = 0;i<pNet->n_rows*pNet->n_cols;i++)
  {
    for(int j = 0;j<pNet->n_rows*pNet->n_cols;j++)
    {
      std::cout<<pSSP->transMatrix.coeffRef(i, j)<<" ";
    }
    std::cout<<std::endl;
  }
  */

//  my_file.close();
    //mdp_planner::Trail_Based_RTDP(pNet);
  return 0;
}
