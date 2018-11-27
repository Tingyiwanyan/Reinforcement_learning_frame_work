#include "ssp.h"
#include "methods.h"
#include "mdp_state.h"
#include "geometry_utils/Vector2.h"
#include <map>
#include <queue>
#include<ctime>
#include<cstdlib>
#include<math.h>
#include<vector>
#include<iostream>
#include<fstream>

using namespace geometry_utils;
using namespace mdp_planner;

double gaussian_pdf(double x, double m, double s) {
  static const double inv_sqrt_2pi = 0.3989422804014327;
  double a = (x - m) / s;

  return inv_sqrt_2pi / s * std::exp(-0.5 * a * a);
}

void mdp_planner::getTransitionModel(mdp_state_t* s, action_t act,MDP_Net::Ptr& pNet)
{
  //assert(s->successors[act]!=s);

  double v_max = 1;//pParams->getParamNode()["vehicle_model"]["v_max"].as<double>();
  double sigma = 0.3;//pParams->getParamNode()["vehicle_model"]["gaussian_sigma"].as<double>();
  //double sigma_d = pParams->getParamNode()["disturbance"]["gaussian_sigma"].as<double>();

  //get the unit applied vec for action a
  Vec2 v_app = mdp_state_t::getActionVector(act);
  v_app = v_max*v_app;
  Vec2 v_net = v_app;

  //each prob is associated with a action/transition link
  s->probs.resize(NUM_ACTIONS, 0);

  double sum = 0;
  for(uint i=NORTH; i<NUM_ACTIONS; i++){
      if(s->successors[i]->id != s->id){
          Vec2 v_succ = pNet->cell_centers[s->successors[i]->id] -
                  pNet->cell_centers[s->id];
          double theta, cos_theta;
          if(v_net.norm() > EPSILON){
              cos_theta = v_net.dot(v_succ)/(v_net.norm()*v_succ.norm());
              theta = std::acos(std::min(std::max(cos_theta, -1.0), 1.0));
              //method1: using gaussian pdf to emulate gaussian distribution on a polar coordinate
              s->probs[i] = gaussian_pdf(theta, 0, sigma);
              //method2: using cosine curve to emulate gaussian like distribution with p\in[0, 2]
              //s->probs[i] = (cos_theta+1);
          }
          else{
              theta = 0; sigma = 1e+2;
              //large sigma generates pretty much like "uniform" distriubtion
              s->probs[i] = gaussian_pdf(theta, 0, sigma);
          }
          //if only forward transitions are possible, set reverse transitions 0
          if(cos_theta < 0){
              s->probs[i] = 0;
          }
      }
      else
          s->probs[i] = 0;

      sum += s->probs[i];
  }

  //normalize to prob
  for(uint i=NORTH; i<NUM_ACTIONS; i++)
      s->probs[i] = (sum<EPSILON) ? 0 : s->probs[i]/sum;
}

void mdp_planner::uniform_transition_initialization(MDP_Net::Ptr& pNet)
{
  std::cout<<"IM here at uniform"<<std::endl;
  for(int i=0;i<pNet->mdp_states.size();i++)
  {
    if(pNet->mdp_states[i]->type != OBSTACLE){
      for(uint j=NORTH; j<NUM_ACTIONS; j++){
        pNet->mdp_states[i]->probs[j]=1.0/9.0;
        //std::cout<<pNet->mdp_states[i]->probs[j]<<std::endl;
      }
    }
  }
}

double mdp_planner::addSuccessorValue(mdp_state_t* s,action_t succ_a, MDP_Net::Ptr& pNet)
{
      mdp_planner::getTransitionModel(s,succ_a, pNet);
      double obst_penalty   = -100;//-pParams->getParamNode()["mdp_methods"]["obst_penalty"].as<double>();
      //action_cost    = pParams->getParamNode()["mdp_methods"]["action_cost"].as<double>();
      double value=0;
      for(uint i=NORTH; i<NUM_ACTIONS; i++){

          if(s->successors[i] != s){

              mdp_state_t* next_st = (s->successors[action_t(i)]);

              double imm_rew = (next_st->type == GOAL) ? 100 : -1;
              value += s->probs[i] * (imm_rew + 0.9* next_st->optimal_value);

          }
          else
              value += s->probs[i] * obst_penalty;

      }
      //if obstacles, or walls such that robot cannot move forward but return to current state, put a penalty on it
      return value;
}

void mdp_planner::updateState(mdp_state_t* s,MDP_Net::Ptr& pNet)
{
  if(s->type == BODY || s->type == START){
      //std::fill(s->q_values.begin(), s->q_values.end(), 0);
      //double max_val =  -INF; //s->optimal_value;
      //s->q_values[ZERO] = -INF;
      double max_val = -INF;
      action_t optimal_act;
      for(uint j=NORTH; j<NUM_ACTIONS; j++)
      {
          action_t act = (action_t)j;
          if(s->successors[act] != s) //the act should be legal
          {
              getTransitionModel(s, act,pNet);
              //double direct_action_value = getQvalue(s, act);
              double direct_action_value = mdp_planner::addSuccessorValue(s,act,pNet);
              if (direct_action_value > max_val)
              {
                max_val = direct_action_value;
                s->optimal_action = (action_t)j;
              }
          }
      }
      //fill the max value as current value
      s->optimal_value = max_val;
      getTransitionModel(s,s->optimal_action,pNet);
}
}

mdp_state_t* mdp_planner::one_time_step_sample(mdp_state_t* s, action_t act, MDP_Net::Ptr& pNet)
{
  double random_num;
  random_num = rand()%100;
  mdp_planner::getTransitionModel(s, act, pNet);
  double percent = random_num/100.0;
  double probability = 0.0;
  mdp_state_t* sample_next_state;
  for(uint i=NORTH;i<NUM_ACTIONS;i++)
  {
    probability += s->probs[i];
    if(percent < probability || percent == probability)
    {
      action_t sample_direct = (action_t)i;
      sample_next_state = s->successors[sample_direct];
      return sample_next_state;
    }
  }
}

double mdp_planner::Trail_update_(mdp_state_t* s, MDP_Net::Ptr& pNet)
{
  int max_iter_num = 30;//pParams->getParamNode()["mdp_methods"]["num_iterations"].as<int>();
//  std::cout<<"Im in trail_update"<<std::endl;
  //std::cout<<"max_iter_num "<<max_iter_num<<std::endl;
  int iter_num = 0;
  mdp_state_t* iter_state = s;
  mdp_state_t* iter_state_next;
  double error_all = 0;
  double error_current;
  while(iter_state->type != GOAL && iter_num < max_iter_num)
  {
    //std::cout<<"Im in trail_while"<<std::endl;
    double previous_value = iter_state->optimal_value;
    mdp_planner::updateState(iter_state, pNet);
    double current_value = iter_state->optimal_value;
    error_current = std::abs(previous_value - current_value);
    if(error_current > error_all)
    {
      error_all = error_current;
    }
    action_t opt_act = iter_state->optimal_action;
    for(uint j=NORTH;j<NUM_ACTIONS;j++)
    {
      action_t act_curr = (action_t)j;
      iter_state->actions[act_curr]=false;
    }
    iter_state->actions[opt_act]=true;
    iter_state_next = mdp_planner::one_time_step_sample(iter_state,opt_act,pNet);
    while(iter_state_next->type == OBSTACLE)
    {
      iter_state_next = mdp_planner::one_time_step_sample(iter_state,opt_act,pNet);
    }
    iter_state = iter_state_next;
    //if(iter_state->type == OBSTACLE)
    //{
    //  continue;
    //}
    iter_num++;
  }
  return error_all;
}

void mdp_planner::Trail_Based_RTDP(MDP_Net::Ptr& pNet)
{
  double error = 0;
  double error_one_trail;
  for(int i=0;i<pNet->mdp_states.size();i++)
  {
    if(pNet->mdp_states[i]->type != OBSTACLE)
    {
      error_one_trail = mdp_planner::Trail_update_(pNet->mdp_states[i],pNet);
      if(error<error_one_trail)
      {
        error = error_one_trail;
      }
    }
  }
  std::cout<<"error is"<<error<<std::endl;
  while(error > 0.5)
  {
    error = 0;
    //error = mdp_planner::Trail_update_(pNet->mdp_states[0],pParams,pNet);

    for(int i=0;i<pNet->mdp_states.size();i++)
    {
      if(pNet->mdp_states[i]->type != OBSTACLE)
      {
        error_one_trail = mdp_planner::Trail_update_(pNet->mdp_states[i],pNet);
        if(error<error_one_trail)
        {
          error = error_one_trail;
        }
      }
    }
    std::cout<<"error is"<<error<<std::endl;

  }
}
/*
void mdp_planner::MFPT_RTDP(MDP_Net::Ptr& pNet)
{
  std::cout<<"Im here"<<std::endl;
  double error = 0;
  double error_one_trail;

  vector<Transform2> tf2_starts_1, tf2_goals_1;
  vector<double> vec = pParams->getParamNode()["start_goal_config"]["tf2_starts"]["s0"].as< vector<double> >();
  std::cout<<"start position"<<std::endl<<vec[0]<<"  "<<vec[1]<<"  "<<vec[2]<<std::endl;
  tf2_starts_1.push_back(Transform2(vec[0], vec[1], vec[2]));
  vector<double> vec1 = pParams->getParamNode()["start_goal_config"]["tf2_goals"]["g0"].as< vector<double> >();
  tf2_goals_1.push_back(Transform2(vec1[0], vec1[1],vec[2]));
  mdp_planner::uniform_transition_initialization(pNet);
  mdp_state_t* start;
  start = pNet->getState(tf2_starts_1[0].translation);

  for(int j=0;j<10;j++){
  for(int i=0;i<pNet->mdp_states.size();i++){

    if(pNet->mdp_states[i]->type != OBSTACLE)
    {
      error_one_trail = mdp_planner::Trail_update_(pNet->mdp_states[i],pParams,pNet);
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


  SSP::Ptr pSSP = SSP::Ptr(new SSP(pNet));

  //high_resolution_clock::time_point t1 = high_resolution_clock::now();
  //_prioritizedstates.clear();
  //optimalActionTransitionDistribution(_states);
  pSSP->initTransMatrix(pNet);
  std::cout<<"goals "<<pNet->getState(tf2_goals_1[0].translation)->id<<std::endl;
  pSSP->MFPT(pNet, pNet->getState(tf2_goals_1[0].translation)->id);
  std::cout << "reachabilities" << std::endl;
  ofstream my_file;
  my_file.open("reachability.txt");

  for (int i = pNet -> n_rows - 1; i >= 0; i--) {
      for (int j = 0; j < pNet -> n_cols; j++) {
          //std::cout << i * pGrid2d -> n_cols + j << " ";
          std::cout<< (int)pSSP->reachability[i * pNet -> n_cols + j]  << " ";
          my_file<<pSSP->reachability[i * pNet -> n_cols + j]<<",";
      }
      std::cout << std::endl;
  }
  my_file.close();

for(int i =0;i<pNet->n_rows*pNet->n_cols;i++)
{
  for(int j=0;j<pNet->n_rows*pNet->n_cols;j++)
  {
    std::cout<<pSSP->transMatrix.coeffRef(i,j)<<" ";
  }
  std::cout<<std::endl;
}

std::cout<<"state values"<<std::endl;
for (int i = pNet -> n_rows - 1; i >= 0; i--) {
      for (int j = 0; j < pNet -> n_cols; j++) {
          //std::cout << i * pGrid2d -> n_cols + j << " ";
          std::cout<< (int)pNet->mdp_states[i*pNet->n_cols+j]->optimal_value<<" ";
          //my_file2<<pNet->mdp_states[i*pNet->n_cols+j]->optimal_value<<",";
      }
      std::cout << std::endl;
  }

}
*/
