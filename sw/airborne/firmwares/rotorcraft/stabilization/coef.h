/*Copyright (C) Florian Sansou <florian.sansou@enac.fr> 
 This file is directly generated with MATLAB to obtain the coefficient set of the looping
*/

#define CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE  2
#define CTRL_HOVER_WIND_INPUT 11
#define CTRL_HOVER_WIND_NUM_ACT 4

const float kf = 0.000000014047000;
const float mot_max_speed = 15769.000000000000000;

const float ueq[CTRL_HOVER_WIND_NUM_ACT][1] = {{1.1229},{1.1229},{0},{0}};

const float H[CTRL_HOVER_WIND_NUM_INTEGRATOR_STATE][CTRL_HOVER_WIND_INPUT] = {{-0.0011717,0.31103,0.26206,0.21279,-0.56428,-2.3279,-1.6503,-2.6974,-0.94502,-0.35911,-1.4043}, 
{-0.33323,-0.015029,0.037479,2.2011,0.26086,0.049195,-0.091841,-0.88079,0.13119,-0.83053,-0.18889}};

const float K[CTRL_HOVER_WIND_NUM_ACT][CTRL_HOVER_WIND_INPUT] = {{-0.0685006,-0.893192,-1.10171,-0.855695,6.50831,14.666,11.1447,18.1199,3.53513,3.30993,12.444}, 
{0.636478,-0.681366,-0.160426,-2.05888,-7.35769,21.5883,6.68889,-4.69494,5.35087,0.00962274,-6.4564}, 
{0.827446,0.56992,-0.560788,-14.922,-4.17019,-1.84899,-1.09902,-3.06815,5.40289,14.0842,-0.135449}, 
{0.706084,-0.524046,0.285638,-13.6654,-0.645657,2.52428,-0.00831569,7.57345,-6.21705,13.9808,1.42099}};

const float num[3] = {-0.000937625039959,-0.000010054651112,0.000927570388847};

const float den[3] = {1.000000000000000,-1.946339688780586,0.946339688786502};

