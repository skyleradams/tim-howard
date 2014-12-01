#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define POSITION_ADDRESS 36

/*define useful data types */
struct twotuple
{
  double first;
  double second;
};

struct twoInts
{
  int first;
  int second;
};

struct fourtuple
{
  double first;
  double second;
  double third;
  double fourth;
};

struct trajectoryItem
{
  float t;
  float yleft;
  float zleft;
  float yright;
  float zright;
};



/* Dynamixel IDs */
#define ID_106_LEFT 1
#define ID_64_LEFT 2
#define ID_106_RIGHT 3
#define ID_64_RIGHT 4
Dynamixel Dxl(DXL_BUS_SERIAL1); //create dynamixel instance

// conversion motorposition to motorangle and offsets
const float ticksPerRad = 4096/(2*PI);
const float theta1Offset_left = 126*PI/180;
const float theta2Offset_left = 0.4*20*PI/180;

//physical goal dimension/position [m]
const float goal_leftY = 0.457;
const float goal_rightY = 1.981;
const float goal_bottomZ = 0;
const float goal_topZ = 0.914;

//robot base positions
const float baseoffset_leftY = 1.0;
const float baseoffset_rightY = 1.3;
const float baseoffsetZ = 0.4;

//robot armlength
const double l = 0.3;

const double angle_threshold = .09;

const float ballRadius = 0.06;

const int noOfWaypoints = 12;
float t_goal, y_goal, z_goal;


fourtuple robotTargets; //TODO: initialize to something for first few loops without data
trajectoryItem trajectory[noOfWaypoints+1]; //TODO: initialize to something for first few loops without data
//IMPORTANT: time in the trajectory is in milliseconds!!!


/* Define USB communication functions */
const char readyAck = 6;

void clearbuffer()
{
  while( SerialUSB.available() )
  {
    SerialUSB.read();
  }
}

int readIntFromBytes() //USB communication
{
  union u_tag {
    byte b[2];
    int ulval;
  } u;
  u.b[0] = SerialUSB.read();
  u.b[1] = SerialUSB.read();
  if (u.ulval > pow(2,15))
    u.ulval = u.ulval-pow(2,16);
  return u.ulval;
}



/* Define robot specific functions */
struct fourtuple worldTOlocaltargets(double y_global, double z_global)
{ //function sets target for each robot in its coordinate system
  
  struct fourtuple robottargets; // [yleft, zleft, yright, zright]
  float safetyDistance = 0.1; //[m]
  
  /* z values */
  if (z_global > goal_topZ)
    z_global = goal_topZ;
  if (z_global < ballRadius)
    z_global = ballRadius;
  robottargets.second = z_global - baseoffsetZ;
  robottargets.fourth = z_global - baseoffsetZ;
  
  /* y values */
  if (y_global > goal_rightY)
    y_global = goal_rightY;
  if (y_global < goal_leftY)
    y_global = goal_leftY;
  
  if (y_global > (goal_leftY+goal_rightY)/2 ) //right half of goal
  {
    robottargets.first = (goal_leftY+goal_rightY)/2 - baseoffset_leftY  - safetyDistance; //move to boundary
    robottargets.third = y_global - baseoffset_rightY; //move to given position
  }
  else
  {
    robottargets.first = y_global - baseoffset_leftY; //move to given position
    robottargets.third = (goal_leftY+goal_rightY)/2 - baseoffset_rightY + safetyDistance; //move to boundary
  }
  
  return robottargets;
}

struct twotuple invKinematics(twotuple endpointxy)
{
  //first check if within reach. If not reset to something sensible TODO
  /*
  if (pow(endpointxy.first,2) + pow(endpointxy.second,2) > pow(2*l,2))
  {
    endpointxy.first = 0;
    endpointxy.second = -0.1;
  }
  */
  
  //TODO: how to treat r=0 cases -- ignore, hope it doesnt happen
  double theta2 = 0.5*acos((endpointxy.first*endpointxy.first+endpointxy.second*endpointxy.second-2*l*l)/(2*l*l)); //0.5 due to gearing
  double theta1 = atan2(endpointxy.second, endpointxy.first)-acos(sqrt(endpointxy.first*endpointxy.first+endpointxy.second*endpointxy.second)/(2*l));
  
  twotuple result = {theta1,theta2};
  return result;
}

struct twotuple forwardKinematics(twotuple theta)
{
  twotuple endpointyz;
  theta.second = 2*theta.second; //gearing
  endpointyz.first = l*(cos(theta.first)+cos(theta.first+theta.second));
  endpointyz.second = l*(sin(theta.first)+sin(theta.first+theta.second));
  
  return endpointyz;
}

void generateTrajectory(fourtuple target,float t_goal,float currtime)
{
  //target = [yleft, zleft, yright, zright]
  fourtuple CurrPos = getCurrPosition();
  
  for (int i=0; i<=noOfWaypoints; i++) //linear interpolation in time and position
  {   
    //trajectory[i].t = (1.0*i/noOfWaypoints)*(t_goal*1000) + currtime;
    trajectory[i].t = 0.0;
    trajectory[i].yleft = (1.0*i/noOfWaypoints)*(target.first-CurrPos.first) + CurrPos.first;
    trajectory[i].zleft = (1.0*i/noOfWaypoints)*(target.second-CurrPos.second) + CurrPos.second;
    trajectory[i].yright = (1.0*i/noOfWaypoints)*(target.third-CurrPos.third) + CurrPos.third;
    trajectory[i].zright = (1.0*i/noOfWaypoints)*(target.fourth-CurrPos.fourth) + CurrPos.fourth;
  }
}

struct fourtuple twoTOfourTuple( twotuple one, twotuple two)
{
  fourtuple result = {one.first,one.second,two.first,two.second};
  return result;
}

struct fourtuple getCurrPosition()
{
  //Returns Currposition as [yleft, zleft, yright, zright] in robot coords
  fourtuple CurrPosition;
  twotuple motorpos_left, motorpos_right;
  
  motorpos_left.first = (Dxl.readWord(ID_106_LEFT, POSITION_ADDRESS)/ticksPerRad)-theta1Offset_left;
  motorpos_left.second = (Dxl.readWord(ID_64_LEFT, POSITION_ADDRESS)/ticksPerRad)-theta2Offset_left;
  motorpos_right.first = 0;//Dxl.readWord(ID_106_RIGHT, POSITION_ADDRESS)/ticksPerRad;
  motorpos_right.second = 0; //Dxl.readWord(ID_64_RIGHT, POSITION_ADDRESS)/ticksPerRad;
  
  /*SerialUSB.println("Current position theta");
  SerialUSB.println(motorpos_left.first);
  SerialUSB.println(motorpos_left.second);
  SerialUSB.println(motorpos_right.first);
  SerialUSB.println(motorpos_right.second);
  */
  
  CurrPosition = twoTOfourTuple(forwardKinematics(motorpos_left),forwardKinematics(motorpos_right));
  
  /*
  SerialUSB.println("current position xy");
  SerialUSB.println(CurrPosition.first);
  SerialUSB.println(CurrPosition.second);
  */
  
  return CurrPosition;
}

double BoundDouble(double upper, double lower, double num){
   while(num>=upper){
    num-=upper;
  }
  
  while(num< lower){
    num+= lower;
  }
  
  return num;
}

double FindMinimumDirection(double current, double goal){
  //scales angle so that motors always take the shortest path
  goal = BoundDouble(2*PI, 0, goal);
  //goal is now between 0 and 2pi
  current = BoundDouble(2*PI, 0, current);
  //current is now between 0 and 2pi
  double delta = 0;
  
  if (abs(goal-current) > PI){ //too far, go the other way
   delta = -2*PI + (goal - current);
  }
  else{
   delta = (goal-current);
  }
  
  return delta;

struct twoInts pickNextWaypoint()
{
  /* This function chooses the next waypoint. Returns twoInts nextWaypoint = {nextWaypoint_left, nextWaypoint_right}
     Idea: Check which waypoint we are closest to and choose the next.
  */
  fourtuple CurrPos = getCurrPosition();
  twoInts nextWaypoint;
  
  float distance2_left = 1000; //initialize squared distances to large value
  float distance2_right = 1000;
  
  for (int i=0; i<=noOfWaypoints; i++)
  {
    if ( (CurrPos.first-trajectory[i].yleft)*(CurrPos.first-trajectory[i].yleft)+(CurrPos.second-trajectory[i].zleft)*(CurrPos.second-trajectory[i].zleft) < distance2_left ) //check if i th waypoint is closer than any previous ones
    {
      nextWaypoint.first = i;
      distance2_left = (CurrPos.first-trajectory[i].yleft)*(CurrPos.first-trajectory[i].yleft)+(CurrPos.second-trajectory[i].zleft)*(CurrPos.second-trajectory[i].zleft);
    }
    
    if ( (CurrPos.third-trajectory[i].yright)*(CurrPos.third-trajectory[i].yright)+(CurrPos.fourth-trajectory[i].zright)*(CurrPos.fourth-trajectory[i].zright) < distance2_right )
    {
      nextWaypoint.second = i;
      distance2_left = (CurrPos.third-trajectory[i].yright)*(CurrPos.third-trajectory[i].yright)+(CurrPos.fourth-trajectory[i].zright)*(CurrPos.fourth-trajectory[i].zright);
    }
  }

  /* increment next waypoint unless we are already at the last one */
  if (nextWaypoint.first != noOfWaypoints)
    nextWaypoint.first++;
  if (nextWaypoint.second != noOfWaypoints)
    nextWaypoint.second++;
  
  return nextWaypoint;
}
  


void setup() //this runs once after startup or reset
{
  // Dynamixel 2.0 Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps 
  Dxl.begin(3);
  //enable multi turn mode
  Dxl.writeWord(BROADCAST_ID, 6, 4095);
  Dxl.writeWord(BROADCAST_ID, 8, 4095);
  
  SerialUSB.begin();
  delay(3000);
  clearbuffer();
  SerialUSB.print(readyAck);
  
  //initialize trajectory
  for (int i=0; i<=noOfWaypoints; i++)
  {
    trajectory[i].t = 0;
    trajectory[i].yleft = 0;
    trajectory[i].zleft = 0;
    trajectory[i].yright = 0;
    trajectory[i].zright = 0;
  }
}


/* void loop() must remain at end of code. Define all functions above */

twotuple nextWaypointTheta_left, nextWaypointTheta_right;
twotuple nextWaypointYZ_left, nextWaypointYZ_right;
fourtuple currpos;
int encPosition1, encPosition2;
int speed = 50; //max: 1023

void loop() //this runs repeatedly during operation
{
  
  
  /* Get new target positions and calculate trajectory */
  if (SerialUSB.available() >= 6)
  {
    if (SerialUSB.available() != 6) //something went wrong. clear and try again
    {
      clearbuffer();
      SerialUSB.print(readyAck);
    }
    else
    {
      t_goal = 0.01*readIntFromBytes();
      y_goal = 0.01*readIntFromBytes();
      z_goal = 0.01*readIntFromBytes();
      SerialUSB.print(readyAck);
      
      /*SerialUSB.println("y_goal, z_goal");
      SerialUSB.println(y_goal);
      SerialUSB.println(z_goal); */
      
      robotTargets = worldTOlocaltargets(y_goal,z_goal);
      generateTrajectory(robotTargets, t_goal, millis());
      
      /*SerialUSB.println("Theta:");
      SerialUSB.println(nextWaypointTheta_left.first);
      SerialUSB.println(nextWaypointTheta_left.second);*/

    } 
  }
  
  /* Command the robot */
  //choose correct waypoint
  

  /*
  for(int i=0; i<noOfWaypoints; i++) //could do with search function
  {
    if (trajectory[i].t > millis()) //found next waypoint
      waypointNumber = i;
      break;
  }
  /*
  
  if (i==noOfWaypoints) //we ran through the loop without break --> exceeded time of trajectory
    waypointNumber = noOfWaypoints-1; //choose last one
  */
  
//  int waypointNumber = 12;
//  
//  nextWaypointYZ_left.first = trajectory[waypointNumber].yleft;
//  nextWaypointYZ_left.second = trajectory[waypointNumber].zleft;
//  nextWaypointYZ_right.first = trajectory[waypointNumber].yright;
//  nextWaypointYZ_right.second = trajectory[waypointNumber].zright;
//  
//  nextWaypointTheta_left = invKinematics(nextWaypointYZ_left);
//  
//  encPosition1 = (int) floor((nextWaypointTheta_left.first+theta1Offset_left)*ticksPerRad);
//  encPosition2 = (int) floor((nextWaypointTheta_left.second+theta2Offset_left)*ticksPerRad);
//  1,
  fourtuple CurrPos = getCurrPosition();
//  double t1 = FindMinimumDirection(CurrPos.first,t_goal*ticksPerRad);
//  double t2 = FindMinimumDirection(CurrPos.second,y_goal*ticksPerRad);
  
  
  encPosition1 = (int) floor(t_goal*ticksPerRad);
  encPosition2 = (int) floor(y_goal*ticksPerRad);
  
  /*SerialUSB.println("encoder position");
  SerialUSB.println(encPosition1);
  SerialUSB.println(encPosition2); */

  Dxl.setPosition(ID_106_LEFT, encPosition1, speed);
  Dxl.setPosition(ID_64_LEFT, encPosition2, speed);

  
  //convert to motor angles
  
  //nextWaypointTheta_right = invKinematics(nextWaypointYZ_right);
  
  //convert to motor positions (ticks) and send (syncwrite)
 
  
  
  /*
  Dxl.setPosition(ID_106_LEFT, nextWaypointPhi_left.first*ticksPerRad, speed);
  Dxl.setPosition(ID_64_LEFT, nextWaypointPhi_left.second*ticksPerRad, speed);
  */
  

  /*
  SerialUSB.println("nextWaypointTheta");
  SerialUSB.println(nextWaypointTheta_left.first);
  SerialUSB.println(nextWaypointTheta_left.second);

  SerialUSB.println(ticksPerRad);
  */
   
//  SerialUSB.println("Current xy position in loop");
//  SerialUSB.println(currpos.first);
//  SerialUSB.println(currpos.second);
//  SerialUSB.println(currpos.third);
//  SerialUSB.println(currpos.fourth);
//  delay(500);




 SerialUSB.println("thetas");
 SerialUSB.println(Dxl.readWord(ID_106_LEFT, POSITION_ADDRESS));
 SerialUSB.println(Dxl.readWord(ID_64_LEFT, POSITION_ADDRESS));
 delay(100);

  
  
  /* outline of code
  check if target position is available from serialUSB and get it if yes. What if no? what do at first run?
  convert target position to local robot systems xy
  get current endpoint positions (read -> phi -> theta -> xy)
  generate trajectory waypoints xy space
  choose which waypoint to go to next and which speed
  convert this waypoint to motor positions (xy -> theta -> phi -> motorpos)
  send to motor: Dxl.setPosition(ID_64_LEFT, position, speed);
  */
  
}
