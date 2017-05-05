/*
 * This is code for a fotball playing robot.
 * This code is intended to talk to a modified version of the balanduino base code, and some code for communication with an overhead pixy, that is where the position data comes from.
 * The purpuse of this code is to make the decisions on where to move and how to rotate based on the data from the overhead camera. 
 * There are two parrallel statemachines running in this code, the one for the basic movement, in MovementStates, that deals with the spesific instructions to the balandruino code, and also 
 * the one planing and larger decisions, in PlanStates. The PlanStates dictates when to wait and when to try to score and so fourth.
 * 
 * In this tab there is a switch depending on the planState, that leads to functions for each state that contains their own actions and tests that should do what is desiered in that state every 
 * frame and also decides if the state should be switched to another one, depending on what test returns true.
 * 
 * To add to this code: find the funcions you want to use and att the "activly doing stuff" code there and put the organizing code in the tests.
 * 
 * The three states of motion that builds upp all physical activity.
 *  By Anders Elfverson 2017
 */



enum MovementState {
  idle,
  turnR,
  turnL,
  movE,
  moveFast
};
MovementState movementState = idle;
/*
 * The five states describing what plans can be carried out, 
 * score goes to a place near the ball and the goes diretly on the ball, kicking it.
 * defend goes to a place close to home
 * wait makes the robot stay idle until one of the stopwaiting condisions turns true
 * avoid goes away form the closes player a little bit.
 * 
 */
enum PlanState{
  score,
  defend,
  wait,
  avoiD 
};
PlanState planState = wait;

/*Score starts by claculating where to go and then moves on to the turnstate where it turns toward that position and moves to the move state where it moves straight ahead until
 * the position is belived to have been reached. Then it makes desicions based on the diffrendce between what it intended to to and what it acctually did.
 * The states defend and avoid have a very similar layout. Specific for the score state is that it moves to several positions, two or three, before it is done. That is 
 * handelled by the considerresult function.
 */

enum ScoreState {
  calculateScore,
  scoreScore,
  waitScore,
  goHome,
  moveFastScore,
  turnValidateScore,
  movnvalidateScore,
  considerResultScore
};
ScoreState scoreState = calculateScore;

//Moves close to home
enum DefendState {
  calculateDefend,
  turnValidateDefend,
  movnvalidateDefend,
  considerResultDefend
};
DefendState defendState = calculateDefend;

//Moves away from others
enum AvoidState {
  calculateAvoid,
  turnValidateAvoid,
  movnvalidateAvoid,
  considerResultAvoid
};
AvoidState avoidState = calculateAvoid;

// the "#include":s are done here to let the states be visible when compiling the subfunctions contained in these files.
#include "MoveFunctions.h"
#include "ScoreFunctions.h"
#include "WaitFunctions.h"
#include "DefendFunctions.h"
#include "AvoidFunctions.h"
#include "PlanFunctions.h"


//this function runs the statemachines  
void runStateMachine(){
//Basic movement switch
     switch(movementState) {
      
     case idle  :
        idleMovement();
        break;
   
     case turnR  :
        turnRightMovement();
        break;

     case turnL :
        turnLeftMovement();
        break;
  
     case movE  :
        moveMovement();
        break;

     case moveFast  :
        fastMovement();
        break;
     
     default :
        break;
     }
//Plan switch
    switch(planState) {
  
     case score  :
        scorePlan();
        break;
   
     case defend  :
        defendPlan();
        break;
  
     case wait  :
        waitPlan(); 
        break;

     case avoiD :
        avoidPlan();
        break;
     default :
        break;
     }
}


  

