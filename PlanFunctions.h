 //while the system runns with two parralell statemachines running there is also some other statemachines running for each plan state

  void scorePlan(){
          //Score switch 
              switch(scoreState) {
        
           case calculateScore  :
              CalculateScore();
              break;

           case scoreScore    :
              Score();
              break;
           
         
           case waitScore    :
              waitToScore();
              break;

           case moveFastScore    :
              MoveFast();
              break;

           case turnValidateScore  :
              TurnValidateScore();
              break;
        
           case movnvalidateScore  :
              MovnvalidateScore();
              break;
      
           case considerResultScore :
              ConsiderResultScore();
              break;
           
           default :
              break;
      }
    }
    
  void defendPlan(){
          //Defend switch
              switch(defendState) {
        
           case calculateDefend  :
              CalculateDefend();
              break;
         
           case turnValidateDefend  :
              TurnValidateDefend();
              break;
        
           case movnvalidateDefend  :
              MovnvalidateDefend();
              break;
      
           case considerResultDefend :
              ConsiderResultDefend();
              break;
           
           default :
              break;
      }
    }
    
  void waitPlan(){
      waitRoutine();
    }
    
  void avoidPlan(){
            //Avoid switch
              switch(avoidState) {
        
           case calculateAvoid  :
              CalculateAvoid();
              break;
         
           case turnValidateAvoid  :
              TurnValidateAvoid();
              break;
        
           case movnvalidateAvoid  :
              MovnvalidateAvoid();
              break;
      
           case considerResultAvoid :
              ConsiderResultAvoid();
              break;
           
           default :
              break;
      }
    }

