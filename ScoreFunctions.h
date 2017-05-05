  //int32_t pulses = 1920; // pulses per revolution
  //double circumference = 30.9211; // the wheels circumference
  int32_t rightpos; 
  int32_t leftpos;
  int32_t tmprightpos;
  int32_t tmpleftpos;
  int32_t startTurnRightPos;
  int32_t startTurnLeftPos;
  //int32_t encoderVinkel1 = 350;
  int32_t runDistance = 400;
  double degreeToEncoder=21.67;
  double timeLastLoop = 0;
  double cmToEncoder = (1920/30.9211);

  bool shouldScore = true;
  bool shouldMoveFast = false;
  double scoreWaitTime = 3000;

///simons variabler

int32_t wheelPos =  abs((motor.readRightEncoder()+motor.readLeftEncoder())/2);

int32_t tmpright;
int32_t sek=millis();

double radToDegree = 57.29;
double robotAngle;
double posBall[2];
double posRobot[2];
double posGoal[2]={240,94};
double posKick[2];
double posHome[2]= {91,94};
double distBehindBall = 50; 
double length1 = 0;
double length2 = 0;
double length3 = 0;
double angelToBall = 0;
double angelToGoal = 0;
String a ="Hej";
double angleToBall = 0;
double angleToGoal = 0;
double angleToKick;
double encoderVinkel1;
double encoderVinkel2;
int16_t turnAngle;
int16_t encoderAngle;
double perimeterCircle = 20*2*3.14159265; 
static const double perimeterWheel = 30.9211;
double encoderStracka1 = 0;
double encoderStracka2 = 0;
double turningAngle=0;
double encoderVinkel=0;
static const double encoder = 1920; 
int i = 0;
double stopDist = 25*encoder/perimeterWheel; 


///simons variabler end
double calculateTurn(double angle){ // Beräkna rotation, posRobot robot, posBall boll, robotAngle robotens vinkel
  double dirAngle[2];
  dirAngle[0]=posBall[0]-posRobot[0];
  dirAngle[1]=posBall[1]-posRobot[1];
  String str1 = "x:"+String(dirAngle[0]);
  String str2 = "y:"+String(dirAngle[1]);
  String str3 = "robotAngle: "+String(robotAngle);
  String str4 = "AngletoBall: "+String(angle);
  //Serial.println(str3);
  //Serial.println(str4);
  //Serial.println(str1);
  //Serial.println(str2);
  //int16_t vAngle = atan(abs(dirAngle[0])/abs(dirAngle[1]))*radToDegree;
  
  int16_t encoderAngle;
  if(dirAngle[0]<0 && dirAngle[1]<0){
    turnAngle=robotAngle-180-angle;
    if(turnAngle<0){
      turnAngle=turnAngle+360;
    }
    Serial.println("turnAngle1: "+String(turnAngle));
    Serial.println("encoderAngle1: "+String(((abs(turnAngle)-20)*degreeToEncoder)));
    return ((abs(turnAngle))*degreeToEncoder);
  }else if(dirAngle[0]<0 && dirAngle[1]>0){
    turnAngle=robotAngle-360+angle;
    if(turnAngle<0){
      turnAngle=turnAngle+360;
    }
    Serial.println("turnAngle2: "+String(turnAngle));
    Serial.println("encoderAngle2: "+String(((abs(turnAngle)-20)*degreeToEncoder)));
    return ((abs(turnAngle))*degreeToEncoder);
  }else if(dirAngle[0]>0 && dirAngle[1]>0){  
    turnAngle=robotAngle-angle;
    if(turnAngle<0){
      turnAngle=turnAngle+360;
    }
    Serial.println("turnAngle3: "+String(turnAngle));
    Serial.println("encoderAngle3: "+String(((abs(turnAngle)-20)*degreeToEncoder)));
    return ((abs(turnAngle))*degreeToEncoder);
  }else if(dirAngle[0]>0 && dirAngle[1]<0){
    turnAngle=robotAngle-180+angle;
    if(turnAngle<0){
      turnAngle=turnAngle+360;
    }
    Serial.println("turnAngle4: "+String(turnAngle));
    Serial.println("encoderAngle4: "+String(((abs(turnAngle)-20)*degreeToEncoder)));
    return ((abs(turnAngle))*degreeToEncoder);
  }
}
  
  bool CalculationsDoneScore(){
    if(encoderVinkel1)
    return true;
    else
    return false;
  }

  bool TurnDoneScore(){
    rightpos = motor.readRightEncoder();
    leftpos = motor.readLeftEncoder();
      if(((abs(rightpos-startTurnRightPos))+ (abs(leftpos-startTurnLeftPos)))>abs(encoderVinkel1)){
        startTurnRightPos = motor.readRightEncoder();
        startTurnLeftPos = motor.readLeftEncoder();
      return true;
      }
      return false;
    }
    
   bool MoveDoneScore(){
    rightpos = motor.readRightEncoder();
    leftpos = motor.readLeftEncoder();
    //Serial.println(encoderStracka1);
    //Serial.println((((abs(rightpos)-abs(startTurnRightPos))+ (abs(leftpos)-abs(startTurnLeftPos)))/2));
      if((((abs(rightpos)-abs(startTurnRightPos))+ (abs(leftpos)-abs(startTurnLeftPos)))/2)>(abs(encoderStracka1)-20*cmToEncoder)){
        startTurnRightPos = motor.readRightEncoder()+1220;
        startTurnLeftPos = motor.readLeftEncoder()+1220;
        timeLastLoop = millis();
      return true;
      }
      return false;
    }

  bool ShouldTurnAgainScore(){
    if(abs(RobotAx - posKick[0]) > 40 || abs(RobotAy - posKick[1]) > 40){
      return true;
      }
    return false;
    }

  bool ShouldCalculateAgainScore(){
    if(millis()-timeLastLoop > 1000){
      return true;
      }
    
    return false;
    }
  bool ShouldGoHome(){
    
  }

  bool doneWaiting(){
    if(millis() - scoreWaitTime > 750){
        return true;
      }
      return false;
    }
  
  void CalculateScore(){
    
    encoderVinkel1 = NULL;
    robotAngle = RobotAalfa; 
    posBall[0] = bollx;
    posBall[1] = bolly;
    posRobot[0] = RobotAx;
    posRobot[1] = RobotAy;
    startTurnRightPos = motor.readRightEncoder();
    startTurnLeftPos = motor.readLeftEncoder();

    length2 = sqrt(pow((posGoal[0]-posBall[0]),2)+pow((posGoal[1]-posBall[1]),2));

    length1 = sqrt(pow((posBall[0]-posRobot[0]),2)+pow((posBall[1]-posRobot[1]),2))*1.35;
    
    posKick[0] = -(abs(posGoal[0]-posBall[0])*distBehindBall/length2)+posBall[0];
    
    if(posBall[1]>posGoal[1]){
    posKick[1] = (abs(posGoal[1]-posBall[1])*distBehindBall/length2)+posBall[1];
    }else{
      posKick[1] = -(abs(posGoal[1]-posBall[1])*distBehindBall/length2)+posBall[1];
    }
    length3 = sqrt(pow((posKick[0]-posRobot[0]),2)+pow((posKick[1]-posRobot[1]),2))*1.15;//*1.35
    Serial.println("posKickx: "+String(posKick[0]));
    Serial.println("posKickx: "+String(posKick[1]));
    Serial.println("length1: "+String(length3));
    double y1=posBall[0]-posRobot[0];
    double y2=posBall[1]-posRobot[1];
    y1=abs(y1);
    y2=abs(y2);
    double x1=y1/y1;
    angleToBall = (atan(x1)*radToDegree);
    
    double y3=posKick[0]-posRobot[0];
    double y4=posKick[1]-posRobot[1];
    y3=abs(y3);
    y4=abs(y4);
    double x2=y3/y4;
    angleToKick = (atan(x2)*radToDegree);

    double y5=posGoal[0]-posBall[0];
    double y6=posGoal[1]-posBall[1];
    y5=abs(y5);
    y6=abs(y6);
    double x3=y5/y6;
    angleToGoal = (atan(x3)*radToDegree);

    //räkna ut distans separat i en egen metod.
    encoderStracka1=(length3-20)*cmToEncoder;
    //encoderStracka2=(length2-20)*cmToEncoder;
    //Serial.println(encoderStracka1);
    encoderVinkel2 = calculateTurn(angleToKick);
    if(turnAngle<=180){
      encoderVinkel1 = calculateTurn(angleToKick)-20*degreeToEncoder;
    }else{
      encoderVinkel1 = ((360*degreeToEncoder)-calculateTurn(angleToKick))-20*degreeToEncoder;
    }
    
    encoderVinkel2 = calculateTurn(angleToGoal);
    //Serial.println(encoderVinkel1);   
    if(CalculationsDoneScore()){
        scoreState = turnValidateScore;
      }
    movementState = idle;
  }

  void Score(){
    shouldMoveFast = true;
    encoderVinkel1 = NULL;
    robotAngle = RobotAalfa; 
    posBall[0] = bollx;
    posBall[1] = bolly;
    posRobot[0] = RobotAx;
    posRobot[1] = RobotAy;
    startTurnRightPos = motor.readRightEncoder();
    startTurnLeftPos = motor.readLeftEncoder();

    //length2 = sqrt(pow((posGoal[0]-posBall[0]),2)+pow((posGoal[1]-posBall[1]),2));

    length1 = sqrt(pow((posBall[0]-posRobot[0]),2)+pow((posBall[1]-posRobot[1]),2))*1.35;
    
    posKick[0] = posBall[0];
    posKick[1] = posBall[1];
    
    length3 = 40;
    Serial.println("length2: "+String(length3));
    double y1=posBall[0]-posRobot[0];
    double y2=posBall[1]-posRobot[1];
    y1=abs(y1);
    y2=abs(y2);
    double x1=y1/y1;
    angleToBall = (atan(x1)*radToDegree);
    
    double y3=posKick[0]-posRobot[0];
    double y4=posKick[1]-posRobot[1];
    y3=abs(y3);
    y4=abs(y4);
    double x2=y3/y4;
    angleToKick = (atan(x2)*radToDegree);

    double y5=posGoal[0]-posBall[0];
    double y6=posGoal[1]-posBall[1];
    y5=abs(y5);
    y6=abs(y6);
    double x3=y5/y6;
    angleToGoal = (atan(x3)*radToDegree);

    //räkna ut distans separat i en egen metod.
    encoderStracka1=(length3-20)*cmToEncoder;
    encoderStracka2=(length2-20)*cmToEncoder;
    //Serial.println(encoderStracka1);
    encoderVinkel2 = calculateTurn(angleToKick);
     if(turnAngle<=180){
      encoderVinkel1 = calculateTurn(angleToKick);
    }else{
      encoderVinkel1 =((360*degreeToEncoder)-calculateTurn(angleToKick));
    }
    encoderVinkel2 = calculateTurn(angleToGoal);
    //Serial.println(encoderVinkel1);   
    if(CalculationsDoneScore()){
        scoreState = turnValidateScore;
      }
    movementState = idle;    
    }

// void goHome(){
  
 //}

  void waitToScore(){
    if(doneWaiting()){
      scoreState = scoreScore;
      }
    movementState = idle;
    }


  void TurnValidateScore(){
     if(TurnDoneScore()){
      scoreState = movnvalidateScore;
      if(shouldMoveFast){
        scoreState = moveFastScore;
        }
      }
      if(turnAngle<=180){
      movementState = turnR;
      }else{
        movementState = turnL;
      }
  }

  void MoveFast(){
    shouldMoveFast = false;
    if(MoveDoneScore()){
      scoreState = considerResultScore;
      }
      movementState = moveFast;
    }

  void MovnvalidateScore(){
     if(MoveDoneScore()){
        scoreState = considerResultScore;
      }
      movementState = movE;
  }

  void ConsiderResultScore(){
      
      //if(ShouldTurnAgainScore()){
      //  scoreState = calculateScore;
      //  }
      //else 
      if(shouldScore){
        scoreState = waitScore;
        scoreWaitTime = millis();
        shouldScore =false;
        }
      else{
        planState = wait;
        shouldScore = true;
        }
    movementState = idle;
  }


    
 

