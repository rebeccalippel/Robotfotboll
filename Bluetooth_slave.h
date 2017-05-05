/*
 * By Sten Elling T. Jacobsen 2017 
 */

  int timer4 = 0;
void setupCommunication() 
{
  
}
double commsTimer = 0;
char input;
String package;
void runCommunication(){
    int timer3 = millis();
  
 if(Serial.available()>0){
    input=Serial.read();
  }
      while(input!='!'){ // Add characters to a string (package) until ! is received 
        if(millis()-commsTimer>80){
          return;
        }
        if(Serial.available()>0){
        package+=String(input);
        input=Serial.read();
        
        }
    }
    //Serial.print(package);
    sortString(package);
    
    package="";
  //Serial.print("Bluetooth: ");  
  //Serial.println(timer3-timer4);
  //timer4=timer3;
 }
  

