//globala variabler
int bollx;
int bolly;

int evBollx; 
int evBolly; 


int RobotAx;
int RobotAy;
int RobotAalfa;

int evRobotAx;
int evRobotAy;
int evRobotAalfa;


int RobotBx;
int RobotBy;
int RobotBalfa;

int evRobotBx;
int evRobotBy;
int evRobotBalfa;




int Rightx(int nyx, int variabel){
    if(0<= nyx && nyx <= 199){ //ställ in våra värden på planen sen 
      variabel = nyx;  
    }
    return variabel;
  }
  
int Righty(int nyy, int variabel){
    if(0<= nyy && nyy <= 319){ //ställ in våra värden på planen sen 
      variabel = nyy;  
    }
    return variabel;
  }

  int Rightalfa(int nyalfa, int variabel){
    if(0<= nyalfa && nyalfa <= 180){ //ställ in våra värden på planen sen 
      return nyalfa;  
    }else if(-180<= nyalfa && nyalfa <= 0){
      return (nyalfa+360);
    }
    return variabel;
  }



  int getVariable(String Variable){
  if(Variable == "bollx"){
    return bollx;
  }
  else if (Variable == "bolly"){
    return bolly;
  }
  else if(Variable == "RobotAx"){
    return RobotAx;
  }
  else if(Variable == "RobotAy"){
    return RobotAy;
  }
  else if(Variable == "RobotAalfa"){
    return RobotAalfa;
  }
  else if(Variable == "RobotBx"){
    return RobotBx;
  }
  else if(Variable == "RobotBy"){
    return RobotBy;
  }
  else if(Variable == "RobotBalfa"){
    return RobotBalfa;
  }
  return 9999;
  }

// splitting a string and return the part nr index split by separator
String getStringPartByNr(String data, char separator, int index) {
    int stringData = 0;        //variable to count data part nr 
    String dataPart;      //variable to hole the return text

    for(int i = 0; i<data.length(); i++) {    //Walk through the text one letter at a time
        if(data[i]==separator) {
            //Count the number of times separator character appears in the text
            stringData++;
        } else if(stringData==index) {
            //get the text when separator is the rignt one
            dataPart.concat(data[i]);
        } else if(stringData>index) {
            //return text and stop if the next separator appears - to save CPU-time
            return dataPart;
            break;
        }
    }
    //return text if this is the last part
    return dataPart;
}

String getRightStringPart(String data, char boks1, char boks2){
  String RensaData1 = getStringPartByNr(data, boks1, 1); //De tecken från strängen data som kommer efter(1) bokstaven boks1
  String RensaData2 = getStringPartByNr(RensaData1, boks2, 0); //De tecken från strängen data som kommer innan(0) bokstaven boks2
  return RensaData2;
  }

void sortString(String data){
  
if(getStringPartByNr(data, 'x', 0)=="1"){//om den returnerande strängen är 1 gäller datan boll
  
  evBollx = getRightStringPart( data, 'x', 'y').toInt(); //De tecken som komemr mellan x och y samt att vi gör om den till en int
  bollx = Rightx(evBollx, bollx); //kontrollera om evBollx ligger i rätt intervall

  evBolly = getRightStringPart( data, 'y', 'a').toInt(); //De tecken som komemr mellan y och a samt att vi gör om den till en int
  bolly = Righty(evBolly, bolly);
}

else if (getStringPartByNr(data, 'x', 0)=="37"){ //om den returnerande strängen är 2 gäller datan robotA
  
  evRobotAx = getRightStringPart( data, 'x', 'y').toInt(); //De tecken som komemr mellan x och y samt att vi gör om den till en int
  RobotAx = Rightx(evRobotAx, RobotAx); 
  
  evRobotAy = getRightStringPart( data, 'y', 'a').toInt(); //De tecken som komemr mellan y och a samt att vi gör om den till en 
  RobotAy = Rightx(evRobotAy, RobotAy);
  
  evRobotAalfa = getStringPartByNr(data, 'a', 1).toInt(); //De tecken från strängen data som kommer efter bokstaven a samt att vi gör om den till en int
  RobotAalfa = Rightalfa(evRobotAalfa, RobotAalfa);
}


else if (getStringPartByNr(data, 'x', 0)== "3"){ //om den returnerande strängen är 3 gäller datan robotB
  
  evRobotBx = getRightStringPart( data, 'x', 'y').toInt(); //De tecken som komemr mellan x och y samt att vi gör om den till en int
  RobotBx = Rightx(evRobotBx, RobotBx);
  
  evRobotBy = getRightStringPart( data, 'y', 'a').toInt(); //De tecken som komemr mellan y och a samt att vi gör om den till en int
  RobotBy = Rightx(evRobotBy, RobotBy);

  evRobotBalfa = getStringPartByNr(data, 'a', 1).toInt(); //De tecken från strängen data som kommer efter bokstaven a samt att vi gör om den till en int 
  RobotBalfa = Rightalfa(evRobotBalfa, RobotBalfa); 
}
}


