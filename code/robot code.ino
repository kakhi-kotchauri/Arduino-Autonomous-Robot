
// libraryes
#include <AsyncDelay.h>
#include <IRremote.h>    
#include <QMC5883LCompass.h>
#include <Servo.h>


// for servo
Servo headservo;
int servo_position = 0;

// for ir recive  
const int RECV_PIN = 12;     
IRrecv irrecv(RECV_PIN);     
decode_results results; 

int oldir = 0;


// for ir timer
bool irtimeractive = false;
AsyncDelay irtimer;


// for usesonic
const int trigPin = 11;
const int echoPin = 10;
long duration;
int distance;


// for useaveragenum
int calculate = 0;
int averagedistance = 40;


// for movements

const int leftforward = 2;
const int leftbackward = 4;

const int rightforward = 7;
const int rightbackward = 8;

bool stopforward = false;
bool blockreverse = true;


// for mode

int setmode = 0;
int ledinterval;
const int ledpin = 9;

// for turn functions
bool turnrightactive = false;
bool turnedonright = false;
bool turnedonleft = false;
bool isturning = false;

bool turnleftactive = false;




int deg;


// for usehybridmode

AsyncDelay servotimer;
AsyncDelay backwardtimer;
AsyncDelay headingtimer;
bool rightcorrected = false;
bool rightcorrected2 = false;
bool leftcorrected = false;
bool leftcorrected2 = false;
bool blockright = false;
bool blocktimercheck = false;
bool movingonright = false;
bool movingonleft = false;
bool headingsetted = false;

int leftturndistance;
int rightturndistance;


// for useautonomy
bool blockreset = false;
bool stopcheckaround = true;
bool callstoper = true;
bool stop1 = true;
bool stop2 = true;
bool stop3 = true;
bool stop4 = true;
bool stoper1 = false;
bool moveblocker = false;
bool once = false;

int leftdistance;
int rightdistance;

AsyncDelay delay1;
AsyncDelay delay2;
AsyncDelay delay3;
AsyncDelay delay4;
AsyncDelay reversingtimer1;
AsyncDelay reversingtimer2;
AsyncDelay turningtimer;



// for compass
AsyncDelay compasstimer;
QMC5883LCompass compass;

 int correctheading;



// functions for distance measure

void usesonic() {

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
  // Serial.print("Distance: ");
}


void useaveragenum(int measurements, int number) {

for(int i = 0; i < measurements; ++i)
{
  calculate += number;
  delay(0.3);
}

calculate /= measurements;


if(calculate >= 0 && calculate < 200) {
  averagedistance = calculate;
}

}


// for compass

int azimuth;

void usecompass() {
    compass.read();
    azimuth = compass.getAzimuth();

  if(compasstimer.isExpired()) {
    // Serial.println(azimuth);
    compasstimer.start(200, AsyncDelay::MILLIS);
  }


}



// functions for movement

void forward(bool status) {

  if(!stopforward) {

    if(status) {
    digitalWrite(leftforward, 1);
    digitalWrite(rightforward, 1);
    } else {
    digitalWrite(leftforward, 0);
    digitalWrite(rightforward, 0);
    }

  }

}

void backward(bool status) {
  if(status) {
  digitalWrite(leftbackward, 1);
  digitalWrite(rightbackward, 1);
  } else {
  digitalWrite(leftbackward, 0);
  digitalWrite(rightbackward, 0);
  }
}

void left(bool status) {
  if(status) {
  digitalWrite(leftbackward, 1);
  digitalWrite(rightforward, 1);
  } else {
  digitalWrite(leftbackward, 0);
  digitalWrite(rightforward, 0);
  }
}

void right(bool status) {
  if(status) {
  digitalWrite(leftforward, 1);
  digitalWrite(rightbackward, 1);
  } else {
  digitalWrite(leftforward, 0);
  digitalWrite(rightbackward, 0);
  }
}


// functions for ir reciving

// function for reset ir

bool isreseted = false;

void reset() {

 if(headingtimer.isExpired() && !headingsetted && !turnrightactive && !turnleftactive)
 {
    correctheading = azimuth;
    headingsetted = true;
    movingonright = false;
    movingonleft = false;
    Serial.println("RR");
    Serial.println(azimuth);
 }

 if(isreseted == false) {

  if(irtimeractive == false) {
    irtimer.start(110, AsyncDelay::MILLIS);
    irtimeractive = true;
  }

  if(irtimer.isExpired()) {
    forward(false); 
    backward(false); 
    right(false); 
    left(false); 
    if(!turnrightactive && !turnleftactive) {
      correctheading = azimuth;
    }
    headingtimer.start(170, AsyncDelay::MILLIS);
    Serial.println("reset");
    headservo.write(90);
    backwardtimer.start(180, AsyncDelay::MILLIS);
    blocktimercheck = false;
    irtimeractive = false;
    isreseted = true;
    headingsetted = false;
    rightturndistance = 0;
    leftturndistance = 0;
  }

 }


}

bool testt = false;
bool testt2 = false;

// functions for turning 

void turnright() { 

 if(azimuth < correctheading + deg && turnrightactive && !testt && turningtimer.isExpired() == false) {

   if(azimuth < 10 && azimuth >= 5 && correctheading + deg - 359 >= 0 ) {
     correctheading = correctheading + deg - 359;
    //  Serial.println("metia 359 ze");
    //  Serial.println(correctheading);
     testt = true;
   }

   right(true);
   isturning = true;
  //  Serial.println("turningonrightside");

 } else if(azimuth < correctheading && testt && turningtimer.isExpired() == false) {
   Serial.println(correctheading);
   Serial.println(azimuth);
   right(true);
   isturning = true;

 } else if(!turnedonright) {
   right(false);
  //  correctheading = azimuth;
   isturning = false;
  //  Serial.println("finished turning on right side");
   moveblocker = false;
   turnedonright = true;
   testt = false;
   turnrightactive = false;
   headingtimer.start(150, AsyncDelay::MILLIS);
   headingsetted = false;
 }

}



void turnleft() { 

 if(azimuth > correctheading - deg && turnleftactive && !testt2 && turningtimer.isExpired() == false) {

   if(azimuth >= 350 && azimuth < 355 && correctheading - deg + 359 <= 350) {
     correctheading = correctheading - deg + 359;
    //  Serial.println("naklebia 359 ze");
     Serial.println(correctheading);
     testt2 = true;
   }
 
   left(true);
   isturning = true;
  //  Serial.println("leftside");
   Serial.println(azimuth);

 } else if(azimuth > correctheading && testt2 && turningtimer.isExpired() == false) {

   Serial.println(correctheading);
   Serial.println(azimuth);
   left(true);
   isturning = true;

 } else if(!turnedonleft) {
   left(false);
   isturning = false;
  //  Serial.println("finished turning on left side");
   moveblocker = false;
   turnedonleft = true;
   testt2 = false;
   turnleftactive = false;
   headingtimer.start(150, AsyncDelay::MILLIS);
   headingsetted = false;
 }

}


void reciveir() {

if (irrecv.decode(&results)){

    irrecv.resume();
    isreseted = false;
    irtimer.start(110, AsyncDelay::MILLIS);
    
    if(results.value != 4294967295) {
        oldir = results.value;
    }

    Serial.println(oldir);

    if(oldir == -12241) {
      forward(true); 
    } else if(oldir == 28815) {
      backward(true);
    } else if(oldir == -30601 && blockreverse) {      
      if(setmode == 0 && !blocktimercheck) {
        headservo.write(0);
        servotimer.start(200, AsyncDelay::MILLIS);
        blocktimercheck = true;
      }
      if(servotimer.isExpired() && headservo.read() == 0) {
        rightturndistance = averagedistance;
      }
      if( rightturndistance >= 22) {
        headservo.write(90);
        right(true);
        movingonright = true;
        correctheading = azimuth;
      } else if(setmode == 1) {
        right(true);
      }
    } else if(oldir == 2295 && blockreverse) {
      if(setmode == 0 && !blocktimercheck) {
        headservo.write(180);
        servotimer.start(200, AsyncDelay::MILLIS);
        blocktimercheck = true;
      }
      if(servotimer.isExpired() && headservo.read() == 180) {
          leftturndistance = averagedistance;
      }
      if(leftturndistance >= 22) {
        headservo.write(90);
        left(true);
        movingonleft = true;
        correctheading = azimuth;
      } else if(setmode == 1) {
        left(true);
      }
    } else if(oldir == 6375) {
      setmode = 0;
    } else if(oldir == 26775) {
      setmode = 1;
      digitalWrite(ledpin, HIGH);
      stopforward = false;
    } else if (oldir == -26521) {
       setmode = 2;
    } else if(oldir == -17851) {
      digitalWrite(ledpin, LOW);
    } else if(oldir == 30855) {
      deg = 90;
      turnrightactive = true;
      turnedonright = false;
      correctheading = azimuth;
      turningtimer.start(10000, AsyncDelay::MILLIS);
    } else if(oldir == 14535) {
      deg = 90;
      turnleftactive = true;
      turnedonleft = false;
      correctheading = azimuth;
      turningtimer.start(10000, AsyncDelay::MILLIS);
    }



  } else {
      reset();
  }

}



// for blinkled

AsyncDelay ledtimer;

void blinkled(int timedelay) {

  ledinterval = timedelay;

  if(ledtimer.isExpired()) {
    digitalWrite(ledpin, HIGH);
    ledtimer.start(ledinterval, AsyncDelay::MILLIS);
  } else {
    digitalWrite(ledpin, LOW);
  }


}




// function for hybrid mode
int maxtolerance = 18;
int tolerance = 18;

AsyncDelay correctiontimer;

void usehybridmode() {
 
    blinkled(500);

    // Serial.println(movingonright);
    // Serial.println(movingonleft);

  if(!movingonright && !movingonleft && !isturning) {

    // Serial.println("shemodis kompasshi");


    if(correctheading > 350 && azimuth < 10) {
      correctheading = 5;
    } else if(correctheading < 10 && azimuth > 350) {
      correctheading = 355;
    }

    if(correctheading + tolerance < azimuth && azimuth <= 180 ) {
      left(true);
      tolerance = 0;
      rightcorrected = false;
      // Serial.println("leftcorrect1");
      // Serial.println(correctheading);
      // Serial.println(azimuth);
    } else if(!rightcorrected) { 
      left(false);
      tolerance = maxtolerance;
      rightcorrected = true;
    }

    if(correctheading - tolerance > azimuth && azimuth <= 180 ) {
      right(true);
      tolerance = 0;
      leftcorrected = false;
    } else if(!leftcorrected) {
      right(false);
      tolerance = maxtolerance;
      leftcorrected = true;
    }


    if(correctheading + tolerance < azimuth && azimuth > 180 ) {
      left(true);
      tolerance = 0;
      // Serial.println("leftcorrect2");
      rightcorrected2 = false;
    } else if(!rightcorrected2) {
      left(false);
      tolerance = maxtolerance;
      rightcorrected2 = true;
    }


    if(correctheading - tolerance > azimuth && azimuth > 180 ) {
      right(true);
      tolerance = 0;
      leftcorrected2 = false;
    } else if(!leftcorrected2) {
      right(false);
      tolerance = maxtolerance;
      leftcorrected2 = true;
    }

  }



/////////


    if(averagedistance  <= 20 && !stopforward && headservo.read() == 90 || digitalRead(6) == 0 && !stopforward && headservo.read() == 90) {
      // Serial.println("gacherdi");
      forward(false);
      stopforward = true;
    } else if(stopforward && averagedistance > 20) {
      // Serial.println("gaushvi");
      stopforward = false;
    }

    if(averagedistance <= 12 && headservo.read() == 90 && backwardtimer.isExpired()) {
      left(false);
      right(false);
      backward(true);
      blockreverse = false;
    } else if(!blockreverse) {
      backward(false);
      blockreverse = true;
    }


}




// useautonomy

void useautonomy() {

    blinkled(1000);

    if(!moveblocker && !once) {
      forward(true);
      once = true;
      // Serial.println("gaxsna");
    } else if(moveblocker && once) {
      forward(false);
      once = false;
      // Serial.println("dabloka");
    }




  if(averagedistance < 20 && callstoper && !isturning || digitalRead(6) == 0 && callstoper && !isturning) {

    stopcheckaround = false; 
    moveblocker = true;
    callstoper = false;
    leftdistance = 0;
    rightdistance = 0;
    // Serial.println("charte tavidan");

  } else if(averagedistance >= 30 && stopcheckaround && !callstoper ) {
    callstoper = true;
    // Serial.println("gaachere");
  }

    if(averagedistance < 20 || digitalRead(6) == 0) {
      forward(false);
      stopforward = true;
      moveblocker = true;
    // Serial.println("tormuzi");
    } else if(averagedistance >= 20 && stopforward && stopcheckaround) {
    // Serial.println("tormuzis gauqmeba");
      stopforward = false;
    }

  if(!stopcheckaround) {

  
    if(!blockreset) {
   
      stop1= false;
      blockreset = true;
      delay1.start(500, AsyncDelay::MILLIS);
      delay2.start(1000, AsyncDelay::MILLIS);
      delay3.start(1500, AsyncDelay::MILLIS);
      delay4.start(2000, AsyncDelay::MILLIS);
    }

  
    if(delay1.isExpired() && !stop1) {
        headservo.write(25);
      // Serial.println("a");
      stop1 = true;
      stop2 = false;
    }

    if(delay2.isExpired() && !stop2) {
      rightdistance = averagedistance;
        headservo.write(90);
      Serial.println("R");
      Serial.println(rightdistance);
      stop2 = true;
      stop3 = false;
    }

    if(delay3.isExpired() && !stop3) {
        headservo.write(155);
      // Serial.println("c");
      stop3 = true;
      stop4 = false;
    }

    if(delay4.isExpired() && !stop4) {
      leftdistance = averagedistance;
        headservo.write(90);
      Serial.println("L");
      Serial.println(leftdistance);
      stop4 = true;
      delay(700);
      // averagedistance = 0;
      blockreset = false;
      stoper1 = false;
      backward(true);
      reversingtimer1.start(200, AsyncDelay::MILLIS);
      // Serial.println("zadni");
      stopcheckaround = true;
    }
    

  }


 if(rightdistance > 0 && leftdistance > 0 && stoper1 && reversingtimer2.isExpired()) {

//  Serial.println("shemovida aq");
  correctheading = azimuth;

  if(rightdistance >= leftdistance) {
    // Serial.println("mibrundi1");
      deg = 50;
      correctheading = azimuth;
      turningtimer.start(10000, AsyncDelay::MILLIS);
      turnrightactive = true;
      turnedonright = false;
      leftdistance = 0;
      rightdistance = 0;
  } else if(leftdistance > rightdistance) {
    // Serial.println("mibrundi2");
      deg = 50;
      correctheading = azimuth;
      turningtimer.start(10000, AsyncDelay::MILLIS);
      turnleftactive = true;
      turnedonleft = false;
      leftdistance = 0;
      rightdistance = 0;
  }

}

if(reversingtimer1.isExpired() && !stoper1) {
  stoper1 = true;
  backward(false);
  reversingtimer2.start(200, AsyncDelay::MILLIS);
  // Serial.println("gachereba");
} 



}


//////////////////////////////////////////////// /


void setup()     
{     
  Serial.begin(9600);     
  irrecv.enableIRIn();     
  ledtimer.start(ledinterval, AsyncDelay::MILLIS);
  compasstimer.start(200, AsyncDelay::MILLIS);
  compass.init();
  compass.setCalibration(-1156, 303, 0, 2006, -1128, 0);
  pinMode(leftforward, OUTPUT);
  pinMode(leftbackward, OUTPUT);
  pinMode(rightforward, OUTPUT);
  pinMode(rightbackward, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT); 
  pinMode(ledpin, OUTPUT); 
  headservo.attach (3);
  headservo.write(90); 
  usecompass();
  correctheading = azimuth;
  usesonic();
  useaveragenum(30, distance);
  reset();
}  






void loop(){

  if(setmode == 0) { 
    usecompass();
    usesonic();
    useaveragenum(30, distance);
    usehybridmode();
    turnright();
    turnleft();
  } else if(setmode == 2) {
    useautonomy();
    usecompass();
    usesonic();
    useaveragenum(30, distance);
    turnright();
    turnleft();
  }

  reciveir();

//  Serial.println(averagedistance);
    
}
