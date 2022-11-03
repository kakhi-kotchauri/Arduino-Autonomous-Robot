
// libraryes
#include <AsyncDelay.h>
#include <IRremote.h>    
#include <QMC5883LCompass.h>
#include <Servo.h>


// for servo
Servo headservo;
int servo_position = 0;

// for ir recive  
int RECV_PIN = 12;     
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

int leftforward = 2;
int leftbackward = 4;

int rightforward = 7;
int rightbackward = 8;

bool stopforward = false;
bool blockreverse = true;


// for mode

int setmode = 0;
int ledinterval;
int ledpin = 9;

// for usehybridmode

AsyncDelay servotimer;
AsyncDelay backwardtimer;
bool rightcorrected = false;
bool rightcorrected2 = false;
bool leftcorrected = false;
bool leftcorrected2 = false;
bool blockright = false;
bool blocktimercheck = false;
bool movingonright = false;
bool movingonleft = false;

int leftturndistance;
int rightturndistance;




// for compass
AsyncDelay compasstimer;
QMC5883LCompass compass;

 int correctheading;



// functions for distance measure

void usesonic() {
  // ultrasound
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
    distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  // Serial.print("Distance: ");
}


void useaveragenum(int measurements, int number) {

for(int i = 0; i < measurements; ++i)
{
  calculate += number;
  delay(1);
}

calculate /= measurements;


if(calculate >= 0) {
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
    correctheading = azimuth;
    Serial.println("reset");
    headservo.write(90);
    backwardtimer.start(180, AsyncDelay::MILLIS);
    blocktimercheck = false;
    irtimeractive = false;
    movingonright = false;
    movingonleft = false;
    isreseted = true;
    rightturndistance = 0;
    leftturndistance = 0;
  }

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

    if(oldir == -12241) {
      forward(true); 
      Serial.println("forward");
    } else if(oldir == 28815) {
      backward(true);
      Serial.println("backward");
    } else if(oldir == -30601 && blockreverse) {

       if(setmode == 0 && !blocktimercheck) {
          headservo.write(0);
          servotimer.start(200, AsyncDelay::MILLIS);
          blocktimercheck = true;
       }

       if(servotimer.isExpired() && headservo.read() == 0) {
          rightturndistance = averagedistance;
       }

       if( rightturndistance >= 15) {
          headservo.write(90);
          right(true);
          movingonright = true;
          correctheading = azimuth;
          Serial.println("right");
       } else if(setmode == 1) {
          right(true);
          Serial.println("right");
       }

    } else if(oldir == 2295 && blockreverse) {
      // left(true);

        if(setmode == 0 && !blocktimercheck) {
          headservo.write(180);
          servotimer.start(200, AsyncDelay::MILLIS);
          blocktimercheck = true;
       }

        if(servotimer.isExpired() && headservo.read() == 180) {
            leftturndistance = averagedistance;
       }

       if(leftturndistance >= 13) {
          headservo.write(90);
          left(true);
          movingonleft = true;
          correctheading = azimuth;
          Serial.println("left");
       } else if(setmode == 1) {
          left(true);
          Serial.println("left");
       }

    } else if(oldir == 6375) {
      setmode = 0;
    } else if(oldir == 26775) {
      setmode = 1;
      digitalWrite(ledpin, HIGH);
      stopforward = false;
    } else if(oldir == -17851) {
      digitalWrite(ledpin, LOW);
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

void usehybridmode() {
 
    blinkled(500);

    // Serial.println(movingonright);
    // Serial.println(movingonleft);

  if(!movingonright && !movingonleft) {

    // Serial.println("shemodis kompasshi");


    if(correctheading > 350 && azimuth < 10) {
      correctheading = 5;
    } else if(correctheading < 10 && azimuth > 350) {
      correctheading = 355;
    }

    if(correctheading + 15 < azimuth && azimuth <= 180) {
      left(true);
      rightcorrected = false;
      // Serial.println("leftcorrect1");
      // Serial.println(correctheading);
      // Serial.println(azimuth);
    } else if(!rightcorrected) {
      left(false);
      rightcorrected = true;
    }

    if(correctheading - 15 > azimuth && azimuth <= 180) {
      right(true);
      leftcorrected = false;
    } else if(!leftcorrected) {
      right(false);
      leftcorrected = true;
    }


    if(correctheading + 15 < azimuth && azimuth > 180) {
      left(true);
      // Serial.println("leftcorrect2");
      rightcorrected2 = false;
    } else if(!rightcorrected2) {
      left(false);
      rightcorrected2 = true;
    }


    if(correctheading - 15 > azimuth && azimuth > 180) {
      right(true);
      leftcorrected2 = false;
    } else if(!leftcorrected2) {
      right(false);
      leftcorrected2 = true;
    }

  }



/////////


    if(averagedistance <= 17 && !stopforward && headservo.read() == 90) {
      Serial.println("gacherdi");
      forward(false);
      stopforward = true;
    } else if(stopforward && averagedistance > 17) {
      Serial.println("gaushvi");
      stopforward = false;
    }

    if(averagedistance <= 8 && headservo.read() == 90 && backwardtimer.isExpired()) {
      left(false);
      right(false);
      backward(true);
      blockreverse = false;
    } else if(!blockreverse) {
      backward(false);
      blockreverse = true;
    }


}


////////////////////////////////////////////////


void setup()     
{     
  Serial.begin(9600);     
  irrecv.enableIRIn();     
  ledtimer.start(ledinterval, AsyncDelay::MILLIS);
  compasstimer.start(200, AsyncDelay::MILLIS);
  compass.init();
  compass.setCalibration(-1823, 1000, -823, 1987, -1672, 1270);
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
  }


  reciveir();

}
