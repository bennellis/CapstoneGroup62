#define A 2
#define B 3
#define C 4
#define D 5
#define sensor A0 // Sharp IR GP2Y0A41SK0F (4-30cm, analog)
 
#define NUMBER_OF_STEPS_PER_REV 512
boolean moving = 0;

void setup(){
  pinMode(A,OUTPUT);
  pinMode(B,OUTPUT);
  pinMode(C,OUTPUT);
  pinMode(D,OUTPUT);
  Serial.begin(9600); // start the serial port
}

void write(int a,int b,int c,int d){
  digitalWrite(A,a);
  digitalWrite(B,b);
  digitalWrite(C,c);
  digitalWrite(D,d);
}

void onestep(int d){
  write(0,0,0,1);
  delayMicroseconds(d);
  write(0,0,1,1);
  delayMicroseconds(d);
  write(0,0,1,0);
  delayMicroseconds(d);
  write(0,1,1,0);
  delayMicroseconds(d);
  write(0,1,0,0);
  delayMicroseconds(d);
  write(1,1,0,0);
  delayMicroseconds(d);
  write(1,0,0,0);
  delayMicroseconds(d);
  write(1,0,0,1);
  delayMicroseconds(d);
}

void loop(){
  if (!moving){
    float volts = analogRead(sensor)*0.0048828125;  // value from sensor * (5/1024)
    int distance = 13*pow(volts, -1); // worked out from datasheet graph
    delay(1000); // slow down serial port 
      
    if (distance <= 30){
      Serial.println(distance);   // print the distance
    }
    if(distance >5){
      moving = 1;
    }
  }
  else {
    int i;
    int d;
    d = 5000;
    i=0;
    while(i<NUMBER_OF_STEPS_PER_REV*2){
      onestep(d);
      i++;
    }
    moving = 0;
  }
}
