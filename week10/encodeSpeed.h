
float revCounts=0;
float motorSpeed=0; //current motor speed
unsigned long prevTime = 0;  //previous time used to calculate
float speedFreq=0;            //motor speed in HZ

void stepsCounter(void pvParameters){  //count motor steps
  while(1){
    if digitalRead(25) == HIGH{
      revCounts++;
    }
  }
}
void read_speed(){
if (revCounts >= 24){                   //calculate motor speed
      unsigned long currentTime= millis();
      int timeDiff = currentTime - prevTime;
      speedFreq = (revCounts *1000) / (24* (timeDiff));
      prevTime = currentTime;
      revCounts = 0;
      motorSpeed = speedFreq * 60;
      Serial.print("Motor Speed: ");
    Serial.println(motorSpeed);
    delay(10);
    }

}
