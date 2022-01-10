
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19
//#define stepSize 1
//#define gearRatio 1

#define DEGREES_STEP 0.1125

float desiredAngle = 0;
float currentAngle = 0;
float diffAngle = 0;
int t = 0;

void setup()
{ 
  Serial.begin(9600);

  pinMode(Z_STEP_PIN, OUTPUT);
  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);

  digitalWrite(Z_ENABLE_PIN, LOW);

}

void loop()
{  

  desiredAngle = 180*sin(4*0.00018*(++t));

  diffAngle = desiredAngle - currentAngle;

//  Serial.print("Current: ");
//  Serial.println(currentAngle);
//  Serial.print("Desired: ");
//  Serial.println(desiredAngle);
//  Serial.print("Diff: ");
//  Serial.println(diffAngle);

  if (diffAngle > 0) {
    digitalWrite(Z_DIR_PIN, HIGH);
            currentAngle += DEGREES_STEP;
  } else { 
    digitalWrite(Z_DIR_PIN, LOW);
            currentAngle -= DEGREES_STEP;
  }
  
  if (abs(diffAngle) >= DEGREES_STEP) {
    digitalWrite(Z_STEP_PIN, HIGH);
    delay(1);
    digitalWrite(Z_STEP_PIN, LOW);
    
   // diffAngle = desiredAngle - currentAngle;
//    Serial.print("Current: ");
//    Serial.println(currentAngle);
//    Serial.print("Desired: ");
//    Serial.println(desiredAngle);
//    Serial.print("Diff: ");
//    Serial.println(diffAngle);
      //if (diffAngle > 0) {
        //currentAngle += DEGREES_STEP;
      //} else { 
        //currentAngle -= DEGREES_STEP;
  //}

  }

  
  delay(1);
}
