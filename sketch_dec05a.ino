const byte  trigPinR = 2 ; // 右邊超音波 觸發腳Trig
const byte  echoPinR = 3 ;  //右邊超音波 接收腳 Echo
unsigned long distanceR ; // 距離 cm

const byte  trigPinL = 4 ; //左邊超音波 觸發腳Trig
const byte  echoPinL = 5 ;  //左邊超音波 接收腳 Echo
unsigned long distanceL ; // 距離 cm

unsigned long pingR() {
    digitalWrite(trigPinR,HIGH) ; //觸發腳位設定為高電位
    delayMicroseconds(10);   //持續5微秒
    digitalWrite(trigPinR,LOW) ;
    return  ( pulseIn(echoPinR,HIGH)/58)  ;  // 換算成 cm 並傳回
  }


unsigned long pingL() {
    digitalWrite(trigPinL,HIGH) ; //觸發腳位設定為高電位
    delayMicroseconds(10);   //持續5微秒
    digitalWrite(trigPinL,LOW) ;
    return  ( pulseIn(echoPinL,HIGH)/58)  ;  // 換算成 cm 並傳回
  }
  
void setup() {
  // put your setup code here, to run once:
  pinMode(trigPinR,OUTPUT) ;
  pinMode(echoPinR,INPUT) ;

  pinMode(trigPinL,OUTPUT) ;
  pinMode(echoPinL,INPUT) ;
  Serial.begin(9600) ;
}

void loop() {
  // put your main code here, to run repeatedly:
    distanceR  = pingR()  ;
    distanceL  = pingL()  ;
//    String str1 ="";
    if (distanceR > 100 & distanceL > 100){
      Serial.print('F') ;
      }
    else if (distanceR <= 100 & distanceL <= 100){
      Serial.print('H') ;
      }
    else if (distanceR <= 100 & distanceL > 100){
      Serial.print('R') ;
      }
    else if (distanceR > 100 & distanceL <= 100){
      Serial.print('L') ;
      }   
//    str1 = " Left=" + String(distanceL) + "cm , Right=" + String(distanceR) + " cm" ;
//    Serial.println(str1) ;
    delay(100) ;
}
