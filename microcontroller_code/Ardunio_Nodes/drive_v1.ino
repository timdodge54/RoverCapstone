#include <ros.h>
#include <drive/int8List.h>
#include <drive/boolList.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>


int a=0;
int b=0;
int c=0;
int d=0;
int e=0;
int f=0;

bool forward = false;
bool back = false;
bool quad = false;
int currentCamera = 44;
int quadPin = 52;

ros::NodeHandle ArdMegDrive;
std_msgs::String str_msg;
ros::Publisher log_info("rover_log", &str_msg);

std_msgs::Int16 heading;
ros::Publisher head("heading", &heading);


void rosmsg( const drive::int8List& msg){

  a=msg.data[0];
  b=msg.data[1];
  c=msg.data[2];
  d=msg.data[3];
  e=msg.data[4];
  f=msg.data[5];

  sendSerial();



  str_msg.data="ArdMegDrive:Running";
  log_info.publish(&str_msg);
}

void cammsg(const drive::boolList& msg){
  digitalWrite(44, HIGH);
  digitalWrite(46, HIGH);
  digitalWrite(48, HIGH);
  digitalWrite(50, HIGH);
  //digitalWrite(52, HIGH);
  
  forward = msg.data[0];
  back = msg.data[1];
  quad = msg.data[2];

  if(forward > 0)
  {
    currentCamera += 2;

    if(currentCamera > 50)
    {
      currentCamera = 44;
    }

    digitalWrite(currentCamera, HIGH);
    delay(50);
    digitalWrite(currentCamera, LOW);
  }
  if(back > 0)
  {
    currentCamera -= 2;

    if(currentCamera < 44)
    {
      currentCamera = 50;
    }

    digitalWrite(currentCamera, HIGH);
    delay(50);
    digitalWrite(currentCamera, LOW);
  }
  if(quad > 0)
  {
    digitalWrite(currentCamera, HIGH);
    delay(50);
    digitalWrite(currentCamera, LOW);
  }
}

void getstat(const std_msgs::Empty& getstat){
    str_msg.data="ArdMegDrive:Running";
    log_info.publish(&str_msg);

  
    ArdMegDrive.spinOnce();  
}


ros::Subscriber<drive::int8List> sub("cDrive", rosmsg );
ros::Subscriber<drive::boolList> sub2("cCamera", cammsg );

ros::Subscriber<std_msgs::Empty> sublog("get_rover_log", getstat);

void setup() {
  
  //open the serial connection via ros
  //the following command must be run on the host to open the connection on that end
  //rosrun rosserial_arduino serial_node.py /dev/ttyACM0 
  //ACM0 may need to change depending on the mounting order
  ArdMegDrive.initNode();
  //subscribe to the node
  ArdMegDrive.subscribe(sub);
  ArdMegDrive.subscribe(sub2);
  ArdMegDrive.subscribe(sublog);
  ArdMegDrive.advertise(log_info);
  ArdMegDrive.advertise(head);
  //attach each servo to respective pin
  str_msg.data="Drive arduino intilized";
  delay(5000);
  log_info.publish(&str_msg);

  pinMode(currentCamera, OUTPUT);
  pinMode(currentCamera + 2, OUTPUT);
  pinMode(currentCamera + 4, OUTPUT);
  pinMode(currentCamera + 6, OUTPUT);

  digitalWrite(currentCamera, HIGH);
  digitalWrite(currentCamera + 2, HIGH);
  digitalWrite(currentCamera + 4, HIGH);
  digitalWrite(currentCamera + 6, HIGH);
  
  Serial2.begin(115200);
  Serial2.print("<0,0,-3,-1,-1,-1>");
  
}


void sendSerial(){
Serial2.print("<"+String(a)+","+String(b)+","+String(c)+","+String(d)+","+String(e)+","+String(f)+">");  

}

void loop() {
  ArdMegDrive.spinOnce();

  if (Serial2.available()>0){
  Serial2.read();
  }else{
    
  
    ;//this is if we want to read data back
 }

  
  delay(100);
}


