#include <iostream>

bool init_var(bool* rc_car, bool* cartox,int* marker, bool* lane,
float* motor_ki, float* motor_kp, float* motor_kd,
float* servo_ki, float* servo_kp, float* servo_kd, float* motor_speed,
bool* img_stream, float* car_distance
)
{
  bool ret = true;
  std::string line;
  std::ifstream myfile ("src/car_vision/include/HSCD.txt");

  if(myfile.is_open())
  {
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "rc_car")
      *rc_car =
        (bool)atoi(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
    std::cout << line.substr(0,line.find("=")) << " " << *rc_car<<'\n' ;
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "cartox")
      *cartox =
        (bool)atoi(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
    std::cout << line.substr(0,line.find("=")) << " " << *cartox <<'\n' ;
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "marker")
      *marker =
        atoi(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
    std::cout << line.substr(0,line.find("=")) << " " << *marker <<'\n' ;
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "lane")
      *lane =
        (bool)atoi(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
    std::cout << line.substr(0,line.find("=")) << " " << *lane <<'\n' ;
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "motor_ki")
      *motor_ki =
        atof(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
    std::cout << line.substr(0,line.find("=")) << " " << *motor_ki
      <<'\n' ;
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "motor_kp")
      *motor_kp =
        atof(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
    std::cout << line.substr(0,line.find("=")) << " " << *motor_kp
      <<'\n' ;
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "motor_kd")
      *motor_kd =
        atof(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
   std::cout << line.substr(0,line.find("=")) << " " << *motor_kd
      <<'\n' ;
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "servo_ki")
      *servo_ki =
        atof(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
    std::cout << line.substr(0,line.find("=")) << " " << *servo_ki
      <<'\n' ;
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "servo_kp")
      *servo_kp =
        atof(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
    std::cout << line.substr(0,line.find("=")) << " " << *servo_kp
      <<'\n' ;
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "servo_kd")
      *servo_kd =
        atof(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
    std::cout << line.substr(0,line.find("=")) << " " << *servo_kd
      <<'\n' ;
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "motor_speed")
      *motor_speed =
        atof(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
    std::cout << line.substr(0,line.find("=")) << " " << *motor_speed
      <<'\n' ;
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "img_stream")
      *img_stream =
        (bool)atoi(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
    std::cout << line.substr(0,line.find("=")) << " " << *img_stream <<'\n' ;
    getline(myfile, line);
    if(line.substr(0,line.find("="))== "car_distance")
      *img_stream =
        atoi(line.substr(line.find("=")+1,line.find("\n")).c_str());
    else ret = false;
    std::cout << line.substr(0,line.find("=")) << " " << *car_distance <<'\n' ;


    myfile.close();
  }else
  {
    std::cout << "Unable to open file";
    return false;
  }

  std::cout << "Finish reading car parameters " << ret;
  return ret;
}
