// init hips

void OdriveInit1() {

      Serial.println("ODrive 1 - back hips");

      for (int axis = 0; axis < 2; ++axis) {
          Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 50000.0f << '\n';
          Serial1 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
          Serial1 << "w axis" << axis << ".motor.config.calibration_current " << 20.0f << '\n';
  
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive1.run_state(axis, requested_state, false); // don't wait 
      }               
}

void OdriveInit4() {

      Serial.println("ODrive 4 - front hips");

      for (int axis = 0; axis < 2; ++axis) {
          Serial4 << "w axis" << axis << ".controller.config.vel_limit " << 50000.0f << '\n';
          Serial4 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
          Serial4 << "w axis" << axis << ".motor.config.calibration_current " << 20.0f << '\n';
   
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive4.run_state(axis, requested_state, false); // don't wait 
      }             
}

void OdriveInit2() {

      Serial.println("init all axis 0s");

          Serial2 << "w axis" << 0 << ".controller.config.vel_limit " << 50000.0f << '\n';
          Serial2 << "w axis" << 0 << ".motor.config.current_lim " << 20.0f << '\n';
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(0, requested_state, false); // don't wait 
                    
          Serial3 << "w axis" << 0 << ".controller.config.vel_limit " << 50000.0f << '\n';
          Serial3 << "w axis" << 0 << ".motor.config.current_lim " << 20.0f << '\n'; 
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive3.run_state(0, requested_state, false); // don't wait 

          Serial5 << "w axis" << 0 << ".controller.config.vel_limit " << 50000.0f << '\n';
          Serial5 << "w axis" << 0 << ".motor.config.current_lim " << 20.0f << '\n';    
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive5.run_state(0, requested_state, false); // don't wait      

          Serial6 << "w axis" << 0 << ".controller.config.vel_limit " << 50000.0f << '\n';
          Serial6 << "w axis" << 0 << ".motor.config.current_lim " << 20.0f << '\n'; 
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 0 << ": Requesting state " << requested_state << '\n';
          odrive6.run_state(0, requested_state, false); // don't wait           
            
}

void OdriveInit3() {

          Serial.println("init all axis 1s");
    
          Serial2 << "w axis" << 1 << ".controller.config.vel_limit " << 50000.0f << '\n';
          Serial2 << "w axis" << 1 << ".motor.config.current_lim " << 20.0f << '\n';
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(1, requested_state, false); // don't wait
    
          Serial3 << "w axis" << 1 << ".controller.config.vel_limit " << 50000.0f << '\n';
          Serial3 << "w axis" << 1 << ".motor.config.current_lim " << 20.0f << '\n';
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive3.run_state(1, requested_state, false); // don't wait
    
          Serial5 << "w axis" << 1 << ".controller.config.vel_limit " << 50000.0f << '\n';
          Serial5 << "w axis" << 1 << ".motor.config.current_lim " << 20.0f << '\n';
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive5.run_state(1, requested_state, false); // don't wait
    
          Serial6 << "w axis" << 1 << ".controller.config.vel_limit " << 50000.0f << '\n';
          Serial6 << "w axis" << 1 << ".motor.config.current_lim " << 20.0f << '\n';
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << 1 << ": Requesting state " << requested_state << '\n';
          odrive6.run_state(1, requested_state, false); // don't wait
                   
}

void modifyGains() {

          Serial.println("modfy gains");

          float posGain = 60.0;        // 20
          float velGain = 0.0008;       // 0.0005
          float integrator = 0.0005;     // 0.0005

          Serial1 << "w axis" << 0 << ".controller.config.pos_gain " << posGain << '\n';    // back hip
          Serial1 << "w axis" << 1 << ".controller.config.pos_gain " << posGain << '\n';    // back hip

          Serial2 << "w axis" << 0 << ".controller.config.pos_gain " << posGain << '\n';    // back left shoulder
          Serial2 << "w axis" << 1 << ".controller.config.pos_gain " << posGain << '\n';    // back left knee

          Serial3 << "w axis" << 0 << ".controller.config.pos_gain " << posGain << '\n';    // back right shoulder
          Serial3 << "w axis" << 1 << ".controller.config.pos_gain " << posGain << '\n';    // back right knee

          Serial4 << "w axis" << 0 << ".controller.config.pos_gain " << posGain << '\n';    // front hip
          Serial4 << "w axis" << 1 << ".controller.config.pos_gain " << posGain << '\n';    // front hip

          Serial5 << "w axis" << 0 << ".controller.config.pos_gain " << posGain << '\n';    // front shoulder
          Serial5 << "w axis" << 1 << ".controller.config.pos_gain " << posGain << '\n';    // front knee

          Serial6 << "w axis" << 0 << ".controller.config.pos_gain " << posGain << '\n';    // front shoulder
          Serial6 << "w axis" << 1 << ".controller.config.pos_gain " << posGain << '\n';    // front knee

          // ******

          Serial1 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';    // back hip
          Serial1 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';    // back hip

          Serial2 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';    // back left shoulder
          Serial2 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';    // back left knee

          Serial3 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';    // back right shoulder
          Serial3 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';    // back right knee

          Serial4 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';    // front hip
          Serial4 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';    // front hip

          Serial5 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';    // front shoulder
          Serial5 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';    // front knee

          Serial6 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';    // front shoulder
          Serial6 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';    // front knee

          // ******

          Serial1 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';    // back hip
          Serial1 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';    // back hip

          Serial2 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';    // back left shoulder
          Serial2 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';    // back left knee

          Serial3 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';    // back right shoulder
          Serial3 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';    // back right knee

          Serial4 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';    // front hip
          Serial4 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';    // front hip

          Serial5 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';    // front shoulder
          Serial5 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';    // front knee

          Serial6 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';    // front shoulder
          Serial6 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';    // front knee
          


}

void applyOffsets() {
          // apply initial offsets to ODrives taking into account encoder index position

          // hips
          odrive1.SetPosition(0, offSet10);         // back left
          odrive1.SetPosition(1, offSet11);         // back right
          odrive4.SetPosition(0, offSet40);         // front left
          odrive4.SetPosition(1, offSet41);         // front right

          // back right leg
          odrive2.SetPosition(0, offSet20);         // shoulder
          odrive2.SetPosition(1, offSet21);         // knee

          // back left leg
          odrive3.SetPosition(0, offSet30);         // shoulder
          odrive3.SetPosition(1, offSet31);         // knee

          // front left leg
          odrive5.SetPosition(0, offSet50);         // shoulder
          odrive5.SetPosition(1, offSet51);         // knee

          // front right leg
          odrive6.SetPosition(0, offSet60);         // shoulder
          odrive6.SetPosition(1, offSet61);         // knee
}

void driveJoints(int joint, int pos) {
          // takes into account the original setup offsets for motor postions, and also turns around directions so they are consistent
          // also constrains the motion limts for each joint          

          // hips
          if (joint == 10) {
              pos = constrain(pos, -1500,1500);
              odrive1.SetPosition(0, (pos*-1)+offSet10);    // back left hip
          }        
          else if (joint == 11) {
              pos = constrain(pos, -1500,1500);
              odrive1.SetPosition(1, pos+offSet11);    // back right hip
          }        
          else if (joint == 40) {
              pos = constrain(pos, -1500,1500);
              odrive4.SetPosition(0, (pos*-1)+offSet40);    // front left hip      
          }        
          else if (joint == 41) {
              pos = constrain(pos, -1500,1500);
              odrive4.SetPosition(1, pos+offSet41);    // front right hip       
          }

          // shoulders
          else if (joint == 20) {
              pos = constrain(pos, -2000,4000);
              odrive2.SetPosition(0, pos+offSet20);    // back left shoulder
          }
          else if (joint == 30) {
              pos = constrain(pos, -2000,4000);
              odrive3.SetPosition(0, (pos*-1)+offSet30);    // front right shoulder
          }
          else if (joint == 50) {
              pos = constrain(pos, -2000,4000);
              odrive5.SetPosition(0, pos+offSet50);    // front left shoulder
          }
          else if (joint == 60) {
              pos = constrain(pos, -2000,4000);
              odrive6.SetPosition(0, (pos*-1)+offSet60);    // front right shoulder
          }

          // hips
          else if (joint == 21) {
              pos = constrain(pos, -2500,3000);
              odrive2.SetPosition(1, pos+offSet21);    // back left knee
          }
          else if (joint == 31) {
              pos = constrain(pos, -2500,3000);
              odrive3.SetPosition(1, (pos*-1)+offSet31);    // back right knee
          }
          else if (joint == 51) {
              pos = constrain(pos, -2500,3000);
              odrive5.SetPosition(1, pos+offSet51);    // front left knee
          }
          else if (joint == 61) {
              pos = constrain(pos, -2500,3000);
              odrive6.SetPosition(1, (pos*-1)+offSet61);    // front right knee
          }

          

          
}
















