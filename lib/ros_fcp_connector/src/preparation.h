#include <motor2.h>
#include <cmath>
#include <parameters.h>
#include <robot.h>
#include <properties.h>

/* preparation of walking */
namespace Preparation {

    /*std::array<std::array<float, 3>, 6> standing_position{
    {{320, -210, -BODY_HEIGHT},
     {0, -210, -BODY_HEIGHT},
     {-250, -210, -BODY_HEIGHT},
     {-250, 210, -BODY_HEIGHT},
     {0, 210, -BODY_HEIGHT},
     {320, 210, -BODY_HEIGHT}}};*/

    float LINK_LENGTH012=LINK_LENGTH[0]*std::sin(INCLINATION)+LINK_LENGTH[1]*std::cos(INCLINATION)+LINK_LENGTH[2];
    std::array<std::array<float, 3>, 6> standing_position{{
        {RELATIVE_POS[0][0]+std::cos(RELATIVE_ANGLE[0])*LINK_LENGTH012, RELATIVE_POS[0][1]+std::sin(RELATIVE_ANGLE[0])*LINK_LENGTH012, -BODY_HEIGHT},
        {RELATIVE_POS[1][0]+std::cos(RELATIVE_ANGLE[1])*LINK_LENGTH012, RELATIVE_POS[1][1]+std::sin(RELATIVE_ANGLE[1])*LINK_LENGTH012, -BODY_HEIGHT},
        {RELATIVE_POS[2][0]+std::cos(RELATIVE_ANGLE[2])*LINK_LENGTH012, RELATIVE_POS[2][1]+std::sin(RELATIVE_ANGLE[2])*LINK_LENGTH012, -BODY_HEIGHT},
        {RELATIVE_POS[3][0]+std::cos(RELATIVE_ANGLE[3])*LINK_LENGTH012, RELATIVE_POS[3][1]+std::sin(RELATIVE_ANGLE[3])*LINK_LENGTH012, -BODY_HEIGHT},
        {RELATIVE_POS[4][0]+std::cos(RELATIVE_ANGLE[4])*LINK_LENGTH012, RELATIVE_POS[4][1]+std::sin(RELATIVE_ANGLE[4])*LINK_LENGTH012, -BODY_HEIGHT},
        {RELATIVE_POS[5][0]+std::cos(RELATIVE_ANGLE[5])*LINK_LENGTH012, RELATIVE_POS[5][1]+std::sin(RELATIVE_ANGLE[5])*LINK_LENGTH012, -BODY_HEIGHT}
    }};

    /**
     * @brief standing motion for initialization
     * @param standing_position: contact point in each leg
     * @param max_speed: max speed
     * @param acc: acceleration at beginning and end of motion  
     */
    void Stand(Robot& r, Motor& m, int number_of_trying,std::array<std::array<float, 3>, 6> standing_position,
    const float max_speed,const float acc) {
        std::array<PTP,6> ptp_;

        /* number_of_trying == 0 : move position of each tip of legs to standby point */ 
        /* number_of_trying == 1 : put each tip of leg down to the ground */ 
        std::array<std::array<float, 3>, 6> target_posL;
        m.getSyncAllPosition(r.legs_, false);
        for(Leg& leg: r.legs_){
            Kinematics::ConvertBodyToLeg(standing_position[leg.getID()], target_posL[leg.getID()], leg);
            if(number_of_trying == 0){ /*target_posL[leg.getID()][0] += 40.0;*/ target_posL[leg.getID()][2] = 0.0; }
        }

        while (true){
            int ready_legs = 0;
            m.getSyncAllPosition(r.legs_, false);
            for (Leg& leg: r.legs_){
                Kinematics::FwdKinematics(leg.getAngle(), leg.getTipLpos(), leg);
                if (ptp_[leg.getID()].point_to_point_profile(leg.getTipLpos(), target_posL[leg.getID()], max_speed,acc) < 10.0) ready_legs++;
                Kinematics::ConvertLegToBody(leg.getTipLpos(), leg.getTipBpos(), leg);
                Kinematics::InvKinematics(leg);
            }
            m.setSyncAllPosition(r.legs_);
            if (ready_legs == 6) break;
            delay(15);
        }

        delay(200); // motor の収束待ち
        m.getSyncAllPosition(r.legs_, true);
        for (Leg& leg : r.legs_) { Kinematics::FwdKinematicsAll(leg, true); leg.sync();}
    }


    void set_raise_order(Robot& r, std::array<int8_t,6>& raise_order){
		raise_order.fill(0);
		for(Leg& leg : r.legs_){
			int id = leg.getID();
			for(Leg& leg2 : r.legs_){
				int id2 = leg2.getID();
				if(id != id2){
                    bool intersected_flag=false;
					for(int i=0 ; i<3 and intersected_flag==false ; i++){
						for(int j=0 ; j<3 and intersected_flag==false ; j++){
							intersected_flag = ArrayMath2::LineIntersection(leg.node_bpos_[i],leg.node_bpos_[i+1],leg2.node_bpos_[j],leg2.node_bpos_[j+1]);
							if(intersected_flag==true){
								Serial.println();
								Serial.print(leg.getID()); Serial.print(":,");Serial.print(i); Serial.print(",");Serial.println(j);
								std::array<float,3> dist;
								ArrayMath2::DistanceBetweenTwoLines(dist,leg.node_bpos_[i],leg.node_bpos_[i+1],leg2.node_bpos_[j],leg2.node_bpos_[j+1]);
								if(dist[2]>0)
									raise_order[id]++;
							}
						}
					}
				}
			}
		}
	}

	void Raise_under_raise_order(Robot& r, Motor& m) {

		m.getSyncAllPosition(r.legs_,false);
		for(Leg& leg : r.legs_)
			Kinematics::FwdKinematicsAll(leg, false);
		std::array<int8_t,6> raise_order;
		set_raise_order(r,raise_order);

    PTP ptp;
    const float speed = 20.0f;
		int8_t max_raise_order=0;
    for (int i = 0; i < 6; ++i) {
			if(max_raise_order<raise_order[i]) max_raise_order=raise_order[i];
    }

    for (Leg& leg: r.legs_){
      m.setEnableTorque(leg,0,false);
      m.setEnableTorque(leg,2,false);
    }

		for(int order=0;order<=max_raise_order;order++){
			int ordered_num=0;
			for(Leg& leg: r.legs_){
				if(raise_order[leg.getID()]<=order) ordered_num++;
			}

			while (true){
				int ready_legs = 0;
				m.getSyncAllPosition(r.legs_, false);
				for (Leg& leg: r.legs_){
					leg.setObsAngle(leg.getAngle());
					const int id = leg.getID();
					Kinematics::FwdKinematics(leg.getAngle(), leg.getTipLpos(), leg);
					std::array<float, 3> target_pos,target_angle;
					Kinematics::ConvertBodyToLeg(standing_position[id], target_pos, leg);
					target_pos[0] +=100.0;
					target_pos[2] = 150.0;
					Kinematics::InvKinematics(target_pos,target_angle,leg);
					if(raise_order[id]<=order){
						if (ptp.point_to_point(leg.getTipLpos(), target_pos, speed) < 5.0) ready_legs++;
					}
					Kinematics::ConvertLegToBody(leg.getTipLpos(), leg.getTipBpos(), leg);
					Kinematics::InvKinematics(leg);
					
					m.setPosition(leg.getAngle()[1],id,1);
					if(std::abs(target_angle[1]-leg.getObsAngle()[1])>M_PI/32.0*4){
						m.setEnableTorque(leg,0,false);
						m.setEnableTorque(leg,2,false);
					}else{
						m.setEnableTorque(leg,true);
						//delay(15);
						m.setSyncPosition(leg);
						//m.setEnableTorque(leg,2,true);
						//m.setPosition(leg.getAngle()[2],id,2);
						/*if(std::abs(target_angle[2]-leg.getObsAngle()[2])<=M_PI/32.0){
							m.setEnableTorque(leg,0,true); 
							m.setPosition(leg.getAngle()[0],id,0);
							//m.setEnableTorque(leg,true);
							//m.setSyncPosition(leg);
						}*/
					}
				}
				if (ready_legs == ordered_num) break;
				delay(15);
			}
		}
		m.enableTorque(); 
		m.setSyncAllPosition(r.legs_);
    }

};