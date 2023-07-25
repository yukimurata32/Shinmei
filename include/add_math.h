#pragma once

#include <array>
#include <cmath>

class ArrayMath2{
    public:

    /* 
    Judge whether two lines cross each other with 2-D perspective or not.
    Input:
        *line1's startpoint: line1_a
        *line1's endpoint:   line1_b
        *line2's startpoint: line2_a
        *line2's endpoint:   line2_b
    Output:
        *Judgement: true or false
    */
    static bool LineIntersection(const std::array<float, 3>& line1_a,const std::array<float, 3>& line1_b,const std::array<float, 3>& line2_a,const std::array<float, 3>& line2_b){
        float s,t;
        s = (line1_a[0] - line1_b[0]) * (line2_a[1] - line1_a[1]) - (line1_a[1] - line1_b[1]) * (line2_a[0] - line1_a[0]);
        t = (line1_a[0] - line1_b[0]) * (line2_b[1] - line1_a[1]) - (line1_a[1] - line1_b[1]) * (line2_b[0] - line1_a[0]);
        if (s * t >= 0)
            return false; //Two lines don't intersect each others.
        //Serial.print("s="); Serial.println(s);
        s = (line2_a[0] - line2_b[0]) * (line1_a[1] - line2_a[1]) - (line2_a[1] - line2_b[1]) * (line1_a[0] - line2_a[0]);
        t = (line2_a[0] - line2_b[0]) * (line1_b[1] - line2_a[1]) - (line2_a[1] - line2_b[1]) * (line1_b[0] - line2_a[0]);
        if (s * t >= 0)
            return false; //Two lines don't intersect each others.
        return true; //Two lines intersect each others.
    }

    /* 
    Derive smallest distance between two lines. 
    Input:
        *line1's startpoint: line1_a
        *line1's endpoint:   line1_b
        *line2's startpoint: line2_a
        *line2's endpoint:   line2_b
    Output:
        *distance between two lines : dist
    */
    static void DistanceBetweenTwoLines(std::array<float,3>& output, const std::array<float, 3>& line1_a,const std::array<float, 3>& line1_b,const std::array<float, 3>& line2_a,const std::array<float, 3>& line2_b){
        /*float t1=0.5,t2=0.5;
        float delta_t1=0.01,delta_t2=0.01;
        std::array<float,3> OptimizedPoint1;
        std::array<float,3> OptimizedPoint2;
        for(int i=0;i<3;i++){
            OptimizedPoint1[i] = t1 * (line1_b[i]- line1_a[i]) + line1_a[i];
            OptimizedPoint2[i] = t2 * (line2_b[i]- line2_a[i]) + line2_a[i];
        }
        float OptimizedDistance = ArrayMath::distance(OptimizedPoint1,OptimizedPoint2);

        for(int j=0 ; j < 50 ; j++){
            std::array<float,3> Point1_,Point2_;
            for(int i=0;i<3;i++){
                Point1_[i] = (t1 + delta_t1) * (line1_b[i]- line1_a[i]) + line1_a[i];
                Point2_[i] = (t2 + delta_t2) * (line2_b[i]- line2_a[i]) + line2_a[i];
            }

            float Distance1 = ArrayMath::distance(Point1_,OptimizedPoint2);
            float Distance2 = ArrayMath::distance(OptimizedPoint1,Point2_);
            delta_t1 = - 0.5* (Distance1 - OptimizedDistance)/(delta_t1);
            delta_t2 = - 0.5* (Distance2 - OptimizedDistance)/(delta_t2);
            t1 += delta_t1;
            t2 += delta_t2;
            OptimizedPoint1 = Point1_;
            OptimizedPoint2 = Point2_;
            OptimizedDistance = ArrayMath::distance(OptimizedPoint1,OptimizedPoint2);
        }
        for(int i=0;i<3;i++){
            output[i] = OptimizedPoint2[i] - OptimizedPoint1[i];
        }
        //return OptimizedDistance;
        */



        /*std::array<float,3> p1 = line1_a,p2 = line2_a,v1,v2,p12;
        for(int i=0;i<3;i++){
            v1[i] = line1_b[i] - line1_a[i];
            v2[i] = line2_b[i] - line2_a[i];
            p12[i] = p2[i] - p1[i];
        }

        std::array<float,3> v12_outer,direct;
        ArrayMath::outer(v1, v2, v12_outer);
        float v12_outer_norm = ArrayMath::norm(v12_outer);
        if(v12_outer_norm!=0){
        }else{
            
        }*/
        
        //https://www.mathcal.org/canvas/vector/suisen.html
        std::array<float,3> a,b,c,d;
        a = line1_a;
        b = line2_a;
        for(int i=0;i<3;++i){
            c[i] = line1_b[i] - line1_a[i];
            d[i] = line2_b[i] - line2_a[i];
        }
        float A,B,C,D,E,F;
        A =   ArrayMath::dot(b,c) - ArrayMath::dot(a,c);
        B =   ArrayMath::dot(d,c); 
        C = - ArrayMath::dot(c,c); 
        D =   ArrayMath::dot(b,d) - ArrayMath::dot(a,d);
        E =   ArrayMath::dot(d,d); 
        F = - ArrayMath::dot(c,d); 

        float s,t;
        s = - (A*E - B*D)/(E*C - B*F);
        t = + (A*F - C*D)/(E*C - B*F);

        std::array<float,3> p,q;
        // p = a + s*c;
        // q = b + t*d; 
        for(int i=0;i<3;++i){
            p[i] = a[i] + s * c[i];
            q[i] = b[i] + t * d[i];
            output[i] = q[i] -p[i];
        }
    }

    /*void PointOnLine(std::array<float,3>& output, const std::array<float, 3>& start,const std::array<float, 3>& end, const float t){
        for(int i=0;i<3;i++)
            output[i] = t * (end[i]- start[i]) + start[i];
    }*/
};