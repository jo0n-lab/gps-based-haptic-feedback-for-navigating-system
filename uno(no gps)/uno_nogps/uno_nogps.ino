// arduino uno 16MHz(클럭) : 보통 pc의 1%
// deg units (lat, lon) 위도, 경도
// v_:벡터(경로) p_:점(위치)
#include <stdio.h>
#include "math.h"
#include "data2.h"
#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#define _USE_MATH_DEFINES
#define R 6371000

int log_num = 0;
int errcnt = 0;
int err = 7;

double dtr_45 = 0.785398; // deg to rad (45)

SoftwareSerial serial_connection(10, 11); // RX=pin10, TX=pin11
TinyGPSPlus gps;                          // gps 객체 생성

double ref_ax[3] = {-0.48, 0.63, 0.61};

double to_xyz(double v[])
{
    double lat = v[0] * M_PI / 180;
    double lon = v[1] * M_PI / 180;
    v[0] = R * cos(lat) * cos(lon);
    v[1] = R * cos(lat) * sin(lon);
    return R * sin(lat);
}

double cal_dist(double v[])
{
    return sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2));
}

double cal_cos(double v1[], double v2[], double dist1, double dist2)
{
    return (v1[0] * v2[0] + v1[1] * v2[1]) / (dist1 * dist2);
} // 내적; cos(theta)

double *cross_prod(double v1[], double v2[])
{
    static double cross[3];
    cross[0] = v1[1] * v2[2] - v1[2] * v2[1];
    cross[1] = v1[2] * v2[0] - v1[0] * v2[2];
    cross[2] = v1[0] * v2[1] - v2[0] * v1[1];
    return cross;
} // 외적; sin(theta)

double cal_err(double v1[], double v2[], double dist1, double dist2)
{
    return cal_dist(cross_prod(v1, v2)) / dist2;
} // 패딩 계산; d

int cal_dir(double v1[], double v2[], double dist1, double dist2)
{
    // v1=way_v, v2=cur_v
    double *cross = cross_prod(v1, v2);
    if (cross[0] * ref_ax[0] + cross[1] * ref_ax[1] + cross[2] * ref_ax[2] > 0)
    {
        return 2;
    }
    else
        return 3;
} // 방향{왼쪽 오른쪽} 계산(외적)

void hp_feedback(int flag)
{
    log_num++;
    //주석 부분에 진동 반응 코딩해주시면 됩니다~
    switch (flag)
    {

    case 1:
        //           out of padding
        Serial.print(log_num);
        Serial.println(" : out of padding");
        break;

        // left direction feedback
    case 2:
        //           left right
        Serial.print(log_num);
        Serial.println(" : left-right");
        break;
    case 3:
        //           left out
        Serial.print(log_num);
        Serial.println(" : left-out");
        break;
    case 4:
        //           left in
        Serial.print(log_num);
        Serial.println(" : left-in");
        break;

        // right direction feedback
    case 5:
        //           right right
        Serial.print(log_num);
        Serial.println(" : right-right");
        break;
    case 6:
        //           right out
        Serial.print(log_num);
        Serial.println(" : right-out");
        break;
    case 7:
        //           right in
        Serial.print(log_num);
        Serial.println(" : right-in");
        break;

    case 8:
        //            arrived
        Serial.print(log_num);
        Serial.println(" : arrived");
        serial_connection.end();
        // while (1);
        break;
    default:
        //            error
        break;
    }
}

double way_p[3] = {w_points[0][0], w_points[0][1], w_points[0][2]};                            //인접한 웨이포인트(시작점 이후)
double way_v[3] = {w_points[0][0] - p_0[0], w_points[0][1] - p_0[1], w_points[0][2] - p_0[2]}; //개별 경로 벡터(끼인각의 기준축)
double cur_p[3];                                                                               //현재 위치 gps.read()
double cur_v[3];
// double p_heading[2]; //imu.read()

double cur_dist;
double way_dist;

int way = 0;
int dir = 0;

void setup()
{
    Serial.begin(9600);
    // serial_connection.begin(9600); // gps 통신 시작
    // Serial.println("GPS Start");
}

void loop()
{
    // while (serial_connection.available())
    // { // gps 통신 끝나는 시점(도착)까지
    for (int i = 0; i < g_size; i++)
    {
        // gps.encode(serial_connection.read()); // gps 센서의 값 읽기
        // if (gps.location.isUpdated())
        // {
        errcnt = 0;
        cur_p[0] = g_points[i][0];
        cur_p[1] = g_points[i][1];
        cur_p[2] = to_xyz(cur_p);

        //     // p_heading[0]=imu.comp.x();
        //     // p_heading[1]=imu.comp.y();

        cur_v[0] = way_p[0] - cur_p[0];
        cur_v[1] = way_p[1] - cur_p[1];
        cur_v[2] = way_p[2] - cur_p[2];
        //     // cur_v 갱신
        Serial.print("current gps(");
        Serial.print(i + 1);
        Serial.print(") : ");
        Serial.print(cur_p[0]);
        Serial.print(" ");
        Serial.print(cur_p[1]);
        Serial.print(" ");
        Serial.println(cur_p[2]);

        Serial.print("waypoint(");
        Serial.print(way + 1);
        Serial.print(") : ");
        Serial.print(way_p[0]);
        Serial.print(" ");
        Serial.print(way_p[1]);
        Serial.print(" ");
        Serial.println(way_p[2]);

        cur_dist = cal_dist(cur_v);
        way_dist = w_dists[way];

        Serial.print("cur_dist :");
        Serial.println(cur_dist);

        if (cur_dist < err)
        {
            log_num++;
            //웨이포인트 도착하면
            way++;
            way_p[0] = w_points[way][0];
            way_p[1] = w_points[way][1];
            way_p[2] = w_points[way][2];
            //새로운 웨이포인트
            way_v[0] = w_points[way][0] - w_points[way - 1][0];
            way_v[1] = w_points[way][1] - w_points[way - 1][1];
            way_v[2] = w_points[way][2] - w_points[way - 1][2];
            //새로운 웨이포인트 벡터

            // let it happen hp_feedback
            Serial.print(log_num);
            Serial.println(" : next waypoint");
        }
        else
        {
            if (cal_err(cur_v, way_v, cur_dist, way_dist) > err)
            { //±5m padding 이탈 시
                hp_feedback(1);
                Serial.print("(err : ");
                Serial.print(cal_err(cur_v, way_v, cur_dist, way_dist));
                Serial.print(" sin : ");
                Serial.println(cal_dist(cross_prod(cur_v, way_v)) / (cur_dist * way_dist));
            }
            // else if (cur_dist < 0.40 * way_dist && cur_dist > 0.33 * way_dist)
            else
            { // 근접한 웨이포인트 방향 알림
                // double theta=p_heading-acos(cal_cos(cur_v,way_v))*180/M_PI;
                hp_feedback(dirs[dir]);
            }
        }
        // }
        // else if (errcnt < 10)
        // {
        //     errcnt++;
        // }
        // else
        // {
        //     Serial.println("error : check your gps module works properly");
        // }
        delay(100);
    }
    // }
    while (1)
        ;
}
